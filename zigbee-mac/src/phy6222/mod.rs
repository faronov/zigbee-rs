//! PHY6222/PHY6252 MAC backend — pure Rust, zero vendor blobs.
//!
//! Implements `MacDriver` for the Phyplus PHY6222/6252 ARM Cortex-M0 SoC.
//! The PHY62x2 has a multi-protocol radio (BLE + IEEE 802.15.4) and is found
//! in many low-cost Tuya smart home devices (THB2, TH05F, BTH01, etc.).
//!
//! All radio configuration uses direct register access in [`driver`]; the
//! register values are transcribed from the PHYPLUS PHY62xx SDK
//! (`rf_phy_driver.c`, `ll_hw_drv.c`) as redistributed by pvvx (THB2 and
//! phy62x2-SDK). No vendor libraries or ROM radio routines are linked.
//! ACKs, CSMA-CA and address filtering are implemented in software here.

pub mod driver;

use crate::frames::{
    build_association_request, build_beacon_request, build_data_frame, build_data_request,
    build_data_request_short, frame_is_for_us, is_exact_destination, parse_association_response,
    parse_beacon, parse_mac_addresses,
};
use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError, PlatformServices};
use driver::{Phy6222Driver, RadioConfig, RadioError};
use zigbee_types::*;

use embassy_time::{Instant, Timer};

/// How long a poll waits for the parent's frame after a frame-pending ACK
/// (the nRF backend's hardware-proven value; the parent's indirect
/// transmission competes with other traffic on a busy channel).
const POLL_DATA_WAIT_MS: u64 = 1500;

const fn scan_duration_ms(scan_duration: u8) -> Option<u32> {
    if scan_duration > 14 {
        return None;
    }
    Some(((1u32 << scan_duration) * 384 / 25) + 1)
}

/// PHY6222 IEEE 802.15.4 MAC driver.
///
/// Pure-Rust implementation using direct register access — no vendor FFI.
pub struct Phy6222Mac {
    driver: Phy6222Driver,
    short_address: ShortAddress,
    pan_id: PanId,
    channel: u8,
    extended_address: IeeeAddress,
    coord_short_address: ShortAddress,
    coord_extended_address: IeeeAddress,
    associated_pan_coord: bool,
    rx_on_when_idle: bool,
    association_permit: bool,
    auto_request: bool,
    dsn: u8,
    bsn: u8,
    beacon_payload: PibPayload,
    max_csma_backoffs: u8,
    min_be: u8,
    max_be: u8,
    max_frame_retries: u8,
    promiscuous: bool,
    /// Buffer for frames received during association (e.g. Transport-Key).
    /// Returned by the next mlme_poll() call.
    pending_assoc_frame: Option<MacFrame>,
}

impl Default for Phy6222Mac {
    fn default() -> Self {
        Self::new()
    }
}

impl Phy6222Mac {
    pub fn new() -> Self {
        Self::take().expect("the PHY6222 radio driver has already been acquired")
    }

    /// Acquire the single PHY6222 MAC/radio instance with the default
    /// radio configuration (48 MHz HCLK, QFN32 PA table).
    pub fn take() -> Option<Self> {
        Self::take_with_config(RadioConfig::default())
    }

    /// Acquire the single MAC/radio instance with a product-selected radio
    /// configuration (HCLK, PA table, FCS mode, TX power).
    pub fn take_with_config(config: RadioConfig) -> Option<Self> {
        let ieee = Self::read_factory_ieee();
        // The driver acknowledges frames for this address from `LL_IRQ`.
        let driver = Phy6222Driver::take(RadioConfig {
            extended_address: ieee,
            ..config
        })?;
        log::info!(
            "[MAC] IEEE: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            ieee[0],
            ieee[1],
            ieee[2],
            ieee[3],
            ieee[4],
            ieee[5],
            ieee[6],
            ieee[7]
        );
        Some(Self {
            driver,
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: ieee,
            coord_short_address: ShortAddress(0xFFFF),
            coord_extended_address: [0; 8],
            associated_pan_coord: false,
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            dsn: 0,
            bsn: 0,
            beacon_payload: PibPayload::new(),
            max_csma_backoffs: 4,
            min_be: 3,
            max_be: 5,
            max_frame_retries: 3,
            promiscuous: false,
            pending_assoc_frame: None,
        })
    }

    pub fn extended_address(&self) -> IeeeAddress {
        self.extended_address
    }

    /// Read the factory-programmed 6-byte BLE public address and convert it
    /// to the 8-byte IEEE 802.15.4 EUI-64 used by the MAC.
    ///
    /// Sources, in the vendor SDK's priority order:
    /// 1. the one-bit-hot chip MADDR words at `0x11000900` (SDK `flash.c`
    ///    `read_chip_mAddr`; `phy_sec_ext.c` `pplus_LoadMACFromChipMAddr`
    ///    lets it override the flash record);
    /// 2. the public-address record at flash `0x4000` (`OWN_PUBLIC_ADDR_LOC`,
    ///    SDK `patch.c:6348`; byte order from `ll.c` `LL_Init`). Ai-Thinker
    ///    PB-03F modules ship with only this record.
    ///
    /// Returns the address in IEEE 802.15.4 over-the-air (little-endian)
    /// order, like every other `IeeeAddress` in the stack. Falls back to a
    /// shared placeholder that production products must reject.
    fn read_factory_ieee() -> [u8; 8] {
        /// Chip MADDR: 6 one-bit-hot words after the 64-word chip ID.
        const CHIP_MADDR_ADDR: u32 = 0x1100_0900;
        /// `OWN_PUBLIC_ADDR_LOC` (XIP address of flash offset 0x4000).
        const OWN_PUBLIC_ADDR_LOC: u32 = 0x1100_4000;

        let mut words = [0u32; 6];
        for (i, word) in words.iter_mut().enumerate() {
            // SAFETY: XIP flash is always mapped and readable.
            *word =
                unsafe { core::ptr::read_volatile((CHIP_MADDR_ADDR + i as u32 * 4) as *const u32) };
        }
        let mut record = [0u8; 6];
        for (i, byte) in record.iter_mut().enumerate() {
            // SAFETY: as above.
            *byte =
                unsafe { core::ptr::read_volatile((OWN_PUBLIC_ADDR_LOC + i as u32) as *const u8) };
        }

        match chip_maddr_ble_address(&words).or_else(|| public_addr_record_ble_address(record)) {
            Some(mac) => ble_address_to_eui64(mac),
            None => {
                log::warn!("phy6222: no factory MAC — using fallback");
                [0x00, 0x0D, 0x6F, 0xFF, 0xFE, 0xDE, 0xAD, 0x01]
            }
        }
    }

    fn next_dsn(&mut self) -> u8 {
        let s = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        s
    }

    fn next_bsn(&mut self) -> u8 {
        let s = self.bsn;
        self.bsn = self.bsn.wrapping_add(1);
        s
    }

    /// Power down the radio to save battery between poll cycles.
    /// Saves ~5–8 mA. Call `radio_wake()` before next TX/RX.
    pub fn radio_sleep(&self) {
        self.driver.radio_sleep();
    }

    /// Re-enable the radio after `radio_sleep()`.
    pub fn radio_wake(&mut self) {
        self.driver.radio_wake();
    }

    /// Radio counters and calibration results, for bring-up diagnostics.
    pub fn radio_stats(&self) -> driver::RadioStats {
        self.driver.stats()
    }

    /// Current low-level radio configuration.
    pub fn radio_config(&self) -> &RadioConfig {
        self.driver.config()
    }

    /// Adjust low-level radio settings (e.g. FCS mode for lab A/B tests).
    pub fn update_radio_config(&mut self, update_fn: impl FnOnce(&mut RadioConfig)) {
        self.driver.update_config(update_fn);
    }

    fn map_radio_err(e: RadioError) -> MacError {
        match e {
            RadioError::CcaFailure => MacError::ChannelAccessFailure,
            RadioError::HardwareError => MacError::RadioError,
            RadioError::InvalidFrame => MacError::FrameTooLong,
            RadioError::CrcError => MacError::RadioError,
            RadioError::NotInitialized => MacError::RadioError,
            RadioError::RxTimeout => MacError::NoData,
        }
    }

    /// Simple PRNG: deterministic hash from seed byte.
    fn prng(seed: u8) -> u32 {
        (seed as u32).wrapping_mul(1103515245).wrapping_add(12345)
    }

    /// Next CRC-good frame received before `deadline` (`None` on timeout or
    /// radio error).
    ///
    /// The driver only returns MPDUs of at least 3 bytes (PSDU >= 5), so
    /// callers may index the frame control and sequence number directly.
    /// Shared by every receive loop so the `receive`/timer race is
    /// generated once.
    async fn recv_until(&mut self, deadline: Instant) -> Option<driver::RxFrame> {
        embassy_time::with_deadline(deadline, self.driver.receive())
            .await
            .ok()?
            .ok()
    }

    /// Unslotted CSMA-CA + TX + ACK wait + retries.
    ///
    /// Implements IEEE 802.15.4-2011 §5.1.1.4 (unslotted CSMA-CA) with
    /// optional ACK reception and retry loop per `macMaxFrameRetries`.
    /// Returns the matching ACK's frame-pending bit (`false` without ACK
    /// request).
    async fn csma_ca_transmit(
        &mut self,
        frame: &[u8],
        ack_requested: bool,
    ) -> Result<bool, MacError> {
        let max_retries = if ack_requested {
            self.max_frame_retries
        } else {
            0
        };
        // 802.15.4 timing constants for 2.4 GHz
        const SYMBOL_PERIOD_US: u64 = 16; // 62.5 ksym/s
        const UNIT_BACKOFF_SYMBOLS: u64 = 20; // aUnitBackoffPeriod

        for attempt in 0..=max_retries {
            // ── Unslotted CSMA-CA ──
            let mut nb: u8 = 0;
            let mut be = self.min_be;

            let channel_clear = loop {
                // Random backoff: 0 to (2^BE - 1) unit backoff periods
                let max_val = (1u32 << be) - 1;
                let seed = self.dsn.wrapping_add(nb).wrapping_add(attempt);
                let random = Self::prng(seed);
                let backoff = (random % (max_val + 1)) as u64;
                let delay_us = backoff * UNIT_BACKOFF_SYMBOLS * SYMBOL_PERIOD_US;
                if delay_us > 0 {
                    Timer::after_micros(delay_us).await;
                }

                // CCA
                let busy = self.driver.channel_busy().map_err(Self::map_radio_err)?;

                if !busy {
                    break true;
                }

                nb += 1;
                be = core::cmp::min(be + 1, self.max_be);
                if nb > self.max_csma_backoffs {
                    break false;
                }
            };

            if !channel_clear {
                if attempt == max_retries {
                    return Err(MacError::ChannelAccessFailure);
                }
                continue;
            }

            // ── TX (+ hardware ACK window) ──
            if !ack_requested {
                self.driver
                    .transmit_blocking(frame)
                    .map_err(Self::map_radio_err)?;
                return Ok(false);
            }
            let seq = frame[2];
            if let Some(frame_pending) = self
                .driver
                .transmit_wait_ack(frame)
                .map_err(Self::map_radio_err)?
            {
                return Ok(frame_pending);
            }

            // No valid ACK — retry if attempts remain
            if attempt == max_retries {
                return Err(MacError::NoAck);
            }
            log::debug!(
                "phy6222: no ACK for seq={}, retry {}/{}",
                seq,
                attempt + 1,
                max_retries
            );
        }

        Err(MacError::NoAck)
    }

    async fn associate_with(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        // `build_association_request` derives the destination PAN from
        // `req.coord_address` itself and always uses the broadcast PAN
        // (0xFFFF) with PAN ID compression clear for our own (unassociated)
        // source, since we have no PAN yet — see its doc comment.
        let seq = self.next_dsn();
        let frame = build_association_request(
            seq,
            &req.coord_address,
            &self.extended_address,
            &req.capability_info,
        );

        // Use CSMA-CA for the association request (ACK requested)
        self.csma_ca_transmit(&frame, true).await?;

        // Wait for association response — poll coordinator multiple times
        Timer::after(embassy_time::Duration::from_millis(200)).await;

        // Try up to 5 polls, waiting up to 1.5s each
        for poll_attempt in 0..5u8 {
            if poll_attempt > 0 {
                Timer::after(embassy_time::Duration::from_millis(500)).await;
            }

            // Send Data Request to poll for association response. We are
            // still unassociated (no short address yet), so this uses our
            // IEEE address as source; `build_data_request`'s FCF dst-mode
            // matches whether `req.coord_address` is short or extended.
            let data_req =
                build_data_request(self.next_dsn(), &req.coord_address, &self.extended_address);
            let _ = self.csma_ca_transmit(&data_req, true).await;

            if let Some(confirm) = self.assoc_listen(1500, true).await {
                // After the response, listen briefly for the Transport-Key.
                if self.pending_assoc_frame.is_none() {
                    self.assoc_listen(2000, false).await;
                }
                return Ok(confirm);
            }
        }
        Err(MacError::NoBeacon)
    }

    /// Receive during association for up to `ms` milliseconds.
    ///
    /// ACKs frames addressed to us and saves the first data frame (normally
    /// the Transport-Key) for the next `mlme_poll`. Returns the parsed
    /// Association Response when `want_response`; otherwise returns once a
    /// data frame has been saved.
    async fn assoc_listen(&mut self, ms: u64, want_response: bool) -> Option<MlmeAssociateConfirm> {
        let deadline = Instant::now() + embassy_time::Duration::from_millis(ms);
        while let Some(rx) = self.recv_until(deadline).await {
            let data = &rx.data[..rx.len];
            let frame_type = data[0] & 0x07;
            // Frames for this node were already acknowledged by `LL_IRQ`.
            let (_, dst, payload_offset, _) = parse_mac_addresses(data);

            // Association Response (MAC command, type 3)
            if want_response
                && frame_type == 0x03
                && let Some((addr, status_byte)) = parse_association_response(data)
            {
                let status = match status_byte {
                    0x00 => AssociationStatus::Success,
                    0x01 => AssociationStatus::PanAtCapacity,
                    _ => AssociationStatus::PanAccessDenied,
                };
                if status_byte == 0 {
                    self.short_address = addr;
                    self.driver.update_config(|c| c.short_address = addr.0);
                }
                return Some(MlmeAssociateConfirm {
                    short_address: addr,
                    status,
                });
            }

            // Unicast data frame received during association — save it
            // (normally the Transport-Key from the coordinator). Broadcasts
            // are not saved so they cannot displace the Transport-Key.
            if frame_type == 0x01
                && self.pending_assoc_frame.is_none()
                && is_exact_destination(
                    &dst,
                    self.pan_id,
                    self.short_address,
                    &self.extended_address,
                )
                && let Some(frame) = data
                    .get(payload_offset..)
                    .filter(|p| !p.is_empty())
                    .and_then(MacFrame::from_slice)
            {
                log::info!("phy6222: saved post-assoc frame ({} bytes)", frame.len());
                self.pending_assoc_frame = Some(frame);
                if !want_response {
                    break;
                }
            }
        }
        None
    }

    /// Accept `rx` as a data frame (type 1) for this node: destination
    /// filter (promiscuous mode honoured only if `allow_promiscuous`) and
    /// MSDU copy; `LL_IRQ` already sent any required ACK. Returns `(src,
    /// dst, security_use, payload)`. Shared by poll and data indication.
    #[inline(never)]
    fn take_data_frame(
        &mut self,
        rx: &driver::RxFrame,
        allow_promiscuous: bool,
    ) -> Option<(MacAddress, MacAddress, bool, MacFrame)> {
        let data = &rx.data[..rx.len];
        if data.len() < 5 || data[0] & 0x07 != 0x01 {
            return None;
        }
        let (src, dst, payload_offset, security_use) = parse_mac_addresses(data);
        let accepted = if allow_promiscuous {
            self.accepts_destination(&dst)
        } else {
            frame_is_for_us(
                &dst,
                self.pan_id,
                self.short_address,
                &self.extended_address,
            )
        };
        if !accepted {
            return None;
        }
        let payload = data
            .get(payload_offset..)
            .filter(|p| !p.is_empty())
            .and_then(MacFrame::from_slice)?;
        Some((src, dst, security_use, payload))
    }

    /// Synchronous PIB read behind `mlme_get` (keeps the PIB match out of
    /// the async state machine).
    #[inline(never)]
    fn get_pib(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        use PibAttribute::*;
        Ok(match attr {
            MacShortAddress => PibValue::ShortAddress(self.short_address),
            MacPanId => PibValue::PanId(self.pan_id),
            PhyCurrentChannel => PibValue::U8(self.channel),
            MacExtendedAddress => PibValue::ExtendedAddress(self.extended_address),
            MacCoordShortAddress => PibValue::ShortAddress(self.coord_short_address),
            MacCoordExtendedAddress => PibValue::ExtendedAddress(self.coord_extended_address),
            MacAssociatedPanCoord => PibValue::Bool(self.associated_pan_coord),
            MacRxOnWhenIdle => PibValue::Bool(self.rx_on_when_idle),
            MacAssociationPermit => PibValue::Bool(self.association_permit),
            MacAutoRequest => PibValue::Bool(self.auto_request),
            MacBeaconPayload => PibValue::Payload(self.beacon_payload.clone()),
            MacMaxCsmaBackoffs => PibValue::U8(self.max_csma_backoffs),
            MacMinBe => PibValue::U8(self.min_be),
            MacMaxBe => PibValue::U8(self.max_be),
            MacMaxFrameRetries => PibValue::U8(self.max_frame_retries),
            MacPromiscuousMode => PibValue::Bool(self.promiscuous),
            MacDsn => PibValue::U8(self.dsn),
            MacBsn => PibValue::U8(self.bsn),
            PhyTransmitPower => PibValue::U8(self.driver.config().tx_power as u8),
            PhyChannelsSupported => PibValue::U32(ChannelMask::ALL_2_4GHZ.0),
            PhyCurrentPage => PibValue::U8(0),
            _ => return Err(MacError::InvalidParameter),
        })
    }

    /// Synchronous PIB write behind `mlme_set`.
    #[inline(never)]
    fn set_pib(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        use PibAttribute::*;
        match (attr, value) {
            (MacShortAddress, PibValue::ShortAddress(v)) => {
                self.short_address = v;
                self.driver.update_config(|c| c.short_address = v.0);
            }
            (MacPanId, PibValue::PanId(v)) => {
                self.pan_id = v;
                self.driver.update_config(|c| c.pan_id = v.0);
            }
            (PhyCurrentChannel, PibValue::U8(v)) => {
                self.channel = v;
                self.driver.update_config(|c| c.channel = v);
            }
            (MacExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.extended_address = v;
                self.driver.update_config(|c| c.extended_address = v);
            }
            (MacCoordShortAddress, PibValue::ShortAddress(v)) => {
                self.coord_short_address = v;
            }
            (MacCoordExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.coord_extended_address = v;
            }
            (MacAssociatedPanCoord, PibValue::Bool(v)) => {
                self.associated_pan_coord = v;
            }
            (MacRxOnWhenIdle, PibValue::Bool(v)) => {
                self.rx_on_when_idle = v;
            }
            (MacAssociationPermit, PibValue::Bool(v)) => {
                self.association_permit = v;
            }
            (MacAutoRequest, PibValue::Bool(v)) => {
                self.auto_request = v;
            }
            (MacBeaconPayload, PibValue::Payload(v)) => {
                self.beacon_payload = v;
            }
            (MacMaxCsmaBackoffs, PibValue::U8(v)) => {
                self.max_csma_backoffs = v;
            }
            (MacMinBe, PibValue::U8(v)) => {
                self.min_be = v;
            }
            (MacMaxBe, PibValue::U8(v)) => {
                self.max_be = v;
            }
            (MacMaxFrameRetries, PibValue::U8(v)) => {
                self.max_frame_retries = v;
            }
            (MacPromiscuousMode, PibValue::Bool(v)) => {
                self.promiscuous = v;
                self.driver.update_config(|c| c.promiscuous = v);
            }
            (MacDsn, PibValue::U8(v)) => {
                self.dsn = v;
            }
            (MacBsn, PibValue::U8(v)) => {
                self.bsn = v;
            }
            (PhyTransmitPower, PibValue::U8(v)) => {
                self.driver.update_config(|c| c.tx_power = v as i8);
            }
            _ => return Err(MacError::InvalidParameter),
        }
        Ok(())
    }

    /// IEEE 802.15.4 third-level destination filter (shared policy).
    fn accepts_destination(&self, dst: &MacAddress) -> bool {
        self.promiscuous
            || frame_is_for_us(dst, self.pan_id, self.short_address, &self.extended_address)
    }
}

impl MacDriver for Phy6222Mac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();
        let mut energy_list: EdList = heapless::Vec::new();

        // IEEE 802.15.4 scan duration is
        // aBaseSuperframeDuration * (2^n + 1) symbols. This backend keeps the
        // existing rounded per-channel window, but evaluates the bounded
        // n <= 14 result in u32 so Cortex-M0 does not retain a general u64
        // division runtime solely for scan timing.
        let Some(scan_duration_ms) = scan_duration_ms(req.scan_duration) else {
            return Err(MacError::InvalidParameter);
        };

        for ch in 11u8..=26 {
            if req.channel_mask.0 & (1u32 << ch) == 0 {
                continue;
            }

            self.driver.update_config(|c| c.channel = ch);

            match req.scan_type {
                ScanType::Ed => {
                    let (rssi, _busy) = self
                        .driver
                        .energy_detect_blocking()
                        .map_err(Self::map_radio_err)?;
                    let ed = ((rssi as i16 + 100).clamp(0, 255)) as u8;
                    let _ = energy_list.push(EdValue {
                        channel: ch,
                        energy: ed,
                    });
                }
                ScanType::Active | ScanType::Passive => {
                    if req.scan_type == ScanType::Active {
                        let seq = self.next_bsn();
                        let beacon_req = build_beacon_request(seq);
                        let _ = self.driver.transmit_blocking(&beacon_req);
                    }

                    // Collect multiple beacons within the scan duration window
                    let deadline = embassy_time::Instant::now()
                        + embassy_time::Duration::from_millis(u64::from(scan_duration_ms));
                    while !pan_descriptors.is_full() {
                        let Some(frame) = self.recv_until(deadline).await else {
                            break;
                        };
                        if let Some(pd) = parse_beacon(ch, &frame.data[..frame.len], frame.lqi) {
                            let _ = pan_descriptors.push(pd);
                        }
                    }
                }
                ScanType::Orphan => {}
            }
        }

        self.driver.update_config(|c| c.channel = self.channel);

        Ok(MlmeScanConfirm {
            scan_type: req.scan_type,
            pan_descriptors,
            energy_list,
        })
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        self.channel = req.channel;
        // IEEE 802.15.4 §6.4.1: MLME-ASSOCIATE.request sets macPANId to the
        // coordinator's PAN before the exchange, so the association response
        // and the following Transport-Key pass the software address filter
        // and are acknowledged.
        let previous_pan = self.pan_id;
        self.pan_id = req.coord_address.pan_id();
        let pan = self.pan_id.0;
        self.driver.update_config(|c| {
            c.channel = req.channel;
            c.pan_id = pan;
        });
        let result = self.associate_with(req).await;
        if !matches!(
            result,
            Ok(MlmeAssociateConfirm {
                status: AssociationStatus::Success,
                ..
            })
        ) {
            self.pan_id = previous_pan;
            self.driver.update_config(|c| c.pan_id = previous_pan.0);
        }
        self.associated_pan_coord = matches!(
            result,
            Ok(MlmeAssociateConfirm {
                status: AssociationStatus::Success,
                ..
            })
        );
        result
    }

    async fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(&mut self, _req: MlmeDisassociateRequest) -> Result<(), MacError> {
        self.short_address = ShortAddress(0xFFFF);
        self.pan_id = PanId(0xFFFF);
        self.coord_short_address = ShortAddress(0xFFFF);
        self.coord_extended_address = [0; 8];
        self.associated_pan_coord = false;
        self.driver.update_config(|c| {
            c.short_address = 0xFFFF;
            c.pan_id = 0xFFFF;
        });
        Ok(())
    }

    fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.channel = 11;
            self.coord_short_address = ShortAddress(0xFFFF);
            self.coord_extended_address = [0; 8];
            self.associated_pan_coord = false;
            self.rx_on_when_idle = false;
            self.dsn = 0;
            self.bsn = 0;
        }
        self.driver.update_config(|c| {
            c.channel = self.channel;
            c.short_address = self.short_address.0;
            c.pan_id = self.pan_id.0;
        });
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        // `Phy6222Mac` does not implement `ParentMacDriver`: it has no
        // frame-pending ACK path and keeps every parent primitive at its
        // `Unsupported` default. Fail explicitly.
        start_requires_parent_capability(&req)
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        self.get_pib(attr)
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        self.set_pib(attr, value)
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Return any frame saved during association first (e.g. Transport-Key)
        if let Some(frame) = self.pending_assoc_frame.take() {
            log::info!(
                "[MAC:Poll] Returning saved association frame ({} bytes)",
                frame.len()
            );
            return Ok(Some(frame));
        }

        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let has_short = self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE;

        // 2 passes: SHORT address first (most indirect frames), then IEEE (Transport-Key)
        let passes: u8 = if has_short { 2 } else { 1 };

        for pass in 0..passes {
            // Pass 0 (once associated) polls with our real short address —
            // never the unassigned/broadcast 0xFFFF placeholder — so the
            // parent's indirect-transmission queue lookup matches us.
            // Pass 1 (IEEE) catches frames (e.g. Transport-Key) the parent
            // queued against our extended address before it learned our
            // short address.
            let data_req: heapless::Vec<u8, 24> = if pass == 0 && has_short {
                build_data_request_short(self.next_dsn(), &parent, self.short_address)
            } else {
                build_data_request(self.next_dsn(), &parent, &self.extended_address)
            };

            // The Data Request ACK's frame-pending bit says whether the
            // parent holds data for us (same policy as the nRF backend).
            let frame_pending = match self.csma_ca_transmit(&data_req, true).await {
                Ok(frame_pending) => frame_pending,
                // Parent never acknowledged the first poll: report it so
                // parent-loss handling runs.
                Err(error) if pass == 0 => return Err(error),
                // The short pass already proved the parent reachable.
                Err(_) => return Ok(None),
            };
            if !frame_pending {
                continue;
            }

            // Wait for the parent's indirect data frame.
            let deadline = Instant::now() + embassy_time::Duration::from_millis(POLL_DATA_WAIT_MS);
            // Only data frames (type 1) addressed to us complete the poll.
            while let Some(rx) = self.recv_until(deadline).await {
                if let Some((_, _, _, mac_frame)) = self.take_data_frame(&rx, false) {
                    return Ok(Some(mac_frame));
                }
            }
        }

        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let msdu_handle = req.msdu_handle;
        let ack_requested = req.tx_options.ack_tx;

        // Build the MAC frame via the shared, header-size-aware builder:
        // it rejects frames whose *total* PSDU (MHR + MSDU, excluding the
        // 2-byte hardware FCS) would exceed 125 bytes — the same bound the
        // radio driver enforces in `Phy6222Driver::transmit` — instead of
        // checking `req.payload.len()` against a fixed constant that does
        // not account for addressing-mode overhead or the FCS.
        let seq = self.next_dsn();
        let mac_frame = build_data_frame(
            seq,
            req.src_addr_mode,
            self.short_address,
            &self.extended_address,
            &req.dst_address,
            req.payload,
            ack_requested,
            req.tx_options.frame_pending,
        )
        .map_err(|_| MacError::FrameTooLong)?;

        // CSMA-CA + TX + ACK wait + retries
        self.csma_ca_transmit(&mac_frame, ack_requested).await?;

        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        self.mcps_data_indication_timeout(5_000_000).await
    }

    async fn mcps_data_indication_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<McpsDataIndication, MacError> {
        let deadline = Instant::now() + embassy_time::Duration::from_micros(timeout_us as u64);

        // Only data frames (type 1) for this node are delivered; frames
        // naming this node exactly are ACKed.
        while let Some(rx) = self.recv_until(deadline).await {
            let Some((src_address, dst_address, security_use, payload)) =
                self.take_data_frame(&rx, true)
            else {
                continue;
            };
            log::trace!("phy6222: rx {} bytes lqi={}", rx.len, rx.lqi);
            return Ok(McpsDataIndication {
                src_address,
                dst_address,
                lqi: rx.lqi,
                payload,
                security_use,
            });
        }
        Err(MacError::NoData)
    }

    fn capabilities(&self) -> MacCapabilities {
        // Nominal payload for the common short-dst/short-src case
        // (9-byte MHR, 125-byte PSDU-minus-FCS budget → 116 bytes, of
        // which 102 is a conservative figure matching other backends).
        // `mcps_data` does not rely on this constant for correctness:
        // the actual bound is enforced per-frame by
        // `frames::build_data_frame`, which accounts for the real
        // addressing-mode overhead and the 2-byte hardware FCS.
        MacCapabilities::non_parent(102, TxPower(0), TxPower(10))
    }
}

impl zigbee_crypto::ForwardAesProvider for Phy6222Mac {}
impl PlatformServices for Phy6222Mac {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(duration_us as u64).await;
    }

    fn fill_random(&mut self, _output: &mut [u8]) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }
}

// ── Frame builders ──────────────────────────────────────────────
//
// Beacon Request, Association Request, Data Request, data-frame framing and
// beacon parsing all use the shared, host-tested helpers in `crate::frames`.

/// Decode the SDK chip MADDR (six one-bit-hot words, most significant
/// byte first) into a little-endian BLE address (`ownPublicAddr` order).
fn chip_maddr_ble_address(words: &[u32; 6]) -> Option<[u8; 6]> {
    let mut mac = [0u8; 6];
    for (i, &word) in words.iter().enumerate() {
        mac[5 - i] = one_bit_hot_decode(word)?;
    }
    valid_ble_address(mac)
}

/// Decode the `OWN_PUBLIC_ADDR_LOC` record into a little-endian BLE address.
/// SDK `LL_Init`: `ownPublicAddr[3,2,1,0,5,4] = flash[0..6]`.
fn public_addr_record_ble_address(b: [u8; 6]) -> Option<[u8; 6]> {
    valid_ble_address([b[3], b[2], b[1], b[0], b[5], b[4]])
}

fn valid_ble_address(mac: [u8; 6]) -> Option<[u8; 6]> {
    (mac != [0xFF; 6] && mac != [0x00; 6]).then_some(mac)
}

/// EUI-48 → EUI-64 (`AA:BB:CC:DD:EE:FF` → `AA:BB:CC:FF:FE:DD:EE:FF`), both
/// in little-endian (over-the-air) byte order.
fn ble_address_to_eui64(m: [u8; 6]) -> [u8; 8] {
    [m[0], m[1], m[2], 0xFE, 0xFF, m[3], m[4], m[5]]
}

/// Decode one chip-MADDR word, as SDK `chip_id_one_bit_hot_convter()`
/// (`lib/sec/phy_sec_ext.c:197`): each 16-bit half must have exactly one
/// bit set; the bit index of the high half is the upper nibble and the bit
/// index of the low half is the lower nibble. Blank (`0xFFFFFFFF`) or
/// malformed words return `None`.
fn one_bit_hot_decode(word: u32) -> Option<u8> {
    let high = (word >> 16) as u16;
    let low = word as u16;
    if high.count_ones() != 1 || low.count_ones() != 1 {
        return None;
    }
    Some(((high.trailing_zeros() as u8) << 4) | low.trailing_zeros() as u8)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scan_parses_captured_zha_coordinator_beacon() {
        // nRF sniffer capture, channel 15: ZHA coordinator beacon answering
        // a PB-03F beacon request (FCS stripped by the sniffer).
        let beacon = [
            0x00, 0x80, 0x9E, 0xE9, 0xDF, 0x00, 0x00, 0xFF, 0xCF, 0x00, 0x00, 0x00, 0x22, 0x84,
            0xB1, 0x4C, 0xC0, 0x1E, 0xA6, 0xC7, 0xAA, 0x20, 0xFF, 0xFF, 0xFF, 0x00,
        ];
        let pd = parse_beacon(15, &beacon, 200).unwrap();
        assert_eq!(pd.zigbee_beacon.protocol_id, 0);
        assert_eq!(pd.zigbee_beacon.stack_profile, 2);
        assert!(pd.superframe_spec.association_permit);
        assert_eq!(pd.lqi, 200);
        assert_eq!(
            pd.coord_address,
            MacAddress::Short(PanId(0xDFE9), ShortAddress(0x0000))
        );
        // A MAC-only beacon carries no Zigbee payload and is not a network.
        assert!(parse_beacon(15, &beacon[..11], 200).is_none());
    }

    #[test]
    fn public_addr_record_matches_sdk_byte_order() {
        // Ai-Thinker PB-03F factory record at flash 0x4000 (BD_ADDR
        // 24:42:E3:5F:6D:04 per SDK `LL_Init`).
        let mac = public_addr_record_ble_address([0xE3, 0x5F, 0x6D, 0x04, 0x24, 0x42]).unwrap();
        assert_eq!(mac, [0x04, 0x6D, 0x5F, 0xE3, 0x42, 0x24]);
        // EUI-64 24:42:E3:FF:FE:5F:6D:04, little-endian on air.
        assert_eq!(
            ble_address_to_eui64(mac),
            [0x04, 0x6D, 0x5F, 0xFE, 0xFF, 0xE3, 0x42, 0x24]
        );
        assert_eq!(public_addr_record_ble_address([0xFF; 6]), None);
        assert_eq!(public_addr_record_ble_address([0x00; 6]), None);
    }

    #[test]
    fn one_bit_hot_decode_matches_sdk_convter() {
        // byte 0xA5: high half bit 10, low half bit 5.
        assert_eq!(one_bit_hot_decode((1 << (16 + 10)) | (1 << 5)), Some(0xA5));
        assert_eq!(one_bit_hot_decode(0x8000_0001), Some(0xF0));
        assert_eq!(one_bit_hot_decode(0xFFFF_FFFF), None);
        assert_eq!(one_bit_hot_decode(0x0000_0000), None);
        assert_eq!(one_bit_hot_decode(0x0003_0001), None);
    }

    #[test]
    fn chip_maddr_words_are_most_significant_first() {
        let enc = |b: u8| (1u32 << (16 + (b >> 4))) | (1 << (b & 0x0F));
        let words = [
            enc(0x24),
            enc(0x42),
            enc(0xE3),
            enc(0x5F),
            enc(0x6D),
            enc(0x04),
        ];
        assert_eq!(
            chip_maddr_ble_address(&words),
            Some([0x04, 0x6D, 0x5F, 0xE3, 0x42, 0x24])
        );
        let mut partial = words;
        partial[3] = 0xFFFF_FFFF;
        assert_eq!(chip_maddr_ble_address(&partial), None);
    }

    // These tests exercise the shared `crate::frames` builders/parsers
    // exactly as this backend's `MacDriver` impl calls them (same argument
    // shapes), pinning golden frame bytes / FCF bit decoding for the
    // specific bugs the audit found here. `Phy6222Mac`/`Phy6222Driver`
    // themselves cannot be constructed on a host test target (see
    // `driver::tests` for why), so these test the frame layer directly.

    const OWN_IEEE: IeeeAddress = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];

    fn fc_of(frame: &[u8]) -> u16 {
        u16::from_le_bytes([frame[0], frame[1]])
    }

    #[test]
    fn scan_duration_is_bounded_and_preserves_existing_rounding() {
        assert_eq!(scan_duration_ms(0), Some(16));
        assert_eq!(scan_duration_ms(5), Some(492));
        assert_eq!(scan_duration_ms(14), Some(251_659));
        assert_eq!(scan_duration_ms(15), None);
    }

    // ── (1) data-frame base FCF must not always set ACK request ─────

    #[test]
    fn data_frame_without_ack_request_clears_ar_bit() {
        let dst = MacAddress::Short(PanId(0x1234), ShortAddress(0xBEEF));
        let frame = build_data_frame(
            0x10,
            AddressMode::Short,
            ShortAddress(0x0042),
            &OWN_IEEE,
            &dst,
            &[0xAA, 0xBB],
            /* ack_request = */ false,
            /* frame_pending = */ false,
        )
        .expect("frame within PSDU limit");

        let fc = fc_of(&frame);
        assert_eq!(fc & 0x07, 0x01, "frame type must be Data");
        assert_eq!(
            fc & 0x0020,
            0,
            "AR bit must be clear when ack not requested"
        );
    }

    #[test]
    fn data_frame_with_ack_request_sets_ar_bit() {
        let dst = MacAddress::Short(PanId(0x1234), ShortAddress(0xBEEF));
        let frame = build_data_frame(
            0x11,
            AddressMode::Short,
            ShortAddress(0x0042),
            &OWN_IEEE,
            &dst,
            &[0xAA, 0xBB],
            /* ack_request = */ true,
            /* frame_pending = */ false,
        )
        .expect("frame within PSDU limit");

        let fc = fc_of(&frame);
        assert_ne!(fc & 0x0020, 0, "AR bit must be set when ack requested");
    }

    // ── (2) Association Request FCF PAN ID compression / addressing ──

    #[test]
    fn association_request_fcf_matches_short_coordinator_and_uncompressed_src_pan() {
        let coord = MacAddress::Short(PanId(0xDFE9), ShortAddress(0x0000));
        let cap = CapabilityInfo {
            rx_on_when_idle: false,
            allocate_address: true,
            ..CapabilityInfo::default()
        };
        let frame = build_association_request(0x42, &coord, &OWN_IEEE, &cap);

        let fc = fc_of(&frame);
        assert_eq!(fc & 0x07, 0x03, "frame type must be MAC command");
        assert_eq!((fc >> 10) & 0x03, 0x02, "dst addr mode must be short");
        assert_eq!(
            (fc >> 6) & 1,
            0,
            "PAN ID compression must be clear — we have no PAN yet"
        );
        assert_eq!((fc >> 14) & 0x03, 0x03, "src addr mode must be extended");
        // Own (unassociated) source PAN is the broadcast PAN, present
        // uncompressed right after the destination PAN + address.
        assert_eq!(&frame[7..9], &[0xFF, 0xFF]);
    }

    #[test]
    fn association_request_fcf_matches_extended_coordinator() {
        let coord = MacAddress::Extended(PanId(0xDFE9), [0x99; 8]);
        let frame = build_association_request(0x43, &coord, &OWN_IEEE, &CapabilityInfo::default());

        let fc = fc_of(&frame);
        assert_eq!(
            (fc >> 10) & 0x03,
            0x03,
            "dst addr mode must be extended to match the coordinator's address"
        );
    }

    // ── (3) short-address Data Request carries the real child address ──

    #[test]
    fn data_request_short_uses_actual_assigned_short_address_not_broadcast() {
        let parent = MacAddress::Short(PanId(0x1234), ShortAddress(0x0001));
        let our_short = ShortAddress(0x9ABC);
        let frame = build_data_request_short(0x20, &parent, our_short);

        // Source address is the last 2 bytes before the command ID.
        let cmd_id_index = frame.len() - 1;
        assert_eq!(frame[cmd_id_index], 0x04, "Data Request command ID");
        let src = u16::from_le_bytes([frame[cmd_id_index - 2], frame[cmd_id_index - 1]]);
        assert_eq!(src, our_short.0);
        assert_ne!(src, 0xFFFF, "must not fall back to the broadcast address");
    }

    // ── (4) IEEE-source Data Request FCF dst mode vs short/extended dst ──

    #[test]
    fn data_request_ieee_source_fcf_dst_mode_follows_short_coordinator() {
        let coord = MacAddress::Short(PanId(0x1234), ShortAddress(0x5678));
        let frame = build_data_request(0x30, &coord, &OWN_IEEE);
        let fc = fc_of(&frame);
        assert_eq!((fc >> 10) & 0x03, 0x02, "dst addr mode must be short");
        assert_eq!((fc >> 14) & 0x03, 0x03, "src addr mode must be extended");
    }

    #[test]
    fn data_request_ieee_source_fcf_dst_mode_follows_extended_coordinator() {
        let coord = MacAddress::Extended(PanId(0x1234), [0x77; 8]);
        let frame = build_data_request(0x31, &coord, &OWN_IEEE);
        let fc = fc_of(&frame);
        assert_eq!(
            (fc >> 10) & 0x03,
            0x03,
            "dst addr mode must switch to extended when the coordinator's \
             address is extended"
        );
    }

    // ── (5) MAC frame/PSDU length enforcement (incl. 2-byte FCS) ─────

    #[test]
    fn data_frame_at_max_psdu_minus_fcs_is_accepted() {
        // MHR for short/short addressing is 2(FC)+1(seq)+2(dstPan)+2(dstAddr)
        // +2(srcAddr) = 9 bytes; 125 - 9 = 116 bytes of payload exactly
        // fills the PSDU-without-FCS budget.
        let dst = MacAddress::Short(PanId(1), ShortAddress(2));
        let payload = [0u8; 116];
        let frame = build_data_frame(
            0,
            AddressMode::Short,
            ShortAddress(3),
            &OWN_IEEE,
            &dst,
            &payload,
            false,
            false,
        )
        .expect("exactly at the 125-byte PSDU-minus-FCS bound");
        assert_eq!(frame.len(), 125);
    }

    #[test]
    fn data_frame_over_max_psdu_minus_fcs_is_rejected() {
        let dst = MacAddress::Short(PanId(1), ShortAddress(2));
        let payload = [0u8; 117];
        let result = build_data_frame(
            0,
            AddressMode::Short,
            ShortAddress(3),
            &OWN_IEEE,
            &dst,
            &payload,
            false,
            false,
        );
        assert!(result.is_err(), "one byte over budget must be rejected");
    }
}
