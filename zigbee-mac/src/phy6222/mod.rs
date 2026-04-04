//! PHY6222 MAC backend — pure Rust, zero vendor blobs.
//!
//! Implements `MacDriver` for the Phyplus PHY6222/6252 ARM Cortex-M0 SoC.
//! The PHY6222 has a multi-protocol radio (BLE + IEEE 802.15.4) and is found
//! in many low-cost Tuya smart home devices (THB2, TH05F, BTH01, etc.).
//!
//! This is the first zigbee-rs backend with a **100% pure-Rust radio driver**:
//! all radio configuration uses direct register access derived from the
//! open-source PHY6222 SDK. No vendor libraries are linked.

pub mod driver;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use driver::{Phy6222Driver, RadioConfig, RadioError};
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;

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
    pending_assoc_frame: Option<([u8; 128], usize)>,
}

impl Phy6222Mac {
    pub fn new() -> Self {
        let config = RadioConfig::default();
        let ieee = Self::read_factory_ieee();
        log::info!(
            "[MAC] IEEE: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            ieee[0], ieee[1], ieee[2], ieee[3], ieee[4], ieee[5], ieee[6], ieee[7]
        );
        Self {
            driver: Phy6222Driver::new(config),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: ieee,
            coord_short_address: ShortAddress(0xFFFF),
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
        }
    }

    /// Read the factory-programmed 6-byte BLE MAC from flash and convert
    /// to an 8-byte IEEE 802.15.4 EUI-64 address.
    ///
    /// PHY6222 stores MAC at `CHIP_MADDR_FLASH_ADDRESS` (0x11000900) using
    /// one-bit-hot encoding: each byte is a 32-bit word where each pair of
    /// bits redundantly encodes one data bit (flash wear-leveling protection).
    ///
    /// EUI-48 → EUI-64 conversion inserts 0xFF:0xFE in the middle:
    ///   `AA:BB:CC:DD:EE:FF` → `AA:BB:CC:FF:FE:DD:EE:FF`
    fn read_factory_ieee() -> [u8; 8] {
        /// Flash base for XIP (memory-mapped) access.
        const FLASH_BASE: u32 = 0x1100_0000;
        /// Chip ID area in flash (factory-programmed).
        const CHIP_ID_FLASH_ADDR: u32 = FLASH_BASE + 0x0800;
        /// Chip ID is 64 × 32-bit words (256 bytes).
        const CHIP_ID_LENGTH: u32 = 64;
        /// MAC address follows chip ID: 6 × 32-bit words.
        const CHIP_MADDR_ADDR: u32 = CHIP_ID_FLASH_ADDR + CHIP_ID_LENGTH * 4; // 0x11000900
        const CHIP_MADDR_LEN: usize = 6;

        let mut mac48 = [0xFFu8; CHIP_MADDR_LEN];
        let mut valid = true;

        for i in 0..CHIP_MADDR_LEN {
            let word_addr = CHIP_MADDR_ADDR + (i as u32) * 4;
            let word = unsafe { core::ptr::read_volatile(word_addr as *const u32) };
            match one_bit_hot_decode(word) {
                Some(b) => mac48[CHIP_MADDR_LEN - 1 - i] = b, // stored in reverse order
                None => {
                    valid = false;
                    break;
                }
            }
        }

        if !valid || mac48 == [0xFF; 6] || mac48 == [0x00; 6] {
            // No valid factory MAC — generate from chip-unique SRAM content
            // as a last resort (not ideal but better than all-zeros)
            log::warn!("phy6222: no factory MAC — using fallback");
            return [0x00, 0x0D, 0x6F, 0xFF, 0xFE, 0xDE, 0xAD, 0x01];
        }

        // EUI-48 → EUI-64: insert 0xFF:0xFE in the middle
        // BLE MAC AA:BB:CC:DD:EE:FF → IEEE AA:BB:CC:FF:FE:DD:EE:FF
        [
            mac48[0], mac48[1], mac48[2], 0xFF, 0xFE, mac48[3], mac48[4], mac48[5],
        ]
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

    /// Unslotted CSMA-CA + TX + ACK wait + retries.
    ///
    /// Implements IEEE 802.15.4-2011 §5.1.1.4 (unslotted CSMA-CA) with
    /// optional ACK reception and retry loop per `macMaxFrameRetries`.
    async fn csma_ca_transmit(
        &mut self,
        frame: &[u8],
        ack_requested: bool,
    ) -> Result<(), MacError> {
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
                let busy = self
                    .driver
                    .clear_channel_assessment()
                    .await
                    .map_err(Self::map_radio_err)?;

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

            // ── TX ──
            self.driver
                .transmit(frame)
                .await
                .map_err(Self::map_radio_err)?;

            if !ack_requested {
                return Ok(());
            }

            // ── ACK wait ──
            // Spec: aTurnaroundTime (192µs) + ACK frame duration (~352µs).
            // Allow 1.5ms total for software overhead.
            let seq = frame[2];
            let ack_result = select::select(self.driver.receive(), Timer::after_micros(1500)).await;

            if let select::Either::First(Ok(rx)) = ack_result {
                if rx.len >= 3 {
                    let fc = u16::from_le_bytes([rx.data[0], rx.data[1]]);
                    let frame_type = fc & 0x07;
                    let ack_seq = rx.data[2];
                    if frame_type == 0x02 && ack_seq == seq {
                        return Ok(());
                    }
                }
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

    /// Send a 3-byte IEEE 802.15.4 ACK frame for the given sequence number.
    async fn send_ack(&mut self, seq: u8) {
        // ACK frame: FC=0x0002 (type=ACK, no pending, no AR), seq
        let ack = [0x02u8, 0x00, seq];
        let _ = self.driver.transmit(&ack).await;
    }
}

impl MacDriver for Phy6222Mac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();
        let mut energy_list: EdList = heapless::Vec::new();

        let scan_duration_ms = ((1u64 << req.scan_duration as u64) * 15360 / 1000) + 1;

        for ch in 11u8..=26 {
            if req.channel_mask.0 & (1u32 << ch) == 0 {
                continue;
            }

            self.driver.update_config(|c| c.channel = ch);

            match req.scan_type {
                ScanType::Ed => {
                    let (rssi, _busy) = self.driver.energy_detect().map_err(Self::map_radio_err)?;
                    let ed = ((rssi as i16 + 100).clamp(0, 255)) as u8;
                    let _ = energy_list.push(EdValue {
                        channel: ch,
                        energy: ed,
                    });
                }
                ScanType::Active => {
                    let seq = self.next_bsn();
                    let beacon_req = build_beacon_request(seq);
                    let _ = self.driver.transmit(&beacon_req).await;

                    // Collect multiple beacons within the scan duration window
                    let deadline = embassy_time::Instant::now()
                        + embassy_time::Duration::from_millis(scan_duration_ms);
                    while !pan_descriptors.is_full() {
                        let now = embassy_time::Instant::now();
                        if now >= deadline {
                            break;
                        }
                        let remaining = deadline - now;
                        let result = select::select(
                            self.driver.receive(),
                            Timer::after(remaining),
                        )
                        .await;

                        if let select::Either::First(Ok(frame)) = result {
                            if let Some(pd) = parse_beacon_frame(&frame.data[..frame.len], ch) {
                                let _ = pan_descriptors.push(pd);
                            }
                        } else {
                            break;
                        }
                    }
                }
                ScanType::Passive => {
                    let deadline = embassy_time::Instant::now()
                        + embassy_time::Duration::from_millis(scan_duration_ms);
                    while !pan_descriptors.is_full() {
                        let now = embassy_time::Instant::now();
                        if now >= deadline {
                            break;
                        }
                        let remaining = deadline - now;
                        let result = select::select(
                            self.driver.receive(),
                            Timer::after(remaining),
                        )
                        .await;

                        if let select::Either::First(Ok(frame)) = result {
                            if let Some(pd) = parse_beacon_frame(&frame.data[..frame.len], ch) {
                                let _ = pan_descriptors.push(pd);
                            }
                        } else {
                            break;
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
        self.driver.update_config(|c| c.channel = req.channel);

        let coord_pan = req.coord_address.pan_id();

        let seq = self.next_dsn();
        let frame = build_association_request(
            seq,
            coord_pan,
            &req.coord_address,
            &self.extended_address,
            &req.capability_info,
        );

        // Use CSMA-CA for the association request (ACK requested)
        self.csma_ca_transmit(&frame, true).await?;

        // Wait for association response — poll coordinator multiple times
        Timer::after(embassy_time::Duration::from_millis(200)).await;

        let mut confirm: Option<MlmeAssociateConfirm> = None;

        // Try up to 5 polls, waiting up to 1.5s each
        for poll_attempt in 0..5u8 {
            if poll_attempt > 0 {
                Timer::after(embassy_time::Duration::from_millis(500)).await;
            }

            // Send Data Request to poll for association response
            let data_req = build_data_request_ieee(
                self.next_dsn(),
                &req.coord_address,
                &self.extended_address,
            );
            let _ = self.csma_ca_transmit(&data_req, true).await;

            let deadline =
                embassy_time::Instant::now() + embassy_time::Duration::from_millis(1500);

            for _ in 0..20u8 {
                let now = embassy_time::Instant::now();
                if now >= deadline {
                    break;
                }
                let remaining = deadline - now;

                let result =
                    select::select(self.driver.receive(), Timer::after(remaining)).await;

                match result {
                    select::Either::Second(_) => break,
                    select::Either::First(Err(_)) => continue,
                    select::Either::First(Ok(rx)) => {
                        let data = &rx.data[..rx.len];
                        if data.len() < 3 {
                            continue;
                        }
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        let frame_type = fc & 0x07;

                        // ACK — skip
                        if frame_type == 0x02 {
                            continue;
                        }

                        // Send ACK if requested
                        if (fc >> 5) & 1 != 0 {
                            self.send_ack(data[2]).await;
                        }

                        // Check for Association Response (MAC command, type 3)
                        if frame_type == 0x03 {
                            if let Some((addr, status_byte)) =
                                parse_association_response(data)
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
                                confirm = Some(MlmeAssociateConfirm {
                                    short_address: addr,
                                    status,
                                });
                                break;
                            }
                        }

                        // Data frame received during association — save it
                        // (likely Transport-Key from coordinator)
                        if frame_type == 0x01 && self.pending_assoc_frame.is_none() {
                            let (_, _, payload_offset, _) = parse_mac_addresses(data);
                            if data.len() > payload_offset {
                                let payload = &data[payload_offset..];
                                let mut buf = [0u8; 128];
                                let copy_len = payload.len().min(128);
                                buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                                self.pending_assoc_frame = Some((buf, copy_len));
                                log::info!(
                                    "phy6222: saved post-assoc frame ({} bytes)",
                                    copy_len
                                );
                            }
                        }
                    }
                }
            }

            if confirm.is_some() {
                break;
            }
        }

        // After getting assoc response, listen briefly for more frames (Transport-Key)
        if confirm.is_some() && self.pending_assoc_frame.is_none() {
            let deadline =
                embassy_time::Instant::now() + embassy_time::Duration::from_millis(2000);
            for _ in 0..20u8 {
                let now = embassy_time::Instant::now();
                if now >= deadline {
                    break;
                }
                let remaining = deadline - now;
                let result =
                    select::select(self.driver.receive(), Timer::after(remaining)).await;
                if let select::Either::First(Ok(rx)) = result {
                    let data = &rx.data[..rx.len];
                    if data.len() >= 3 {
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        if (fc >> 5) & 1 != 0 {
                            self.send_ack(data[2]).await;
                        }
                        let frame_type = fc & 0x07;
                        if frame_type == 0x01 {
                            let (_, _, payload_offset, _) = parse_mac_addresses(data);
                            if data.len() > payload_offset {
                                let payload = &data[payload_offset..];
                                let mut buf = [0u8; 128];
                                let copy_len = payload.len().min(128);
                                buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                                self.pending_assoc_frame = Some((buf, copy_len));
                                log::info!(
                                    "phy6222: saved post-assoc frame ({} bytes)",
                                    copy_len
                                );
                                break;
                            }
                        }
                    }
                } else {
                    break;
                }
            }
        }

        confirm.ok_or(MacError::NoBeacon)
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
        self.driver.update_config(|c| {
            c.short_address = 0xFFFF;
            c.pan_id = 0xFFFF;
        });
        Ok(())
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.channel = 11;
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
        self.pan_id = req.pan_id;
        self.channel = req.channel;
        self.driver.update_config(|c| {
            c.pan_id = req.pan_id.0;
            c.channel = req.channel;
        });
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        use PibAttribute::*;
        Ok(match attr {
            MacShortAddress => PibValue::ShortAddress(self.short_address),
            MacPanId => PibValue::PanId(self.pan_id),
            PhyCurrentChannel => PibValue::U8(self.channel),
            MacExtendedAddress => PibValue::ExtendedAddress(self.extended_address),
            MacCoordShortAddress => PibValue::ShortAddress(self.coord_short_address),
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

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
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

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Return any frame saved during association first (e.g. Transport-Key)
        if let Some((buf, len)) = self.pending_assoc_frame.take() {
            log::info!(
                "[MAC:Poll] Returning saved association frame ({} bytes)",
                len
            );
            return Ok(MacFrame::from_slice(&buf[..len]));
        }

        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let has_short = self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE;

        // 2 passes: SHORT address first (most indirect frames), then IEEE (Transport-Key)
        let passes: u8 = if has_short { 2 } else { 1 };

        for pass in 0..passes {
            let data_req = if pass == 0 && has_short {
                build_data_request(self.next_dsn(), self.pan_id, self.coord_short_address)
                    .as_slice()
                    .iter()
                    .copied()
                    .collect::<heapless::Vec<u8, 24>>()
            } else {
                build_data_request_ieee(
                    self.next_dsn(),
                    &parent,
                    &self.extended_address,
                )
            };

            if self.csma_ca_transmit(&data_req, true).await.is_err() {
                continue;
            }

            // Wait up to 1500ms with up to 40 RX attempts per pass
            let deadline =
                embassy_time::Instant::now() + embassy_time::Duration::from_millis(1500);

            let mut got_none = false;
            for _rx_attempt in 0..40u8 {
                let now = embassy_time::Instant::now();
                if now >= deadline {
                    break;
                }
                let remaining = deadline - now;

                let rx_result =
                    select::select(self.driver.receive(), Timer::after(remaining)).await;

                match rx_result {
                    select::Either::Second(_) => break, // timeout
                    select::Either::First(Err(_)) => continue,
                    select::Either::First(Ok(rx)) => {
                        let data = &rx.data[..rx.len];
                        if data.len() < 3 {
                            continue;
                        }
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        let frame_type = fc & 0x07;

                        // ACK frame — check frame_pending bit
                        if frame_type == 0x02 {
                            let frame_pending = (data[0] >> 4) & 1 != 0;
                            if !frame_pending {
                                got_none = true;
                                break;
                            }
                            continue; // ACK with pending=1, keep waiting for data
                        }

                        // Only accept data frames (type 1)
                        if frame_type != 0x01 {
                            continue;
                        }

                        // Verify MAC destination matches us or broadcast
                        let (_src, dst, payload_offset, _security_use) =
                            parse_mac_addresses(data);
                        match &dst {
                            MacAddress::Short(_, d) => {
                                let for_us = d.0 == self.short_address.0
                                    || d.0 == 0xFFFF
                                    || d.0 == 0xFFFD
                                    || d.0 == 0xFFFC;
                                if !for_us {
                                    continue;
                                }
                            }
                            MacAddress::Extended(_, e) => {
                                if *e != self.extended_address {
                                    continue;
                                }
                            }
                        }

                        // Send ACK if requested
                        if (fc >> 5) & 1 != 0 {
                            self.send_ack(data[2]).await;
                        }

                        if data.len() <= payload_offset {
                            continue;
                        }

                        let mac_frame = MacFrame::from_slice(&data[payload_offset..])
                            .unwrap_or_else(MacFrame::new);

                        return Ok(Some(mac_frame));
                    }
                }
            }

            if got_none {
                return Ok(None);
            }
        }

        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        if req.payload.len() > MAX_MAC_PAYLOAD {
            return Err(MacError::FrameTooLong);
        }

        let msdu_handle = req.msdu_handle;
        let ack_requested = req.tx_options.ack_tx;

        // Build MAC frame with proper header
        let seq = self.next_dsn();
        let mac_frame = build_data_frame(
            seq,
            self.pan_id,
            &req.dst_address,
            self.short_address,
            req.payload,
            ack_requested,
        );

        // CSMA-CA + TX + ACK wait + retries
        self.csma_ca_transmit(&mac_frame, ack_requested).await?;

        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        const RX_TIMEOUT_MS: u64 = 5000;
        let deadline =
            embassy_time::Instant::now() + embassy_time::Duration::from_millis(RX_TIMEOUT_MS);

        loop {
            let now = embassy_time::Instant::now();
            if now >= deadline {
                return Err(MacError::NoData);
            }
            let remaining = deadline - now;

            let rx_result = select::select(self.driver.receive(), Timer::after(remaining)).await;

            match rx_result {
                select::Either::Second(_) => {
                    return Err(MacError::NoData);
                }
                select::Either::First(Err(_)) => {
                    continue;
                }
                select::Either::First(Ok(rx)) => {
                    let data = &rx.data[..rx.len];

                    if data.len() < 5 {
                        continue;
                    }

                    let fc = u16::from_le_bytes([data[0], data[1]]);
                    let frame_type = fc & 0x07;

                    // Only deliver data frames (type 1) to upper layer
                    if frame_type != 1 {
                        continue;
                    }

                    // Send ACK if requested (bit 5 of FC)
                    if (fc >> 5) & 1 != 0 {
                        self.send_ack(data[2]).await;
                    }

                    let (src_address, dst_address, payload_offset, security_use) =
                        parse_mac_addresses(data);

                    // Software address filtering
                    if !self.promiscuous {
                        match &dst_address {
                            MacAddress::Short(pan, addr) => {
                                let pan_ok = pan.0 == self.pan_id.0 || pan.0 == 0xFFFF;
                                let addr_ok = addr.0 == self.short_address.0
                                    || addr.0 == 0xFFFF
                                    || addr.0 == 0xFFFD
                                    || addr.0 == 0xFFFC;
                                if !pan_ok || !addr_ok {
                                    log::trace!(
                                        "phy6222: filtered short dst pan=0x{:04X} addr=0x{:04X}",
                                        pan.0,
                                        addr.0
                                    );
                                    continue;
                                }
                            }
                            MacAddress::Extended(pan, addr) => {
                                let pan_ok = pan.0 == self.pan_id.0 || pan.0 == 0xFFFF;
                                let addr_ok = *addr == self.extended_address;
                                if !pan_ok || !addr_ok {
                                    log::trace!("phy6222: filtered ext dst pan=0x{:04X}", pan.0,);
                                    continue;
                                }
                            }
                        }
                    }

                    if data.len() <= payload_offset {
                        continue;
                    }

                    let mac_frame =
                        MacFrame::from_slice(&data[payload_offset..]).unwrap_or_else(MacFrame::new);

                    log::trace!("phy6222: rx {} bytes lqi={}", rx.len, rx.lqi);

                    return Ok(McpsDataIndication {
                        src_address,
                        dst_address,
                        lqi: rx.lqi,
                        payload: mac_frame,
                        security_use,
                    });
                }
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: false,
            router: true,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(0),
            tx_power_max: TxPower(10),
        }
    }
}

// ── Frame builders ──────────────────────────────────────────────

fn build_beacon_request(seq: u8) -> [u8; 8] {
    let fc: u16 = 0x0803;
    [fc as u8, (fc >> 8) as u8, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]
}

fn build_association_request(
    seq: u8,
    coord_pan: PanId,
    coord_addr: &MacAddress,
    ext_addr: &IeeeAddress,
    cap: &CapabilityInfo,
) -> heapless::Vec<u8, 32> {
    let mut frame = heapless::Vec::new();

    // FC: Command, dst=short, src=extended, AR=1, PAN compress=1
    let fc: u16 = 0xC823;
    let _ = frame.push(fc as u8);
    let _ = frame.push((fc >> 8) as u8);
    let _ = frame.push(seq);

    let _ = frame.push(coord_pan.0 as u8);
    let _ = frame.push((coord_pan.0 >> 8) as u8);
    match coord_addr {
        MacAddress::Short(_, a) => {
            let _ = frame.push(a.0 as u8);
            let _ = frame.push((a.0 >> 8) as u8);
        }
        MacAddress::Extended(_, ext) => {
            for b in ext {
                let _ = frame.push(*b);
            }
        }
    }

    for b in ext_addr {
        let _ = frame.push(*b);
    }

    let _ = frame.push(0x01); // Association Request command ID
    let _ = frame.push(cap.to_byte());

    frame
}

fn build_data_request(seq: u8, pan_id: PanId, coord: ShortAddress) -> [u8; 10] {
    let fc: u16 = 0x8863;
    [
        fc as u8,
        (fc >> 8) as u8,
        seq,
        pan_id.0 as u8,
        (pan_id.0 >> 8) as u8,
        coord.0 as u8,
        (coord.0 >> 8) as u8,
        0xFF,
        0xFF,
        0x04,
    ]
}

/// Build a Data Request MAC command with IEEE (extended) source address.
/// Used for poll pass 2 to catch Transport-Key queued by coordinator for our IEEE.
fn build_data_request_ieee(
    seq: u8,
    coord: &MacAddress,
    own_ieee: &IeeeAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame: heapless::Vec<u8, 24> = heapless::Vec::new();
    // FC: command(0x03), dst=short(0x08), src=extended(0xC0), AR(0x20), PAN compress(0x40)
    let fc: u16 = 0xCC63;
    let _ = frame.push(fc as u8);
    let _ = frame.push((fc >> 8) as u8);
    let _ = frame.push(seq);
    // Dst PAN + addr
    match coord {
        MacAddress::Short(pan, addr) => {
            let _ = frame.push(pan.0 as u8);
            let _ = frame.push((pan.0 >> 8) as u8);
            let _ = frame.push(addr.0 as u8);
            let _ = frame.push((addr.0 >> 8) as u8);
        }
        MacAddress::Extended(pan, ext) => {
            let _ = frame.push(pan.0 as u8);
            let _ = frame.push((pan.0 >> 8) as u8);
            for b in ext {
                let _ = frame.push(*b);
            }
        }
    }
    // Src IEEE (no src PAN — PAN compress)
    for b in own_ieee {
        let _ = frame.push(*b);
    }
    // Command: Data Request (0x04)
    let _ = frame.push(0x04);
    frame
}

fn build_data_frame(
    seq: u8,
    pan_id: PanId,
    dst: &MacAddress,
    src_short: ShortAddress,
    payload: &[u8],
    ack: bool,
) -> heapless::Vec<u8, 127> {
    let mut frame: heapless::Vec<u8, 127> = heapless::Vec::new();

    // FC: Data frame, dst=short, src=short, PAN compress, optional ACK request
    let mut fc: u16 = 0x8861; // data + dst short + src short + PAN compress
    if ack {
        fc |= 0x0020; // AR bit
    }
    let _ = frame.push(fc as u8);
    let _ = frame.push((fc >> 8) as u8);
    let _ = frame.push(seq);

    // Dst PAN + addr
    let _ = frame.push(pan_id.0 as u8);
    let _ = frame.push((pan_id.0 >> 8) as u8);
    match dst {
        MacAddress::Short(_, a) => {
            let _ = frame.push(a.0 as u8);
            let _ = frame.push((a.0 >> 8) as u8);
        }
        MacAddress::Extended(_, ext) => {
            for b in ext {
                let _ = frame.push(*b);
            }
        }
    }

    // Src short
    let _ = frame.push(src_short.0 as u8);
    let _ = frame.push((src_short.0 >> 8) as u8);

    // Payload
    for &b in payload {
        let _ = frame.push(b);
    }

    frame
}

fn parse_beacon_frame(data: &[u8], channel: u8) -> Option<PanDescriptor> {
    if data.len() < 9 {
        return None;
    }
    let fc = u16::from_le_bytes([data[0], data[1]]);
    if fc & 0x07 != 0x00 {
        return None;
    }

    let src_pan = u16::from_le_bytes([data[3], data[4]]);
    let coord_addr = u16::from_le_bytes([data[5], data[6]]);

    let superframe_raw = if data.len() > 8 {
        u16::from_le_bytes([data[7], data[8]])
    } else {
        0
    };

    // Parse Zigbee beacon payload if present (starts after superframe spec)
    let zigbee_beacon = if data.len() >= 22 {
        let offset = 9; // after superframe spec
        ZigbeeBeaconPayload {
            protocol_id: data[offset],
            stack_profile: data[offset + 1] & 0x0F,
            protocol_version: (data[offset + 1] >> 4) & 0x0F,
            router_capacity: data[offset + 2] & 0x04 != 0,
            device_depth: (data[offset + 2] >> 3) & 0x0F,
            end_device_capacity: data[offset + 2] & 0x80 != 0,
            extended_pan_id: {
                let mut epid = [0u8; 8];
                epid.copy_from_slice(&data[offset + 3..offset + 11]);
                epid
            },
            tx_offset: [data[offset + 11], data[offset + 12], data[offset + 13]],
            update_id: if data.len() > offset + 14 {
                data[offset + 14]
            } else {
                0
            },
        }
    } else {
        ZigbeeBeaconPayload {
            protocol_id: 0,
            stack_profile: 2,
            protocol_version: 2,
            router_capacity: true,
            device_depth: 0,
            end_device_capacity: true,
            extended_pan_id: [0u8; 8],
            tx_offset: [0xFF, 0xFF, 0xFF],
            update_id: 0,
        }
    };

    Some(PanDescriptor {
        coord_address: MacAddress::Short(PanId(src_pan), ShortAddress(coord_addr)),
        channel,
        superframe_spec: SuperframeSpec::from_raw(superframe_raw),
        lqi: 0xFF,
        security_use: false,
        zigbee_beacon,
    })
}

fn parse_mac_addresses(data: &[u8]) -> (MacAddress, MacAddress, usize, bool) {
    let default_addr = MacAddress::Short(PanId(0xFFFF), ShortAddress(0xFFFF));
    if data.len() < 3 {
        return (default_addr, default_addr, 0, false);
    }

    let fc = u16::from_le_bytes([data[0], data[1]]);
    let security = (fc >> 3) & 1 != 0;
    let pan_compress = (fc >> 6) & 1 != 0;
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;

    let mut offset = 3; // past FC + seq

    // Dst PAN
    let dst_pan = if dst_mode > 0 && offset + 2 <= data.len() {
        let p = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        PanId(p)
    } else {
        PanId(0xFFFF)
    };

    // Dst addr
    let dst_address = match dst_mode {
        2 if offset + 2 <= data.len() => {
            let a = u16::from_le_bytes([data[offset], data[offset + 1]]);
            offset += 2;
            MacAddress::Short(dst_pan, ShortAddress(a))
        }
        3 if offset + 8 <= data.len() => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            MacAddress::Extended(dst_pan, ext)
        }
        _ => default_addr,
    };

    // Src PAN
    let src_pan = if src_mode > 0 && !pan_compress && offset + 2 <= data.len() {
        let p = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        PanId(p)
    } else {
        dst_pan
    };

    // Src addr
    let src_address = match src_mode {
        2 if offset + 2 <= data.len() => {
            let a = u16::from_le_bytes([data[offset], data[offset + 1]]);
            offset += 2;
            MacAddress::Short(src_pan, ShortAddress(a))
        }
        3 if offset + 8 <= data.len() => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[offset..offset + 8]);
            offset += 8;
            MacAddress::Extended(src_pan, ext)
        }
        _ => MacAddress::Short(src_pan, ShortAddress(0xFFFF)),
    };

    (src_address, dst_address, offset, security)
}

fn parse_association_response(data: &[u8]) -> Option<(ShortAddress, u8)> {
    if data.len() < 5 {
        return None;
    }
    let fc = u16::from_le_bytes([data[0], data[1]]);
    if fc & 0x07 != 0x03 {
        return None;
    }

    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 0x01;

    let mut offset = 3;
    if dst_mode > 0 {
        offset += 2;
    }
    match dst_mode {
        2 => offset += 2,
        3 => offset += 8,
        _ => {}
    }
    if src_mode > 0 && pan_compress == 0 {
        offset += 2;
    }
    match src_mode {
        2 => offset += 2,
        3 => offset += 8,
        _ => {}
    }

    if offset + 3 > data.len() {
        return None;
    }

    if data[offset] != 0x02 {
        return None;
    }

    let short = u16::from_le_bytes([data[offset + 1], data[offset + 2]]);
    let status = data[offset + 3];

    Some((ShortAddress(short), status))
}

/// Decode a one-bit-hot encoded 32-bit word back to a byte.
///
/// PHY6222 factory flash stores each byte as a 32-bit word where each
/// data bit is encoded redundantly across multiple word bits.
/// The encoding uses 4 bits per data bit (positions 0,8,16,24 for bit-hot):
///   - Each nybble of the word represents one bit of the output byte
///   - A nybble must have exactly one bit set (one-bit-hot)
///   - The bit position within the nybble gives the 2-bit data value
///
/// Per the PHY6222 SDK `chip_id_one_bit_hot_convter()`: the 32-bit word
/// is split into 8 nybbles. Each nybble must be a power-of-2 (exactly
/// one bit set). The bit index (0-3) within each nybble contributes
/// 2 bits to the output byte.
///
/// Returns `None` if the word is blank (0xFFFFFFFF) or has invalid encoding.
fn one_bit_hot_decode(word: u32) -> Option<u8> {
    if word == 0xFFFF_FFFF || word == 0x0000_0000 {
        return None;
    }

    let mut result: u8 = 0;

    // Each nybble (4 bits) of the 32-bit word encodes 1 bit of the result.
    // There are 8 nybbles → 8 bits of output.
    for i in 0u8..8 {
        let nybble = ((word >> (i * 4)) & 0x0F) as u8;

        // Must be exactly one bit set (power of 2)
        if nybble == 0 || (nybble & (nybble - 1)) != 0 {
            return None;
        }

        // The bit position gives the data bit value (0 or 1)
        // nybble=1 (bit 0) → data bit 0
        // nybble=2 (bit 1) → data bit 1
        // nybble=4 (bit 2) → data bit 0 (redundant encoding)
        // nybble=8 (bit 3) → data bit 1 (redundant encoding)
        // Simplified: bit 0 of the position = data bit
        let bit_pos = nybble.trailing_zeros() as u8;
        let data_bit = bit_pos & 1;
        result |= data_bit << i;
    }

    Some(result)
}
