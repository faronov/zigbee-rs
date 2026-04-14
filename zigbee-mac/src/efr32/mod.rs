//! EFR32MG1P MAC backend — pure Rust, zero vendor blobs.
//!
//! Implements `MacDriver` for the Silicon Labs EFR32MG1P ARM Cortex-M4F SoC.
//! The EFR32MG1P has a multi-protocol radio (BLE + IEEE 802.15.4) and is found
//! in devices like the IKEA TRÅDFRI motion sensor and many Zigbee modules.
//!
//! This is a **pure-Rust radio driver**: all radio configuration uses direct
//! register access. No RAIL library, no GSDK binary blobs are linked.
//!
//! # IMPORTANT — Scaffold Implementation
//! The radio register values are simplified approximations. The exact register
//! sequences for 802.15.4 mode need verification against the EFR32xG1 Reference
//! Manual or extraction from the RAIL library source. The driver compiles and
//! has the correct structure, but register values need to be verified before
//! use on real hardware.

pub mod driver;
mod rac_seq;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use driver::{Efr32Driver, RadioConfig, RadioError};
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;

/// Maximum MAC payload size (127 - MAC overhead).
const MAX_MAC_PAYLOAD: usize = 102;

/// EFR32MG1P IEEE 802.15.4 MAC driver.
///
/// Pure-Rust implementation using direct register access — no RAIL FFI.
pub struct Efr32Mac {
    driver: Efr32Driver,
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

impl Efr32Mac {
    pub fn new() -> Self {
        let config = RadioConfig::default();
        let ieee = Self::read_factory_ieee();
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
        Self {
            driver: Efr32Driver::new(config),
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

    /// Read the factory-programmed IEEE 802.15.4 EUI-64 address.
    ///
    /// EFR32MG1P stores a unique 64-bit EUI in the Device Information (DI) page
    /// at address 0x0FE0_81A0. This is programmed at the factory and cannot be
    /// changed.
    fn read_factory_ieee() -> [u8; 8] {
        // EFR32MG1P Device Information page — EUI64
        const DI_EUI64_ADDR: u32 = 0x0FE0_81A0; // TODO: verify against EFR32xG1 RM

        let mut eui64 = [0u8; 8];
        let lo = unsafe { core::ptr::read_volatile(DI_EUI64_ADDR as *const u32) };
        let hi = unsafe { core::ptr::read_volatile((DI_EUI64_ADDR + 4) as *const u32) };

        eui64[0] = (lo >> 0) as u8;
        eui64[1] = (lo >> 8) as u8;
        eui64[2] = (lo >> 16) as u8;
        eui64[3] = (lo >> 24) as u8;
        eui64[4] = (hi >> 0) as u8;
        eui64[5] = (hi >> 8) as u8;
        eui64[6] = (hi >> 16) as u8;
        eui64[7] = (hi >> 24) as u8;

        // Validate — if all-zeros or all-ones, use fallback
        if eui64 == [0xFF; 8] || eui64 == [0x00; 8] {
            log::warn!("efr32: no factory EUI64 — using fallback");
            return [0x00, 0x0D, 0x6F, 0xFF, 0xFE, 0xDE, 0xAD, 0x02];
        }

        eui64
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
    /// Saves ~5–10 mA. Call `radio_wake()` before next TX/RX.
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
        const SYMBOL_PERIOD_US: u64 = 16; // 62.5 ksym/s
        const UNIT_BACKOFF_SYMBOLS: u64 = 20; // aUnitBackoffPeriod

        for attempt in 0..=max_retries {
            // ── Unslotted CSMA-CA ──
            let mut nb: u8 = 0;
            let mut be = self.min_be;

            let channel_clear = loop {
                let max_val = (1u32 << be) - 1;
                let seed = self.dsn.wrapping_add(nb).wrapping_add(attempt);
                let random = Self::prng(seed);
                let backoff = (random % (max_val + 1)) as u64;
                let delay_us = backoff * UNIT_BACKOFF_SYMBOLS * SYMBOL_PERIOD_US;
                if delay_us > 0 {
                    Timer::after_micros(delay_us).await;
                }

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

            if attempt == max_retries {
                return Err(MacError::NoAck);
            }
            log::debug!(
                "efr32: no ACK for seq={}, retry {}/{}",
                seq,
                attempt + 1,
                max_retries
            );
        }

        Err(MacError::NoAck)
    }

    /// Send a 3-byte IEEE 802.15.4 ACK frame for the given sequence number.
    async fn send_ack(&mut self, seq: u8) {
        let ack = [0x02u8, 0x00, seq];
        let _ = self.driver.transmit(&ack).await;
    }
}

impl MacDriver for Efr32Mac {
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
                    rtt_target::rprintln!("scan ED ch{}", ch);
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
                    // Dump first beacon request frame bytes
                    if ch == 11 {
                        rtt_target::rprintln!(
                            "beacon_req[{}]: {:02X?}",
                            beacon_req.len(),
                            &beacon_req
                        );
                    }
                    rtt_target::rprintln!("scan TX ch{}", ch);
                    let tx_result = self.driver.transmit(&beacon_req).await;
                    rtt_target::rprintln!("  tx={}", if tx_result.is_ok() { "ok" } else { "FAIL" });

                    // Debug: check FRC_STATUS and RSSI during RX listen
                    if ch == 15 {
                        // Read radio state while listening
                        let frc_status =
                            unsafe { core::ptr::read_volatile(0x40080000 as *const u32) };
                        let rac_status =
                            unsafe { core::ptr::read_volatile(0x40084004 as *const u32) };
                        let agc_rssi =
                            unsafe { core::ptr::read_volatile(0x40087008 as *const u32) };
                        let frc_if = unsafe { core::ptr::read_volatile(0x40080060 as *const u32) };
                        rtt_target::rprintln!(
                            "  ch15 RX: FRC_ST={:#X} RAC={:#X} RSSI={:#X} FRC_IF={:#X}",
                            frc_status,
                            (rac_status >> 24) & 0xF,
                            agc_rssi,
                            frc_if
                        );
                    }

                    let deadline = embassy_time::Instant::now()
                        + embassy_time::Duration::from_millis(scan_duration_ms);
                    while !pan_descriptors.is_full() {
                        let now = embassy_time::Instant::now();
                        if now >= deadline {
                            break;
                        }
                        let remaining = deadline - now;
                        let result =
                            select::select(self.driver.receive(), Timer::after(remaining)).await;

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
                        let result =
                            select::select(self.driver.receive(), Timer::after(remaining)).await;

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

        self.csma_ca_transmit(&frame, true).await?;

        Timer::after(embassy_time::Duration::from_millis(200)).await;

        let mut confirm: Option<MlmeAssociateConfirm> = None;

        for poll_attempt in 0..5u8 {
            if poll_attempt > 0 {
                Timer::after(embassy_time::Duration::from_millis(500)).await;
            }

            let data_req = build_data_request_ieee(
                self.next_dsn(),
                &req.coord_address,
                &self.extended_address,
            );
            let _ = self.csma_ca_transmit(&data_req, true).await;

            let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(1500);

            for _ in 0..20u8 {
                let now = embassy_time::Instant::now();
                if now >= deadline {
                    break;
                }
                let remaining = deadline - now;

                let result = select::select(self.driver.receive(), Timer::after(remaining)).await;

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

                        if frame_type == 0x02 {
                            continue;
                        }

                        if (fc >> 5) & 1 != 0 {
                            self.send_ack(data[2]).await;
                        }

                        if frame_type == 0x03 {
                            if let Some((addr, status_byte)) = parse_association_response(data) {
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

                        if frame_type == 0x01 && self.pending_assoc_frame.is_none() {
                            let (_, _, payload_offset, _) = parse_mac_addresses(data);
                            if data.len() > payload_offset {
                                let payload = &data[payload_offset..];
                                let mut buf = [0u8; 128];
                                let copy_len = payload.len().min(128);
                                buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                                self.pending_assoc_frame = Some((buf, copy_len));
                                log::info!("efr32: saved post-assoc frame ({} bytes)", copy_len);
                            }
                        }
                    }
                }
            }

            if confirm.is_some() {
                break;
            }
        }

        // Listen briefly for Transport-Key after association
        if confirm.is_some() && self.pending_assoc_frame.is_none() {
            let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(2000);
            for _ in 0..20u8 {
                let now = embassy_time::Instant::now();
                if now >= deadline {
                    break;
                }
                let remaining = deadline - now;
                let result = select::select(self.driver.receive(), Timer::after(remaining)).await;
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
                                log::info!("efr32: saved post-assoc frame ({} bytes)", copy_len);
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
        if let Some((buf, len)) = self.pending_assoc_frame.take() {
            log::info!(
                "[MAC:Poll] Returning saved association frame ({} bytes)",
                len
            );
            return Ok(MacFrame::from_slice(&buf[..len]));
        }

        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let has_short = self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE;

        let passes: u8 = if has_short { 2 } else { 1 };

        for pass in 0..passes {
            let data_req = if pass == 0 && has_short {
                build_data_request(self.next_dsn(), self.pan_id, self.coord_short_address)
                    .as_slice()
                    .iter()
                    .copied()
                    .collect::<heapless::Vec<u8, 24>>()
            } else {
                build_data_request_ieee(self.next_dsn(), &parent, &self.extended_address)
            };

            if self.csma_ca_transmit(&data_req, true).await.is_err() {
                continue;
            }

            let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(1500);

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
                    select::Either::Second(_) => break,
                    select::Either::First(Err(_)) => continue,
                    select::Either::First(Ok(rx)) => {
                        let data = &rx.data[..rx.len];
                        if data.len() < 3 {
                            continue;
                        }
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        let frame_type = fc & 0x07;

                        if frame_type == 0x02 {
                            let frame_pending = (data[0] >> 4) & 1 != 0;
                            if !frame_pending {
                                got_none = true;
                                break;
                            }
                            continue;
                        }

                        if frame_type != 0x01 {
                            continue;
                        }

                        let (_src, dst, payload_offset, _security_use) = parse_mac_addresses(data);
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

        let seq = self.next_dsn();
        let mac_frame = build_data_frame(
            seq,
            self.pan_id,
            &req.dst_address,
            self.short_address,
            req.payload,
            ack_requested,
        );

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

                    if frame_type != 1 {
                        continue;
                    }

                    if (fc >> 5) & 1 != 0 {
                        self.send_ack(data[2]).await;
                    }

                    let (src_address, dst_address, payload_offset, security_use) =
                        parse_mac_addresses(data);

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
                                        "efr32: filtered short dst pan=0x{:04X} addr=0x{:04X}",
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
                                    log::trace!("efr32: filtered ext dst pan=0x{:04X}", pan.0,);
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

                    log::trace!("efr32: rx {} bytes lqi={}", rx.len, rx.lqi);

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
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(19),
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

fn build_data_request_ieee(
    seq: u8,
    coord: &MacAddress,
    own_ieee: &IeeeAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame: heapless::Vec<u8, 24> = heapless::Vec::new();
    let fc: u16 = 0xCC63;
    let _ = frame.push(fc as u8);
    let _ = frame.push((fc >> 8) as u8);
    let _ = frame.push(seq);
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
    for b in own_ieee {
        let _ = frame.push(*b);
    }
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

    let mut fc: u16 = 0x8861; // data + dst short + src short + PAN compress
    if ack {
        fc |= 0x0020;
    }
    let _ = frame.push(fc as u8);
    let _ = frame.push((fc >> 8) as u8);
    let _ = frame.push(seq);

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

    let _ = frame.push(src_short.0 as u8);
    let _ = frame.push((src_short.0 >> 8) as u8);

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

    let zigbee_beacon = if data.len() >= 22 {
        let offset = 9;
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

    let mut offset = 3;

    let dst_pan = if dst_mode > 0 && offset + 2 <= data.len() {
        let p = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        PanId(p)
    } else {
        PanId(0xFFFF)
    };

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

    let src_pan = if src_mode > 0 && !pan_compress && offset + 2 <= data.len() {
        let p = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        PanId(p)
    } else {
        dst_pan
    };

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
