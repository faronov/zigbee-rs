//! ESP32 MAC backend (ESP32-C6 / ESP32-H2).
//!
//! Implements `MacDriver` using `esp-radio::ieee802154` for ESP32 chips
//! with built-in IEEE 802.15.4 radio (RISC-V based).
//!
//! # Supported chips
//! - ESP32-C6 (RISC-V, 2.4 GHz WiFi + BLE + 802.15.4)
//! - ESP32-H2 (RISC-V, BLE + 802.15.4)
//!
//! # Features
//! - Real beacon parsing during active/passive scan
//! - IEEE 802.15.4 association flow (request → poll → response)
//! - Software address filtering in RX path
//! - CSMA-CA with backoff for TX
//! - EUI-64 address from eFuse factory MAC

mod driver;

use crate::pib::{self, PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use driver::Ieee802154Driver;
use zigbee_types::*;

use embassy_time::{Duration, Instant, Timer};
use esp_radio::ieee802154::{Config, Ieee802154};

/// ESP32 802.15.4 MAC driver.
pub struct EspMac<'a> {
    driver: Ieee802154Driver<'a>,
    // PIB state
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
    tx_power: i8,
}

impl<'a> EspMac<'a> {
    pub fn new(ieee802154: Ieee802154<'a>, config: Config) -> Self {
        let ieee = Self::read_efuse_ieee();

        Self {
            driver: Ieee802154Driver::new(ieee802154, config),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: ieee,
            coord_short_address: ShortAddress(0x0000),
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
            tx_power: 0,
        }
    }

    /// Read IEEE EUI-64 address from ESP32 eFuse factory MAC.
    ///
    /// The 6-byte factory MAC is stored in eFuse and converted to EUI-64
    /// by inserting FF:FE in the middle (standard MAC-48 → EUI-64).
    fn read_efuse_ieee() -> IeeeAddress {
        // eFuse read registers for factory MAC — same offset on C6 and H2
        // EFUSE_RD_MAC_SYS_0 = base + 0x44, EFUSE_RD_MAC_SYS_1 = base + 0x48
        const EFUSE_RD_MAC_SYS_0: *const u32 = 0x600B_0844 as *const u32;
        const EFUSE_RD_MAC_SYS_1: *const u32 = 0x600B_0848 as *const u32;

        let lo = unsafe { core::ptr::read_volatile(EFUSE_RD_MAC_SYS_0) };
        let hi = unsafe { core::ptr::read_volatile(EFUSE_RD_MAC_SYS_1) };

        // Base MAC is 6 bytes: lo[31:0] = MAC[31:0], hi[15:0] = MAC[47:32]
        let mac0 = (lo & 0xFF) as u8;
        let mac1 = ((lo >> 8) & 0xFF) as u8;
        let mac2 = ((lo >> 16) & 0xFF) as u8;
        let mac3 = ((lo >> 24) & 0xFF) as u8;
        let mac4 = (hi & 0xFF) as u8;
        let mac5 = ((hi >> 8) & 0xFF) as u8;

        // Convert MAC-48 to EUI-64 by inserting FF:FE in middle
        // and flipping the universal/local bit (bit 1 of byte 0)
        [mac0 ^ 0x02, mac1, mac2, 0xFF, 0xFE, mac3, mac4, mac5]
    }

    fn next_dsn(&mut self) -> u8 {
        let seq = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        seq
    }

    /// Transmit a frame with a small post-TX settle delay.
    async fn transmit_frame(&mut self, frame: &[u8]) -> Result<(), MacError> {
        self.driver
            .transmit(frame)
            .map_err(|_| MacError::RadioError)?;
        Timer::after_micros(200).await;
        Ok(())
    }

    /// Collect beacons during scan with timeout.
    async fn collect_beacons(
        &mut self,
        channel: u8,
        duration_us: u64,
        descriptors: &mut PanDescriptorList,
    ) {
        let deadline = Instant::now() + Duration::from_micros(duration_us);
        self.driver.start_receive();

        loop {
            if Instant::now() >= deadline {
                break;
            }
            if let Some(Ok(rx)) = self.driver.poll_receive() {
                let data = &rx.data[..rx.len];
                if let Some(pd) = parse_beacon(channel, data, rx.lqi) {
                    if descriptors.push(pd).is_err() {
                        break;
                    }
                }
                self.driver.start_receive();
            } else {
                Timer::after_micros(200).await;
            }
        }
    }

    /// Wait for association response (MAC command 0x02) with timeout.
    async fn wait_assoc_response(
        &mut self,
        timeout_ms: u64,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        self.driver.start_receive();

        loop {
            if Instant::now() >= deadline {
                return Err(MacError::NoAck);
            }
            if let Some(Ok(rx)) = self.driver.poll_receive() {
                let data = &rx.data[..rx.len];
                if data.len() < 5 {
                    self.driver.start_receive();
                    continue;
                }
                let fc = u16::from_le_bytes([data[0], data[1]]);
                // Must be a MAC command frame (type 3)
                if fc & 0x07 != 3 {
                    self.driver.start_receive();
                    continue;
                }
                let cmd_offset = 3 + addressing_size(fc);
                if data.len() < cmd_offset + 4 {
                    self.driver.start_receive();
                    continue;
                }
                // Association Response = command ID 0x02
                if data[cmd_offset] == 0x02 {
                    let short_addr =
                        u16::from_le_bytes([data[cmd_offset + 1], data[cmd_offset + 2]]);
                    let status = match data[cmd_offset + 3] {
                        0x00 => AssociationStatus::Success,
                        0x01 => AssociationStatus::PanAtCapacity,
                        _ => AssociationStatus::PanAccessDenied,
                    };
                    if status == AssociationStatus::Success {
                        self.short_address = ShortAddress(short_addr);
                    }
                    return Ok(MlmeAssociateConfirm {
                        short_address: ShortAddress(short_addr),
                        status,
                    });
                }
                self.driver.start_receive();
            } else {
                Timer::after_micros(200).await;
            }
        }
    }
}

impl MacDriver for EspMac<'_> {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();
        let mut energy_list: EdList = heapless::Vec::new();

        let scan_us = pib::scan_duration_us(req.scan_duration);

        log::info!(
            "[ESP MLME-SCAN] Starting {:?} scan, duration={}",
            req.scan_type,
            req.scan_duration
        );

        for ch in req.channel_mask.iter() {
            let ch_num = ch.number();
            log::debug!("[ESP MLME-SCAN] Scanning ch {}…", ch_num);
            self.channel = ch_num;
            self.driver.update_config(|cfg| cfg.channel = ch_num);

            match req.scan_type {
                ScanType::Active => {
                    // Send beacon request then listen for responses
                    let beacon_req = build_beacon_request(self.next_dsn());
                    let _ = self.transmit_frame(&beacon_req).await;
                    self.collect_beacons(ch_num, scan_us, &mut pan_descriptors)
                        .await;
                }
                ScanType::Passive => {
                    // Just listen for beacons (no beacon request sent)
                    self.collect_beacons(ch_num, scan_us, &mut pan_descriptors)
                        .await;
                }
                ScanType::Ed => {
                    // Brief listen for energy measurement
                    self.driver.start_receive();
                    Timer::after_millis(2).await;
                    let energy = if let Some(Ok(rx)) = self.driver.poll_receive() {
                        rx.lqi
                    } else {
                        0
                    };
                    let _ = energy_list.push(EdValue {
                        channel: ch_num,
                        energy,
                    });
                }
                ScanType::Orphan => {
                    log::warn!("[ESP] Orphan scan not yet implemented");
                }
            }
        }

        if matches!(req.scan_type, ScanType::Active | ScanType::Passive)
            && pan_descriptors.is_empty()
        {
            log::warn!("[ESP MLME-SCAN] No beacons found on any channel");
            return Err(MacError::NoBeacon);
        }

        log::info!(
            "[ESP MLME-SCAN] Scan complete: {} PAN descriptor(s)",
            pan_descriptors.len()
        );

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
        self.pan_id = req.coord_address.pan_id();
        self.driver.update_config(|cfg| {
            cfg.channel = req.channel;
            cfg.pan_id = Some(req.coord_address.pan_id().0);
        });

        log::info!(
            "[ESP MLME-ASSOC] Associating on ch {} with {:?}",
            req.channel,
            req.coord_address
        );

        // Build and send Association Request command frame
        let frame = build_association_request(
            self.next_dsn(),
            &req.coord_address,
            &self.extended_address,
            &req.capability_info,
        );
        self.transmit_frame(&frame).await?;

        // Per IEEE 802.15.4 §5.3.2.1: wait, then poll with Data Request
        Timer::after_millis(100).await;

        // Send Data Request to poll for indirect Association Response
        let data_req =
            build_data_request(self.next_dsn(), &req.coord_address, &self.extended_address);
        self.transmit_frame(&data_req).await?;

        // Wait for Association Response with generous timeout (3 seconds)
        self.wait_assoc_response(3000).await
    }

    async fn mlme_associate_response(
        &mut self,
        _resp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        // TODO: coordinator/router role
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(&mut self, _req: MlmeDisassociateRequest) -> Result<(), MacError> {
        self.short_address = ShortAddress(0xFFFF);
        self.pan_id = PanId(0xFFFF);
        Ok(())
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.channel = 11;
            self.rx_on_when_idle = false;
            self.association_permit = false;
            self.auto_request = true;
            self.dsn = 0;
            self.bsn = 0;
            self.beacon_payload = PibPayload::new();
            self.max_csma_backoffs = 4;
            self.min_be = 3;
            self.max_be = 5;
            self.max_frame_retries = 3;
            self.promiscuous = false;
            self.tx_power = 0;
        }
        self.driver.update_config(|cfg| {
            cfg.channel = self.channel;
            cfg.pan_id = Some(self.pan_id.0);
            cfg.short_addr = Some(self.short_address.0);
        });
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.channel;
        self.driver.update_config(|cfg| {
            cfg.channel = req.channel;
            cfg.pan_id = Some(req.pan_id.0);
        });
        Ok(())
    }

    async fn mlme_get(&self, attribute: PibAttribute) -> Result<PibValue, MacError> {
        match attribute {
            PibAttribute::MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            PibAttribute::MacPanId => Ok(PibValue::PanId(self.pan_id)),
            PibAttribute::PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PibAttribute::MacExtendedAddress => {
                Ok(PibValue::ExtendedAddress(self.extended_address))
            }
            PibAttribute::MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            PibAttribute::MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            PibAttribute::MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            PibAttribute::MacDsn => Ok(PibValue::U8(self.dsn)),
            PibAttribute::MacBsn => Ok(PibValue::U8(self.bsn)),
            PibAttribute::MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            PibAttribute::MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
            PibAttribute::MacMinBe => Ok(PibValue::U8(self.min_be)),
            PibAttribute::MacMaxBe => Ok(PibValue::U8(self.max_be)),
            PibAttribute::MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
            PibAttribute::MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PibAttribute::PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
            PibAttribute::PhyChannelsSupported => Ok(PibValue::U32(ChannelMask::ALL_2_4GHZ.0)),
            _ => Ok(PibValue::U8(0)),
        }
    }

    async fn mlme_set(&mut self, attribute: PibAttribute, value: PibValue) -> Result<(), MacError> {
        match (attribute, &value) {
            (PibAttribute::MacShortAddress, PibValue::ShortAddress(v)) => {
                self.short_address = *v;
                self.driver.update_config(|cfg| cfg.short_addr = Some(v.0));
            }
            (PibAttribute::MacPanId, PibValue::PanId(v)) => {
                self.pan_id = *v;
                self.driver.update_config(|cfg| cfg.pan_id = Some(v.0));
            }
            (PibAttribute::PhyCurrentChannel, PibValue::U8(v)) => {
                if !(11..=26).contains(v) {
                    return Err(MacError::InvalidParameter);
                }
                self.channel = *v;
                self.driver.update_config(|cfg| cfg.channel = *v);
            }
            (PibAttribute::MacExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.extended_address = *v;
            }
            (PibAttribute::MacRxOnWhenIdle, PibValue::Bool(v)) => self.rx_on_when_idle = *v,
            (PibAttribute::MacAssociationPermit, PibValue::Bool(v)) => {
                self.association_permit = *v;
            }
            (PibAttribute::MacAutoRequest, PibValue::Bool(v)) => self.auto_request = *v,
            (PibAttribute::MacBeaconPayload, PibValue::Payload(v)) => {
                self.beacon_payload = v.clone();
            }
            (PibAttribute::MacMaxCsmaBackoffs, PibValue::U8(v)) => self.max_csma_backoffs = *v,
            (PibAttribute::MacMinBe, PibValue::U8(v)) => self.min_be = *v,
            (PibAttribute::MacMaxBe, PibValue::U8(v)) => self.max_be = *v,
            (PibAttribute::MacMaxFrameRetries, PibValue::U8(v)) => self.max_frame_retries = *v,
            (PibAttribute::MacPromiscuousMode, PibValue::Bool(v)) => self.promiscuous = *v,
            (PibAttribute::PhyTransmitPower, PibValue::I8(v)) => self.tx_power = *v,
            (PibAttribute::MacCoordShortAddress, PibValue::ShortAddress(v)) => {
                self.coord_short_address = *v;
            }
            _ => return Err(MacError::Unsupported),
        }
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Build and send MAC Data Request to parent (coordinator)
        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let data_req = build_data_request(self.next_dsn(), &parent, &self.extended_address);
        self.transmit_frame(&data_req).await?;

        // Wait for response with polling
        self.driver.start_receive();
        let deadline = Instant::now() + Duration::from_millis(500);

        loop {
            if Instant::now() >= deadline {
                return Ok(None);
            }
            if let Some(result) = self.driver.poll_receive() {
                let received = result.map_err(|_| MacError::RadioError)?;
                if received.len < 5 {
                    self.driver.start_receive();
                    continue;
                }
                let data = &received.data[..received.len];
                let fc = u16::from_le_bytes([data[0], data[1]]);
                let frame_type = fc & 0x07;

                // Only deliver data frames (type 1)
                if frame_type != 1 {
                    self.driver.start_receive();
                    continue;
                }

                let header_len = 3 + addressing_size(fc);
                if data.len() <= header_len {
                    return Ok(None);
                }

                let payload_data = &data[header_len..];
                return Ok(MacFrame::from_slice(payload_data));
            } else {
                Timer::after_micros(200).await;
            }
        }
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let msdu_handle = req.msdu_handle;
        let ack_requested = req.tx_options.ack_tx;
        let mut frame_buf = [0u8; 127];
        let len = build_data_frame(
            &mut frame_buf,
            self.next_dsn(),
            self.short_address,
            self.pan_id,
            &self.extended_address,
            &req,
        )?;

        // Unslotted CSMA-CA (IEEE 802.15.4-2011 §5.1.1.4)
        let mut be = self.min_be;
        let mut nb: u8 = 0;
        let symbol_period_us: u64 = 16; // 2.4 GHz = 62.5 ksym/s
        let unit_backoff_symbols: u64 = 20; // aUnitBackoffPeriod

        loop {
            let max_val = (1u32 << be) - 1;
            let random = (self.dsn as u32)
                .wrapping_mul(1103515245)
                .wrapping_add(12345);
            let backoff = (random % (max_val + 1)) as u64;
            let delay_us = backoff * unit_backoff_symbols * symbol_period_us;
            if delay_us > 0 {
                Timer::after_micros(delay_us).await;
            }

            match self.driver.transmit(&frame_buf[..len]) {
                Ok(()) => {
                    Timer::after_micros(200).await;
                    break;
                }
                Err(_) => {
                    nb += 1;
                    be = core::cmp::min(be + 1, self.max_be);
                    if nb > self.max_csma_backoffs {
                        return Err(MacError::ChannelAccessFailure);
                    }
                }
            }
        }

        // Best-effort retransmit for ACK-requested frames
        if ack_requested {
            for retransmit in 0..4u8 {
                Timer::after_millis(3 + retransmit as u64 * 2).await;
                let _ = self.driver.transmit(&frame_buf[..len]);
                Timer::after_micros(200).await;
            }
        }

        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        const RX_TIMEOUT_MS: u64 = 5000;
        let deadline = Instant::now() + Duration::from_millis(RX_TIMEOUT_MS);
        self.driver.start_receive();

        loop {
            if Instant::now() >= deadline {
                return Err(MacError::NoData);
            }

            if let Some(result) = self.driver.poll_receive() {
                let received = result.map_err(|_| MacError::RadioError)?;
                let data = &received.data[..received.len];

                if data.len() < 5 {
                    self.driver.start_receive();
                    continue;
                }

                let fc = u16::from_le_bytes([data[0], data[1]]);
                let frame_type = fc & 0x07;

                // Only deliver data frames (type 1) to upper layer
                if frame_type != 1 {
                    self.driver.start_receive();
                    continue;
                }

                let header_len = 3 + addressing_size(fc);
                if data.len() <= header_len {
                    self.driver.start_receive();
                    continue;
                }

                let src = parse_source_address(data, fc)
                    .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));
                let dst = parse_dest_address(data, fc)
                    .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));

                // Software address filtering — only accept frames for us
                if !self.promiscuous {
                    let accepted = match &dst {
                        MacAddress::Short(pan, addr) => {
                            (pan.0 == self.pan_id.0 || pan.0 == 0xFFFF)
                                && (addr.0 == self.short_address.0 || addr.0 == 0xFFFF)
                        }
                        MacAddress::Extended(pan, addr) => {
                            (pan.0 == self.pan_id.0 || pan.0 == 0xFFFF)
                                && *addr == self.extended_address
                        }
                    };
                    if !accepted {
                        self.driver.start_receive();
                        continue;
                    }
                }

                log::info!(
                    "[ESP RX] Accepted frame {} bytes, LQI {}",
                    data.len(),
                    received.lqi
                );

                let payload_data = &data[header_len..];
                if let Some(mac_frame) = MacFrame::from_slice(payload_data) {
                    return Ok(McpsDataIndication {
                        src_address: src,
                        dst_address: dst,
                        lqi: received.lqi,
                        payload: mac_frame,
                        security_use: (fc >> 3) & 1 != 0,
                    });
                }
                self.driver.start_receive();
            } else {
                Timer::after_micros(200).await;
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: true,
            router: true,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(-24),
            tx_power_max: TxPower(21),
        }
    }
}

// ── Frame parsing utilities ─────────────────────────────────────

/// Calculate addressing field size from Frame Control word.
fn addressing_size(fc: u16) -> usize {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    let mut size = 0;
    // Destination
    match dst_mode {
        0x02 => size += 2 + 2, // PAN(2) + Short(2)
        0x03 => size += 2 + 8, // PAN(2) + Extended(8)
        _ => {}
    }
    // Source
    match src_mode {
        0x02 => size += if pan_compress { 2 } else { 4 }, // Short ± PAN
        0x03 => size += if pan_compress { 8 } else { 10 }, // Extended ± PAN
        _ => {}
    }
    size
}

/// Parse source address from raw MAC frame.
fn parse_source_address(data: &[u8], fc: u16) -> Option<MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

    // Skip past FC(2) + Seq(1) + dst addressing
    let mut offset = 3;
    let dst_pan = if dst_mode >= 2 && data.len() > offset + 1 {
        let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        Some(pan)
    } else {
        None
    };
    match dst_mode {
        0x02 => offset += 2,
        0x03 => offset += 8,
        _ => {}
    }

    // Source PAN
    let src_pan = if !pan_compress && src_mode >= 2 && data.len() > offset + 1 {
        let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;
        pan
    } else {
        dst_pan.unwrap_or(0xFFFF)
    };

    match src_mode {
        0x02 if data.len() >= offset + 2 => {
            let addr = u16::from_le_bytes([data[offset], data[offset + 1]]);
            Some(MacAddress::Short(PanId(src_pan), ShortAddress(addr)))
        }
        0x03 if data.len() >= offset + 8 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[offset..offset + 8]);
            Some(MacAddress::Extended(PanId(src_pan), ext))
        }
        _ => None,
    }
}

/// Parse destination address from raw MAC frame.
fn parse_dest_address(data: &[u8], fc: u16) -> Option<MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let offset = 3; // After FC(2) + Seq(1)

    if data.len() < offset + 2 {
        return None;
    }
    let pan = u16::from_le_bytes([data[offset], data[offset + 1]]);
    let addr_offset = offset + 2;

    match dst_mode {
        0x02 if data.len() >= addr_offset + 2 => {
            let addr = u16::from_le_bytes([data[addr_offset], data[addr_offset + 1]]);
            Some(MacAddress::Short(PanId(pan), ShortAddress(addr)))
        }
        0x03 if data.len() >= addr_offset + 8 => {
            let mut ext = [0u8; 8];
            ext.copy_from_slice(&data[addr_offset..addr_offset + 8]);
            Some(MacAddress::Extended(PanId(pan), ext))
        }
        _ => None,
    }
}

/// Parse Zigbee beacon payload from raw bytes (at least 15 bytes).
fn parse_zigbee_beacon(data: &[u8]) -> ZigbeeBeaconPayload {
    let protocol_id = data[0];
    let nwk_info = u16::from_le_bytes([data[1], data[2]]);

    let mut extended_pan_id = [0u8; 8];
    extended_pan_id.copy_from_slice(&data[3..11]);
    let mut tx_offset = [0u8; 3];
    tx_offset.copy_from_slice(&data[11..14]);

    ZigbeeBeaconPayload {
        protocol_id,
        stack_profile: (nwk_info & 0x0F) as u8,
        protocol_version: ((nwk_info >> 4) & 0x0F) as u8,
        router_capacity: (nwk_info >> 10) & 1 != 0,
        device_depth: ((nwk_info >> 11) & 0x0F) as u8,
        end_device_capacity: (nwk_info >> 15) & 1 != 0,
        extended_pan_id,
        tx_offset,
        update_id: data[14],
    }
}

/// Parse a received frame as a beacon. Returns None if not a beacon.
fn parse_beacon(channel: u8, data: &[u8], lqi: u8) -> Option<PanDescriptor> {
    if data.len() < 5 {
        return None;
    }

    let fc = u16::from_le_bytes([data[0], data[1]]);
    let frame_type = fc & 0x07;

    // Must be a beacon frame (type 0)
    if frame_type != 0 {
        return None;
    }

    // Parse superframe spec from beacon payload
    let superframe_offset = 3 + addressing_size(fc);
    if data.len() < superframe_offset + 2 {
        return None;
    }

    let sf_raw = u16::from_le_bytes([data[superframe_offset], data[superframe_offset + 1]]);
    let superframe_spec = SuperframeSpec::from_raw(sf_raw);

    // Zigbee beacon payload follows superframe + GTS + pending address fields
    let beacon_payload_offset = superframe_offset + 4; // +2 sf, +1 gts, +1 pending
    if data.len() < beacon_payload_offset + 15 {
        return None;
    }

    let zigbee_beacon = parse_zigbee_beacon(&data[beacon_payload_offset..]);
    let coord_address = parse_source_address(data, fc)?;

    Some(PanDescriptor {
        channel,
        coord_address,
        superframe_spec,
        lqi,
        security_use: (fc >> 3) & 1 != 0,
        zigbee_beacon,
    })
}

// ── Frame-building utilities ────────────────────────────────────

/// Build a Beacon Request MAC command frame.
fn build_beacon_request(seq: u8) -> [u8; 8] {
    // FC: MAC command(3), dst=short(0x0800), broadcast PAN
    [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]
}

/// Build an Association Request MAC command frame.
fn build_association_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
    cap: &CapabilityInfo,
) -> heapless::Vec<u8, 32> {
    let mut frame = heapless::Vec::new();
    // FC: MAC command, ack req, PAN compress, dst=short, src=extended
    let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);
    let dst_pan = coord.pan_id();
    let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());
    match coord {
        MacAddress::Short(_, addr) => {
            let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
        }
        MacAddress::Extended(_, addr) => {
            let _ = frame.extend_from_slice(addr);
        }
    }
    let _ = frame.extend_from_slice(own_extended);
    let _ = frame.push(0x01); // Association Request command ID
    let _ = frame.push(cap.to_byte());
    frame
}

/// Build a MAC Data Request command frame (used for polling parent).
fn build_data_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    // FC: MAC command, ack req, PAN compress, dst=short, src=extended
    let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);
    let dst_pan = coord.pan_id();
    let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());
    match coord {
        MacAddress::Short(_, addr) => {
            let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
        }
        MacAddress::Extended(_, addr) => {
            let _ = frame.extend_from_slice(addr);
        }
    }
    let _ = frame.extend_from_slice(own_extended);
    let _ = frame.push(0x04); // Data Request command ID
    frame
}

/// Build an IEEE 802.15.4 data frame into `buf`, return length.
fn build_data_frame(
    buf: &mut [u8; 127],
    seq: u8,
    short_addr: ShortAddress,
    _pan_id: PanId,
    extended_addr: &IeeeAddress,
    req: &McpsDataRequest<'_>,
) -> Result<usize, MacError> {
    let mut fc: u16 = 0x0001; // Data frame
    if req.tx_options.ack_tx {
        fc |= 0x0020;
    }
    fc |= 0x0040; // PAN ID compression

    match req.dst_address {
        MacAddress::Short(_, _) => fc |= 0x0800,
        MacAddress::Extended(_, _) => fc |= 0x0C00,
    }
    if short_addr.0 != 0xFFFF {
        fc |= 0x8000;
    } else {
        fc |= 0xC000;
    }

    let mut pos = 0;
    buf[pos] = (fc & 0xFF) as u8;
    pos += 1;
    buf[pos] = ((fc >> 8) & 0xFF) as u8;
    pos += 1;
    buf[pos] = seq;
    pos += 1;

    let dst_pan = req.dst_address.pan_id();
    buf[pos..pos + 2].copy_from_slice(&dst_pan.0.to_le_bytes());
    pos += 2;

    match req.dst_address {
        MacAddress::Short(_, addr) => {
            buf[pos..pos + 2].copy_from_slice(&addr.0.to_le_bytes());
            pos += 2;
        }
        MacAddress::Extended(_, addr) => {
            buf[pos..pos + 8].copy_from_slice(&addr);
            pos += 8;
        }
    }

    if short_addr.0 != 0xFFFF {
        buf[pos..pos + 2].copy_from_slice(&short_addr.0.to_le_bytes());
        pos += 2;
    } else {
        buf[pos..pos + 8].copy_from_slice(extended_addr);
        pos += 8;
    }

    if pos + req.payload.len() > 125 {
        return Err(MacError::FrameTooLong);
    }
    buf[pos..pos + req.payload.len()].copy_from_slice(req.payload);
    pos += req.payload.len();

    Ok(pos)
}
