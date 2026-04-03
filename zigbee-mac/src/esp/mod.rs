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
    /// Buffer for frames received during association (e.g. Transport-Key).
    pending_assoc_frame: Option<([u8; 128], usize)>,
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
            pending_assoc_frame: None,
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

    /// Transmit a frame. The driver waits for TX completion internally.
    async fn transmit_frame(&mut self, frame: &[u8]) -> Result<(), MacError> {
        self.driver
            .transmit(frame)
            .map_err(|_| MacError::RadioError)?;
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
        let mut rx_count = 0u32;
        let mut saw_data_on_channel = false;
        let mut data_pan: u16 = 0;
        let mut data_src: u16 = 0;

        loop {
            if Instant::now() >= deadline {
                break;
            }
            if let Some(Ok(rx)) = self.driver.poll_receive() {
                rx_count += 1;
                let data = &rx.data[..rx.len];
                let fc = if data.len() >= 2 { u16::from_le_bytes([data[0], data[1]]) } else { 0 };
                let ftype = fc & 0x07;
                // Log first few frames on each channel for debugging
                if rx_count <= 3 {
                    log::info!("[SCAN] ch{}: #{} fc=0x{:04X} type={} len={}",
                        channel, rx_count, fc, ftype, rx.len);
                }
                if let Some(pd) = parse_beacon(channel, data, rx.lqi) {
                    log::info!("[SCAN] ch{}: BEACON PAN=0x{:04X}",
                        channel, pd.coord_address.pan_id().0);
                    if descriptors.push(pd).is_err() {
                        break;
                    }
                } else if ftype == 1 && data.len() >= 7 && !saw_data_on_channel {
                    // Data frame — extract PAN and source address for synthetic beacon
                    let pan = u16::from_le_bytes([data[3], data[4]]);
                    let src = u16::from_le_bytes([data[5], data[6]]);
                    if pan != 0xFFFF {
                        saw_data_on_channel = true;
                        data_pan = pan;
                        data_src = src;
                    }
                }
                self.driver.start_receive();
            } else {
                Timer::after_micros(200).await;
            }
        }

        // Fallback: if we received data frames but no beacons, synthesize a
        // PanDescriptor. EZSP coordinators may not send standard beacons but
        // the presence of data traffic proves the network exists.
        if descriptors.is_empty() && saw_data_on_channel {
            log::info!("[SCAN] ch{}: no beacons but {} data frames, synth PAN=0x{:04X} src=0x{:04X}",
                channel, rx_count, data_pan, data_src);
            let _ = descriptors.push(PanDescriptor {
                coord_address: MacAddress::Short(PanId(data_pan), ShortAddress(data_src)),
                channel,
                superframe_spec: SuperframeSpec::from_raw(0x8FFF), // permit joining
                lqi: 128,
                security_use: false,
                zigbee_beacon: ZigbeeBeaconPayload {
                    protocol_id: 0,
                    stack_profile: 2,
                    protocol_version: 2,
                    router_capacity: true,
                    device_depth: 0,
                    end_device_capacity: true,
                    extended_pan_id: [0u8; 8],
                    tx_offset: [0xFF, 0xFF, 0xFF],
                    update_id: 0,
                },
            });
        } else if rx_count == 0 {
            log::info!("[SCAN] ch{}: no frames in {}us", channel, duration_us);
        }
    }

    /// Wait for association response (MAC command 0x02) with timeout.
    async fn wait_assoc_response(
        &mut self,
        timeout_ms: u64,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        let deadline = Instant::now() + Duration::from_millis(timeout_ms);
        self.driver.start_receive();
        let mut rx_count = 0u32;

        loop {
            if Instant::now() >= deadline {
                log::info!("[ESP ASSOC-WAIT] timeout, {} frames seen", rx_count);
                return Err(MacError::NoAck);
            }
            if let Some(Ok(rx)) = self.driver.poll_receive() {
                rx_count += 1;
                let data = &rx.data[..rx.len];
                if data.len() < 5 {
                    self.driver.start_receive();
                    continue;
                }
                let fc = u16::from_le_bytes([data[0], data[1]]);
                let ftype = fc & 0x07;
                log::info!("[ESP ASSOC-WAIT] #{} fc=0x{:04X} type={} len={}", rx_count, fc, ftype, rx.len);
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

        // Enable promiscuous mode for scanning (accept beacon responses)
        self.driver.update_config(|cfg| {
            cfg.promiscuous = true;
        });

        for ch in req.channel_mask.iter() {
            let ch_num = ch.number();
            log::info!("[SCAN] ch {}", ch_num);
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

        // Restore normal mode after scan
        self.driver.update_config(|cfg| {
            cfg.promiscuous = false;
        });

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
            cfg.promiscuous = true; // Accept all frames during association
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
        log::info!("[ESP MLME-ASSOC] Sending assoc req ({} bytes)", frame.len());
        match self.transmit_frame(&frame).await {
            Ok(()) => log::info!("[ESP MLME-ASSOC] TX OK"),
            Err(e) => {
                log::warn!("[ESP MLME-ASSOC] TX failed: {:?}", e);
                return Err(e);
            }
        }

        // Poll for Association Response with retries
        Timer::after_millis(200).await;

        let mut confirm: Option<MlmeAssociateConfirm> = None;

        for poll_attempt in 0..5u8 {
            if poll_attempt > 0 {
                Timer::after_millis(500).await;
            }

            // Send Data Request to poll for indirect response
            let data_req = build_data_request(
                self.next_dsn(),
                &req.coord_address,
                &self.extended_address,
            );
            let _ = self.transmit_frame(&data_req).await;

            // Wait for response
            match self.wait_assoc_response(1500).await {
                Ok(c) => {
                    confirm = Some(c);
                    break;
                }
                Err(_) => continue,
            }
        }

        // After getting assoc response, listen for post-assoc frames (Transport-Key)
        if confirm.is_some() && self.pending_assoc_frame.is_none() {
            let deadline = Instant::now() + Duration::from_millis(2000);
            self.driver.start_receive();

            while Instant::now() < deadline {
                if let Some(Ok(rx)) = self.driver.poll_receive() {
                    let data = &rx.data[..rx.len];
                    if data.len() >= 3 {
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        let frame_type = fc & 0x07;

                        // Send ACK if requested
                        if (fc >> 5) & 1 != 0 {
                            let ack = [0x02u8, 0x00, data[2]];
                            let _ = self.driver.transmit(&ack);
                        }

                        // Save data frames (likely Transport-Key)
                        if frame_type == 0x01 {
                            let header_len = 3 + addressing_size(fc);
                            if data.len() > header_len {
                                let payload = &data[header_len..];
                                let mut buf = [0u8; 128];
                                let copy_len = payload.len().min(128);
                                buf[..copy_len].copy_from_slice(&payload[..copy_len]);
                                self.pending_assoc_frame = Some((buf, copy_len));
                                log::info!("[ESP] Saved post-assoc frame ({} bytes)", copy_len);
                                break;
                            }
                        }
                    }
                    self.driver.start_receive();
                } else {
                    Timer::after_micros(200).await;
                }
            }
        }

        // Restore normal mode
        self.driver.update_config(|cfg| cfg.promiscuous = false);
        
        confirm.ok_or(MacError::NoBeacon)
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
            (PibAttribute::PhyTransmitPower, PibValue::I8(v)) => {
                self.tx_power = *v;
                // TX power is applied through driver config if the radio supports it
                log::debug!("[ESP] TX power set to {} dBm", *v);
            }
            (PibAttribute::MacCoordShortAddress, PibValue::ShortAddress(v)) => {
                self.coord_short_address = *v;
            }
            _ => return Err(MacError::Unsupported),
        }
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Return any frame saved during association first (e.g. Transport-Key)
        if let Some((buf, len)) = self.pending_assoc_frame.take() {
            log::info!("[ESP:Poll] Returning saved assoc frame ({} bytes)", len);
            return Ok(MacFrame::from_slice(&buf[..len]));
        }

        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let has_short = self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE;

        // 2 passes: SHORT address first, then IEEE address
        let passes: u8 = if has_short { 2 } else { 1 };

        for pass in 0..passes {
            let data_req = if pass == 0 && has_short {
                build_data_request_short(self.next_dsn(), &parent, self.short_address)
            } else {
                build_data_request(self.next_dsn(), &parent, &self.extended_address)
            };

            if self.transmit_frame(&data_req).await.is_err() {
                continue;
            }

            self.driver.start_receive();
            let deadline = Instant::now() + Duration::from_millis(1500);

            let mut got_none = false;
            for _rx_attempt in 0..40u8 {
                if Instant::now() >= deadline {
                    break;
                }

                if let Some(result) = self.driver.poll_receive() {
                    let received = match result {
                        Ok(r) => r,
                        Err(_) => { self.driver.start_receive(); continue; }
                    };
                    if received.len < 3 {
                        self.driver.start_receive();
                        continue;
                    }
                    let data = &received.data[..received.len];
                    let fc = u16::from_le_bytes([data[0], data[1]]);
                    let frame_type = fc & 0x07;

                    // ACK — check frame_pending bit
                    if frame_type == 0x02 {
                        let frame_pending = (data[0] >> 4) & 1 != 0;
                        if !frame_pending {
                            got_none = true;
                            break;
                        }
                        self.driver.start_receive();
                        continue;
                    }

                    // Only accept data frames
                    if frame_type != 0x01 {
                        self.driver.start_receive();
                        continue;
                    }

                    // Verify destination matches us or broadcast
                    if let Some(dst) = parse_dest_address(data, fc) {
                        let for_us = match &dst {
                            MacAddress::Short(_, d) => {
                                d.0 == self.short_address.0
                                    || d.0 == 0xFFFF
                                    || d.0 == 0xFFFD
                                    || d.0 == 0xFFFC
                            }
                            MacAddress::Extended(_, e) => *e == self.extended_address,
                        };
                        if !for_us {
                            self.driver.start_receive();
                            continue;
                        }
                    }

                    let header_len = 3 + addressing_size(fc);
                    if data.len() <= header_len {
                        self.driver.start_receive();
                        continue;
                    }

                    return Ok(MacFrame::from_slice(&data[header_len..]));
                } else {
                    Timer::after_micros(200).await;
                }
            }

            if got_none {
                return Ok(None);
            }
        }

        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let msdu_handle = req.msdu_handle;
        let ack_requested = req.tx_options.ack_tx;
        let mut frame_buf = [0u8; 127];
        let seq = self.next_dsn();
        let len = build_data_frame(
            &mut frame_buf,
            seq,
            self.short_address,
            self.pan_id,
            &self.extended_address,
            &req,
        )?;

        let max_retries = if ack_requested { self.max_frame_retries } else { 0 };

        for attempt in 0..=max_retries {
            // Unslotted CSMA-CA
            let mut be = self.min_be;
            let mut nb: u8 = 0;
            let symbol_period_us: u64 = 16;
            let unit_backoff_symbols: u64 = 20;

            let channel_clear = loop {
                let max_val = (1u32 << be) - 1;
                let random = (seq as u32)
                    .wrapping_add(nb as u32)
                    .wrapping_add(attempt as u32)
                    .wrapping_mul(1103515245)
                    .wrapping_add(12345);
                let backoff = (random % (max_val + 1)) as u64;
                let delay_us = backoff * unit_backoff_symbols * symbol_period_us;
                if delay_us > 0 {
                    Timer::after_micros(delay_us).await;
                }

                // Try transmit (CCA implicit)
                match self.driver.transmit(&frame_buf[..len]) {
                    Ok(()) => break true,
                    Err(_) => {
                        nb += 1;
                        be = core::cmp::min(be + 1, self.max_be);
                        if nb > self.max_csma_backoffs {
                            break false;
                        }
                    }
                }
            };

            if !channel_clear {
                if attempt == max_retries {
                    return Err(MacError::ChannelAccessFailure);
                }
                continue;
            }

            if !ack_requested {
                return Ok(McpsDataConfirm { msdu_handle, timestamp: None });
            }

            // Wait for ACK (turnaround + ACK duration ≈ 1.5ms)
            self.driver.start_receive();
            let ack_deadline = Instant::now() + Duration::from_millis(2);
            let mut got_ack = false;

            while Instant::now() < ack_deadline {
                if let Some(Ok(rx)) = self.driver.poll_receive() {
                    let data = &rx.data[..rx.len];
                    if data.len() >= 3 {
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        let frame_type = fc & 0x07;
                        if frame_type == 0x02 && data[2] == seq {
                            got_ack = true;
                            break;
                        }
                    }
                    self.driver.start_receive();
                } else {
                    Timer::after_micros(100).await;
                }
            }

            if got_ack {
                return Ok(McpsDataConfirm { msdu_handle, timestamp: None });
            }

            // No ACK — retry if attempts remain
            if attempt < max_retries {
                log::debug!("[ESP] No ACK seq={}, retry {}/{}", seq, attempt + 1, max_retries);
            }
        }

        Err(MacError::NoAck)
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
                                && (addr.0 == self.short_address.0
                                    || addr.0 == 0xFFFF
                                    || addr.0 == 0xFFFD
                                    || addr.0 == 0xFFFC)
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

                // Send software ACK if requested (FC bit 5)
                if (fc >> 5) & 1 != 0 {
                    let ack_frame = [0x02u8, 0x00, data[2]]; // ACK type, seq
                    let _ = self.driver.transmit(&ack_frame);
                }

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
            coordinator: false,
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

/// Build a Data Request with SHORT source address (for poll pass 1).
fn build_data_request_short(
    seq: u8,
    coord: &MacAddress,
    own_short: ShortAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    // FC: command, ACK, PAN compress, dst=short, src=short
    let _ = frame.extend_from_slice(&[0x63, 0x88, seq]);
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
    let _ = frame.extend_from_slice(&own_short.0.to_le_bytes());
    let _ = frame.push(0x04);
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
