//! ESP32-C6 MAC backend.
//!
//! Wraps `esp-ieee802154` radio driver to implement `MacDriver`.
//! Refactored from the original zigbee-rs `EspMlme` (scan-only) to
//! implement the full MacDriver trait with MLME/MCPS support.
//!
//! # Architecture
//! ```text
//! MacDriver trait methods
//!        │
//!        ▼
//! EspMac (this module)
//!   ├── PIB state (addresses, channel, config)
//!   ├── Frame construction (beacon req, assoc req, data)
//!   └── Ieee802154Driver (driver.rs)
//!          ├── esp-radio Ieee802154 peripheral
//!          ├── TX via Signal (interrupt-driven)
//!          └── RX via Signal (interrupt-driven)
//! ```

mod driver;

use crate::pib::{self, PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use driver::Ieee802154Driver;
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;
use esp_radio::ieee802154::{Config, Ieee802154};
use ieee802154::mac::{self, Frame, FrameContent, Header};

/// ESP32-C6 802.15.4 MAC driver.
///
/// Built on `esp-radio::ieee802154` — uses the ESP32-C6's hardware
/// radio with interrupt-driven TX/RX via Embassy signals.
///
/// # Usage
/// ```rust,no_run
/// use esp_radio::ieee802154::Ieee802154;
/// use zigbee_mac::esp::EspMac;
///
/// let ieee = Ieee802154::new(peripherals.IEEE802154);
/// let mac = EspMac::new(ieee, Default::default());
/// let nlme = Nlme::new(storage, mac);
/// ```
pub struct EspMac<'a> {
    driver: Ieee802154Driver<'a>,
    // PIB state
    short_address: ShortAddress,
    pan_id: PanId,
    channel: u8,
    extended_address: IeeeAddress,
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
        Self {
            driver: Ieee802154Driver::new(ieee802154, config),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: [0u8; 8], // Read from hardware on first GET
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

    fn next_dsn(&mut self) -> u8 {
        let seq = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        seq
    }

    /// Construct an IEEE 802.15.4 Beacon Request MAC command frame.
    fn beacon_request_frame(&mut self) -> [u8; 10] {
        let seq = self.next_dsn();
        // Frame Control: MAC command, no security, no frame pending,
        // no ack request, no PAN ID compression, dst=short, src=none
        [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00]
    }

    /// Construct an IEEE 802.15.4 Association Request MAC command frame.
    fn association_request_frame(
        &mut self,
        coord_address: &MacAddress,
        capability_info: &CapabilityInfo,
    ) -> heapless::Vec<u8, 32> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();

        // Frame Control: MAC command, no security, no pending,
        // ack requested, PAN ID compression, dst=short, src=extended
        let _ = frame.extend_from_slice(&[0x23, 0xC8, seq]);

        // Destination PAN + address
        let dst_pan = coord_address.pan_id();
        let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());
        match coord_address {
            MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }

        // Source extended address (no PAN ID due to compression)
        let _ = frame.extend_from_slice(&self.extended_address);

        // Command ID: Association Request = 0x01
        let _ = frame.push(0x01);
        // Capability info byte
        let _ = frame.push(capability_info.to_byte());

        frame
    }

    /// Scan a single channel for beacons (active scan).
    async fn scan_channel_active(
        &mut self,
        channel: u8,
        duration: u8,
    ) -> Result<heapless::Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>, MacError> {
        self.driver.update_config(|config| {
            config.promiscuous = false;
            config.channel = channel;
        });

        let beacon_req = self.beacon_request_frame();
        if let Err(e) = self.driver.transmit(&beacon_req).await {
            log::error!("[MLME-SCAN] beacon TX error on ch {channel}: {e:?}");
        }

        let delay_us = pib::scan_duration_us(duration);
        let mut descriptors = heapless::Vec::new();

        let timer_fut = Timer::after_micros(delay_us);
        let receive_fut = self.receive_beacons(channel, &mut descriptors);
        let _ = select::select(timer_fut, receive_fut).await;

        Ok(descriptors)
    }

    /// Scan a single channel for energy (ED scan).
    async fn scan_channel_ed(&mut self, channel: u8, duration: u8) -> EdValue {
        self.driver.update_config(|config| {
            config.channel = channel;
        });

        let delay_us = pib::scan_duration_us(duration);
        // TODO: Use hardware ED measurement if available.
        // For now, listen for frames and estimate from LQI.
        Timer::after_micros(delay_us).await;

        EdValue {
            channel,
            energy: 0, // Placeholder — real impl uses radio ED register
        }
    }

    /// Receive beacons on the current channel until interrupted by timer.
    async fn receive_beacons(
        &mut self,
        channel: u8,
        descriptors: &mut heapless::Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>,
    ) -> Result<(), MacError> {
        for _ in 0..MAX_PAN_DESCRIPTORS {
            let received = self
                .driver
                .receive()
                .await
                .map_err(|_| MacError::RadioError)?;

            // Parse the IEEE 802.15.4 frame
            if let Ok(frame) = Frame::try_unpack(&received.data[..received.len], false) {
                if let FrameContent::Beacon(beacon) = &frame.content {
                    // Parse Zigbee beacon payload from the frame payload
                    if let Some(payload) = frame.payload {
                        if payload.len() >= 15 {
                            let zigbee_beacon = parse_zigbee_beacon(payload);
                            let source = frame.header.source.unwrap_or(mac::Address::Short(
                                mac::ShortAddress(0xFFFF),
                                mac::ShortAddress(0xFFFF),
                            ));

                            let coord_address = convert_mac_address(&source);
                            let superframe_spec =
                                SuperframeSpec::from_raw(beacon.superframe_spec.0);

                            let pd = PanDescriptor {
                                channel,
                                coord_address,
                                superframe_spec,
                                lqi: received.lqi,
                                security_use: frame.header.has_security,
                                zigbee_beacon,
                            };

                            if descriptors.push(pd).is_err() {
                                break; // Full
                            }
                        }
                    }
                }
            }
        }
        Ok(())
    }
}

// ── MacDriver implementation ────────────────────────────────────

impl MacDriver for EspMac<'_> {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let mut pan_descriptors = heapless::Vec::new();
        let mut energy_list = heapless::Vec::new();

        for channel in req.channel_mask.iter() {
            let ch = channel.number();
            match req.scan_type {
                ScanType::Active | ScanType::Passive => {
                    match self.scan_channel_active(ch, req.scan_duration).await {
                        Ok(pds) => {
                            for pd in pds {
                                let _ = pan_descriptors.push(pd);
                            }
                        }
                        Err(e) => log::error!("[MLME-SCAN] ch {ch} error: {e:?}"),
                    }
                }
                ScanType::Ed => {
                    let ed = self.scan_channel_ed(ch, req.scan_duration).await;
                    let _ = energy_list.push(ed);
                }
                ScanType::Orphan => {
                    // Orphan scan: send Orphan Notification, listen for Coordinator Realignment
                    // TODO: implement orphan scan
                    log::warn!("[MLME-SCAN] Orphan scan not yet implemented");
                }
            }
        }

        if matches!(req.scan_type, ScanType::Active | ScanType::Passive)
            && pan_descriptors.is_empty()
        {
            return Err(MacError::NoBeacon);
        }

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
        // Switch to the association channel
        self.driver.update_config(|config| {
            config.channel = req.channel;
        });
        self.channel = req.channel;

        // Build and send Association Request command frame
        let frame = self.association_request_frame(&req.coord_address, &req.capability_info);
        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| MacError::RadioError)?;

        // Wait for Association Response (with timeout)
        // macResponseWaitTime * aBaseSuperframeDuration symbols
        let timeout_us = (pib::A_BASE_SUPERFRAME_DURATION as u64) * 32 * 1_000_000
            / pib::SYMBOL_RATE_2_4GHZ as u64;

        let result = select::select(
            Timer::after_micros(timeout_us),
            self.wait_for_assoc_response(),
        )
        .await;

        match result {
            select::Either::Second(Ok(confirm)) => Ok(confirm),
            select::Either::Second(Err(e)) => Err(e),
            select::Either::First(_) => Err(MacError::NoAck), // Timeout
        }
    }

    async fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        // TODO: implement for coordinator/router role
        // Build Association Response command frame and transmit (indirect for sleepy)
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(&mut self, _req: MlmeDisassociateRequest) -> Result<(), MacError> {
        // TODO: Send Disassociation Notification command
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
            self.max_csma_backoffs = 4;
            self.min_be = 3;
            self.max_be = 5;
            self.max_frame_retries = 3;
            self.promiscuous = false;
        }
        // Reset radio to idle state
        self.driver.update_config(|config| {
            config.channel = self.channel;
            config.promiscuous = self.promiscuous;
        });
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.channel;

        self.driver.update_config(|config| {
            config.channel = req.channel;
        });

        // TODO: Start beacon transmission for coordinator role
        // In non-beacon mode (Zigbee), this mainly configures the radio
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        match attr {
            PibAttribute::MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            PibAttribute::MacPanId => Ok(PibValue::PanId(self.pan_id)),
            PibAttribute::MacExtendedAddress => {
                Ok(PibValue::ExtendedAddress(self.extended_address))
            }
            PibAttribute::MacCoordShortAddress => Ok(PibValue::ShortAddress(ShortAddress(0x0000))),
            PibAttribute::MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            PibAttribute::MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            PibAttribute::MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            PibAttribute::MacBeaconOrder => Ok(PibValue::U8(15)), // Non-beacon mode
            PibAttribute::MacSuperframeOrder => Ok(PibValue::U8(15)),
            PibAttribute::MacDsn => Ok(PibValue::U8(self.dsn)),
            PibAttribute::MacBsn => Ok(PibValue::U8(self.bsn)),
            PibAttribute::MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
            PibAttribute::MacMinBe => Ok(PibValue::U8(self.min_be)),
            PibAttribute::MacMaxBe => Ok(PibValue::U8(self.max_be)),
            PibAttribute::MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
            PibAttribute::MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PibAttribute::PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PibAttribute::PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
            PibAttribute::PhyChannelsSupported => Ok(PibValue::U32(ChannelMask::ALL_2_4GHZ.0)),
            PibAttribute::PhyCurrentPage => Ok(PibValue::U8(0)), // 2.4 GHz
            PibAttribute::MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            _ => Ok(PibValue::U8(0)),
        }
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        match attr {
            PibAttribute::MacShortAddress => {
                self.short_address = value.as_short_address().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacPanId => {
                self.pan_id = value.as_pan_id().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacRxOnWhenIdle => {
                self.rx_on_when_idle = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacAssociationPermit => {
                self.association_permit = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacAutoRequest => {
                self.auto_request = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::PhyCurrentChannel => {
                let ch = value.as_u8().ok_or(MacError::InvalidParameter)?;
                if !(11..=26).contains(&ch) {
                    return Err(MacError::InvalidParameter);
                }
                self.channel = ch;
                self.driver.update_config(|config| {
                    config.channel = ch;
                });
            }
            PibAttribute::MacPromiscuousMode => {
                self.promiscuous = value.as_bool().ok_or(MacError::InvalidParameter)?;
                self.driver.update_config(|config| {
                    config.promiscuous = self.promiscuous;
                });
            }
            PibAttribute::MacMaxCsmaBackoffs => {
                self.max_csma_backoffs = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacMinBe => {
                self.min_be = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacMaxBe => {
                self.max_be = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacMaxFrameRetries => {
                self.max_frame_retries = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacBeaconPayload => {
                if let PibValue::Payload(p) = value {
                    self.beacon_payload = p;
                }
            }
            PibAttribute::PhyTransmitPower => {
                if let PibValue::I8(p) = value {
                    self.tx_power = p;
                }
            }
            _ => {}
        }
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Send Data Request command to coordinator, wait for response
        // TODO: construct Data Request command frame
        // For now, just check if there's a pending frame in RX
        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        // Build IEEE 802.15.4 data frame
        let mut frame_buf = [0u8; 127];
        let len = self.build_data_frame(&mut frame_buf, req)?;

        // Transmit
        self.driver
            .transmit(&frame_buf[..len])
            .await
            .map_err(|_| MacError::RadioError)?;

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        loop {
            let received = self
                .driver
                .receive()
                .await
                .map_err(|_| MacError::RadioError)?;

            // Parse IEEE 802.15.4 frame
            if let Ok(frame) = Frame::try_unpack(&received.data[..received.len], false) {
                if let FrameContent::Data = &frame.content {
                    let src = frame
                        .header
                        .source
                        .map(|a| convert_mac_address(&a))
                        .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));
                    let dst = frame
                        .header
                        .destination
                        .map(|a| convert_mac_address(&a))
                        .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));

                    if let Some(payload) = frame.payload {
                        if let Some(mac_frame) = MacFrame::from_slice(payload) {
                            return Ok(McpsDataIndication {
                                src_address: src,
                                dst_address: dst,
                                lqi: received.lqi,
                                payload: mac_frame,
                                security_use: frame.header.has_security,
                            });
                        }
                    }
                }
            }
            // Not a data frame — keep listening
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: true,
            router: true,
            hardware_security: false, // ESP32-C6 does AES in HW but not 802.15.4 security
            max_payload: 102,
            tx_power_min: TxPower(-24),
            tx_power_max: TxPower(20),
        }
    }
}

// ── Private helpers ─────────────────────────────────────────────

impl EspMac<'_> {
    /// Wait for an Association Response command frame.
    async fn wait_for_assoc_response(&mut self) -> Result<MlmeAssociateConfirm, MacError> {
        for _ in 0..10 {
            let received = self
                .driver
                .receive()
                .await
                .map_err(|_| MacError::RadioError)?;

            if let Ok(frame) = Frame::try_unpack(&received.data[..received.len], false) {
                if let FrameContent::Command(cmd) = &frame.content {
                    // Association Response command ID = 0x02
                    if cmd.id == mac::command::Id::AssociationResponse {
                        if let Some(payload) = frame.payload {
                            if payload.len() >= 3 {
                                let short_addr = u16::from_le_bytes([payload[0], payload[1]]);
                                let status = match payload[2] {
                                    0x00 => AssociationStatus::Success,
                                    0x01 => AssociationStatus::PanAtCapacity,
                                    _ => AssociationStatus::PanAccessDenied,
                                };

                                if status == AssociationStatus::Success {
                                    self.short_address = ShortAddress(short_addr);
                                    self.pan_id = frame
                                        .header
                                        .source
                                        .map(|a| match a {
                                            mac::Address::Short(pan, _) => PanId(pan.0),
                                            mac::Address::Extended(pan, _) => PanId(pan.0),
                                        })
                                        .unwrap_or(self.pan_id);
                                }

                                return Ok(MlmeAssociateConfirm {
                                    short_address: ShortAddress(short_addr),
                                    status,
                                });
                            }
                        }
                    }
                }
            }
        }
        Err(MacError::NoAck)
    }

    /// Build an IEEE 802.15.4 data frame into the provided buffer.
    fn build_data_frame(
        &mut self,
        buf: &mut [u8; 127],
        req: McpsDataRequest<'_>,
    ) -> Result<usize, MacError> {
        let seq = self.next_dsn();

        // Frame Control (2 bytes)
        let mut fc: u16 = 0x0001; // Data frame
        if req.tx_options.ack_tx {
            fc |= 0x0020; // Ack request
        }
        // PAN ID compression (intra-PAN)
        fc |= 0x0040;
        // Destination addressing mode
        match req.dst_address {
            MacAddress::Short(_, _) => fc |= 0x0800,    // Short
            MacAddress::Extended(_, _) => fc |= 0x0C00, // Extended
        }
        // Source addressing mode (use short if we have one, else extended)
        if self.short_address.0 != 0xFFFF {
            fc |= 0x8000; // Short source
        } else {
            fc |= 0xC000; // Extended source
        }

        let mut pos = 0;

        // Frame Control
        buf[pos] = (fc & 0xFF) as u8;
        pos += 1;
        buf[pos] = ((fc >> 8) & 0xFF) as u8;
        pos += 1;

        // Sequence Number
        buf[pos] = seq;
        pos += 1;

        // Destination PAN ID
        let dst_pan = req.dst_address.pan_id();
        buf[pos..pos + 2].copy_from_slice(&dst_pan.0.to_le_bytes());
        pos += 2;

        // Destination address
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

        // Source address (no PAN ID — compressed)
        if self.short_address.0 != 0xFFFF {
            buf[pos..pos + 2].copy_from_slice(&self.short_address.0.to_le_bytes());
            pos += 2;
        } else {
            buf[pos..pos + 8].copy_from_slice(&self.extended_address);
            pos += 8;
        }

        // Payload
        let payload_len = req.payload.len();
        if pos + payload_len > 125 {
            // 127 - 2 for CRC
            return Err(MacError::FrameTooLong);
        }
        buf[pos..pos + payload_len].copy_from_slice(req.payload);
        pos += payload_len;

        Ok(pos)
    }
}

// ── Utility functions ───────────────────────────────────────────

/// Convert ieee802154 crate Address to our MacAddress type.
fn convert_mac_address(addr: &mac::Address) -> MacAddress {
    match addr {
        mac::Address::Short(pan, short) => MacAddress::Short(PanId(pan.0), ShortAddress(short.0)),
        mac::Address::Extended(pan, ext) => MacAddress::Extended(PanId(pan.0), ext.0.to_le_bytes()),
    }
}

/// Parse Zigbee beacon payload from raw bytes.
fn parse_zigbee_beacon(data: &[u8]) -> ZigbeeBeaconPayload {
    let protocol_id = data[0];
    let nwk_info = u16::from_le_bytes([data[1], data[2]]);
    let stack_profile = (nwk_info & 0x0F) as u8;
    let protocol_version = ((nwk_info >> 4) & 0x0F) as u8;
    let router_capacity = (nwk_info >> 10) & 1 != 0;
    let device_depth = ((nwk_info >> 11) & 0x0F) as u8;
    let end_device_capacity = (nwk_info >> 15) & 1 != 0;

    let mut extended_pan_id = [0u8; 8];
    extended_pan_id.copy_from_slice(&data[3..11]);

    let mut tx_offset = [0u8; 3];
    tx_offset.copy_from_slice(&data[11..14]);

    let update_id = data[14];

    ZigbeeBeaconPayload {
        protocol_id,
        stack_profile,
        protocol_version,
        router_capacity,
        device_depth,
        end_device_capacity,
        extended_pan_id,
        tx_offset,
        update_id,
    }
}
