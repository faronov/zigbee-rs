//! CC2340 MAC backend.
//!
//! Implements `MacDriver` for the Texas Instruments CC2340R5 ARM Cortex-M0+ SoC.
//! The CC2340R5 has a dedicated 2.4 GHz radio supporting IEEE 802.15.4, controlled
//! through TI's Radio Control Layer (RCL) library.
//!
//! This backend uses FFI bindings to TI's precompiled RCL and MAC platform
//! libraries for radio access, with async TX/RX through Embassy signals.
//!
//! # Architecture
//! ```text
//! MacDriver trait methods
//!        │
//!        ▼
//! Cc2340Mac (this module)
//!   ├── PIB state (addresses, channel, config)
//!   ├── Frame construction (beacon req, assoc req, data)
//!   └── Cc2340Driver (driver.rs)
//!          ├── FFI → rcl_cc23x0r5.a + RF patches (TI precompiled)
//!          ├── FFI → mac_ti23xx_* platform shim functions
//!          ├── TX via Signal (interrupt-driven)
//!          └── RX via Signal (interrupt-driven)
//! ```

pub mod driver;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError, PlatformServices};
use driver::Cc2340Driver;
pub use driver::RadioConfig;
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::{Instant, Timer};

/// CC2340 802.15.4 MAC driver.
pub struct Cc2340Mac {
    driver: Cc2340Driver,
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

impl Cc2340Mac {
    /// Create a new CC2340 MAC driver with default PIB values.
    pub fn new() -> Self {
        Self {
            driver: Cc2340Driver::new(RadioConfig::default()),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: [0u8; 8],
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
            tx_power: 5,
        }
    }

    fn next_dsn(&mut self) -> u8 {
        let seq = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
        seq
    }

    fn next_bsn(&mut self) -> u8 {
        let seq = self.bsn;
        self.bsn = self.bsn.wrapping_add(1);
        seq
    }

    /// Construct an IEEE 802.15.4 Beacon Request MAC command frame.
    fn beacon_request_frame(&mut self) -> [u8; 8] {
        let seq = self.next_dsn();
        [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07]
    }

    /// Construct an IEEE 802.15.4 Association Request MAC command frame.
    fn association_request_frame(
        &mut self,
        coord_address: &MacAddress,
        capability_info: &CapabilityInfo,
    ) -> heapless::Vec<u8, 32> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();
        let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);
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

        let _ = frame.extend_from_slice(&self.extended_address);
        let _ = frame.push(0x01);
        let _ = frame.push(capability_info.to_byte());
        frame
    }

    /// Build a MAC Data frame.
    fn build_data_frame(
        &mut self,
        dst_address: &MacAddress,
        payload: &[u8],
        ack_request: bool,
    ) -> heapless::Vec<u8, 127> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();

        let mut fc: u16 = 0x0001; // Data frame
        if ack_request {
            fc |= 0x0020;
        }
        fc |= 0x0040; // PAN ID compression

        match dst_address {
            MacAddress::Short(_, _) => fc |= 0x0800,
            MacAddress::Extended(_, _) => fc |= 0x0C00,
        }

        if self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE {
            fc |= 0x8000; // src=short
        } else {
            fc |= 0xC000; // src=extended
        }

        let _ = frame.extend_from_slice(&fc.to_le_bytes());
        let _ = frame.push(seq);

        let dst_pan = dst_address.pan_id();
        let _ = frame.extend_from_slice(&dst_pan.0.to_le_bytes());

        match dst_address {
            MacAddress::Short(_, addr) => {
                let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
            }
            MacAddress::Extended(_, addr) => {
                let _ = frame.extend_from_slice(addr);
            }
        }

        if self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE {
            let _ = frame.extend_from_slice(&self.short_address.0.to_le_bytes());
        } else {
            let _ = frame.extend_from_slice(&self.extended_address);
        }

        let _ = frame.extend_from_slice(payload);
        frame
    }

    /// Scan a single channel for beacons (active scan).
    async fn scan_channel_active(
        &mut self,
        channel: u8,
        duration_ms: u32,
    ) -> heapless::Vec<PanDescriptor, 8> {
        let mut results = heapless::Vec::new();
        self.driver.update_config(|cfg| cfg.channel = channel);

        let beacon_req = self.beacon_request_frame();
        if self.driver.transmit(&beacon_req).await.is_err() {
            return results;
        }

        let deadline = Timer::after(embassy_time::Duration::from_millis(duration_ms as u64));
        let collect = async {
            loop {
                match self.driver.receive().await {
                    Ok(rx_frame) => {
                        if let Some(desc) = Self::parse_beacon(
                            &rx_frame.data[..rx_frame.len],
                            rx_frame.lqi,
                            channel,
                        ) {
                            let _ = results.push(desc);
                        }
                    }
                    Err(_) => break,
                }
            }
        };
        let _ = select::select(deadline, collect).await;
        results
    }

    /// Scan a single channel for energy (ED scan).
    async fn scan_channel_ed(&mut self, channel: u8) -> u8 {
        self.driver.update_config(|cfg| cfg.channel = channel);
        let mut max_energy: i8 = -128;

        let deadline = Timer::after(embassy_time::Duration::from_millis(100));
        let measure = async {
            loop {
                let rssi = self.driver.read_rssi();
                if rssi > max_energy {
                    max_energy = rssi;
                }
                Timer::after_millis(1).await;
            }
        };
        let _ = select::select(deadline, measure).await;

        ((max_energy as i16 + 128) * 255 / 256) as u8
    }

    /// Parse a received beacon frame into a PAN descriptor.
    fn parse_beacon(frame_data: &[u8], lqi: u8, channel: u8) -> Option<PanDescriptor> {
        if frame_data.len() < 5 {
            return None;
        }
        let fc = u16::from_le_bytes([frame_data[0], frame_data[1]]);
        let frame_type = fc & 0x07;
        if frame_type != 0 {
            return None;
        }

        let superframe_offset = 3 + addressing_size(fc);
        if frame_data.len() < superframe_offset + 2 {
            return None;
        }
        let sf_raw = u16::from_le_bytes([
            frame_data[superframe_offset],
            frame_data[superframe_offset + 1],
        ]);
        let superframe_spec = SuperframeSpec::from_raw(sf_raw);

        // Zigbee beacon payload follows superframe + GTS(1) + pending(1)
        let beacon_payload_offset = superframe_offset + 4;
        if frame_data.len() < beacon_payload_offset + 15 {
            return None;
        }
        let zigbee_beacon = parse_zigbee_beacon(&frame_data[beacon_payload_offset..]);
        let coord_address = parse_source_address(frame_data, fc)?;

        Some(PanDescriptor {
            channel,
            coord_address,
            superframe_spec,
            lqi,
            security_use: (fc >> 3) & 1 != 0,
            zigbee_beacon,
        })
    }

    /// Synchronize the driver's radio config with our PIB state.
    fn sync_radio_config(&mut self) {
        self.driver.update_config(|cfg| {
            cfg.channel = self.channel;
            cfg.pan_id = self.pan_id.0;
            cfg.short_addr = self.short_address.0;
            cfg.ieee_addr = self.extended_address;
            cfg.tx_power_dbm = self.tx_power;
            cfg.rx_on_when_idle = self.rx_on_when_idle;
            cfg.promiscuous = self.promiscuous;
        });
    }
}

/// Build a Data Request MAC command frame.
fn build_data_request(seq: u8, coord: &MacAddress, src_ext: &[u8; 8]) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    let _ = frame.extend_from_slice(&[0x63, 0xC8, seq]);
    let pan = coord.pan_id();
    let _ = frame.extend_from_slice(&pan.0.to_le_bytes());
    match coord {
        MacAddress::Short(_, addr) => {
            let _ = frame.extend_from_slice(&addr.0.to_le_bytes());
        }
        MacAddress::Extended(_, addr) => {
            let _ = frame.extend_from_slice(addr);
        }
    }
    let _ = frame.extend_from_slice(src_ext);
    let _ = frame.push(0x04);
    frame
}

// ── Free-standing frame parsing functions ───────────────────────

/// Compute addressing field size from Frame Control word.
fn addressing_size(fc: u16) -> usize {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;
    let mut size = 0usize;

    match dst_mode {
        2 => size += 2 + 2,
        3 => size += 2 + 8,
        _ => {}
    }
    match src_mode {
        2 => {
            if !pan_compress {
                size += 2;
            }
            size += 2;
        }
        3 => {
            if !pan_compress {
                size += 2;
            }
            size += 8;
        }
        _ => {}
    }
    size
}

/// Parse source address from raw MAC frame.
fn parse_source_address(data: &[u8], fc: u16) -> Option<MacAddress> {
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    let pan_compress = (fc >> 6) & 1 != 0;

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
    let offset = 3;

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

/// Parse Zigbee beacon payload from raw bytes.
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

// ── MacDriver trait implementation ──────────────────────────────

impl MacDriver for Cc2340Mac {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        self.driver.init();
        let scan_duration_ms = ((1u32 << req.scan_duration) + 1) * 15;

        match req.scan_type {
            ScanType::Active | ScanType::Passive => {
                let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if let Some(channel) = Channel::from_number(ch) {
                        if req.channel_mask.contains(channel) {
                            let beacons = self.scan_channel_active(ch, scan_duration_ms).await;
                            for desc in beacons {
                                let _ = pan_descriptors.push(desc);
                            }
                        }
                    }
                }

                self.sync_radio_config();

                if pan_descriptors.is_empty() {
                    Err(MacError::NoBeacon)
                } else {
                    Ok(MlmeScanConfirm {
                        scan_type: req.scan_type,
                        pan_descriptors,
                        energy_list: heapless::Vec::new(),
                    })
                }
            }
            ScanType::Ed => {
                let mut energy_list: EdList = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if let Some(channel) = Channel::from_number(ch) {
                        if req.channel_mask.contains(channel) {
                            let energy = self.scan_channel_ed(ch).await;
                            let _ = energy_list.push(EdValue {
                                channel: ch,
                                energy,
                            });
                        }
                    }
                }

                self.sync_radio_config();

                Ok(MlmeScanConfirm {
                    scan_type: req.scan_type,
                    pan_descriptors: heapless::Vec::new(),
                    energy_list,
                })
            }
            ScanType::Orphan => Err(MacError::Unsupported),
        }
    }

    async fn mlme_associate(
        &mut self,
        req: MlmeAssociateRequest,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        self.channel = req.channel;
        self.pan_id = req.coord_address.pan_id();
        self.driver.update_config(|cfg| {
            cfg.channel = req.channel;
            cfg.pan_id = req.coord_address.pan_id().0;
        });

        log::info!(
            "[CC2340 MLME-ASSOC] ch {} coord {:?}",
            req.channel,
            req.coord_address
        );

        let frame = self.association_request_frame(&req.coord_address, &req.capability_info);
        self.driver
            .transmit(&frame)
            .await
            .map_err(|_| MacError::RadioError)?;

        Timer::after_millis(100).await;

        let data_req =
            build_data_request(self.next_dsn(), &req.coord_address, &self.extended_address);
        let _ = self.driver.transmit(&data_req).await;

        let timeout = Timer::after(embassy_time::Duration::from_millis(3000));
        let wait_response = async {
            for _ in 0..10 {
                match self.driver.receive().await {
                    Ok(rx_frame) => {
                        let data = &rx_frame.data[..rx_frame.len];
                        if data.len() < 5 {
                            continue;
                        }
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        if fc & 0x07 != 3 {
                            continue;
                        }
                        let cmd_offset = 3 + addressing_size(fc);
                        if data.len() < cmd_offset + 4 {
                            continue;
                        }
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
                                self.sync_radio_config();
                            }
                            return Ok(MlmeAssociateConfirm {
                                short_address: ShortAddress(short_addr),
                                status,
                            });
                        }
                    }
                    Err(_) => return Err(MacError::RadioError),
                }
            }
            Err(MacError::NoAck)
        };

        match select::select(timeout, wait_response).await {
            select::Either::First(_) => Err(MacError::NoAck),
            select::Either::Second(result) => result,
        }
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
        self.coord_short_address = ShortAddress(0x0000);
        self.sync_radio_config();
        log::info!("[CC2340] Disassociated");
        Ok(())
    }

    async fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        self.driver.init();
        if set_default_pib {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.channel = 11;
            self.coord_short_address = ShortAddress(0x0000);
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
            self.tx_power = 5;
        }
        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.channel = req.channel;

        self.sync_radio_config();
        log::info!(
            "[CC2340] PAN started: 0x{:04X} ch={}",
            req.pan_id.0,
            req.channel
        );
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        use PibAttribute::*;

        match attr {
            MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            MacPanId => Ok(PibValue::PanId(self.pan_id)),
            MacExtendedAddress => Ok(PibValue::ExtendedAddress(self.extended_address)),
            MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            MacDsn => Ok(PibValue::U8(self.dsn)),
            MacBsn => Ok(PibValue::U8(self.bsn)),
            MacMaxCsmaBackoffs => Ok(PibValue::U8(self.max_csma_backoffs)),
            MacMinBe => Ok(PibValue::U8(self.min_be)),
            MacMaxBe => Ok(PibValue::U8(self.max_be)),
            MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
            MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PhyTransmitPower => Ok(PibValue::U8(self.tx_power as u8)),
            PhyChannelsSupported => Ok(PibValue::U32(zigbee_types::ChannelMask::ALL_2_4GHZ.0)),
            PhyCurrentPage => Ok(PibValue::U8(0)),
            MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            _ => Err(MacError::Unsupported),
        }
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        use PibAttribute::*;

        match (attr, value) {
            (MacShortAddress, PibValue::ShortAddress(v)) => {
                self.short_address = v;
                self.sync_radio_config();
            }
            (MacPanId, PibValue::PanId(v)) => {
                self.pan_id = v;
                self.sync_radio_config();
            }
            (MacExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.extended_address = v;
                self.sync_radio_config();
            }
            (MacRxOnWhenIdle, PibValue::Bool(v)) => self.rx_on_when_idle = v,
            (MacAssociationPermit, PibValue::Bool(v)) => self.association_permit = v,
            (MacAutoRequest, PibValue::Bool(v)) => self.auto_request = v,
            (MacDsn, PibValue::U8(v)) => self.dsn = v,
            (MacBsn, PibValue::U8(v)) => self.bsn = v,
            (MacMaxCsmaBackoffs, PibValue::U8(v)) => self.max_csma_backoffs = v,
            (MacMinBe, PibValue::U8(v)) => self.min_be = v,
            (MacMaxBe, PibValue::U8(v)) => self.max_be = v,
            (MacMaxFrameRetries, PibValue::U8(v)) => self.max_frame_retries = v,
            (MacPromiscuousMode, PibValue::Bool(v)) => {
                self.promiscuous = v;
                self.sync_radio_config();
            }
            (PhyCurrentChannel, PibValue::U8(v)) => {
                self.channel = v;
                self.sync_radio_config();
            }
            (PhyTransmitPower, PibValue::U8(v)) => {
                self.tx_power = v as i8;
                self.sync_radio_config();
            }
            (MacBeaconPayload, PibValue::Payload(v)) => self.beacon_payload = v,
            (MacCoordShortAddress, PibValue::ShortAddress(v)) => {
                self.coord_short_address = v;
            }
            _ => return Err(MacError::Unsupported),
        }

        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let data_req = build_data_request(self.next_dsn(), &parent, &self.extended_address);

        self.driver
            .transmit(&data_req)
            .await
            .map_err(|_| MacError::RadioError)?;

        let result = select::select(Timer::after_millis(500), self.driver.receive()).await;

        match result {
            select::Either::Second(Ok(received)) => {
                if received.len < 5 {
                    return Ok(None);
                }
                let data = &received.data[..received.len];
                let fc = u16::from_le_bytes([data[0], data[1]]);
                let frame_type = fc & 0x07;

                if frame_type != 1 {
                    return Ok(None);
                }

                let header_len = 3 + addressing_size(fc);
                if data.len() <= header_len {
                    return Ok(None);
                }

                Ok(MacFrame::from_slice(&data[header_len..]))
            }
            _ => Ok(None),
        }
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let ack_requested = req.tx_options.ack_tx;
        let frame = self.build_data_frame(&req.dst_address, req.payload, ack_requested);

        // Unslotted CSMA-CA
        let mut be = self.min_be;
        let mut nb: u8 = 0;
        let symbol_period_us: u64 = 16;
        let unit_backoff_symbols: u64 = 20;

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

            match self.driver.transmit(&frame).await {
                Ok(_) => break,
                Err(_) => {
                    nb += 1;
                    be = core::cmp::min(be + 1, self.max_be);
                    if nb > self.max_csma_backoffs {
                        return Err(MacError::ChannelAccessFailure);
                    }
                }
            }
        }

        if ack_requested {
            for retransmit in 0..4u8 {
                Timer::after_millis(3 + retransmit as u64 * 2).await;
                let _ = self.driver.transmit(&frame).await;
            }
        }

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
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
                select::Either::Second(_) => return Err(MacError::NoData),
                select::Either::First(Err(_)) => continue,
                select::Either::First(Ok(rx_frame)) => {
                    let data = &rx_frame.data[..rx_frame.len];
                    if data.len() < 5 {
                        continue;
                    }
                    let fc = u16::from_le_bytes([data[0], data[1]]);
                    if fc & 0x07 != 1 {
                        continue;
                    }
                    let header_len = 3 + addressing_size(fc);
                    if data.len() <= header_len {
                        continue;
                    }

                    let src = parse_source_address(data, fc)
                        .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));
                    let dst = parse_dest_address(data, fc)
                        .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));

                    // Software address filtering
                    if !self.promiscuous {
                        let accepted = match &dst {
                            MacAddress::Short(pan, addr) => {
                                (pan.0 == 0xFFFF
                                    || pan.0 == self.pan_id.0
                                    || self.pan_id.0 == 0xFFFF)
                                    && (addr.0 == 0xFFFF || addr.0 == self.short_address.0)
                            }
                            MacAddress::Extended(pan, addr) => {
                                (pan.0 == 0xFFFF
                                    || pan.0 == self.pan_id.0
                                    || self.pan_id.0 == 0xFFFF)
                                    && *addr == self.extended_address
                            }
                        };
                        if !accepted {
                            continue;
                        }
                    }

                    let payload_data = &data[header_len..];
                    if let Some(mac_frame) = MacFrame::from_slice(payload_data) {
                        return Ok(McpsDataIndication {
                            src_address: src,
                            dst_address: dst,
                            lqi: rx_frame.lqi,
                            payload: mac_frame,
                            security_use: (fc >> 3) & 1 != 0,
                        });
                    }
                }
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: false,
            router: true,
            hardware_security: false,
            max_payload: 116,
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(8),
        }
    }
}

impl PlatformServices for Cc2340Mac {
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
