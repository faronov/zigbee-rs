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
}

impl Phy6222Mac {
    pub fn new() -> Self {
        let config = RadioConfig::default();
        Self {
            driver: Phy6222Driver::new(config),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: [0u8; 8],
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
                    let (rssi, _busy) =
                        self.driver.energy_detect().map_err(Self::map_radio_err)?;
                    let ed = ((rssi as i16 + 100).clamp(0, 255)) as u8;
                    let _ = energy_list.push(EdValue { channel: ch, energy: ed });
                }
                ScanType::Active => {
                    let seq = self.next_bsn();
                    let beacon_req = build_beacon_request(seq);
                    let _ = self.driver.transmit(&beacon_req).await;

                    let result = select::select(
                        self.driver.receive(),
                        Timer::after(embassy_time::Duration::from_millis(scan_duration_ms)),
                    )
                    .await;

                    if let select::Either::First(Ok(frame)) = result {
                        if let Some(pd) = parse_beacon_frame(&frame.data[..frame.len], ch) {
                            let _ = pan_descriptors.push(pd);
                        }
                    }
                }
                ScanType::Passive => {
                    let result = select::select(
                        self.driver.receive(),
                        Timer::after(embassy_time::Duration::from_millis(scan_duration_ms)),
                    )
                    .await;

                    if let select::Either::First(Ok(frame)) = result {
                        if let Some(pd) = parse_beacon_frame(&frame.data[..frame.len], ch) {
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

        self.driver.transmit(&frame).await.map_err(Self::map_radio_err)?;

        Timer::after(embassy_time::Duration::from_millis(500)).await;

        let result = select::select(
            self.driver.receive(),
            Timer::after(embassy_time::Duration::from_secs(5)),
        )
        .await;

        match result {
            select::Either::First(Ok(rx)) => {
                if let Some((addr, status_byte)) = parse_association_response(&rx.data[..rx.len]) {
                    let status = match status_byte {
                        0x00 => AssociationStatus::Success,
                        0x01 => AssociationStatus::PanAtCapacity,
                        _ => AssociationStatus::PanAccessDenied,
                    };
                    if status_byte == 0 {
                        self.short_address = addr;
                        self.driver.update_config(|c| c.short_address = addr.0);
                    }
                    Ok(MlmeAssociateConfirm {
                        short_address: addr,
                        status,
                    })
                } else {
                    Err(MacError::NoBeacon)
                }
            }
            _ => Err(MacError::NoBeacon),
        }
    }

    async fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
    ) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_disassociate(
        &mut self,
        _req: MlmeDisassociateRequest,
    ) -> Result<(), MacError> {
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
            (MacDsn, PibValue::U8(v)) => { self.dsn = v; }
            (MacBsn, PibValue::U8(v)) => { self.bsn = v; }
            (PhyTransmitPower, PibValue::U8(v)) => {
                self.driver.update_config(|c| c.tx_power = v as i8);
            }
            _ => return Err(MacError::InvalidParameter),
        }
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        let seq = self.next_dsn();
        let frame = build_data_request(seq, self.pan_id, self.coord_short_address);
        self.driver.transmit(&frame).await.map_err(Self::map_radio_err)?;

        let result = select::select(
            self.driver.receive(),
            Timer::after(embassy_time::Duration::from_millis(100)),
        )
        .await;

        match result {
            select::Either::First(Ok(rx)) => {
                Ok(MacFrame::from_slice(&rx.data[..rx.len]))
            }
            _ => Ok(None),
        }
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        if req.payload.len() > MAX_MAC_PAYLOAD {
            return Err(MacError::FrameTooLong);
        }

        // Build MAC frame with proper header
        let seq = self.next_dsn();
        let mac_frame = build_data_frame(
            seq,
            self.pan_id,
            &req.dst_address,
            self.short_address,
            req.payload,
            req.tx_options.ack_tx,
        );

        self.driver
            .transmit(&mac_frame)
            .await
            .map_err(Self::map_radio_err)?;

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        let rx = self.driver.receive().await.map_err(Self::map_radio_err)?;
        let data = &rx.data[..rx.len];

        // Parse minimal MAC header for addresses
        let (src_address, dst_address, payload_offset, security_use) = parse_mac_addresses(data);

        let mac_frame = MacFrame::from_slice(&data[payload_offset..]).unwrap_or_else(MacFrame::new);

        Ok(McpsDataIndication {
            src_address,
            dst_address,
            lqi: rx.lqi,
            payload: mac_frame,
            security_use,
        })
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
    [
        fc as u8,
        (fc >> 8) as u8,
        seq,
        0xFF, 0xFF,
        0xFF, 0xFF,
        0x07,
    ]
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
            update_id: if data.len() > offset + 14 { data[offset + 14] } else { 0 },
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
    if dst_mode > 0 { offset += 2; }
    match dst_mode {
        2 => offset += 2,
        3 => offset += 8,
        _ => {}
    }
    if src_mode > 0 && pan_compress == 0 { offset += 2; }
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
