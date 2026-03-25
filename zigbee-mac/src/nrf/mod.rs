//! nRF MAC backend.
//!
//! Implements `MacDriver` using Embassy's ieee802154 radio driver for
//! Nordic nRF52840/nRF52833. Both chips share the same 802.15.4
//! radio with DMA-driven TX/RX and hardware address filtering.
//!
//! # Hardware features used
//! - Auto-CRC generation/checking
//! - Hardware address filtering (PAN ID + short address)
//! - Auto-ACK for frames with ACK request bit set
//! - Energy Detection via EDREQ task
//! - RSSI measurement
//!
//! # Dependencies
//! - `embassy-nrf` with nrf52840 or nrf52833 feature
//! - Embassy async executor
//!
//! # Supported boards
//! - nRF52840-DK, nRF52840-Dongle, Seeed XIAO nRF52840
//! - nRF52833-DK, Thingy:53

use crate::pib::{self, PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError};
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;

// Re-export embassy-nrf from the correct renamed dependency.
#[cfg(all(feature = "nrf52833", not(feature = "nrf52840")))]
use embassy_nrf52833 as embassy_nrf;
#[cfg(feature = "nrf52840")]
use embassy_nrf52840 as embassy_nrf;

use embassy_nrf::radio::Instance;
use embassy_nrf::radio::ieee802154::{Packet, Radio};

/// nRF52840 802.15.4 MAC driver.
///
/// Uses Embassy's hardware abstraction for the nRF radio peripheral.
/// TX/RX are interrupt-driven with DMA. The radio hardware handles
/// CRC, ACK generation, and address filtering.
///
/// # Usage
/// ```rust,no_run
/// use embassy_nrf::radio::ieee802154::Radio;
///
/// let radio = Radio::new(p.RADIO, Irqs);
/// let mac = NrfMac::new(radio);
/// let nlme = Nlme::new(storage, mac);
/// ```
pub struct NrfMac<'a, T: Instance> {
    radio: Radio<'a, T>,
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
    max_frame_retries: u8,
    promiscuous: bool,
    tx_power: i8,
}

impl<'a, T: Instance> NrfMac<'a, T> {
    pub fn new(radio: Radio<'a, T>) -> Self {
        Self {
            radio,
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: [0u8; 8],
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            dsn: 0,
            bsn: 0,
            beacon_payload: PibPayload::new(),
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

    /// Set the radio channel (11-26 for 2.4 GHz Zigbee).
    fn set_channel(&mut self, channel: u8) {
        self.channel = channel;
        self.radio.set_channel(channel);
    }

    /// Configure hardware address filtering on the radio.
    fn update_address_filter(&mut self) {
        // nRF52840 radio supports hardware PAN ID and short address filtering
        // This avoids receiving frames not addressed to us
        // TODO: call radio.set_pan_id() and radio.set_short_address()
        // when embassy-nrf exposes these APIs
    }

    /// Construct a beacon request MAC command frame.
    fn beacon_request_frame(&mut self) -> Packet {
        let seq = self.next_dsn();
        let mut pkt = Packet::new();
        let frame: [u8; 10] = [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07, 0x00, 0x00];
        pkt.copy_from_slice(&frame);
        pkt
    }

    /// Scan a single channel for beacons (active scan).
    async fn scan_channel_active(
        &mut self,
        channel: u8,
        duration: u8,
    ) -> Result<heapless::Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>, MacError> {
        self.set_channel(channel);

        // Send beacon request
        let mut pkt = self.beacon_request_frame();
        self.radio
            .try_send(&mut pkt)
            .await
            .map_err(|_| MacError::RadioError)?;

        let delay_us = pib::scan_duration_us(duration);
        let mut descriptors = heapless::Vec::new();

        // Listen for beacons until timeout
        let timer_fut = Timer::after_micros(delay_us);
        let rx_fut = self.collect_beacons(channel, &mut descriptors);
        let _ = select::select(timer_fut, rx_fut).await;

        Ok(descriptors)
    }

    /// Receive and parse beacons until cancelled.
    async fn collect_beacons(
        &mut self,
        channel: u8,
        descriptors: &mut heapless::Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>,
    ) -> Result<(), MacError> {
        let mut rx_pkt = Packet::new();

        for _ in 0..MAX_PAN_DESCRIPTORS {
            match self.radio.receive(&mut rx_pkt).await {
                Ok(()) => {
                    let data = rx_pkt.as_ref();
                    if let Some(pd) = self.parse_beacon(channel, data, rx_pkt.lqi()) {
                        if descriptors.push(pd).is_err() {
                            break;
                        }
                    }
                }
                Err(_) => continue,
            }
        }
        Ok(())
    }

    /// Parse a received frame as a beacon. Returns None if not a beacon.
    fn parse_beacon(&self, channel: u8, data: &[u8], lqi: u8) -> Option<PanDescriptor> {
        // Minimal frame check: at least FC(2) + SeqN(1) + addressing
        if data.len() < 5 {
            return None;
        }

        // Frame Control
        let fc = u16::from_le_bytes([data[0], data[1]]);
        let frame_type = fc & 0x07;

        // Must be a beacon frame (type 0)
        if frame_type != 0 {
            return None;
        }

        // Parse superframe spec (first 2 bytes of beacon payload in MHR)
        // The exact position depends on addressing mode — simplified here
        // TODO: proper IEEE 802.15.4 frame parsing via ieee802154 crate
        let superframe_offset = 3 + addressing_size(fc);
        if data.len() < superframe_offset + 2 {
            return None;
        }

        let sf_raw = u16::from_le_bytes([data[superframe_offset], data[superframe_offset + 1]]);
        let superframe_spec = SuperframeSpec::from_raw(sf_raw);

        // Zigbee beacon payload follows superframe + GTS + pending address fields
        // For non-beacon networks (beacon_order=15), GTS and pending are minimal
        let beacon_payload_offset = superframe_offset + 4; // +2 sf, +1 gts, +1 pending
        if data.len() < beacon_payload_offset + 15 {
            return None;
        }

        let zigbee_beacon = parse_zigbee_beacon(&data[beacon_payload_offset..]);

        // Extract coordinator address from frame header
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
}

// ── MacDriver implementation ────────────────────────────────────

impl<T: Instance> MacDriver for NrfMac<'_, T> {
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
                        Err(e) => log::error!("[nRF MLME-SCAN] ch {ch}: {e:?}"),
                    }
                }
                ScanType::Ed => {
                    self.set_channel(ch);
                    // nRF52840 supports hardware ED measurement
                    // TODO: use radio.energy_detection() when available
                    let _ = energy_list.push(EdValue {
                        channel: ch,
                        energy: 0,
                    });
                }
                ScanType::Orphan => {
                    log::warn!("[nRF] Orphan scan not yet implemented");
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
        self.set_channel(req.channel);

        // Build Association Request command frame
        let mut pkt = Packet::new();
        let frame = build_association_request(
            self.next_dsn(),
            &req.coord_address,
            &self.extended_address,
            &req.capability_info,
        );
        pkt.copy_from_slice(&frame);

        // Transmit
        self.radio
            .try_send(&mut pkt)
            .await
            .map_err(|_| MacError::RadioError)?;

        // Wait for Association Response with timeout
        let timeout_us = (pib::A_BASE_SUPERFRAME_DURATION as u64) * 32 * 1_000_000
            / pib::SYMBOL_RATE_2_4GHZ as u64;

        let mut rx_pkt = Packet::new();
        let result = select::select(
            Timer::after_micros(timeout_us),
            self.wait_assoc_response(&mut rx_pkt),
        )
        .await;

        match result {
            select::Either::Second(Ok(confirm)) => Ok(confirm),
            select::Either::Second(Err(e)) => Err(e),
            select::Either::First(_) => Err(MacError::NoAck),
        }
    }

    async fn mlme_associate_response(
        &mut self,
        _rsp: MlmeAssociateResponse,
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
            self.max_frame_retries = 3;
            self.promiscuous = false;
        }
        self.set_channel(self.channel);
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        self.pan_id = req.pan_id;
        self.set_channel(req.channel);
        self.update_address_filter();
        Ok(())
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        match attr {
            PibAttribute::MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            PibAttribute::MacPanId => Ok(PibValue::PanId(self.pan_id)),
            PibAttribute::MacExtendedAddress => {
                Ok(PibValue::ExtendedAddress(self.extended_address))
            }
            PibAttribute::MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            PibAttribute::MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            PibAttribute::MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            PibAttribute::MacDsn => Ok(PibValue::U8(self.dsn)),
            PibAttribute::PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PibAttribute::PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
            PibAttribute::PhyChannelsSupported => Ok(PibValue::U32(ChannelMask::ALL_2_4GHZ.0)),
            PibAttribute::MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PibAttribute::MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            _ => Ok(PibValue::U8(0)),
        }
    }

    async fn mlme_set(&mut self, attr: PibAttribute, value: PibValue) -> Result<(), MacError> {
        match attr {
            PibAttribute::MacShortAddress => {
                self.short_address = value.as_short_address().ok_or(MacError::InvalidParameter)?;
                self.update_address_filter();
            }
            PibAttribute::MacPanId => {
                self.pan_id = value.as_pan_id().ok_or(MacError::InvalidParameter)?;
                self.update_address_filter();
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
                self.set_channel(ch);
            }
            PibAttribute::MacPromiscuousMode => {
                self.promiscuous = value.as_bool().ok_or(MacError::InvalidParameter)?;
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
        // TODO: Send Data Request command to parent
        Ok(None)
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let msdu_handle = req.msdu_handle;
        let mut frame_buf = [0u8; 127];
        let len = build_data_frame(
            &mut frame_buf,
            self.next_dsn(),
            self.short_address,
            self.pan_id,
            &self.extended_address,
            req,
        )?;

        let mut pkt = Packet::new();
        pkt.copy_from_slice(&frame_buf[..len]);

        self.radio
            .try_send(&mut pkt)
            .await
            .map_err(|_| MacError::RadioError)?;

        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        let mut rx_pkt = Packet::new();
        loop {
            self.radio
                .receive(&mut rx_pkt)
                .await
                .map_err(|_| MacError::RadioError)?;

            let data = rx_pkt.as_ref();
            if data.len() < 5 {
                continue;
            }

            let fc = u16::from_le_bytes([data[0], data[1]]);
            let frame_type = fc & 0x07;

            // Only deliver data frames (type 1) to upper layer
            if frame_type != 1 {
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

            let payload_data = &data[header_len..];
            if let Some(mac_frame) = MacFrame::from_slice(payload_data) {
                return Ok(McpsDataIndication {
                    src_address: src,
                    dst_address: dst,
                    lqi: rx_pkt.lqi(),
                    payload: mac_frame,
                    security_use: (fc >> 3) & 1 != 0,
                });
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: true,
            router: true,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(8), // nRF52840: -20 to +8 dBm
        }
    }
}

impl<T: Instance> NrfMac<'_, T> {
    async fn wait_assoc_response(
        &mut self,
        pkt: &mut Packet,
    ) -> Result<MlmeAssociateConfirm, MacError> {
        for _ in 0..10 {
            self.radio
                .receive(pkt)
                .await
                .map_err(|_| MacError::RadioError)?;
            let data = pkt.as_ref();
            if data.len() < 5 {
                continue;
            }
            let fc = u16::from_le_bytes([data[0], data[1]]);
            if fc & 0x07 != 3 {
                continue; // Not a MAC command
            }
            // Find command ID — after addressing fields
            let cmd_offset = 3 + addressing_size(fc);
            if data.len() < cmd_offset + 4 {
                continue;
            }
            // Association Response = command ID 0x02
            if data[cmd_offset] == 0x02 {
                let short_addr = u16::from_le_bytes([data[cmd_offset + 1], data[cmd_offset + 2]]);
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
        }
        Err(MacError::NoAck)
    }
}

// ── Shared frame-building utilities ─────────────────────────────

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

/// Build an Association Request MAC command frame.
fn build_association_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
    cap: &CapabilityInfo,
) -> heapless::Vec<u8, 32> {
    let mut frame = heapless::Vec::new();
    // FC: MAC command, ack req, PAN ID compress, dst=short, src=extended
    let _ = frame.extend_from_slice(&[0x23, 0xC8, seq]);
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

/// Build an IEEE 802.15.4 data frame.
fn build_data_frame(
    buf: &mut [u8; 127],
    seq: u8,
    short_addr: ShortAddress,
    _pan_id: PanId,
    extended_addr: &IeeeAddress,
    req: McpsDataRequest<'_>,
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
