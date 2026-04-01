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
    /// macCoordShortAddress — short address of the coordinator/parent
    coord_short_address: ShortAddress,
    /// Buffer for data frame received during association (e.g. Transport-Key)
    pending_assoc_frame: Option<([u8; 128], usize)>,
}

impl<'a, T: Instance> NrfMac<'a, T> {
    pub fn new(radio: Radio<'a, T>) -> Self {
        // Read factory-programmed IEEE address from FICR registers
        let ieee = Self::read_ficr_ieee();
        log::info!(
            "[MAC] IEEE: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
            ieee[0],
            ieee[1],
            ieee[2],
            ieee[3],
            ieee[4],
            ieee[5],
            ieee[6],
            ieee[7],
        );

        Self {
            radio,
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: ieee,
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            dsn: 0,
            bsn: 0,
            beacon_payload: PibPayload::new(),
            max_frame_retries: 3,
            promiscuous: false,
            tx_power: 0,
            coord_short_address: ShortAddress(0x0000),
            pending_assoc_frame: None,
        }
    }

    /// Read the unique device IEEE (EUI-64) address from nRF52840 FICR registers.
    /// FICR.DEVICEID[0] at 0x10000060 (low 32 bits)
    /// FICR.DEVICEID[1] at 0x10000064 (high 32 bits)
    fn read_ficr_ieee() -> IeeeAddress {
        const FICR_DEVICEID0: *const u32 = 0x1000_0060 as *const u32;
        const FICR_DEVICEID1: *const u32 = 0x1000_0064 as *const u32;
        let lo = unsafe { core::ptr::read_volatile(FICR_DEVICEID0) };
        let hi = unsafe { core::ptr::read_volatile(FICR_DEVICEID1) };
        let mut addr = [0u8; 8];
        addr[0..4].copy_from_slice(&lo.to_le_bytes());
        addr[4..8].copy_from_slice(&hi.to_le_bytes());
        addr
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

    /// Set radio TX power in dBm. nRF52840 supports -40 to +8 dBm.
    pub fn set_tx_power(&mut self, dbm: i8) {
        self.tx_power = dbm;
        self.radio.set_transmission_power(dbm);
    }

    /// Configure hardware address filtering on the radio.
    ///
    /// The nRF52840 RADIO peripheral supports automatic MHR (MAC Header) matching
    /// via DAB/DAP registers for address filtering. This programs:
    /// - DAP0/DAB0: short address + PAN ID for destination matching
    /// - DACNF: enable address 0
    ///
    /// With this configured, the radio only delivers frames addressed to us
    /// (or broadcast), reducing CPU wake-ups for sleepy end devices.
    fn update_address_filter(&mut self) {
        // nRF52840 RADIO base address
        const RADIO_BASE: u32 = 0x4000_1000;
        // DAB[n] — Device Address Base (lower 32 bits)
        const DAB0: *mut u32 = (RADIO_BASE + 0x600) as *mut u32;
        // DAP[n] — Device Address Prefix (upper 16 bits)
        const DAP0: *mut u32 = (RADIO_BASE + 0x620) as *mut u32;
        // DACNF — Device Address match Configuration
        const DACNF: *mut u32 = (RADIO_BASE + 0x640) as *mut u32;

        // For IEEE 802.15.4 MHR matching:
        // DAB0 = PAN_ID(16) | SHORT_ADDR(16) packed as little-endian
        let pan = self.pan_id.0;
        let short = self.short_address.0;

        // Only enable filtering if we have a valid short address and PAN
        if short == 0xFFFF || pan == 0xFFFF {
            // Disable address matching — accept all frames
            unsafe { core::ptr::write_volatile(DACNF, 0) };
            return;
        }

        // Pack: lower 16 = PAN ID, upper 16 = short address
        let dab_val = (pan as u32) | ((short as u32) << 16);
        unsafe {
            core::ptr::write_volatile(DAB0, dab_val);
            core::ptr::write_volatile(DAP0, 0); // not used for short addr matching
            // Enable address 0 matching (bit 0 = ENA0)
            // Note: nRF52840 MHR match is best-effort; we still validate in software
            core::ptr::write_volatile(DACNF, 0x01);
        }
        log::debug!(
            "[nRF MAC] Address filter: PAN=0x{:04X} short=0x{:04X}",
            pan,
            short
        );
    }

    /// Construct a beacon request MAC command frame.
    fn beacon_request_frame(&mut self) -> Packet {
        let seq = self.next_dsn();
        let mut pkt = Packet::new();
        // Fix 2: Beacon request is 8 bytes: FC(2) + Seq(1) + DstPAN(2) + DstAddr(2) + CmdID(1)
        let frame: [u8; 8] = [0x03, 0x08, seq, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];
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

    /// Fix 4: Scan a single channel passively (listen-only, no beacon request).
    async fn scan_channel_passive(
        &mut self,
        channel: u8,
        duration: u8,
    ) -> Result<heapless::Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>, MacError> {
        self.set_channel(channel);
        let delay_us = pib::scan_duration_us(duration);
        let mut descriptors = heapless::Vec::new();
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

        // Parse superframe spec from beacon payload.
        // Position depends on addressing mode — computed via addressing_size().
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

        log::info!(
            "[nRF MLME-SCAN] Starting {:?} scan, duration={}",
            req.scan_type,
            req.scan_duration
        );

        for channel in req.channel_mask.iter() {
            let ch = channel.number();
            log::debug!("[nRF MLME-SCAN] Scanning ch {}…", ch);
            match req.scan_type {
                ScanType::Active => match self.scan_channel_active(ch, req.scan_duration).await {
                    Ok(pds) => {
                        if !pds.is_empty() {
                            log::info!("[nRF MLME-SCAN] ch {}: {} beacon(s) found", ch, pds.len());
                        }
                        for pd in pds {
                            let _ = pan_descriptors.push(pd);
                        }
                    }
                    Err(e) => log::error!("[nRF MLME-SCAN] ch {ch}: {e:?}"),
                },
                ScanType::Passive => {
                    // Fix 4: Use passive scan for Passive scan type
                    match self.scan_channel_passive(ch, req.scan_duration).await {
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
                    // Measure energy on channel using RSSI sampling.
                    // nRF52840 doesn't expose a standalone ED task via embassy-nrf,
                    // so we start a brief RX and sample RSSISAMPLE register.
                    const RADIO_BASE: u32 = 0x4000_1000;
                    const RSSISAMPLE: *const u32 = (RADIO_BASE + 0x548) as *const u32;

                    // Brief listen to let AGC settle and capture RSSI
                    let mut dummy = Packet::new();
                    let _ = select::select(Timer::after_millis(2), self.radio.receive(&mut dummy))
                        .await;

                    // Read RSSISAMPLE (value is positive dBm magnitude, negate for actual)
                    let rssi_raw = unsafe { core::ptr::read_volatile(RSSISAMPLE) } as u8;
                    // Convert to 802.15.4 ED value (0-255, higher = more energy)
                    // nRF RSSI is 0..127 (abs dBm), map: ED = 255 - (rssi * 2)
                    let ed = 255u8.saturating_sub(rssi_raw.saturating_mul(2));
                    let _ = energy_list.push(EdValue {
                        channel: ch,
                        energy: ed,
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
            log::warn!("[nRF MLME-SCAN] No beacons found on any channel");
            return Err(MacError::NoBeacon);
        }

        log::info!(
            "[nRF MLME-SCAN] Scan complete: {} PAN descriptor(s)",
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

        // Per IEEE 802.15.4 §5.3.2.1: wait, then poll with Data Request.
        // Poll multiple times — the coordinator may need time to process.
        for poll_attempt in 0..5u8 {
            // First poll after 200ms, subsequent after 500ms
            let delay = if poll_attempt == 0 { 200 } else { 500 };
            Timer::after_millis(delay).await;

            // Send Data Request to poll for indirect Association Response
            let data_req =
                build_data_request(self.next_dsn(), &req.coord_address, &self.extended_address);
            let mut dreq_pkt = Packet::new();
            dreq_pkt.copy_from_slice(&data_req);
            let _ = self.radio.try_send(&mut dreq_pkt).await;

            // Wait up to 1.5s per poll for Association Response
            let timeout_us: u64 = 1_500_000;

            let mut rx_pkt = Packet::new();
            let result = select::select(
                Timer::after_micros(timeout_us),
                self.wait_assoc_response(&mut rx_pkt),
            )
            .await;

            match result {
                select::Either::Second(Ok(confirm)) => return Ok(confirm),
                select::Either::Second(Err(e)) => return Err(e),
                select::Either::First(_) => {
                    // Timeout — try polling again
                    continue;
                }
            }
        }

        Err(MacError::NoAck)
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
                    self.radio.set_transmission_power(p);
                }
            }
            PibAttribute::MacCoordShortAddress => {
                if let PibValue::ShortAddress(addr) = value {
                    self.coord_short_address = addr;
                }
            }
            PibAttribute::MacExtendedAddress => {
                if let PibValue::ExtendedAddress(addr) = value {
                    self.extended_address = addr;
                }
            }
            _ => {}
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

        // Try up to 2 poll passes:
        //   Pass 0: use SHORT address (if available) — matches most indirect frames
        //   Pass 1: use IEEE address — catches Transport-Key which EZSP may queue by IEEE
        // If we don't have a short address, only do one pass with IEEE.
        let passes: u8 = if has_short { 2 } else { 1 };

        for pass in 0..passes {
            let data_req = if pass == 0 && has_short {
                log::debug!(
                    "[MAC:Poll] pass {}: SHORT 0x{:04X}",
                    pass,
                    self.short_address.0
                );
                build_data_request_short(self.next_dsn(), &parent, self.pan_id, self.short_address)
            } else {
                log::debug!("[MAC:Poll] pass {}: IEEE", pass);
                build_data_request(self.next_dsn(), &parent, &self.extended_address)
            };

            let mut pkt = Packet::new();
            pkt.copy_from_slice(&data_req);

            self.radio
                .try_send(&mut pkt)
                .await
                .map_err(|_| MacError::RadioError)?;

            // IEEE 802.15.4 poll sequence:
            // 1. Parent ACKs our Data Request (with frame_pending bit)
            // 2. If frame_pending=1, parent sends the buffered data frame
            // Use a longer window (1500ms) and more attempts (40) to handle
            // busy channels where coordinator transmissions compete with
            // parent indirect frame delivery.
            let deadline = embassy_time::Instant::now() + embassy_time::Duration::from_millis(1500);

            let mut got_none = false;
            for _rx_attempt in 0..40u8 {
                let now = embassy_time::Instant::now();
                if now >= deadline {
                    break;
                }
                let remaining = deadline - now;

                let mut rx_pkt = Packet::new();
                let result =
                    select::select(Timer::after(remaining), self.radio.receive(&mut rx_pkt)).await;

                match result {
                    select::Either::Second(Ok(())) => {
                        let data = rx_pkt.as_ref();
                        if data.len() < 3 {
                            continue;
                        }
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        let frame_type = fc & 0x07;

                        if frame_type == 0x02 {
                            let frame_pending = (data[0] >> 4) & 1 != 0;
                            if !frame_pending {
                                log::debug!("[MAC:Poll] ACK frame_pending=0 (nothing pending)");
                                got_none = true;
                                break;
                            }
                            log::info!("[MAC:Poll] ACK frame_pending=1, waiting for data");
                            continue;
                        }

                        if frame_type != 0x01 {
                            log::info!(
                                "[MAC:Poll] Non-data frame type={} fc={:#06x} len={}",
                                frame_type,
                                fc,
                                data.len()
                            );
                            continue;
                        }

                        // Data frame — in a mesh network, frames can arrive
                        // from ANY router as the next hop (not just our parent).
                        // Only filter by MAC destination (must be for us or broadcast).
                        // The NWK layer handles source validation.
                        let mac_src = parse_source_address(data, fc);

                        // Verify MAC destination matches US (or broadcast).
                        // Without this, we accept coordinator frames for OTHER devices.
                        let mac_dst = parse_dest_address(data, fc);
                        let for_us = match &mac_dst {
                            Some(MacAddress::Short(_, d)) => {
                                d.0 == self.short_address.0
                                    || d.0 == 0xFFFF
                                    || d.0 == 0xFFFD
                                    || d.0 == 0xFFFC
                            }
                            Some(MacAddress::Extended(_, e)) => *e == self.extended_address,
                            None => true, // Can't parse → accept
                        };
                        if !for_us {
                            log::debug!(
                                "[MAC:Poll] SKIP frame for {:?} (we=0x{:04X})",
                                mac_dst,
                                self.short_address.0
                            );
                            continue;
                        }

                        // ACK if requested
                        let ack_requested = (fc >> 5) & 1 != 0;
                        if ack_requested {
                            let seq = data[2];
                            let mut ack_pkt = Packet::new();
                            ack_pkt.copy_from_slice(&[0x02, 0x00, seq]);
                            let _ = self.radio.try_send(&mut ack_pkt).await;
                        }

                        let header_len = 3 + addressing_size(fc);
                        if data.len() <= header_len {
                            return Ok(None);
                        }

                        // Log source info + payload size for diagnosis
                        let src_short = match &mac_src {
                            Some(MacAddress::Short(_, s)) => s.0,
                            _ => 0xFFFF,
                        };
                        log::info!(
                            "[MAC:Poll] frame {} bytes pass={} src=0x{:04X}",
                            data.len() - header_len,
                            pass,
                            src_short,
                        );
                        let payload_data = &data[header_len..];
                        return Ok(MacFrame::from_slice(payload_data));
                    }
                    select::Either::First(_) | select::Either::Second(Err(_)) => {
                        got_none = true;
                        break;
                    }
                }
            }

            // If pass 0 (short addr) got data → already returned above.
            // If pass 0 got nothing (got_none=true), continue to pass 1 (IEEE).
            if !got_none {
                // Exhausted rx_attempts without definitive result
                continue;
            }
        }

        Ok(None)
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
            req,
        )?;

        let mut pkt = Packet::new();
        pkt.copy_from_slice(&frame_buf[..len]);

        // Fix 6: Unslotted CSMA-CA (IEEE 802.15.4-2011 §5.1.1.4)
        let max_backoffs: u8 = 4; // macMaxCsmaBackoffs
        let min_be: u8 = 3; // macMinBE
        let max_be: u8 = 5; // macMaxBE
        let mut be = min_be;
        let mut nb: u8 = 0;
        let symbol_period_us: u64 = 16; // 2.4 GHz = 62.5 ksym/s = 16μs/symbol
        let unit_backoff_symbols: u64 = 20; // aUnitBackoffPeriod

        loop {
            // Random backoff: 0 to 2^BE - 1 unit backoff periods
            let max_val = (1u32 << be) - 1;
            // Simple PRNG: use dsn as seed (not cryptographic, but adequate for CSMA)
            let random = (self.dsn as u32)
                .wrapping_mul(1103515245)
                .wrapping_add(12345);
            let backoff = (random % (max_val + 1)) as u64;
            let delay_us = backoff * unit_backoff_symbols * symbol_period_us;
            if delay_us > 0 {
                Timer::after_micros(delay_us).await;
            }

            match self.radio.try_send(&mut pkt).await {
                Ok(()) => break,
                Err(_) => {
                    nb += 1;
                    be = core::cmp::min(be + 1, max_be);
                    if nb > max_backoffs {
                        return Err(MacError::ChannelAccessFailure);
                    }
                }
            }
        }

        // MAC ACK-based retransmission (IEEE 802.15.4 §5.1.6.4)
        // After try_send(), radio is DISABLED via PHYEND→DISABLE shortcut.
        // Immediately start RX to catch ACK within the 192μs turnaround window.
        // Timing: PHYEND → DISABLED (~1μs) → try_send returns (~20μs) →
        //         receive start (~30μs) → RXREADY (~40μs) = ~90μs total.
        //         ACK preamble starts at 192μs → we have ~100μs margin.
        let dsn = frame_buf[2]; // DSN is byte 2 of MAC frame
        let max_retries: u8 = if ack_requested { 3 } else { 0 };
        let mut ack_ok = !ack_requested; // true if no ACK needed

        for attempt in 0..=max_retries {
            if attempt > 0 {
                // CSMA-CA backoff before retransmit
                let delay = 2000 + attempt as u64 * 1000; // 2ms, 3ms, 4ms
                Timer::after_micros(delay).await;
                pkt.copy_from_slice(&frame_buf[..len]);
                let _ = self.radio.try_send(&mut pkt).await;
            }

            if !ack_requested {
                break; // No ACK needed — single send
            }

            // Listen for ACK: start RX immediately after TX
            let mut ack_pkt = Packet::new();
            let ack_result = select::select(
                Timer::after_micros(1200), // 1200μs timeout (192μs turnaround + margin)
                self.radio.receive(&mut ack_pkt),
            )
            .await;

            match ack_result {
                select::Either::Second(Ok(())) => {
                    let ack_data: &[u8] = &*ack_pkt;
                    if ack_data.len() >= 3 && (ack_data[0] & 0x07) == 0x02 && ack_data[2] == dsn {
                        log::info!("[MAC TX] ACK ok dsn={} attempt={}", dsn, attempt);
                        ack_ok = true;
                        break;
                    }
                    // Wrong frame
                    log::debug!(
                        "[MAC TX] Wrong frame dsn={} got {} bytes fc=0x{:02X}",
                        dsn,
                        ack_data.len(),
                        if !ack_data.is_empty() { ack_data[0] } else { 0 }
                    );
                }
                select::Either::Second(Err(_)) => {
                    log::debug!("[MAC TX] RX error attempt={}", attempt);
                }
                select::Either::First(_) => {
                    log::debug!("[MAC TX] ACK timeout attempt={}", attempt);
                }
            }
        }

        if !ack_ok {
            log::warn!("[MAC TX] No ACK after {} retries dsn={}", max_retries, dsn);
        }

        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        let mut rx_pkt = Packet::new();
        // Absolute deadline — filtered frames don't reset the clock
        const RX_TIMEOUT_MS: u64 = 1000;
        let deadline =
            embassy_time::Instant::now() + embassy_time::Duration::from_millis(RX_TIMEOUT_MS);

        loop {
            let now = embassy_time::Instant::now();
            if now >= deadline {
                return Err(MacError::NoData);
            }
            let remaining = deadline - now;

            // Use remaining time, not a fresh 5s timer
            let rx_result =
                select::select(self.radio.receive(&mut rx_pkt), Timer::after(remaining)).await;

            match rx_result {
                select::Either::Second(_) => {
                    // Timeout — no frame received
                    log::debug!("[nRF RX] Timeout ({}ms) — no frame", RX_TIMEOUT_MS);
                    return Err(MacError::NoData);
                }
                select::Either::First(Err(_)) => {
                    // CRC failure or radio error — discard and keep listening
                    log::debug!("[nRF RX] CRC/radio error, retrying");
                    continue;
                }
                select::Either::First(Ok(())) => {
                    // Frame received — process it below
                }
            }

            let data = rx_pkt.as_ref();
            if data.len() < 5 {
                log::debug!("[nRF RX] Runt frame ({} bytes), skip", data.len());
                continue;
            }

            let fc = u16::from_le_bytes([data[0], data[1]]);
            let frame_type = fc & 0x07;

            // Only deliver data frames (type 1) to upper layer
            if frame_type != 1 {
                continue;
            }

            // Generate ACK if requested (bit 5 of FC)
            let ack_requested = (fc >> 5) & 1 != 0;
            if ack_requested {
                let seq = data[2];
                let mut ack_pkt = Packet::new();
                ack_pkt.copy_from_slice(&[0x02, 0x00, seq]);
                let _ = self.radio.try_send(&mut ack_pkt).await;
            }

            let header_len = 3 + addressing_size(fc);
            if data.len() <= header_len {
                continue;
            }

            let src = parse_source_address(data, fc)
                .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));
            let dst = parse_dest_address(data, fc)
                .unwrap_or(MacAddress::Short(PanId(0), ShortAddress(0)));

            // Fix 5: Software address filtering — only accept frames for us
            if !self.promiscuous {
                match &dst {
                    MacAddress::Short(pan, addr) => {
                        let pan_ok = pan.0 == self.pan_id.0 || pan.0 == 0xFFFF;
                        let addr_ok = addr.0 == self.short_address.0 || addr.0 == 0xFFFF;
                        if !pan_ok || !addr_ok {
                            log::trace!(
                                "[nRF RX] Filtered short dst: pan=0x{:04X} addr=0x{:04X} (ours: pan=0x{:04X} addr=0x{:04X})",
                                pan.0,
                                addr.0,
                                self.pan_id.0,
                                self.short_address.0
                            );
                            continue;
                        }
                    }
                    MacAddress::Extended(pan, addr) => {
                        let pan_ok = pan.0 == self.pan_id.0 || pan.0 == 0xFFFF;
                        let addr_ok = *addr == self.extended_address;
                        if !pan_ok || !addr_ok {
                            log::info!(
                                "[nRF RX] Filtered ext dst: pan=0x{:04X} (ours: 0x{:04X})",
                                pan.0,
                                self.pan_id.0
                            );
                            continue;
                        }
                    }
                }
            }

            log::info!(
                "[nRF RX] Accepted frame {} bytes, LQI {}",
                data.len(),
                rx_pkt.lqi()
            );

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
            let frame_type = fc & 0x07;

            // ACK all frames with ACK-request bit — critical for Transport-Key!
            // Without ACK, the coordinator retries and eventually sends Leave.
            let ack_requested = (fc >> 5) & 1 != 0;
            if ack_requested {
                let seq = data[2];
                let mut ack_pkt = Packet::new();
                ack_pkt.copy_from_slice(&[0x02, 0x00, seq]);
                let _ = self.radio.try_send(&mut ack_pkt).await;
            }

            // Data frame received during association — could be Transport-Key!
            // Save it for later retrieval via mlme_poll.
            if frame_type == 0x01 {
                let header_len = 3 + addressing_size(fc);
                if data.len() > header_len {
                    let payload = &data[header_len..];
                    let len = payload.len().min(128);
                    let mut buf = [0u8; 128];
                    buf[..len].copy_from_slice(&payload[..len]);
                    self.pending_assoc_frame = Some((buf, len));
                    log::info!(
                        "[MAC] Saved+ACKed data frame {} bytes during association",
                        len
                    );
                }
                continue;
            }

            if frame_type != 3 {
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
                // After Association Response, stay in RX to catch Transport-Key.
                // ACK every frame — without ACK, coordinator retries and gives up.
                if status == AssociationStatus::Success {
                    let mut extra_pkt = Packet::new();
                    for _ in 0..20u8 {
                        let rx = select::select(
                            Timer::after_millis(200),
                            self.radio.receive(&mut extra_pkt),
                        )
                        .await;
                        match rx {
                            select::Either::Second(Ok(())) => {
                                let d = extra_pkt.as_ref();
                                if d.len() >= 5 {
                                    let efc = u16::from_le_bytes([d[0], d[1]]);
                                    // ACK if requested
                                    let ack_req = (efc >> 5) & 1 != 0;
                                    if ack_req {
                                        let seq = d[2];
                                        let mut ack_pkt = Packet::new();
                                        ack_pkt.copy_from_slice(&[0x02, 0x00, seq]);
                                        let _ = self.radio.try_send(&mut ack_pkt).await;
                                    }
                                    if efc & 0x07 == 0x01 {
                                        // Data frame — save it (last one wins)
                                        let hl = 3 + addressing_size(efc);
                                        if d.len() > hl {
                                            let pl = &d[hl..];
                                            let plen = pl.len().min(128);
                                            let mut buf = [0u8; 128];
                                            buf[..plen].copy_from_slice(&pl[..plen]);
                                            self.pending_assoc_frame = Some((buf, plen));
                                            log::info!(
                                                "[MAC] Caught+ACKed post-assoc frame {} bytes!",
                                                plen
                                            );
                                        }
                                    }
                                }
                            }
                            _ => break, // Timeout — no more frames
                        }
                    }
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
/// Fix 3: Use PAN ID compression to avoid missing Source PAN ID field
fn build_association_request(
    seq: u8,
    coord: &MacAddress,
    own_extended: &IeeeAddress,
    cap: &CapabilityInfo,
) -> heapless::Vec<u8, 32> {
    let mut frame = heapless::Vec::new();
    // FC: MAC command, ack req, PAN ID compress, dst=short, src=extended
    // 0xC863: type=3(cmd), ack_req=1, pan_compress=1, dst_mode=2(short), src_mode=3(ext)
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

/// Fix 11: Build a MAC Data Request command frame for indirect frame retrieval.
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

/// Build a MAC Data Request using SHORT source address.
/// Used after association when we have a short address assigned.
/// IEEE 802.15.4 §5.1.6.3: indirect frame matching uses the source address
/// mode of the Data Request — if parent queued frames for our short addr,
/// we must poll with short addr to match.
fn build_data_request_short(
    seq: u8,
    coord: &MacAddress,
    own_pan: PanId,
    own_short: ShortAddress,
) -> heapless::Vec<u8, 24> {
    let mut frame = heapless::Vec::new();
    // FC: MAC command (0b011), ack req, PAN compress, dst=short(0b10), src=short(0b10)
    // Bits: type=011, sec=0, pend=0, ack=1, pan_compress=1, rsvd=0, dst=10, ver=00, src=10
    // = 0b 10_00_10_0_0_1_1_0_011 = 0x8863
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
    // PAN compress=1: source PAN is elided (same as dst PAN)
    // Source short address
    let _ = frame.extend_from_slice(&own_short.0.to_le_bytes());
    let _ = frame.push(0x04); // Data Request command ID
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
