//! CC2340 MAC backend.
//!
//! Implements `MacDriver` for the Texas Instruments CC2340R5 ARM Cortex-M0+ SoC.
//! The host-side radio driver is Rust and accesses the LRFD registers directly.
//! TI's BSD-licensed PBE/MCE/RFE firmware images remain embedded as radio
//! microcode data; no RCL, ZBOSS, or MAC shim library is linked.
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
//!          ├── generated TI PHY settings and PA table
//!          ├── direct LRFD trim, synthesizer, and TOPsm setup
//!          ├── official PBE/MCE/RFE firmware loaded as data
//!          └── polling FIFO TX/RX with cooperative async yielding
//! ```
//!
//! Raw TX/RX is implemented but still requires validation on CC2340R5
//! hardware. CCA, hardware filtering/auto-ACK, IRQ completion, dynamic
//! temperature compensation, and production power management remain pending.
//!
//! # Software acknowledgement
//!
//! The PBE runs with auto-ACK disabled, so the MAC does both halves of the
//! IEEE 802.15.4 acknowledgement exchange in software:
//!
//! * **Outgoing:** an ACK-requested frame is followed by an RX window of
//!   [`ACK_WAIT_US`]; only an ACK with the frame's sequence number counts as
//!   delivery. Missing ACKs are retried up to `macMaxFrameRetries` and then
//!   reported as [`MacError::NoAck`].
//! * **Incoming:** a data/command frame with the AR bit set is acknowledged
//!   only when its destination is exactly this node
//!   ([`crate::frames::software_ack_sequence`]); broadcasts and frames for
//!   other nodes are never acknowledged.
//!
//! Frames that arrive while the MAC is waiting for an ACK or a poll response
//! are queued (bounded, [`PENDING_RX_DEPTH`]) for `MCPS-DATA.indication`
//! instead of being dropped. Whether the software TX→RX and RX→TX turnaround
//! meets the parent's `macAckWaitDuration` must still be proven on hardware.

mod config;
pub mod driver;
mod fifo;
mod firmware;
mod hardware;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError, PlatformServices};
use driver::Cc2340Driver;
pub use driver::{RadioConfig, RadioError};
use zigbee_types::*;

use crate::frames::{self, BackoffRng};
use embassy_futures::select;
use embassy_time::{Duration, Instant, Timer};

/// macAckWaitDuration (54 symbols = 864 µs) plus margin for the software
/// TX→RX re-arm of the PBE, which is not hardware-timed on this backend.
pub const ACK_WAIT_US: u64 = 2_000;

/// How long to listen for the parent's indirect frame after an ACK with the
/// frame-pending bit (macMaxFrameTotalWaitTime ≈ 20 ms at default PIB, plus
/// parent software latency).
pub const POLL_DATA_WAIT_MS: u64 = 100;

/// Frames received while waiting for an ACK/poll response, retained for
/// `MCPS-DATA.indication`.
pub const PENDING_RX_DEPTH: usize = 4;

/// A data frame received out of band, kept until indication consumes it.
struct PendingRx {
    data: [u8; 127],
    len: u8,
    lqi: u8,
}

/// Invalid factory or configured IEEE identity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IdentityError {
    UnprogrammedExtendedAddress,
}

/// The CC2340 backend has no independently verified entropy peripheral.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntropyDisposition {
    /// `PlatformServices::fill_random` returns `MacError::Unsupported` and
    /// never substitutes deterministic bytes for cryptographic entropy.
    UnavailableFailClosed,
}

/// Safe radio power state exposed by the current direct-register backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioSleepDisposition {
    /// The radio may be quiesced between operations, but clocks and firmware
    /// stay active. Retention and full-off restoration are not claimed.
    ActiveOnly,
}

/// CC2340 802.15.4 MAC driver.
pub struct Cc2340Mac {
    driver: Cc2340Driver,
    // PIB state
    short_address: ShortAddress,
    pan_id: PanId,
    channel: u8,
    extended_address: IeeeAddress,
    coord_short_address: ShortAddress,
    coord_extended_address: IeeeAddress,
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
    /// Data frames received during ACK/poll waits, oldest first.
    pending_rx: heapless::Deque<PendingRx, PENDING_RX_DEPTH>,
    /// CSMA backoff generator: per-device seed from the EUI-64, stirred with
    /// timer and RSSI jitter before every draw. Not used for key material —
    /// `fill_random` stays fail-closed until a TRNG path is validated.
    backoff_rng: BackoffRng,
}

impl Cc2340Mac {
    /// Create a CC2340 MAC with a validated factory EUI-64.
    ///
    /// Erased all-zero and all-`0xFF` identities are rejected before any
    /// radio hardware is touched.
    pub fn new(extended_address: IeeeAddress) -> Result<Self, IdentityError> {
        if !valid_ieee_address(extended_address) {
            return Err(IdentityError::UnprogrammedExtendedAddress);
        }

        let radio_config = RadioConfig {
            ieee_addr: extended_address,
            ..RadioConfig::default()
        };
        Ok(Self {
            driver: Cc2340Driver::new(radio_config),
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address,
            coord_short_address: ShortAddress(0x0000),
            coord_extended_address: [0; 8],
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
            pending_rx: heapless::Deque::new(),
            backoff_rng: BackoffRng::new(&extended_address, 0),
        })
    }

    /// Return the EUI-64 selected before MAC construction.
    pub const fn extended_address(&self) -> IeeeAddress {
        self.extended_address
    }

    /// Initialize the radio while preserving the low-level availability error.
    ///
    /// Compile-only images therefore distinguish `FirmwareUnavailable` from
    /// `RadioConfigUnavailable` without collapsing either into `MacError`.
    pub fn initialize_radio(&mut self) -> Result<(), RadioError> {
        self.driver.init()
    }

    pub const fn entropy_disposition(&self) -> EntropyDisposition {
        EntropyDisposition::UnavailableFailClosed
    }

    pub const fn radio_sleep_disposition(&self) -> RadioSleepDisposition {
        RadioSleepDisposition::ActiveOnly
    }

    /// Quiesce any in-flight operation before an active-clock wait.
    ///
    /// This deliberately does not call `deinit`: a correct low-power restore
    /// sequence has not yet been proven on CC2340R52 hardware.
    pub fn prepare_active_wait(&mut self) -> Result<(), RadioError> {
        self.driver.prepare_active_wait()
    }

    fn next_dsn(&mut self) -> u8 {
        let seq = self.dsn;
        self.dsn = self.dsn.wrapping_add(1);
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
        frame_pending: bool,
    ) -> heapless::Vec<u8, 127> {
        let mut frame = heapless::Vec::new();
        let seq = self.next_dsn();

        let mut fc: u16 = 0x0001; // Data frame
        if frame_pending {
            fc |= 0x0010;
        }
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
        if let Err(error) = self.driver.transmit(&beacon_req).await {
            log::warn!("[CC2340] Beacon Request TX failed on ch{channel}: {error:?}");
            return results;
        }

        let deadline = Timer::after(embassy_time::Duration::from_millis(duration_ms as u64));
        let collect = async {
            while let Ok(rx_frame) = self.driver.receive().await {
                if let Some(desc) =
                    Self::parse_beacon(&rx_frame.data[..rx_frame.len], rx_frame.lqi, channel)
                {
                    let _ = results.push(desc);
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
    /// Wait a random unslotted CSMA-CA backoff of `0..2^be` unit periods.
    async fn csma_backoff(&mut self, be: u8) {
        let jitter =
            (Instant::now().as_ticks() as u32) ^ ((self.driver.read_rssi() as u8 as u32) << 24);
        self.backoff_rng.mix(jitter);
        let slots = frames::csma_backoff_slots(self.backoff_rng.next_u32(), be);
        if slots > 0 {
            Timer::after_micros(u64::from(slots) * frames::UNIT_BACKOFF_PERIOD_US).await;
        }
    }

    /// Transmit `frame` with unslotted CSMA-CA using the PIB backoff policy.
    ///
    /// The CC2340 backend has no CCA, so a refused/failed transmission is the
    /// only "busy" signal; it consumes one CSMA attempt.
    async fn transmit_csma(&mut self, frame: &[u8]) -> Result<(), MacError> {
        let mut be = self.min_be;
        let mut nb: u8 = 0;
        loop {
            self.csma_backoff(be).await;
            match self.driver.transmit(frame).await {
                Ok(()) => return Ok(()),
                Err(RadioError::InvalidFrame) => return Err(MacError::FrameTooLong),
                Err(RadioError::RadioConfigUnavailable | RadioError::FirmwareUnavailable) => {
                    return Err(MacError::RadioError);
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
    }

    /// Acknowledge `data` in software if it is an AR frame addressed exactly to
    /// this node. ACKs are sent without CCA, as IEEE 802.15.4 requires.
    async fn acknowledge_if_required(&mut self, data: &[u8]) {
        if let Some(sequence) = frames::software_ack_sequence(
            data,
            self.pan_id,
            self.short_address,
            &self.extended_address,
        ) && let Err(error) = self
            .driver
            .transmit(&frames::build_ack(sequence, false))
            .await
        {
            log::warn!("[CC2340] software ACK for seq {sequence} failed: {error:?}");
        }
    }

    /// Retain a data frame that arrived outside `MCPS-DATA.indication`.
    fn retain_frame(&mut self, data: &[u8], lqi: u8) {
        if data.len() < 3 || data[0] & 0x07 != 0x01 || data.len() > 127 {
            return;
        }
        if self.pending_rx.is_full() {
            log::warn!("[CC2340] pending RX queue full, dropping oldest frame");
            let _ = self.pending_rx.pop_front();
        }
        let mut entry = PendingRx {
            data: [0; 127],
            len: data.len() as u8,
            lqi,
        };
        entry.data[..data.len()].copy_from_slice(data);
        let _ = self.pending_rx.push_back(entry);
    }

    /// Handle a frame received while waiting for something else: ACK it if
    /// required and keep it for indication.
    async fn handle_out_of_band(&mut self, data: &[u8], lqi: u8) {
        self.acknowledge_if_required(data).await;
        self.retain_frame(data, lqi);
    }

    /// Listen for the ACK of `sequence` for [`ACK_WAIT_US`].
    ///
    /// Returns `Some(frame_pending)` for a matching ACK and `None` when the
    /// window closes without one. Other frames are queued, not dropped.
    async fn wait_for_ack(&mut self, sequence: u8) -> Result<Option<bool>, MacError> {
        let deadline = Instant::now() + Duration::from_micros(ACK_WAIT_US);
        loop {
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            match select::select(self.driver.receive(), Timer::after(deadline - now)).await {
                select::Either::Second(()) => return Ok(None),
                select::Either::First(Err(
                    RadioError::RadioConfigUnavailable | RadioError::FirmwareUnavailable,
                )) => return Err(MacError::RadioError),
                select::Either::First(Err(_)) => {}
                select::Either::First(Ok(rx)) => {
                    let data = &rx.data[..rx.len];
                    match frames::ack_info(data) {
                        Some((seq, pending)) if seq == sequence => return Ok(Some(pending)),
                        Some(_) => {} // someone else's ACK
                        None => self.handle_out_of_band(data, rx.lqi).await,
                    }
                }
            }
        }
    }

    /// Transmit an ACK-requested frame and retry until its ACK arrives.
    ///
    /// Returns the ACK's frame-pending bit, or [`MacError::NoAck`] after
    /// `macMaxFrameRetries` retransmissions.
    async fn transmit_acknowledged(&mut self, frame: &[u8]) -> Result<bool, MacError> {
        let sequence = frame[2];
        for attempt in 0..=self.max_frame_retries {
            self.transmit_csma(frame).await?;
            if let Some(pending) = self.wait_for_ack(sequence).await? {
                return Ok(pending);
            }
            log::debug!("[CC2340] no ACK for seq {sequence} (attempt {attempt})");
        }
        Err(MacError::NoAck)
    }

    /// Whether `data` is the parent's indirect frame answering our poll: a
    /// data frame addressed exactly to us whose source is the coordinator.
    fn is_poll_response(&self, data: &[u8]) -> bool {
        frames::is_parent_poll_response(
            data,
            self.pan_id,
            self.short_address,
            &self.extended_address,
            self.coord_short_address,
            &self.coord_extended_address,
        )
    }

    /// Third-level filter and decode for `MCPS-DATA.indication`.
    fn indication_from(&self, data: &[u8], lqi: u8) -> Option<McpsDataIndication> {
        if data.len() < 5 {
            return None;
        }
        let fc = u16::from_le_bytes([data[0], data[1]]);
        if fc & 0x07 != 1 {
            return None;
        }
        let header_len = 3 + addressing_size(fc);
        if data.len() <= header_len {
            return None;
        }
        let src = parse_source_address(data, fc)?;
        let dst = parse_dest_address(data, fc)?;
        if !self.promiscuous
            && !frames::frame_is_for_us(
                &dst,
                self.pan_id,
                self.short_address,
                &self.extended_address,
            )
        {
            return None;
        }
        Some(McpsDataIndication {
            src_address: src,
            dst_address: dst,
            lqi,
            payload: MacFrame::from_slice(&data[header_len..])?,
            security_use: (fc >> 3) & 1 != 0,
        })
    }

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
        self.driver.init().map_err(|_| MacError::RadioError)?;
        let scan_duration_ms = ((1u32 << req.scan_duration) + 1) * 15;

        match req.scan_type {
            ScanType::Active | ScanType::Passive => {
                let mut pan_descriptors: PanDescriptorList = heapless::Vec::new();

                for ch in 11u8..=26 {
                    if let Some(channel) = Channel::from_number(ch)
                        && req.channel_mask.contains(channel)
                    {
                        let beacons = self.scan_channel_active(ch, scan_duration_ms).await;
                        for desc in beacons {
                            let _ = pan_descriptors.push(desc);
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
                    if let Some(channel) = Channel::from_number(ch)
                        && req.channel_mask.contains(channel)
                    {
                        let energy = self.scan_channel_ed(ch).await;
                        let _ = energy_list.push(EdValue {
                            channel: ch,
                            energy,
                        });
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
                        // The indirect Association Response is AR-unicast to
                        // our EUI-64; without our ACK the parent retries it
                        // and may expire the allocation.
                        self.acknowledge_if_required(data).await;
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
        self.coord_extended_address = [0; 8];
        self.pending_rx.clear();
        self.sync_radio_config();
        log::info!("[CC2340] Disassociated");
        Ok(())
    }

    fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        self.driver.init().map_err(|_| MacError::RadioError)?;
        if set_default_pib {
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.channel = 11;
            self.coord_short_address = ShortAddress(0x0000);
            self.coord_extended_address = [0; 8];
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
        self.pending_rx.clear();
        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        // `Cc2340Mac` does not implement `ParentMacDriver`: hardware auto-ACK
        // is left disabled (`PBE_IEEE_CFGAUTOACK = 0`) and every parent
        // primitive keeps its `Unsupported` default. Fail explicitly.
        start_requires_parent_capability(&req)
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
            MacCoordShortAddress => Ok(PibValue::ShortAddress(self.coord_short_address)),
            MacCoordExtendedAddress => Ok(PibValue::ExtendedAddress(self.coord_extended_address)),
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
                if !valid_ieee_address(v) {
                    return Err(MacError::InvalidParameter);
                }
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
            (MacCoordExtendedAddress, PibValue::ExtendedAddress(v)) => {
                self.coord_extended_address = v;
            }
            _ => return Err(MacError::Unsupported),
        }

        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let seq = self.next_dsn();
        // IEEE 802.15.4 §6.3.4: use the short source address once assigned.
        let data_req = if self.short_address.0 < 0xFFFE {
            frames::build_data_request_short(seq, &parent, self.short_address)
        } else {
            frames::build_data_request(seq, &parent, &self.extended_address)
        };

        // Data Request is ACK-requested; the ACK's frame-pending bit says
        // whether the parent holds indirect traffic for us.
        if !self.transmit_acknowledged(&data_req).await? {
            return Ok(None);
        }

        let deadline = Instant::now() + Duration::from_millis(POLL_DATA_WAIT_MS);
        loop {
            let now = Instant::now();
            if now >= deadline {
                log::debug!("[CC2340 POLL] frame pending but no data from parent");
                return Ok(None);
            }
            match select::select(self.driver.receive(), Timer::after(deadline - now)).await {
                select::Either::Second(()) => return Ok(None),
                select::Either::First(Err(
                    RadioError::RadioConfigUnavailable | RadioError::FirmwareUnavailable,
                )) => return Err(MacError::RadioError),
                select::Either::First(Err(_)) => {}
                select::Either::First(Ok(rx)) => {
                    let data = &rx.data[..rx.len];
                    if frames::ack_info(data).is_some() {
                        continue;
                    }
                    self.acknowledge_if_required(data).await;
                    if self.is_poll_response(data) {
                        let fc = u16::from_le_bytes([data[0], data[1]]);
                        let header_len = 3 + addressing_size(fc);
                        return Ok(data
                            .get(header_len..)
                            .filter(|payload| !payload.is_empty())
                            .and_then(MacFrame::from_slice));
                    }
                    self.retain_frame(data, rx.lqi);
                }
            }
        }
    }

    async fn mcps_data(&mut self, req: McpsDataRequest<'_>) -> Result<McpsDataConfirm, MacError> {
        let ack_requested = req.tx_options.ack_tx;
        let frame = self.build_data_frame(
            &req.dst_address,
            req.payload,
            ack_requested,
            req.tx_options.frame_pending,
        );

        if ack_requested {
            self.transmit_acknowledged(&frame).await?;
        } else {
            self.transmit_csma(&frame).await?;
        }

        Ok(McpsDataConfirm {
            msdu_handle: req.msdu_handle,
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
        while let Some(entry) = self.pending_rx.pop_front() {
            if let Some(indication) =
                self.indication_from(&entry.data[..entry.len as usize], entry.lqi)
            {
                return Ok(indication);
            }
        }

        let deadline = Instant::now() + Duration::from_micros(timeout_us as u64);

        loop {
            let now = Instant::now();
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
                    self.acknowledge_if_required(data).await;
                    if let Some(indication) = self.indication_from(data, rx_frame.lqi) {
                        return Ok(indication);
                    }
                }
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities::non_parent(116, TxPower(-20), TxPower(8))
    }
}

impl zigbee_crypto::ForwardAesProvider for Cc2340Mac {}
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

/// Validate an EUI-64 without imposing a byte-order interpretation.
pub const fn valid_ieee_address(address: IeeeAddress) -> bool {
    let mut index = 0;
    let mut all_zero = true;
    let mut all_erased = true;
    while index < address.len() {
        all_zero &= address[index] == 0;
        all_erased &= address[index] == 0xFF;
        index += 1;
    }
    !all_zero && !all_erased
}

#[cfg(test)]
mod tests {
    use super::*;

    const VALID_IEEE: IeeeAddress = [0x10, 0x32, 0x54, 0x76, 0x98, 0xBA, 0xDC, 0xFE];

    #[test]
    fn identity_constructor_rejects_unprogrammed_values() {
        assert!(matches!(
            Cc2340Mac::new([0; 8]),
            Err(IdentityError::UnprogrammedExtendedAddress)
        ));
        assert!(matches!(
            Cc2340Mac::new([0xFF; 8]),
            Err(IdentityError::UnprogrammedExtendedAddress)
        ));

        let mac = Cc2340Mac::new(VALID_IEEE).unwrap();
        assert_eq!(mac.extended_address(), VALID_IEEE);
    }

    #[test]
    fn entropy_is_explicitly_fail_closed() {
        let mut mac = Cc2340Mac::new(VALID_IEEE).unwrap();
        let mut output = [0xA5; 8];
        assert_eq!(
            mac.entropy_disposition(),
            EntropyDisposition::UnavailableFailClosed
        );
        assert_eq!(mac.fill_random(&mut output), Err(MacError::Unsupported));
        assert_eq!(output, [0xA5; 8]);
    }

    #[test]
    fn only_active_radio_wait_is_advertised() {
        let mut mac = Cc2340Mac::new(VALID_IEEE).unwrap();
        assert_eq!(
            mac.radio_sleep_disposition(),
            RadioSleepDisposition::ActiveOnly
        );
        assert_eq!(mac.prepare_active_wait(), Ok(()));
    }

    #[test]
    fn compile_only_availability_error_is_preserved_before_mmio() {
        let expected = if firmware::images().is_none() {
            RadioError::FirmwareUnavailable
        } else if config::ieee_802154_phy_writes().is_none() {
            RadioError::RadioConfigUnavailable
        } else {
            // A complete SDK would continue into real MMIO and is therefore a
            // hardware-only test.
            return;
        };

        let mut mac = Cc2340Mac::new(VALID_IEEE).unwrap();
        assert_eq!(mac.initialize_radio(), Err(expected));
    }

    fn associated_mac() -> Cc2340Mac {
        let mut mac = Cc2340Mac::new(VALID_IEEE).unwrap();
        mac.pan_id = PanId(0x1234);
        mac.short_address = ShortAddress(0x5678);
        mac.coord_short_address = ShortAddress(0x0000);
        mac
    }

    /// Data frame, PAN-compressed short→short, AR set.
    fn data_frame(seq: u8, dst: u16, src: u16, payload: &[u8]) -> heapless::Vec<u8, 127> {
        let mut frame = heapless::Vec::new();
        frame
            .extend_from_slice(&[0x61, 0x88, seq, 0x34, 0x12])
            .unwrap();
        frame.extend_from_slice(&dst.to_le_bytes()).unwrap();
        frame.extend_from_slice(&src.to_le_bytes()).unwrap();
        frame.extend_from_slice(payload).unwrap();
        frame
    }

    #[test]
    fn poll_response_must_come_from_parent_and_name_us() {
        let mut mac = associated_mac();
        assert!(mac.is_poll_response(&data_frame(1, 0x5678, 0x0000, &[0xAA])));
        // Another router's unicast to us is not the indirect frame we polled.
        assert!(!mac.is_poll_response(&data_frame(2, 0x5678, 0x1111, &[0xAA])));
        // The parent's broadcast is not a poll response.
        assert!(!mac.is_poll_response(&data_frame(3, 0xFFFF, 0x0000, &[0xAA])));
        // Parent traffic for a sibling.
        assert!(!mac.is_poll_response(&data_frame(4, 0x9999, 0x0000, &[0xAA])));
        // ACK and command frames never qualify.
        assert!(!mac.is_poll_response(&frames::build_ack(5, true)));

        // A parent known only by EUI-64 is matched on its extended source.
        mac.coord_extended_address = [9; 8];
        let mut ext_src = heapless::Vec::<u8, 127>::new();
        ext_src
            .extend_from_slice(&[0x61, 0xC8, 6, 0x34, 0x12, 0x78, 0x56])
            .unwrap();
        ext_src.extend_from_slice(&[9; 8]).unwrap();
        ext_src.push(0xAA).unwrap();
        assert!(mac.is_poll_response(&ext_src));
        ext_src[7] = 8;
        assert!(!mac.is_poll_response(&ext_src));
    }

    #[test]
    fn retained_frames_are_delivered_through_indication_filter() {
        let mut mac = associated_mac();
        mac.retain_frame(&data_frame(1, 0x5678, 0x1111, &[0x01]), 200);
        mac.retain_frame(&data_frame(2, 0x9999, 0x1111, &[0x02]), 200);
        // ACKs and commands are not retained.
        mac.retain_frame(&frames::build_ack(3, false), 200);
        assert_eq!(mac.pending_rx.len(), 2);

        let first = mac.pending_rx.pop_front().unwrap();
        let indication = mac
            .indication_from(&first.data[..first.len as usize], first.lqi)
            .unwrap();
        assert_eq!(indication.payload.as_slice(), &[0x01]);
        assert_eq!(indication.lqi, 200);

        let second = mac.pending_rx.pop_front().unwrap();
        assert!(
            mac.indication_from(&second.data[..second.len as usize], 0)
                .is_none()
        );
    }

    #[test]
    fn pending_queue_is_bounded_and_keeps_newest() {
        let mut mac = associated_mac();
        for seq in 0..(PENDING_RX_DEPTH as u8 + 2) {
            mac.retain_frame(&data_frame(seq, 0x5678, 0x1111, &[seq]), 0);
        }
        assert_eq!(mac.pending_rx.len(), PENDING_RX_DEPTH);
        assert_eq!(mac.pending_rx.front().unwrap().data[2], 2);
    }
}
