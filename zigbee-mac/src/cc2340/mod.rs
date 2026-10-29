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
//!          └── polling FIFO TX/RX, timer-paced so the CPU sleeps between polls
//! ```
//!
//! Each CSMA attempt runs a host-side CCA on the live RX operation (RCL
//! energy-or-signal mode). IRQ-driven completion and production power
//! management remain pending; the radio stays in RX between operations.
//!
//! # Hardware filtering and acknowledgement
//!
//! The PBE does the time-critical parts of IEEE 802.15.4 itself, exactly as
//! TI's RCL `RCL_CMD_IEEE_RX_TX` handler configures it:
//!
//! * **Frame filtering:** only beacons, data and MAC commands addressed to our
//!   PAN/short/extended address (or broadcast) reach the RX FIFO. Everything
//!   else is dropped by the PBE. `macPromiscuousMode` turns filtering off.
//!   An active scan temporarily uses PAN `0xFFFF`, so every beacon passes.
//! * **Incoming:** AR frames that pass the filter are acknowledged by the PBE
//!   192 us after the frame, with frame pending = 0.
//! * **Outgoing:** for an ACK-requested frame the PBE listens for the
//!   immediate ACK itself and reports it (`ACKSTATUS`). Missing ACKs are
//!   retried up to `macMaxFrameRetries` and then reported as
//!   [`MacError::NoAck`]. Only in promiscuous mode, where the PBE cannot
//!   match ACKs, the MAC falls back to a software ACK window of
//!   [`ACK_WAIT_US`].
//!
//! The RX operation keeps running between frames and is stopped only for a
//! TX, after which the PBE resumes RX by itself. Repeated frames (same source
//! and DSN as the previous one, i.e. a retransmission after a lost ACK) are
//! acknowledged by the PBE but not delivered twice. Frames that arrive while
//! the MAC is waiting for an ACK or a poll response are queued (bounded,
//! [`PENDING_RX_DEPTH`]) for `MCPS-DATA.indication` only if they would pass
//! its filter.

mod config;
pub mod driver;
mod fifo;
mod firmware;
mod hardware;

use crate::pib::{PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError, PlatformServices};
use driver::Cc2340Driver;
pub use driver::{DriverStats, RadioConfig, RadioError};
pub use fifo::PbeCounters;
use zigbee_types::*;

use crate::frames::{self, BackoffRng};
use embassy_futures::select;
use embassy_time::{Duration, Instant, Timer};

/// Software ACK window, used only in promiscuous mode where the PBE cannot
/// match ACKs: macAckWaitDuration (54 symbols = 864 µs) plus margin.
pub const ACK_WAIT_US: u64 = 2_000;

/// Interval of the `[CC2340 STATS]` log line.
pub const STATS_LOG_INTERVAL_S: u64 = 60;

/// How long to listen for the parent's indirect frame after an ACK with the
/// frame-pending bit (macMaxFrameTotalWaitTime ≈ 20 ms at default PIB, plus
/// parent software latency).
pub const POLL_DATA_WAIT_MS: u64 = 100;

/// Frames received while waiting for an ACK/poll response, retained for
/// `MCPS-DATA.indication` or a later `MLME-POLL`.
///
/// Each slot is a 129-byte [`PendingRx`], so the depth costs about 129 B of
/// `.bss` per slot. Eight slots absorb a burst of late indirect frames on a
/// lossy link (an OTA download keeps one block response in flight and a
/// retransmitted request can make the parent queue two) without dropping the
/// oldest one, which the four-slot queue did in hardware tests.
pub const PENDING_RX_DEPTH: usize = 8;

/// MAC-level counters, cumulative since boot.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct MacStats {
    /// ACK-requested transmissions that ended in `MacError::NoAck` after all
    /// retries.
    pub no_ack: u32,
    /// Retransmissions because an ACK was missing.
    pub retries: u32,
    /// Repeated frames (same source and DSN) acknowledged but not delivered.
    pub duplicates: u32,
    /// Retained frames dropped because the pending queue was full.
    pub queue_full: u32,
}

/// Source of the last accepted frame and its DSN, for duplicate rejection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct LastRx {
    source: MacAddress,
    sequence: u8,
}

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

/// Radio power states exposed by the current direct-register backend.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RadioSleepDisposition {
    /// [`Cc2340Mac::radio_sleep`] stops the PBE operation, powers down the
    /// TOPsm cores and synthesizer reference and gates the LRFD clocks; the
    /// LRFD stays powered, so firmware images, PHY registers and trims are
    /// retained. The 48 MHz crystal (HFXT) is also stopped (the CPU keeps
    /// running from HFOSC); [`Cc2340Mac::radio_wake`] restarts HFXT (busy-wait
    /// for amplitude settle) and re-enables the clocks. Device standby (LRFD
    /// register loss, LF time base) is not claimed.
    ClockGatedIdle,
    /// [`Self::ClockGatedIdle`], after which the caller may put the whole
    /// device into standby (LRFD power domain off) and report it with
    /// [`Cc2340Mac::radio_note_standby`]; the next wake then re-runs the full
    /// radio bring-up (firmware images, trims, PHY, synthesizer, PA) from the
    /// retained software configuration, as RCL does after
    /// `PowerLPF3_AWAKE_STANDBY`. Standby entry itself (LF clock, RTC wake,
    /// `HapiEnterStandby`) belongs to the board.
    StandbyCapable,
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
    /// `macAssociatedPANCoord`: associated through the PAN coordinator. The
    /// runtime restores it on a silent resume.
    associated_pan_coord: bool,
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
    last_rx: Option<LastRx>,
    stats: MacStats,
    stats_logged_at: Option<Instant>,
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
            associated_pan_coord: false,
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
            last_rx: None,
            stats: MacStats::default(),
            stats_logged_at: None,
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
        RadioSleepDisposition::StandbyCapable
    }

    /// Power the radio down for an idle wait (see
    /// [`RadioSleepDisposition::ClockGatedIdle`]).
    ///
    /// The RX FIFO is drained first, while the RX operation is still running:
    /// a stopped operation keeps its committed frames, and the ACK of every
    /// poll without frame-pending is committed there but never read by
    /// `mlme_poll`. Without the drain those ACKs accumulate across waits
    /// until the PBE drops frames with `RXFIFOFULL` (observed on hardware
    /// after a 120 s fast-poll phase: `rx_ffull` +1 per poll).
    pub fn radio_sleep(&mut self) -> Result<(), RadioError> {
        self.drain_rx_fifo();
        self.driver.sleep()
    }

    /// Restore the radio after [`Self::radio_sleep`]. Every MAC operation
    /// also restores it on demand, so this is an eager restore.
    pub fn radio_wake(&mut self) -> Result<(), RadioError> {
        self.driver.wake()
    }

    /// Whether the radio is currently powered down by [`Self::radio_sleep`].
    pub const fn radio_is_asleep(&self) -> bool {
        self.driver.is_asleep()
    }

    /// Report that the device was in standby since [`Self::radio_sleep`]
    /// (see [`RadioSleepDisposition::StandbyCapable`]). Returns `false` if
    /// the radio was not asleep, i.e. standby was entered out of sequence.
    pub fn radio_note_standby(&mut self) -> bool {
        self.driver.note_standby()
    }

    /// Quiesce any in-flight operation before an active-clock wait.
    ///
    /// This deliberately does not call `deinit`: a correct low-power restore
    /// sequence has not yet been proven on CC2340R52 hardware.
    pub fn prepare_active_wait(&mut self) -> Result<(), RadioError> {
        // Same FIFO hygiene as `radio_sleep`.
        self.drain_rx_fifo();
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
        // IEEE 802.15.4 active scan: macPANId is 0xFFFF for the duration of
        // the scan so the PBE filter accepts beacons from every PAN.
        self.driver.update_config(|cfg| {
            cfg.channel = channel;
            cfg.pan_id = 0xFFFF;
        });

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
    /// The driver performs CCA right before the TX; a busy channel (or any
    /// other refused transmission) consumes one CSMA attempt.
    async fn transmit_csma(&mut self, frame: &[u8]) -> Result<(), MacError> {
        self.transmit_csma_with(frame, false).await.map(|_| ())
    }

    /// [`Self::transmit_csma`]; with `expect_ack` the PBE also receives the
    /// immediate ACK and the result is its frame-pending bit, or
    /// [`MacError::NoAck`] for this single attempt.
    async fn transmit_csma_with(
        &mut self,
        frame: &[u8],
        expect_ack: bool,
    ) -> Result<bool, MacError> {
        let mut be = self.min_be;
        let mut nb: u8 = 0;
        loop {
            self.csma_backoff(be).await;
            self.drain_rx_fifo();
            let result = if expect_ack {
                self.driver.transmit_with_ack(frame).await
            } else {
                self.driver.transmit(frame).await.map(|()| false)
            };
            match result {
                Ok(pending) => return Ok(pending),
                Err(RadioError::NoAck) => return Err(MacError::NoAck),
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

    /// Whether `data` repeats the previously accepted frame: an AR frame with
    /// the same source address and DSN, i.e. a retransmission because our
    /// (hardware) ACK was lost. Records `data` as the last frame otherwise.
    ///
    /// The PBE has already acknowledged the repeat, which is what stops the
    /// sender's retries; it just must not be delivered a second time.
    fn is_duplicate(&mut self, data: &[u8]) -> bool {
        let Some(key) = duplicate_key(data) else {
            return false;
        };
        if self.last_rx == Some(key) {
            self.stats.duplicates = self.stats.duplicates.wrapping_add(1);
            log::debug!(
                "[CC2340] duplicate frame seq {} from {:?} dropped",
                key.sequence,
                key.source
            );
            return true;
        }
        self.last_rx = Some(key);
        false
    }

    /// Read everything the running RX operation has committed to the RX FIFO
    /// and treat it as out-of-band traffic (dedup, then [`Self::retain_frame`]).
    ///
    /// Done before every channel access: a full RX FIFO leaves the receiver
    /// without RSSI readings (CCA would report busy forever) and without room
    /// for the ACK or the poll response that the transmission solicits.
    fn drain_rx_fifo(&mut self) {
        while let Some(rx) = self.driver.try_receive() {
            let data = &rx.data[..rx.len];
            if frames::ack_info(data).is_none() {
                self.handle_out_of_band(data, rx.lqi);
            }
        }
    }

    /// Retain a data frame that arrived outside `MCPS-DATA.indication`, but
    /// only if indication would deliver it.
    ///
    /// A sleepy end device (`macRxOnWhenIdle` false) keeps only frames
    /// addressed to it: with its receiver nominally off it would not hear
    /// PAN broadcasts at all, its parent delivers broadcasts to it indirectly
    /// as unicast, and on a busy PAN the broadcasts otherwise crowd the
    /// unicast frames out of the queue.
    fn retain_frame(&mut self, data: &[u8], lqi: u8) {
        if data.len() > 127 {
            return;
        }
        let Some(indication) = self.indication_from(data, lqi) else {
            return;
        };
        if !self.rx_on_when_idle && is_broadcast(&indication.dst_address) {
            return;
        }
        if self.pending_rx.is_full() {
            self.stats.queue_full = self.stats.queue_full.wrapping_add(1);
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

    /// Handle a frame received while waiting for something else: keep it for
    /// indication unless it is a repeat. The PBE already acknowledged it.
    fn handle_out_of_band(&mut self, data: &[u8], lqi: u8) {
        if !self.is_duplicate(data) {
            self.retain_frame(data, lqi);
        }
    }

    /// Promiscuous-mode fallback: listen for the ACK of `sequence` for
    /// [`ACK_WAIT_US`] in software, because the PBE does not match ACKs
    /// without frame filtering.
    ///
    /// Returns `Some(frame_pending)` for a matching ACK and `None` when the
    /// window closes without one. Other frames are queued, not dropped.
    async fn wait_for_ack_software(&mut self, sequence: u8) -> Result<Option<bool>, MacError> {
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
                        None => self.handle_out_of_band(data, rx.lqi),
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
            if attempt > 0 {
                self.stats.retries = self.stats.retries.wrapping_add(1);
            }
            if self.promiscuous {
                self.transmit_csma(frame).await?;
                if let Some(pending) = self.wait_for_ack_software(sequence).await? {
                    return Ok(pending);
                }
            } else {
                match self.transmit_csma_with(frame, true).await {
                    Ok(pending) => return Ok(pending),
                    Err(MacError::NoAck) => {}
                    Err(error) => return Err(error),
                }
            }
            log::debug!("[CC2340] no ACK for seq {sequence} (attempt {attempt})");
        }
        self.stats.no_ack = self.stats.no_ack.wrapping_add(1);
        let driver = self.driver.stats();
        log::warn!(
            "[CC2340] NoAck for seq {sequence} after {} attempts (ack timeouts {}, ack rx errors {})",
            u16::from(self.max_frame_retries) + 1,
            driver.ack_timeout,
            driver.ack_rx_error
        );
        Err(MacError::NoAck)
    }

    /// MAC and driver counters.
    pub fn stats(&self) -> (MacStats, DriverStats, PbeCounters) {
        (self.stats, self.driver.stats(), self.driver.pbe_counters())
    }

    /// Emit the `[CC2340 STATS]` line at most every [`STATS_LOG_INTERVAL_S`].
    fn log_stats_periodically(&mut self) {
        let now = Instant::now();
        if let Some(last) = self.stats_logged_at
            && now.duration_since(last) < Duration::from_secs(STATS_LOG_INTERVAL_S)
        {
            return;
        }
        self.stats_logged_at = Some(now);
        let (mac, driver, pbe) = self.stats();
        log::info!(
            "[CC2340 STATS] tx={} ack={} ack_fp={} ack_timeout={} ack_rxerr={} retries={} noack={} \
             rx={} dup={} qfull={} rx_restart={} rx_drop={} cca_busy={} sleeps={} stby_reinit={} stop_to={} pa=0x{:04X} | pbe rx_ok={} rx_nok={} \
             rx_ign={} rx_ffull={} tx={} tx_ack={}",
            driver.tx_frames,
            driver.ack_received,
            driver.ack_frame_pending,
            driver.ack_timeout,
            driver.ack_rx_error,
            mac.retries,
            mac.no_ack,
            driver.rx_frames,
            mac.duplicates,
            mac.queue_full,
            driver.rx_restarts,
            driver.rx_dropped,
            driver.cca_busy,
            driver.sleeps,
            driver.standby_reinits,
            driver.stop_timeouts,
            self.driver.pa_value(),
            pbe.rx_ok,
            pbe.rx_nok,
            pbe.rx_ignored,
            pbe.rx_fifo_full,
            pbe.tx,
            pbe.tx_ack
        );
    }

    /// Take the oldest retained frame that is the parent's data for us,
    /// as `MLME-POLL` would have returned it.
    ///
    /// A sleepy device consumes received data only through `MLME-POLL`, so a
    /// poll response that arrived after [`POLL_DATA_WAIT_MS`] (or any other
    /// parent frame picked up while waiting for an ACK) would otherwise sit in
    /// the pending queue until it overflowed. Other retained frames keep
    /// their order for `MCPS-DATA.indication`.
    fn take_retained_poll_response(&mut self) -> Option<MacFrame> {
        let mut found = None;
        for _ in 0..self.pending_rx.len() {
            let Some(entry) = self.pending_rx.pop_front() else {
                break;
            };
            if found.is_none() {
                let data = &entry.data[..entry.len as usize];
                if self.is_poll_response(data) {
                    let fc = u16::from_le_bytes([data[0], data[1]]);
                    let header_len = 3 + addressing_size(fc);
                    found = data
                        .get(header_len..)
                        .filter(|payload| !payload.is_empty())
                        .and_then(MacFrame::from_slice);
                    if found.is_some() {
                        continue;
                    }
                }
            }
            let _ = self.pending_rx.push_back(entry);
        }
        found
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

/// Duplicate-rejection key of an AR data/command frame: its source address
/// and DSN. Frames without AR are never retransmitted, so they have none.
/// Whether `dst` is the MAC broadcast short address.
fn is_broadcast(dst: &MacAddress) -> bool {
    matches!(dst, MacAddress::Short(_, ShortAddress(0xFFFF)))
}

fn duplicate_key(data: &[u8]) -> Option<LastRx> {
    if data.len() < 3 {
        return None;
    }
    let fc = u16::from_le_bytes([data[0], data[1]]);
    let frame_type = fc & 0x07;
    if fc & 0x0020 == 0 || !(frame_type == 1 || frame_type == 3) {
        return None;
    }
    Some(LastRx {
        source: parse_source_address(data, fc)?,
        sequence: data[2],
    })
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
        self.transmit_acknowledged(&frame).await?;

        Timer::after_millis(100).await;

        let data_req =
            build_data_request(self.next_dsn(), &req.coord_address, &self.extended_address);
        if let Err(error) = self.transmit_acknowledged(&data_req).await {
            log::warn!("[CC2340 MLME-ASSOC] Data Request not acknowledged: {error:?}");
        }

        let timeout = Timer::after(embassy_time::Duration::from_millis(3000));
        let wait_response = async {
            for _ in 0..10 {
                match self.driver.receive().await {
                    Ok(rx_frame) => {
                        let data = &rx_frame.data[..rx_frame.len];
                        // The PBE has acknowledged the AR-unicast indirect
                        // Association Response already.
                        if data.len() < 5 || self.is_duplicate(data) {
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
                            self.associated_pan_coord = status == AssociationStatus::Success;
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
        self.associated_pan_coord = false;
        self.pending_rx.clear();
        self.last_rx = None;
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
            self.associated_pan_coord = false;
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
        self.last_rx = None;
        self.sync_radio_config();
        Ok(())
    }

    async fn mlme_start(&mut self, req: MlmeStartRequest) -> Result<(), MacError> {
        // `Cc2340Mac` does not implement `ParentMacDriver`: the PBE auto-ACK
        // always answers with frame pending = 0 (no source-match table) and
        // every parent primitive keeps its `Unsupported` default. Fail
        // explicitly.
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
            MacAssociatedPanCoord => Ok(PibValue::Bool(self.associated_pan_coord)),
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
            (MacAssociatedPanCoord, PibValue::Bool(v)) => self.associated_pan_coord = v,
            _ => return Err(MacError::Unsupported),
        }

        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        self.log_stats_periodically();
        // The parent already sent this frame (a late answer to an earlier
        // poll); deliver it before asking for more.
        if let Some(frame) = self.take_retained_poll_response() {
            return Ok(Some(frame));
        }
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
                    if frames::ack_info(data).is_some() || self.is_duplicate(data) {
                        continue;
                    }
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
                    if self.is_duplicate(data) {
                        continue;
                    }
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
    fn standby_capable_is_advertised_and_harmless_before_radio_init() {
        let mut mac = Cc2340Mac::new(VALID_IEEE).unwrap();
        assert_eq!(
            mac.radio_sleep_disposition(),
            RadioSleepDisposition::StandbyCapable
        );
        assert_eq!(mac.prepare_active_wait(), Ok(()));
        // Before `initialize_radio` there is nothing to power down: no MMIO
        // is touched and the driver never reports itself asleep, so a later
        // init performs the full bring-up.
        assert_eq!(mac.radio_sleep(), Ok(()));
        assert!(!mac.radio_is_asleep());
        // Standby without a sleeping radio is reported as out of sequence and
        // does not arm a re-init.
        assert!(!mac.radio_note_standby());
        assert_eq!(mac.radio_wake(), Ok(()));
        assert_eq!(mac.stats().1.sleeps, 0);
        assert_eq!(mac.stats().1.standby_reinits, 0);
        assert_eq!(mac.driver.read_rssi(), i8::MIN);
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

    /// Drive a future that never actually waits (PIB access) to completion.
    fn ready<F: core::future::Future>(future: F) -> F::Output {
        let mut context = core::task::Context::from_waker(core::task::Waker::noop());
        let mut future = core::pin::pin!(future);
        match future.as_mut().poll(&mut context) {
            core::task::Poll::Ready(output) => output,
            core::task::Poll::Pending => panic!("PIB access must not wait"),
        }
    }

    /// The runtime's silent resume (`configure_restored_network`) restores
    /// these PIB attributes after a reboot; any `Unsupported` makes a
    /// commissioned device fail every resume with `InitFailed`.
    #[test]
    fn silent_resume_pib_writes_are_supported() {
        let mut mac = Cc2340Mac::new(VALID_IEEE).unwrap();
        let writes = [
            (PibAttribute::PhyCurrentChannel, PibValue::U8(15)),
            (PibAttribute::MacPanId, PibValue::PanId(PanId(0xDFE9))),
            (
                PibAttribute::MacShortAddress,
                PibValue::ShortAddress(ShortAddress(0xC07B)),
            ),
            (
                PibAttribute::MacCoordShortAddress,
                PibValue::ShortAddress(ShortAddress(0xD586)),
            ),
            (PibAttribute::MacAssociatedPanCoord, PibValue::Bool(true)),
        ];
        for (attribute, value) in writes {
            assert_eq!(
                ready(mac.mlme_set(attribute, value)),
                Ok(()),
                "{attribute:?}"
            );
        }
        assert_eq!(
            ready(mac.mlme_get(PibAttribute::MacAssociatedPanCoord)),
            Ok(PibValue::Bool(true))
        );
        assert_eq!(
            ready(mac.mlme_get(PibAttribute::MacExtendedAddress)),
            Ok(PibValue::ExtendedAddress(VALID_IEEE))
        );
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

    /// A sleepy end device's receiver is nominally off between polls; PAN
    /// broadcasts it overhears must not crowd unicast frames out of the
    /// queue.
    #[test]
    fn sleepy_device_retains_unicast_but_not_broadcast() {
        let mut mac = associated_mac();
        assert!(!mac.rx_on_when_idle);
        for seq in 0..(PENDING_RX_DEPTH as u8 + 2) {
            mac.retain_frame(&data_frame(seq, 0xFFFF, 0x1111, &[seq]), 200);
        }
        assert!(mac.pending_rx.is_empty());
        assert_eq!(mac.stats.queue_full, 0);

        mac.retain_frame(&data_frame(9, 0x5678, 0x1111, &[0x09]), 200);
        assert_eq!(mac.pending_rx.len(), 1);
    }

    #[test]
    fn only_frames_indication_would_deliver_are_retained() {
        let mut mac = associated_mac();
        mac.rx_on_when_idle = true;
        mac.retain_frame(&data_frame(1, 0x5678, 0x1111, &[0x01]), 200);
        // Unicast to another node in our PAN: indication would drop it.
        mac.retain_frame(&data_frame(2, 0x9999, 0x1111, &[0x02]), 200);
        // Broadcast in our PAN is delivered.
        mac.retain_frame(&data_frame(3, 0xFFFF, 0x1111, &[0x03]), 200);
        // ACKs and commands are not retained.
        mac.retain_frame(&frames::build_ack(4, false), 200);
        // Frame for us by short address but in another PAN.
        let mut foreign_pan = data_frame(5, 0x5678, 0x1111, &[0x05]);
        foreign_pan[3] = 0x99;
        mac.retain_frame(&foreign_pan, 200);
        assert_eq!(mac.pending_rx.len(), 2);

        let first = mac.pending_rx.pop_front().unwrap();
        let indication = mac
            .indication_from(&first.data[..first.len as usize], first.lqi)
            .unwrap();
        assert_eq!(indication.payload.as_slice(), &[0x01]);
        assert_eq!(indication.lqi, 200);

        let second = mac.pending_rx.pop_front().unwrap();
        let indication = mac
            .indication_from(&second.data[..second.len as usize], second.lqi)
            .unwrap();
        assert_eq!(indication.payload.as_slice(), &[0x03]);
    }

    /// Busy-channel regression: foreign traffic never displaces our frame.
    #[test]
    fn foreign_traffic_cannot_fill_the_pending_queue() {
        let mut mac = associated_mac();
        mac.retain_frame(&data_frame(1, 0x5678, 0x0000, &[0x01]), 0);
        for seq in 2..40 {
            mac.retain_frame(&data_frame(seq, 0x4242, 0x1111, &[seq]), 0);
        }
        assert_eq!(mac.pending_rx.len(), 1);
        assert_eq!(mac.stats.queue_full, 0);
        assert_eq!(mac.pending_rx.front().unwrap().data[2], 1);
    }

    #[test]
    fn repeated_frame_from_same_source_and_dsn_is_dropped_once_acked() {
        let mut mac = associated_mac();
        let frame = data_frame(7, 0x5678, 0x0000, &[0x01]);
        assert!(!mac.is_duplicate(&frame));
        // The parent missed our ACK and retransmits the same frame.
        assert!(mac.is_duplicate(&frame));
        assert!(mac.is_duplicate(&frame));
        assert_eq!(mac.stats.duplicates, 2);

        // Next DSN from the same source is new.
        assert!(!mac.is_duplicate(&data_frame(8, 0x5678, 0x0000, &[0x02])));
        // Same DSN from another source is new.
        assert!(!mac.is_duplicate(&data_frame(8, 0x5678, 0x1111, &[0x02])));
        // A DSN that wrapped around after other traffic is accepted again.
        assert!(!mac.is_duplicate(&data_frame(7, 0x5678, 0x0000, &[0x03])));
    }

    #[test]
    fn frames_without_ack_request_are_never_duplicates() {
        let mut mac = associated_mac();
        let mut broadcast = data_frame(9, 0xFFFF, 0x0000, &[0x01]);
        broadcast[0] &= !0x20;
        assert!(!mac.is_duplicate(&broadcast));
        assert!(!mac.is_duplicate(&broadcast));
        assert!(!mac.is_duplicate(&frames::build_ack(9, false)));
        assert_eq!(mac.stats.duplicates, 0);
    }

    #[test]
    fn out_of_band_duplicates_are_not_retained() {
        let mut mac = associated_mac();
        let frame = data_frame(1, 0x5678, 0x0000, &[0x01]);
        mac.handle_out_of_band(&frame, 0);
        mac.handle_out_of_band(&frame, 0);
        assert_eq!(mac.pending_rx.len(), 1);
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

    /// A sleepy device only consumes data through `MLME-POLL`: parent frames
    /// retained by a FIFO drain (late poll responses) must come out of the
    /// next poll in order instead of piling up until the queue drops them.
    #[test]
    fn retained_parent_frames_are_delivered_by_the_next_poll() {
        let mut mac = associated_mac();
        // Unicast for us from a non-parent stays for indication.
        mac.retain_frame(&data_frame(1, 0x5678, 0x1111, &[0x01]), 0);
        mac.retain_frame(&data_frame(2, 0x5678, 0x0000, &[0x02, 0x22]), 0);
        mac.retain_frame(&data_frame(3, 0x5678, 0x0000, &[0x03]), 0);

        let first = mac.take_retained_poll_response().unwrap();
        assert_eq!(first.as_slice(), &[0x02, 0x22]);
        let second = mac.take_retained_poll_response().unwrap();
        assert_eq!(second.as_slice(), &[0x03]);
        assert!(mac.take_retained_poll_response().is_none());
        assert_eq!(mac.pending_rx.len(), 1);
        assert_eq!(mac.pending_rx.front().unwrap().data[2], 1);
    }

    /// A burst of late parent frames as deep as the queue is kept whole.
    #[test]
    fn pending_queue_holds_a_burst_of_parent_frames() {
        let mut mac = associated_mac();
        for seq in 0..PENDING_RX_DEPTH as u8 {
            mac.retain_frame(&data_frame(seq, 0x5678, 0x0000, &[seq]), 0);
        }
        assert_eq!(mac.stats.queue_full, 0);
        for seq in 0..PENDING_RX_DEPTH as u8 {
            assert_eq!(
                mac.take_retained_poll_response().unwrap().as_slice(),
                &[seq]
            );
        }
        assert!(mac.pending_rx.is_empty());
    }
}
