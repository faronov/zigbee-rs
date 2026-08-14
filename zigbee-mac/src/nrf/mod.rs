//! nRF MAC backend.
//!
//! Implements `MacDriver` using Embassy's ieee802154 radio driver for
//! Nordic nRF52840/nRF52833. Both chips share the same 802.15.4
//! radio with DMA-driven TX/RX.
//!
//! # Hardware features used
//! - Auto-CRC generation/checking
//! - Software address filtering (PAN ID + short/extended address)
//! - Software ACK generation and matching
//! - Hardware CCA before transmission
//! - RSSI measurement
//! - Fail-closed AES-128 through the Nordic ECB EasyDMA peripheral
//!
//! # Dependencies
//! - `embassy-nrf` with nrf52840 or nrf52833 feature
//! - Embassy async executor
//!
//! # Supported boards
//! - nRF52840-DK, nRF52840-Dongle, Seeed XIAO nRF52840
//! - nRF52833-DK

mod radio_phy;

pub use radio_phy::{NrfRadioPhy, NrfSoftMac};

use crate::frames::{
    addressing_size, build_association_request, build_beacon_request, build_data_frame,
    build_data_request, build_data_request_short, build_disassociation_notification, parse_beacon,
    parse_dest_address, parse_source_address,
};
use crate::nrf_aes::{EcbDataBlock, EcbRegisters, NrfAesDriver};
use crate::pib::{self, PibAttribute, PibPayload, PibValue};
use crate::primitives::*;
use crate::{MacCapabilities, MacDriver, MacError, PlatformServices};
use core::sync::atomic::{AtomicBool, Ordering};
use zigbee_types::*;

use embassy_futures::select;
use embassy_time::Timer;

// Re-export embassy-nrf from the correct renamed dependency.
#[cfg(all(feature = "nrf52833", not(feature = "nrf52840")))]
use embassy_nrf52833 as embassy_nrf;
#[cfg(feature = "nrf52840")]
use embassy_nrf52840 as embassy_nrf;

use embassy_nrf::radio::Instance as RadioInstance;
use embassy_nrf::radio::ieee802154::{Packet, Radio};
use embassy_nrf::rng::{Instance as RngInstance, Rng};

pub use crate::nrf_aes::NrfAesError;

const ECB_BASE: usize = 0x4000_E000;
const ECB_TASKS_STARTECB: usize = 0x000;
const ECB_TASKS_STOPECB: usize = 0x004;
const ECB_EVENTS_ENDECB: usize = 0x100;
const ECB_EVENTS_ERRORECB: usize = 0x104;
const ECB_INTENCLR: usize = 0x308;
const ECB_ECBDATAPTR: usize = 0x504;
const ECB_WAIT_LIMIT: u32 = 100_000;

/// Depth of the association-time receive queue.
///
/// A channel that already carries an active mesh delivers unrelated broadcast
/// traffic throughout the join, so this has to absorb that background while
/// still retaining the association response and the APS Transport-Key.
const ASSOC_FRAME_QUEUE_LEN: usize = 8;

static ECB_TAKEN: AtomicBool = AtomicBool::new(false);

/// Unique process-wide ownership token for the Nordic ECB peripheral.
///
/// Embassy 0.3 does not expose ECB in its `Peripherals` struct, so this
/// backend provides equivalent singleton ownership locally. This token is
/// not cloneable, and a successful acquisition is never released.
pub struct NrfEcbToken {
    _private: (),
}

impl NrfEcbToken {
    /// Acquire the ECB peripheral exactly once.
    pub fn take() -> Option<Self> {
        ECB_TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .ok()
            .map(|_| Self { _private: () })
    }
}

struct NrfEcbRegisters;

impl NrfEcbRegisters {
    fn prepare() -> Self {
        Self::write(ECB_INTENCLR, 0x03);
        Self::write(ECB_EVENTS_ENDECB, 0);
        Self::write(ECB_EVENTS_ERRORECB, 0);
        // Read-back prevents a following task write from overtaking event
        // clearing on the peripheral bus.
        let _ = Self::read(ECB_EVENTS_ERRORECB);
        Self
    }

    #[inline(always)]
    fn read(offset: usize) -> u32 {
        // SAFETY: offsets are fixed registers in the nRF52833/nRF52840 ECB
        // instance. The owning engine retains Embassy's unique ECB token.
        unsafe { core::ptr::read_volatile((ECB_BASE + offset) as *const u32) }
    }

    #[inline(always)]
    fn write(offset: usize, value: u32) {
        // SAFETY: same exclusive-token and fixed-register argument as `read`.
        unsafe { core::ptr::write_volatile((ECB_BASE + offset) as *mut u32, value) }
    }

    fn clear_event(offset: usize) {
        Self::write(offset, 0);
        let _ = Self::read(offset);
    }
}

impl EcbRegisters for NrfEcbRegisters {
    fn clear_end_event(&mut self) {
        Self::clear_event(ECB_EVENTS_ENDECB);
    }

    fn clear_error_event(&mut self) {
        Self::clear_event(ECB_EVENTS_ERRORECB);
    }

    fn set_data_ptr(&mut self, data: *mut EcbDataBlock) {
        Self::write(ECB_ECBDATAPTR, data as u32);
    }

    fn start(&mut self) {
        Self::write(ECB_TASKS_STARTECB, 1);
    }

    fn end_event(&mut self) -> bool {
        Self::read(ECB_EVENTS_ENDECB) != 0
    }

    fn error_event(&mut self) -> bool {
        Self::read(ECB_EVENTS_ERRORECB) != 0
    }

    fn stop(&mut self) {
        Self::write(ECB_TASKS_STOPECB, 1);
    }
}

struct NrfEcbEngine {
    _token: NrfEcbToken,
    driver: NrfAesDriver<NrfEcbRegisters>,
}

impl NrfEcbEngine {
    fn new(token: NrfEcbToken) -> Self {
        Self {
            _token: token,
            driver: NrfAesDriver::new(NrfEcbRegisters::prepare(), ECB_WAIT_LIMIT),
        }
    }

    fn self_test(&mut self) -> Result<(), NrfAesError> {
        self.driver.self_test()
    }

    #[inline(never)]
    fn encrypt_block(&mut self, key: &[u8; 16], block: &mut [u8; 16]) -> Result<(), NrfAesError> {
        let input = *block;
        self.driver.encrypt(key, &input, block)
    }
}

struct NrfAesState {
    installed: Option<NrfEcbEngine>,
    /// Retain the token and DMA storage after a rejected KAT. This leaves AES
    /// unavailable while ensuring a failed abort can never outlive its buffer.
    rejected: Option<NrfEcbEngine>,
}

impl NrfAesState {
    const fn new() -> Self {
        Self {
            installed: None,
            rejected: None,
        }
    }

    fn install(&mut self, token: NrfEcbToken) -> Result<(), NrfAesError> {
        if self.installed.is_some() || self.rejected.is_some() {
            return Err(NrfAesError::AlreadyInstalled);
        }

        let mut engine = NrfEcbEngine::new(token);
        match engine.self_test() {
            Ok(()) => {
                self.installed = Some(engine);
                Ok(())
            }
            Err(error) => {
                self.rejected = Some(engine);
                Err(error)
            }
        }
    }

    fn engine_mut(&mut self) -> Option<&mut NrfEcbEngine> {
        self.installed.as_mut()
    }
}

struct NrfHardwareAes128<'engine> {
    engine: Option<&'engine mut NrfEcbEngine>,
    key: zigbee_crypto::AesKey,
}

impl zigbee_crypto::Aes128Forward for NrfHardwareAes128<'_> {
    type Error = NrfAesError;

    fn encrypt_block(&mut self, block: &mut [u8; 16]) -> Result<(), Self::Error> {
        self.engine
            .as_deref_mut()
            .ok_or(NrfAesError::NotInstalled)?
            .encrypt_block(&self.key, block)
    }
}

/// nRF52840 802.15.4 MAC driver.
///
/// Uses Embassy's hardware abstraction for the nRF radio peripheral.
/// TX/RX are interrupt-driven with DMA. The radio hardware handles
/// CRC generation/checking and CCA are handled by hardware. MAC destination
/// filtering and ACK matching are implemented in software.
///
/// # Usage
/// ```rust,no_run
/// use embassy_nrf::radio::ieee802154::Radio;
///
/// let radio = Radio::new(p.RADIO, Irqs);
/// let rng = Rng::new(p.RNG, Irqs);
/// let mac = NrfMac::new(radio, rng);
/// let nlme = Nlme::new(storage, mac);
/// ```
pub struct NrfMac<'a, T: RadioInstance, R: RngInstance> {
    radio: Radio<'a, T>,
    rng: Rng<'a, R>,
    // PIB state
    short_address: ShortAddress,
    pan_id: PanId,
    channel: u8,
    extended_address: IeeeAddress,
    rx_on_when_idle: bool,
    association_permit: bool,
    auto_request: bool,
    associated_pan_coord: bool,
    dsn: u8,
    bsn: u8,
    beacon_payload: PibPayload,
    max_frame_retries: u8,
    promiscuous: bool,
    tx_power: i8,
    /// macCoordShortAddress — short address of the coordinator/parent
    coord_short_address: ShortAddress,
    /// macCoordExtendedAddress — extended address of the coordinator/parent
    coord_extended_address: IeeeAddress,
    /// Frames received while association is still completing.
    ///
    /// Sized for a busy channel: the frame that actually matters here is the
    /// APS Transport-Key, and it arrives *after* the association response, so
    /// unrelated mesh broadcasts captured in the meantime must not be able to
    /// crowd it out.
    pending_assoc_frames: heapless::Deque<MacFrame, ASSOC_FRAME_QUEUE_LEN>,
    /// Per-device evolving state for sequence numbers and CSMA backoff.
    random_state: u32,
    /// Exclusively-owned Nordic ECB accelerator. Production composition roots
    /// install it and pass both startup KATs before constructing networking.
    aes: NrfAesState,
}

impl<'a, T: RadioInstance, R: RngInstance> NrfMac<'a, T, R> {
    pub fn new(radio: Radio<'a, T>, rng: Rng<'a, R>) -> Self {
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

        let seed = u32::from_le_bytes([ieee[0], ieee[1], ieee[2], ieee[3]])
            ^ u32::from_le_bytes([ieee[4], ieee[5], ieee[6], ieee[7]])
            ^ 0x9E37_79B9;

        Self {
            radio,
            rng,
            short_address: ShortAddress(0xFFFF),
            pan_id: PanId(0xFFFF),
            channel: 11,
            extended_address: ieee,
            rx_on_when_idle: false,
            association_permit: false,
            auto_request: true,
            associated_pan_coord: false,
            dsn: seed as u8,
            bsn: (seed >> 8) as u8,
            beacon_payload: PibPayload::new(),
            max_frame_retries: 3,
            promiscuous: false,
            tx_power: 0,
            coord_short_address: ShortAddress(0x0000),
            coord_extended_address: [0; 8],
            pending_assoc_frames: heapless::Deque::new(),
            random_state: seed.max(1),
            aes: NrfAesState::new(),
        }
    }

    /// Consume the unique ECB token and install hardware AES after two
    /// back-to-back AES-128 known-answer tests with different keys.
    ///
    /// A failed engine is retained only as rejected/quarantined ownership;
    /// it is never made available to CCM* or AES-MMO.
    pub fn install_aes_engine(&mut self, token: NrfEcbToken) -> Result<(), NrfAesError> {
        self.aes.install(token)
    }

    /// Return the factory-programmed EUI-64 currently used by the MAC.
    pub const fn extended_address(&self) -> IeeeAddress {
        self.extended_address
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

    fn next_random_u32(&mut self) -> u32 {
        let mut value = self.random_state;
        value ^= value << 13;
        value ^= value >> 17;
        value ^= value << 5;
        self.random_state = value.max(1);
        value
    }

    fn queue_pending_assoc_frame(&mut self, payload: &[u8]) {
        let Some(frame) = MacFrame::from_slice(payload) else {
            log::warn!(
                "[MAC] Dropping oversized association-time frame ({} bytes)",
                payload.len()
            );
            return;
        };
        // Evict the oldest entry instead of rejecting the newest. The frames
        // that complete a join (association response, then the APS
        // Transport-Key) are always the most recent ones, so a queue already
        // filled with unrelated mesh broadcasts must never be allowed to
        // permanently discard them.
        if self.pending_assoc_frames.is_full() {
            let _ = self.pending_assoc_frames.pop_front();
            log::debug!("[MAC] Association-time frame queue full — evicted oldest frame");
        }
        let _ = self.pending_assoc_frames.push_back(frame);
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

    async fn try_send_with_csma(&mut self, packet: &mut Packet) -> Result<(), MacError> {
        const MAX_BACKOFFS: u8 = 4;
        const MIN_BE: u8 = 3;
        const MAX_BE: u8 = 5;
        const UNIT_BACKOFF_US: u64 = 20 * 16;

        let mut backoff_count = 0u8;
        let mut backoff_exponent = MIN_BE;
        loop {
            let slots = self.next_random_u32() & ((1u32 << backoff_exponent) - 1);
            if slots != 0 {
                Timer::after_micros(slots as u64 * UNIT_BACKOFF_US).await;
            }

            match self.radio.try_send(packet).await {
                Ok(()) => return Ok(()),
                Err(_) if backoff_count < MAX_BACKOFFS => {
                    backoff_count += 1;
                    backoff_exponent = core::cmp::min(backoff_exponent + 1, MAX_BE);
                }
                Err(_) => return Err(MacError::ChannelAccessFailure),
            }
        }
    }

    async fn send_acknowledged_frame(
        &mut self,
        frame: &[u8],
        max_retries: u8,
    ) -> Result<(), MacError> {
        let dsn = *frame.get(2).ok_or(MacError::InvalidParameter)?;
        for attempt in 0..=max_retries {
            let mut packet = Packet::new();
            packet.copy_from_slice(frame);
            self.try_send_with_csma(&mut packet).await?;

            let mut ack_packet = Packet::new();
            let ack_result = select::select(
                Timer::after_micros(1200),
                self.radio.receive(&mut ack_packet),
            )
            .await;
            if let select::Either::Second(Ok(())) = ack_result {
                let ack = ack_packet.as_ref();
                if ack.len() >= 3 && ack[0] & 0x07 == 0x02 && ack[2] == dsn {
                    log::info!("[MAC TX] ACK ok dsn={} attempt={}", dsn, attempt);
                    return Ok(());
                }
            }
        }

        log::warn!("[MAC TX] No ACK after {} retries dsn={}", max_retries, dsn);
        Err(MacError::NoAck)
    }

    /// Stop the RADIO peripheral between sleepy-device poll windows.
    ///
    /// `&mut self` guarantees no Embassy radio future is active. Interrupts
    /// are cleared before requesting DISABLED, and the next Embassy operation
    /// performs the normal DISABLED-to-RX/TX transition.
    pub fn enter_low_power_idle(&mut self) -> Result<(), MacError> {
        const RADIO_BASE: u32 = 0x4000_1000;
        const TASKS_DISABLE: *mut u32 = (RADIO_BASE + 0x010) as *mut u32;
        const STATE: *const u32 = (RADIO_BASE + 0x550) as *const u32;

        self.radio.clear_all_interrupts();
        if unsafe { core::ptr::read_volatile(STATE) } == 0 {
            return Ok(());
        }
        unsafe { core::ptr::write_volatile(TASKS_DISABLE, 1) };
        for _ in 0..10_000 {
            if unsafe { core::ptr::read_volatile(STATE) } == 0 {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(MacError::RadioError)
    }

    /// Construct a beacon request MAC command frame.
    fn beacon_request_frame(&mut self) -> Packet {
        let seq = self.next_dsn();
        let mut pkt = Packet::new();
        let frame = build_beacon_request(seq);
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

        // The beacon request must survive a busy channel: a single CCA-busy
        // result would otherwise abort this channel entirely, which is exactly
        // what happens on the channel that already carries the mesh we want to
        // join. Use the normal CSMA-CA backoff path instead of a bare TX.
        let mut pkt = self.beacon_request_frame();
        self.try_send_with_csma(&mut pkt).await?;

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

    /// Receive and parse beacons until the scan-duration timer cancels this
    /// future.
    ///
    /// The loop budget must never be spent on frames that are not beacons.
    /// A Zigbee channel that already carries an active mesh delivers a
    /// continuous stream of data frames, ACKs and CRC failures, so a bounded
    /// count of *receive operations* ends the scan long before the scan
    /// duration elapses and hides the networks that beacon later — including
    /// the coordinator, which is usually the node with permit-join open.
    /// Only a full descriptor list stops the scan early; everything else keeps
    /// listening and lets the caller's timer decide when the channel is done.
    async fn collect_beacons(
        &mut self,
        channel: u8,
        descriptors: &mut heapless::Vec<PanDescriptor, MAX_PAN_DESCRIPTORS>,
    ) -> Result<(), MacError> {
        // A radio that fails back-to-back this many times is wedged rather
        // than merely hearing corrupted traffic. Bail out instead of spinning
        // for the rest of the scan window. Reset on every successful receive.
        const MAX_CONSECUTIVE_RADIO_ERRORS: u32 = 64;

        let mut rx_pkt = Packet::new();
        let mut consecutive_errors = 0u32;

        loop {
            match self.radio.receive(&mut rx_pkt).await {
                Ok(()) => {
                    consecutive_errors = 0;
                    let data = rx_pkt.as_ref();
                    // `Packet::lqi()` is the raw Nordic ED byte; PanDescriptor
                    // carries an IEEE 802.15.4 LQI. Normalize exactly once.
                    let lqi = crate::lqi::nrf_from_hardware(rx_pkt.lqi());
                    let Some(pd) = parse_beacon(channel, data, lqi) else {
                        continue;
                    };
                    // Descriptor slots are the scarce resource on a dense mesh.
                    // A router that beacons twice within one window must not
                    // consume a slot that another PAN still needs.
                    if descriptors
                        .iter()
                        .any(|existing| existing.coord_address == pd.coord_address)
                    {
                        continue;
                    }
                    if descriptors.push(pd).is_err() {
                        return Ok(());
                    }
                }
                Err(_) => {
                    consecutive_errors += 1;
                    if consecutive_errors >= MAX_CONSECUTIVE_RADIO_ERRORS {
                        return Err(MacError::RadioError);
                    }
                }
            }
        }
    }
}

// ── MacDriver implementation ────────────────────────────────────

impl<T: RadioInstance, R: RngInstance> MacDriver for NrfMac<'_, T, R> {
    async fn mlme_scan(&mut self, req: MlmeScanRequest) -> Result<MlmeScanConfirm, MacError> {
        let mut pan_descriptors = heapless::Vec::new();
        let energy_list = heapless::Vec::new();

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
                    log::warn!("[nRF] ED scan is not implemented by embassy-nrf");
                    return Err(MacError::Unsupported);
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
        self.associated_pan_coord = false;
        self.set_channel(req.channel);
        match req.coord_address {
            MacAddress::Short(_, address) => {
                self.coord_short_address = address;
                self.coord_extended_address = [0; 8];
            }
            MacAddress::Extended(_, address) => {
                self.coord_short_address = ShortAddress::COORDINATOR;
                self.coord_extended_address = address;
            }
        }

        // Build Association Request command frame
        let frame = build_association_request(
            self.next_dsn(),
            &req.coord_address,
            &self.extended_address,
            &req.capability_info,
        );

        // IEEE 802.15.4 §6.4.1: the association request is an acknowledged
        // MAC command. A bare CCA-gated transmit neither retries a busy
        // channel nor confirms the coordinator actually heard us, so a lost
        // request was previously indistinguishable from a coordinator that
        // never answered — the join then burned all five poll attempts
        // waiting for a response to a frame that was never delivered.
        self.send_acknowledged_frame(&frame, self.max_frame_retries)
            .await?;

        // Per IEEE 802.15.4 §5.3.2.1: wait, then poll with Data Request.
        // Poll multiple times — the coordinator may need time to process.
        for poll_attempt in 0..5u8 {
            // First poll after 200ms, subsequent after 500ms
            let delay = if poll_attempt == 0 { 200 } else { 500 };
            Timer::after_millis(delay).await;

            // Send Data Request to poll for indirect Association Response.
            // CSMA-CA backoff only; the ACK window is deliberately not consumed
            // here so the coordinator's indirect Association Response is left
            // for wait_assoc_response below.
            let data_req =
                build_data_request(self.next_dsn(), &req.coord_address, &self.extended_address);
            let mut dreq_pkt = Packet::new();
            dreq_pkt.copy_from_slice(&data_req);
            let _ = self.try_send_with_csma(&mut dreq_pkt).await;

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

    async fn mlme_disassociate(&mut self, req: MlmeDisassociateRequest) -> Result<(), MacError> {
        if req.tx_indirect {
            return Err(MacError::Unsupported);
        }
        let frame = build_disassociation_notification(
            self.next_dsn(),
            &req.device_address,
            self.short_address,
            &self.extended_address,
            req.reason,
        );
        self.send_acknowledged_frame(&frame, self.max_frame_retries)
            .await?;
        self.short_address = ShortAddress(0xFFFF);
        self.pan_id = PanId(0xFFFF);
        self.associated_pan_coord = false;
        Ok(())
    }

    fn mlme_reset(&mut self, set_default_pib: bool) -> Result<(), MacError> {
        if set_default_pib {
            let random = self.next_random_u32();
            self.short_address = ShortAddress(0xFFFF);
            self.pan_id = PanId(0xFFFF);
            self.channel = 11;
            self.rx_on_when_idle = false;
            self.association_permit = false;
            self.auto_request = true;
            self.associated_pan_coord = false;
            self.dsn = random as u8;
            self.bsn = (random >> 8) as u8;
            self.max_frame_retries = 3;
            self.promiscuous = false;
            self.coord_short_address = ShortAddress::COORDINATOR;
            self.coord_extended_address = [0; 8];
            self.pending_assoc_frames.clear();
        }
        self.set_channel(self.channel);
        Ok(())
    }

    async fn mlme_start(&mut self, _req: MlmeStartRequest) -> Result<(), MacError> {
        Err(MacError::Unsupported)
    }

    async fn mlme_get(&self, attr: PibAttribute) -> Result<PibValue, MacError> {
        match attr {
            PibAttribute::MacShortAddress => Ok(PibValue::ShortAddress(self.short_address)),
            PibAttribute::MacPanId => Ok(PibValue::PanId(self.pan_id)),
            PibAttribute::MacExtendedAddress => {
                Ok(PibValue::ExtendedAddress(self.extended_address))
            }
            PibAttribute::MacCoordShortAddress => {
                Ok(PibValue::ShortAddress(self.coord_short_address))
            }
            PibAttribute::MacCoordExtendedAddress => {
                Ok(PibValue::ExtendedAddress(self.coord_extended_address))
            }
            PibAttribute::MacAssociatedPanCoord => Ok(PibValue::Bool(self.associated_pan_coord)),
            PibAttribute::MacRxOnWhenIdle => Ok(PibValue::Bool(self.rx_on_when_idle)),
            PibAttribute::MacAssociationPermit => Ok(PibValue::Bool(self.association_permit)),
            PibAttribute::MacAutoRequest => Ok(PibValue::Bool(self.auto_request)),
            PibAttribute::MacDsn => Ok(PibValue::U8(self.dsn)),
            PibAttribute::MacBsn => Ok(PibValue::U8(self.bsn)),
            PibAttribute::MacMaxFrameRetries => Ok(PibValue::U8(self.max_frame_retries)),
            PibAttribute::PhyCurrentChannel => Ok(PibValue::U8(self.channel)),
            PibAttribute::PhyTransmitPower => Ok(PibValue::I8(self.tx_power)),
            PibAttribute::PhyChannelsSupported => Ok(PibValue::U32(ChannelMask::ALL_2_4GHZ.0)),
            PibAttribute::MacPromiscuousMode => Ok(PibValue::Bool(self.promiscuous)),
            PibAttribute::MacBeaconPayload => Ok(PibValue::Payload(self.beacon_payload.clone())),
            PibAttribute::MacBeaconPayloadLength => {
                Ok(PibValue::U8(self.beacon_payload.as_slice().len() as u8))
            }
            PibAttribute::PhyCurrentPage => Ok(PibValue::U8(0)),
            _ => Err(MacError::Unsupported),
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
                self.set_channel(ch);
            }
            PibAttribute::MacPromiscuousMode => {
                self.promiscuous = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacBeaconPayload => {
                self.beacon_payload = match value {
                    PibValue::Payload(payload) => payload,
                    _ => return Err(MacError::InvalidParameter),
                };
            }
            PibAttribute::PhyTransmitPower => {
                let PibValue::I8(power) = value else {
                    return Err(MacError::InvalidParameter);
                };
                self.tx_power = power;
                self.radio.set_transmission_power(power);
            }
            PibAttribute::MacCoordShortAddress => {
                self.coord_short_address =
                    value.as_short_address().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacCoordExtendedAddress => {
                self.coord_extended_address = value
                    .as_extended_address()
                    .ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacAssociatedPanCoord => {
                self.associated_pan_coord = value.as_bool().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacExtendedAddress => {
                self.extended_address = value
                    .as_extended_address()
                    .ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacDsn => {
                self.dsn = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacBsn => {
                self.bsn = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            PibAttribute::MacMaxFrameRetries => {
                self.max_frame_retries = value.as_u8().ok_or(MacError::InvalidParameter)?;
            }
            _ => return Err(MacError::Unsupported),
        }
        Ok(())
    }

    async fn mlme_poll(&mut self) -> Result<Option<MacFrame>, MacError> {
        // Return any frame saved during association first (e.g. Transport-Key)
        if let Some(frame) = self.pending_assoc_frames.pop_front() {
            log::info!(
                "[MAC:Poll] Returning saved association frame ({} bytes)",
                frame.len()
            );
            return Ok(Some(frame));
        }

        let parent = MacAddress::Short(self.pan_id, self.coord_short_address);
        let has_short = self.short_address.0 != 0xFFFF && self.short_address.0 != 0xFFFE;

        // Try up to 2 poll passes:
        //   Pass 0: use SHORT address (if available) — matches most indirect frames
        //   Pass 1: use IEEE address — catches Transport-Key which EZSP may queue by IEEE
        // If we don't have a short address, only do one pass with IEEE.
        let passes: u8 = if has_short { 2 } else { 1 };

        for pass in 0..passes {
            let poll_dsn = self.next_dsn();
            let data_req = if pass == 0 && has_short {
                log::debug!(
                    "[MAC:Poll] pass {}: SHORT 0x{:04X}",
                    pass,
                    self.short_address.0
                );
                build_data_request_short(poll_dsn, &parent, self.short_address)
            } else {
                log::debug!("[MAC:Poll] pass {}: IEEE", pass);
                build_data_request(poll_dsn, &parent, &self.extended_address)
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
                            if data[2] != poll_dsn {
                                log::debug!(
                                    "[MAC:Poll] Ignoring stale ACK dsn={} expected={}",
                                    data[2],
                                    poll_dsn
                                );
                                continue;
                            }
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
        let dsn = self.next_dsn();
        let frame = build_data_frame(
            dsn,
            req.src_addr_mode,
            self.short_address,
            &self.extended_address,
            &req.dst_address,
            req.payload,
            ack_requested,
            req.tx_options.frame_pending,
        )
        .map_err(|_| MacError::FrameTooLong)?;

        if ack_requested {
            self.send_acknowledged_frame(&frame, self.max_frame_retries)
                .await?;
        } else {
            let mut packet = Packet::new();
            packet.copy_from_slice(&frame);
            self.try_send_with_csma(&mut packet).await?;
        }

        Ok(McpsDataConfirm {
            msdu_handle,
            timestamp: None,
        })
    }

    async fn mcps_data_indication(&mut self) -> Result<McpsDataIndication, MacError> {
        self.mcps_data_indication_timeout(1_000_000).await
    }

    async fn mcps_data_indication_timeout(
        &mut self,
        timeout_us: u32,
    ) -> Result<McpsDataIndication, MacError> {
        let mut rx_pkt = Packet::new();
        // Absolute deadline — filtered frames don't reset the clock
        let deadline =
            embassy_time::Instant::now() + embassy_time::Duration::from_micros(timeout_us as u64);

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
                    log::debug!("[nRF RX] Timeout ({}us) — no frame", timeout_us);
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

            // `Packet::lqi()` is the raw Nordic ED byte; the indication and
            // this log line must both report the IEEE 802.15.4 LQI.
            let lqi = crate::lqi::nrf_from_hardware(rx_pkt.lqi());
            log::info!("[nRF RX] Accepted frame {} bytes, LQI {}", data.len(), lqi);

            let payload_data = &data[header_len..];
            if let Some(mac_frame) = MacFrame::from_slice(payload_data) {
                return Ok(McpsDataIndication {
                    src_address: src,
                    dst_address: dst,
                    lqi,
                    payload: mac_frame,
                    security_use: (fc >> 3) & 1 != 0,
                });
            }
        }
    }

    fn capabilities(&self) -> MacCapabilities {
        MacCapabilities {
            coordinator: false,
            router: false,
            hardware_security: false,
            max_payload: 102,
            tx_power_min: TxPower(-20),
            tx_power_max: TxPower(8), // nRF52840: -20 to +8 dBm
        }
    }
}

impl<T: RadioInstance, R: RngInstance> zigbee_crypto::ForwardAesProvider for NrfMac<'_, T, R> {
    fn forward_cipher(
        &mut self,
        key: &zigbee_crypto::AesKey,
    ) -> impl zigbee_crypto::Aes128Forward + '_ {
        NrfHardwareAes128 {
            engine: self.aes.engine_mut(),
            key: *key,
        }
    }
}
impl<T: RadioInstance, R: RngInstance> PlatformServices for NrfMac<'_, T, R> {
    fn monotonic_micros(&self) -> u32 {
        embassy_time::Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(duration_us as u64).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.rng.blocking_fill_bytes(output);
        Ok(())
    }
}

impl<T: RadioInstance, R: RngInstance> NrfMac<'_, T, R> {
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
                    self.queue_pending_assoc_frame(payload);
                    log::info!(
                        "[MAC] Saved+ACKed data frame {} bytes during association",
                        payload.len()
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
                self.associated_pan_coord = status == AssociationStatus::Success;
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
                                            self.queue_pending_assoc_frame(pl);
                                            log::info!(
                                                "[MAC] Caught+ACKed post-assoc frame {} bytes!",
                                                pl.len()
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
