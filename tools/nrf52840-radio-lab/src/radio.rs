//! Promiscuous IEEE 802.15.4 receiver and raw transmitter on the nRF52840
//! RADIO.
//!
//! The register sequences mirror `zigbee_mac::nrf::NrfIsrRadioPhy` (MODE 15,
//! PCNF/CRC configuration, fast ramp-up, CCACTRL defaults and the
//! pre-programmed PPI channel RADIO.ADDRESS → TIMER0.CAPTURE[1]). That PHY is
//! not reused directly because a sniffer needs behaviour the MAC must not
//! have: every valid frame including foreign Imm-Acks, RSSI and a timestamp
//! per frame, TX without CCA, and no automatic acknowledgement. Keeping the
//! lab driver separate leaves the shipped router and end-device images
//! untouched.
//!
//! Reception is interrupt driven: `END → DISABLE` stops the radio after each
//! frame, the RADIO handler commits a valid frame to a ring of EasyDMA slots
//! and restarts reception into the next free slot. Transmission runs in task
//! context with the RADIO interrupt masked and is bounded by TIMER0
//! deadlines (at most ~12 ms including the ACK window), then reception
//! resumes.

use core::cell::{Cell, UnsafeCell};
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicUsize, Ordering};

use embassy_nrf::interrupt::Priority;
use embassy_nrf::interrupt::typelevel::{self, Interrupt as _};
use embassy_nrf::{Peripheral, PeripheralRef, peripherals};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use radio_lab_protocol::{self as protocol, Stats, TimestampExtender, TxStatus};

// ── Registers ────────────────────────────────────────────────

const RADIO: usize = 0x4000_1000;
const TASKS_TXEN: usize = RADIO;
const TASKS_RXEN: usize = RADIO + 0x004;
const TASKS_DISABLE: usize = RADIO + 0x010;
const EVENTS_READY: usize = RADIO + 0x100;
const EVENTS_ADDRESS: usize = RADIO + 0x104;
const EVENTS_END: usize = RADIO + 0x10C;
const EVENTS_DISABLED: usize = RADIO + 0x110;
const EVENTS_RSSIEND: usize = RADIO + 0x11C;
const EVENTS_CRCOK: usize = RADIO + 0x130;
const EVENTS_CRCERROR: usize = RADIO + 0x134;
const EVENTS_FRAMESTART: usize = RADIO + 0x138;
const EVENTS_CCAIDLE: usize = RADIO + 0x144;
const EVENTS_CCABUSY: usize = RADIO + 0x148;
const EVENTS_CCASTOPPED: usize = RADIO + 0x14C;
const EVENTS_TXREADY: usize = RADIO + 0x154;
const EVENTS_RXREADY: usize = RADIO + 0x158;
const EVENTS_PHYEND: usize = RADIO + 0x16C;
const SHORTS: usize = RADIO + 0x200;
const INTENSET: usize = RADIO + 0x304;
const INTENCLR: usize = RADIO + 0x308;
const CRCSTATUS: usize = RADIO + 0x400;
const PACKETPTR: usize = RADIO + 0x504;
const FREQUENCY: usize = RADIO + 0x508;
const TXPOWER: usize = RADIO + 0x50C;
const MODE: usize = RADIO + 0x510;
const PCNF0: usize = RADIO + 0x514;
const PCNF1: usize = RADIO + 0x518;
const CRCCNF: usize = RADIO + 0x534;
const CRCPOLY: usize = RADIO + 0x538;
const CRCINIT: usize = RADIO + 0x53C;
const RSSISAMPLE: usize = RADIO + 0x548;
const STATE: usize = RADIO + 0x550;
const MODECNF0: usize = RADIO + 0x650;
const SFD: usize = RADIO + 0x660;
const CCACTRL: usize = RADIO + 0x66C;
const POWER: usize = RADIO + 0xFFC;

const SHORT_END_DISABLE: u32 = 1 << 1;
const SHORT_ADDRESS_RSSISTART: u32 = 1 << 4;
const SHORT_RXREADY_CCASTART: u32 = 1 << 11;
const SHORT_CCAIDLE_TXEN: u32 = 1 << 12;
const SHORT_CCABUSY_DISABLE: u32 = 1 << 13;
const SHORT_TXREADY_START: u32 = 1 << 18;
const SHORT_RXREADY_START: u32 = 1 << 19;
const SHORT_PHYEND_DISABLE: u32 = 1 << 20;

const SHORTS_RX: u32 = SHORT_RXREADY_START | SHORT_END_DISABLE | SHORT_ADDRESS_RSSISTART;
const SHORTS_TX: u32 = SHORT_TXREADY_START | SHORT_PHYEND_DISABLE;
const SHORTS_CCA_TX: u32 = SHORT_RXREADY_CCASTART
    | SHORT_CCABUSY_DISABLE
    | SHORT_CCAIDLE_TXEN
    | SHORT_TXREADY_START
    | SHORT_PHYEND_DISABLE;

const INT_DISABLED: u32 = 1 << 4;
const STATE_DISABLED: u32 = 0;

/// MODE = Ieee802154_250Kbit.
const MODE_IEEE802154: u32 = 15;
/// LFLEN = 8 bits, PLEN = 32-bit zero preamble, CRCINC = include.
const PCNF0_IEEE802154: u32 = 8 | (2 << 24) | (1 << 26);
/// MAXLEN = 127 (aMaxPHYPacketSize).
const PCNF1_IEEE802154: u32 = 127;
/// LEN = 2 bytes, SKIPADDR = Ieee802154.
const CRCCNF_IEEE802154: u32 = 2 | (2 << 8);
const CRCPOLY_IEEE802154: u32 = 0x0001_1021;
const SFD_IEEE802154: u32 = 0xA7;
/// MODECNF0.RU = Fast (40 µs ramp-up), DTX = B1.
const MODECNF0_FAST_RAMP_UP: u32 = 1;
/// ED-threshold CCA (mode 1) with Nordic's driver defaults.
const CCACTRL_DEFAULT: u32 = (2 << 24) | (45 << 16) | (0x14 << 8);

const TIMER0: usize = 0x4000_8000;
const TIMER0_TASKS_START: usize = TIMER0;
const TIMER0_TASKS_STOP: usize = TIMER0 + 0x004;
const TIMER0_TASKS_CLEAR: usize = TIMER0 + 0x00C;
const TIMER0_TASKS_CAPTURE3: usize = TIMER0 + 0x04C;
const TIMER0_SHORTS: usize = TIMER0 + 0x200;
const TIMER0_INTENCLR: usize = TIMER0 + 0x308;
const TIMER0_MODE: usize = TIMER0 + 0x504;
const TIMER0_BITMODE: usize = TIMER0 + 0x508;
const TIMER0_PRESCALER: usize = TIMER0 + 0x510;
const TIMER0_CC1: usize = TIMER0 + 0x544;
const TIMER0_CC3: usize = TIMER0 + 0x54C;

const PPI: usize = 0x4001_F000;
const PPI_CHENSET: usize = PPI + 0x504;
/// Pre-programmed: RADIO.ADDRESS → TIMER0.CAPTURE[1] (end of SFD).
const PPI_CH_ADDRESS_CAPTURE1: u32 = 1 << 26;

const FICR_DEVICEID0: usize = 0x1000_0060;
const FICR_DEVICEID1: usize = 0x1000_0064;

// ── Deadlines (µs on TIMER0) ─────────────────────────────────

/// Radio state changes (disable, ramp-up) complete within a few µs.
const DISABLE_TIMEOUT_US: u32 = 200;
/// CCA (128 µs) + ramp-up + the longest PPDU (133 octets = 4256 µs).
const TX_TIMEOUT_US: u32 = 6_000;
/// macAckWaitDuration is 54 symbols (864 µs) after the frame; the ACK's SFD
/// must have arrived by then. Allow the RX ramp-up on top.
const ACK_START_TIMEOUT_US: u32 = 1_000;
/// Once an SFD is seen, allow the longest frame to finish.
const ACK_END_TIMEOUT_US: u32 = 4_500;

#[inline(always)]
fn read(address: usize) -> u32 {
    // SAFETY: fixed, always-mapped RADIO/TIMER0/PPI/FICR registers of the
    // nRF52840. Ownership of RADIO, TIMER0 and PPI channel 26 is held by
    // `Radio` through the Embassy singletons.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline(always)]
fn write(address: usize, value: u32) {
    // SAFETY: as for `read`.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

/// Clear an event and read it back so the clear reaches the peripheral
/// before a following task or interrupt return.
#[inline(always)]
fn clear_event(address: usize) {
    write(address, 0);
    let _ = read(address);
}

fn clear_all_events() {
    for event in [
        EVENTS_READY,
        EVENTS_ADDRESS,
        EVENTS_END,
        EVENTS_DISABLED,
        EVENTS_RSSIEND,
        EVENTS_CRCOK,
        EVENTS_CRCERROR,
        EVENTS_FRAMESTART,
        EVENTS_CCAIDLE,
        EVENTS_CCABUSY,
        EVENTS_CCASTOPPED,
        EVENTS_TXREADY,
        EVENTS_RXREADY,
        EVENTS_PHYEND,
    ] {
        clear_event(event);
    }
}

// ── Clock ────────────────────────────────────────────────────

static CLOCK: Mutex<CriticalSectionRawMutex, Cell<TimestampExtender>> =
    Mutex::new(Cell::new(TimestampExtender::new()));

/// Raw TIMER0 value. Only called inside a critical section or from the
/// RADIO handler, so CAPTURE[3]/CC[3] is never shared concurrently.
fn timer_raw() -> u32 {
    write(TIMER0_TASKS_CAPTURE3, 1);
    read(TIMER0_CC3)
}

/// 64-bit time of an earlier TIMER0 capture.
fn extend_capture(capture: u32) -> u64 {
    CLOCK.lock(|clock| {
        let mut extender = clock.get();
        let now = timer_raw();
        let now64 = extender.extend(now);
        clock.set(extender);
        TimestampExtender::earlier(now64, now, capture)
    })
}

/// Microseconds since boot on the clock used by `received:`/`transmitted:`.
pub fn now_us() -> u64 {
    CLOCK.lock(|clock| {
        let mut extender = clock.get();
        let now = extender.extend(timer_raw());
        clock.set(extender);
        now
    })
}

fn elapsed_since(start: u32) -> u32 {
    critical_section::with(|_| timer_raw()).wrapping_sub(start)
}

fn timer_start_mark() -> u32 {
    critical_section::with(|_| timer_raw())
}

/// Spin until `done()` or `timeout_us`; true when `done()` held.
fn spin_until(timeout_us: u32, mut done: impl FnMut() -> bool) -> bool {
    let start = timer_start_mark();
    loop {
        if done() {
            return true;
        }
        if elapsed_since(start) > timeout_us {
            return done();
        }
    }
}

// ── Receive ring ─────────────────────────────────────────────

const RING_SLOTS: usize = 32;

#[repr(C, align(4))]
#[derive(Clone, Copy)]
struct Slot {
    /// PHR followed by up to 127 PSDU octets (EasyDMA target).
    buffer: [u8; 1 + protocol::MAX_PSDU_LEN],
    rssi_dbm: i8,
    time_us: u64,
}

const EMPTY_SLOT: Slot = Slot {
    buffer: [0; 1 + protocol::MAX_PSDU_LEN],
    rssi_dbm: 0,
    time_us: 0,
};

struct Ring(UnsafeCell<[Slot; RING_SLOTS]>);

// SAFETY: slot `RX_WRITE` belongs to the radio/handler, slots in
// `RX_READ..RX_WRITE` to the consumer; the indices hand slots over with
// Release/Acquire ordering on this single-core MCU.
unsafe impl Sync for Ring {}

static RING: Ring = Ring(UnsafeCell::new([EMPTY_SLOT; RING_SLOTS]));
static RX_WRITE: AtomicUsize = AtomicUsize::new(0);
static RX_READ: AtomicUsize = AtomicUsize::new(0);
/// Whether the RADIO handler restarts reception after each frame.
static RX_ACTIVE: AtomicBool = AtomicBool::new(false);

static RX_COUNT: AtomicU32 = AtomicU32::new(0);
static CRC_ERRORS: AtomicU32 = AtomicU32::new(0);
static OVERFLOWS: AtomicU32 = AtomicU32::new(0);
static TX_COUNT: AtomicU32 = AtomicU32::new(0);

/// Raised when a frame is committed to the ring.
pub static RX_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

fn slot_ptr(index: usize) -> *mut Slot {
    // SAFETY: index < RING_SLOTS by construction of the ring indices.
    unsafe { (RING.0.get() as *mut Slot).add(index % RING_SLOTS) }
}

/// Point EasyDMA at the current write slot and start reception.
fn start_rx() {
    write(SHORTS, SHORTS_RX);
    // SAFETY: the write slot is owned by the radio until committed.
    let buffer = unsafe {
        (*slot_ptr(RX_WRITE.load(Ordering::Relaxed)))
            .buffer
            .as_ptr()
    };
    write(PACKETPTR, buffer as u32);
    clear_all_events();
    write(TASKS_RXEN, 1);
}

/// Commit the frame in the write slot if reception ended with a valid FCS.
/// Runs in the RADIO handler or with that interrupt masked.
fn complete_rx() {
    if read(EVENTS_END) == 0 {
        return;
    }
    clear_event(EVENTS_END);
    if read(CRCSTATUS) & 1 == 0 {
        CRC_ERRORS.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let write_index = RX_WRITE.load(Ordering::Relaxed);
    let slot = slot_ptr(write_index);
    // SAFETY: the write slot is owned by the radio until committed below.
    let length = unsafe { usize::from((*slot).buffer[0] & 0x7f) };
    if length < 1 + protocol::FCS_LEN {
        return;
    }
    let next = (write_index + 1) % RING_SLOTS;
    if next == RX_READ.load(Ordering::Acquire) {
        OVERFLOWS.fetch_add(1, Ordering::Relaxed);
        return;
    }
    let rssi_dbm = protocol::rssi_dbm(read(RSSISAMPLE));
    let time_us = extend_capture(read(TIMER0_CC1));
    // SAFETY: still the radio-owned write slot.
    unsafe {
        (*slot).rssi_dbm = rssi_dbm;
        (*slot).time_us = time_us;
    }
    RX_WRITE.store(next, Ordering::Release);
    RX_COUNT.fetch_add(1, Ordering::Relaxed);
    RX_SIGNAL.signal(());
}

/// Stop the radio and wait for DISABLED (bounded).
fn disable() {
    write(SHORTS, 0);
    if read(STATE) != STATE_DISABLED {
        write(TASKS_DISABLE, 1);
        spin_until(DISABLE_TIMEOUT_US, || read(STATE) == STATE_DISABLED);
    }
}

/// RADIO interrupt handler; bind with `RADIO => radio::RadioIrq`.
pub struct RadioIrq {
    _private: (),
}

impl typelevel::Handler<typelevel::RADIO> for RadioIrq {
    unsafe fn on_interrupt() {
        if read(EVENTS_DISABLED) == 0 {
            return;
        }
        clear_event(EVENTS_DISABLED);
        if !RX_ACTIVE.load(Ordering::Relaxed) {
            return;
        }
        complete_rx();
        start_rx();
    }
}

/// One frame taken from the ring.
pub struct RxFrame {
    /// PSDU including a recomputed FCS.
    pub psdu: [u8; protocol::MAX_PSDU_LEN],
    pub len: usize,
    pub rssi_dbm: i8,
    pub lqi: u8,
    pub time_us: u64,
}

impl RxFrame {
    pub fn psdu(&self) -> &[u8] {
        &self.psdu[..self.len]
    }
}

// ── Transmit buffer ──────────────────────────────────────────

struct TxBuffer(UnsafeCell<[u8; 1 + protocol::MAX_PSDU_LEN]>);

// SAFETY: only `Radio::transmit` (one owner, RADIO interrupt masked) touches it.
unsafe impl Sync for TxBuffer {}

static TX_BUFFER: TxBuffer = TxBuffer(UnsafeCell::new([0; 1 + protocol::MAX_PSDU_LEN]));

// ── Driver ───────────────────────────────────────────────────

/// Owner of RADIO, TIMER0 and PPI channel 26.
pub struct Radio<'d> {
    _radio: PeripheralRef<'d, peripherals::RADIO>,
    _timer: PeripheralRef<'d, peripherals::TIMER0>,
    _address: PeripheralRef<'d, peripherals::PPI_CH26>,
    channel: u8,
    power_dbm: i8,
    receiving: bool,
}

impl<'d> Radio<'d> {
    /// Configure IEEE 802.15.4 mode, start the 1 MHz TIMER0 clock and enable
    /// the RADIO interrupt. The radio starts disabled (Nordic's `sleep`).
    ///
    /// HFXO must already be running (`HfclkSource::ExternalXtal`).
    pub fn new(
        radio: impl Peripheral<P = peripherals::RADIO> + 'd,
        timer: impl Peripheral<P = peripherals::TIMER0> + 'd,
        address: impl Peripheral<P = peripherals::PPI_CH26> + 'd,
        _irq: impl typelevel::Binding<typelevel::RADIO, RadioIrq> + 'd,
    ) -> Self {
        typelevel::RADIO::disable();

        write(POWER, 0);
        write(POWER, 1);
        write(MODE, MODE_IEEE802154);
        write(MODECNF0, MODECNF0_FAST_RAMP_UP);
        write(CRCCNF, CRCCNF_IEEE802154);
        write(CRCPOLY, CRCPOLY_IEEE802154);
        write(CRCINIT, 0);
        write(PCNF0, PCNF0_IEEE802154);
        write(PCNF1, PCNF1_IEEE802154);
        write(SFD, SFD_IEEE802154);
        write(CCACTRL, CCACTRL_DEFAULT);
        write(SHORTS, 0);
        write(INTENCLR, 0xFFFF_FFFF);
        write(INTENSET, INT_DISABLED);

        write(TIMER0_TASKS_STOP, 1);
        write(TIMER0_INTENCLR, 0xFFFF_FFFF);
        write(TIMER0_SHORTS, 0);
        write(TIMER0_MODE, 0);
        write(TIMER0_BITMODE, 3);
        write(TIMER0_PRESCALER, 4);
        write(TIMER0_TASKS_CLEAR, 1);
        write(TIMER0_TASKS_START, 1);
        write(PPI_CHENSET, PPI_CH_ADDRESS_CAPTURE1);

        let radio = Self {
            _radio: radio.into_ref(),
            _timer: timer.into_ref(),
            _address: address.into_ref(),
            channel: protocol::DEFAULT_CHANNEL,
            power_dbm: protocol::DEFAULT_TX_POWER_DBM,
            receiving: false,
        };
        radio.apply_channel();
        radio.apply_power();
        clear_all_events();

        typelevel::RADIO::set_priority(Priority::P0);
        typelevel::RADIO::unpend();
        // SAFETY: the handler is bound through `_irq` and only touches the
        // statics initialised above.
        unsafe { typelevel::RADIO::enable() };
        radio
    }

    pub fn channel(&self) -> u8 {
        self.channel
    }

    pub fn power_dbm(&self) -> i8 {
        self.power_dbm
    }

    pub fn receiving(&self) -> bool {
        self.receiving
    }

    fn apply_channel(&self) {
        if let Some(frequency) = protocol::channel_frequency(self.channel) {
            write(FREQUENCY, frequency);
        }
    }

    fn apply_power(&self) {
        if let Some(value) = protocol::tx_power_register(self.power_dbm) {
            write(TXPOWER, value);
        }
    }

    /// Mask the handler and stop the radio, keeping a frame that completed
    /// while the interrupt was pending.
    fn pause(&mut self) {
        typelevel::RADIO::disable();
        RX_ACTIVE.store(false, Ordering::Relaxed);
        disable();
        if self.receiving {
            complete_rx();
        }
        clear_all_events();
        typelevel::RADIO::unpend();
    }

    /// Restart reception if it was on, then unmask the handler.
    fn resume(&mut self) {
        clear_all_events();
        if self.receiving {
            RX_ACTIVE.store(true, Ordering::Relaxed);
            start_rx();
        }
        typelevel::RADIO::unpend();
        // SAFETY: as in `new`.
        unsafe { typelevel::RADIO::enable() };
    }

    /// Promiscuous reception (no address filter, no auto-ACK).
    pub fn receive(&mut self) {
        self.pause();
        self.receiving = true;
        self.resume();
    }

    /// Disable the radio.
    pub fn sleep(&mut self) {
        self.pause();
        self.receiving = false;
        self.resume();
    }

    /// Retune; reception continues on the new channel.
    pub fn set_channel(&mut self, channel: u8) {
        self.pause();
        self.channel = channel;
        self.apply_channel();
        self.resume();
    }

    /// Set the TX power (a validated `protocol::TX_POWER_LEVELS_DBM` value).
    pub fn set_power(&mut self, dbm: i8) {
        self.pause();
        self.power_dbm = dbm;
        self.apply_power();
        self.resume();
    }

    /// Counters since reset.
    pub fn stats(&self) -> Stats {
        Stats {
            rx: RX_COUNT.load(Ordering::Relaxed),
            crc_error: CRC_ERRORS.load(Ordering::Relaxed),
            overflow: OVERFLOWS.load(Ordering::Relaxed),
            tx: TX_COUNT.load(Ordering::Relaxed),
        }
    }

    /// Take the oldest received frame.
    pub fn next_frame(&mut self) -> Option<RxFrame> {
        let read_index = RX_READ.load(Ordering::Relaxed);
        if read_index == RX_WRITE.load(Ordering::Acquire) {
            return None;
        }
        // SAFETY: slots in RX_READ..RX_WRITE are owned by the consumer.
        let slot = unsafe { *slot_ptr(read_index) };
        RX_READ.store((read_index + 1) % RING_SLOTS, Ordering::Release);

        let len = usize::from(slot.buffer[0] & 0x7f);
        let mut frame = RxFrame {
            psdu: [0; protocol::MAX_PSDU_LEN],
            len,
            rssi_dbm: slot.rssi_dbm,
            // The RADIO writes the energy-detect octet over the first FCS
            // octet; restore the real FCS for the capture.
            lqi: protocol::lqi_from_hardware(slot.buffer[len - 1]),
            time_us: slot.time_us,
        };
        frame.psdu[..len].copy_from_slice(&slot.buffer[1..=len]);
        protocol::append_fcs(&mut frame.psdu[..len]);
        Some(frame)
    }

    /// Send `psdu` (without FCS) once. With `ack_sequence`, listen for the
    /// matching Imm-Ack afterwards; frames heard in that window are also
    /// queued for output when reception is on. Returns the outcome and the
    /// end-of-SFD time of the transmission (or the time of the failure).
    pub fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
        ack_sequence: Option<u8>,
    ) -> (TxStatus, u64) {
        if psdu.is_empty() || psdu.len() > protocol::MAX_TX_LEN {
            return (TxStatus::Error, now_us());
        }
        self.pause();

        // SAFETY: the RADIO interrupt is masked and the radio is disabled, so
        // nothing else reads or DMAs the TX buffer.
        let buffer = unsafe { &mut *TX_BUFFER.0.get() };
        buffer[0] = (psdu.len() + protocol::FCS_LEN) as u8;
        buffer[1..=psdu.len()].copy_from_slice(psdu);
        write(PACKETPTR, buffer.as_ptr() as u32);
        clear_all_events();
        if cca {
            write(SHORTS, SHORTS_CCA_TX);
            write(TASKS_RXEN, 1);
        } else {
            write(SHORTS, SHORTS_TX);
            write(TASKS_TXEN, 1);
        }
        let finished = spin_until(TX_TIMEOUT_US, || read(EVENTS_DISABLED) != 0);

        let mut status = if !finished {
            disable();
            TxStatus::Error
        } else if read(EVENTS_CCABUSY) != 0 {
            TxStatus::CcaBusy
        } else if read(EVENTS_PHYEND) != 0 {
            TxStatus::Sent
        } else {
            TxStatus::Error
        };
        let time_us = if status == TxStatus::Sent {
            TX_COUNT.fetch_add(1, Ordering::Relaxed);
            extend_capture(read(TIMER0_CC1))
        } else {
            now_us()
        };

        if status == TxStatus::Sent
            && let Some(sequence) = ack_sequence
        {
            status = if self.wait_ack(sequence) {
                TxStatus::Acked
            } else {
                TxStatus::NoAck
            };
        }

        self.resume();
        (status, time_us)
    }

    /// Receive into the current ring slot for one acknowledgement window.
    fn wait_ack(&mut self, sequence: u8) -> bool {
        clear_all_events();
        write(SHORTS, SHORTS_RX);
        let slot = slot_ptr(RX_WRITE.load(Ordering::Relaxed));
        // SAFETY: the write slot is owned by the radio; the interrupt is
        // masked, so only this function uses it until `complete_rx`.
        write(PACKETPTR, unsafe { (*slot).buffer.as_ptr() } as u32);
        write(TASKS_RXEN, 1);

        let started = spin_until(ACK_START_TIMEOUT_US, || read(EVENTS_ADDRESS) != 0);
        let ended = started && spin_until(ACK_END_TIMEOUT_US, || read(EVENTS_DISABLED) != 0);
        if !ended {
            disable();
            return false;
        }
        // SAFETY: as above; reception has ended.
        let buffer = unsafe { &(*slot).buffer };
        let len = usize::from(buffer[0] & 0x7f);
        let acked = read(CRCSTATUS) & 1 != 0
            && len == 3 + protocol::FCS_LEN
            && protocol::is_ack_for(&buffer[1..4], sequence);
        if self.receiving {
            complete_rx();
        }
        acked
    }
}

/// USB serial number of this chip in Zephyr's format.
pub fn usb_serial() -> [u8; 16] {
    protocol::usb_serial([read(FICR_DEVICEID0), read(FICR_DEVICEID1)])
}
