//! Interrupt-driven nRF52833/nRF52840 [`RadioPhy`] with hardware-timed
//! Imm-Ack, for the router (parent) and always-on end-device roles.
//!
//! Unlike [`super::NrfRadioPhy`] (Embassy's task-driven radio, software ACK
//! from the MAC task), this PHY owns the RADIO interrupt. The handler
//! classifies every received frame with [`crate::nrf_ack::classify`] and,
//! when the frame requests an ACK and names this node exactly, schedules the
//! Imm-Ack in hardware:
//!
//! ```text
//! RADIO END ──PPI CH27──▶ TIMER0 CAPTURE[2]            (rx_end, always on)
//! RADIO END ──SHORT────▶ DISABLE ──▶ DISABLED IRQ        (classify, FP lookup)
//! TIMER0 CC[0] = rx_end + 192 µs − 40 µs TX ramp-up − 23 µs END latency
//! TIMER0 COMPARE[0] ──PPI CH20──▶ RADIO TXEN ──SHORT──▶ START   (ACK on air)
//! RADIO ADDRESS ──PPI CH26──▶ TIMER0 CAPTURE[1]        (measured turnaround)
//! ```
//!
//! The ACK therefore starts aTurnaroundTime (192 µs) after the received
//! frame's last bit, independent of task scheduling, with the same ramp-up
//! and END-latency constants as Nordic's `nrf_802154_trx_transmit_ack`. If the handler runs so late that the
//! compare can no longer be met, the ACK is abandoned (the sender retries)
//! and `acks_late` is incremented — an ACK is never sent at an undefined
//! time.
//!
//! Frame Pending comes from a bounded source-address table armed by the
//! parent role through [`RadioPhy::set_ack_frame_pending`] (the
//! `Parent<EVENTS, ASSOC, 0>` model); a full table is reported, never
//! truncated.
//!
//! Transmit is one CCA-gated attempt (`RXEN → CCASTART → TXEN → START`) and,
//! for acknowledged frames, an RX window matched against the sequence number
//! in the handler ([`crate::AckWaitTx`]). CSMA backoff and retries stay in
//! `SoftMacCore`.
//!
//! Peripheral ownership: RADIO, TIMER0 (1 MHz free-running) and the
//! pre-programmed PPI channels 20, 26 and 27 are taken as Embassy singletons.
//! The Embassy time driver must use an RTC (`time-driver-rtc1`).

use core::cell::{RefCell, UnsafeCell};
use core::future::poll_fn;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_futures::select::{Either, select};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Instant, Timer};
use zigbee_types::{IeeeAddress, MacAddress};

use super::embassy_nrf::interrupt::typelevel::{self, Interrupt as _};
use super::embassy_nrf::{Peripheral, PeripheralRef, into_ref, peripherals};
use super::{NrfAesError, NrfAesState, NrfEcbToken, NrfHardwareAes128, Rng, RngInstance};
use crate::nrf_ack::{
    NrfRadioCounters, PendingTable, RxClass, ack_dma_image, ack_turnaround, ack_txen_compare,
    classify,
};
use crate::pib::MacPib;
use crate::{
    AckResult, EndDevice, MAX_PHY_FRAME_LEN, MacError, Parent, PhyAddressFilter, PhyCapabilities,
    PhyError, PhyRxFrame, PlatformServices, RadioPhy, SoftMacCore,
};

// ── Registers ────────────────────────────────────────────────

const RADIO: usize = 0x4000_1000;
const RADIO_TASKS_RXEN: usize = RADIO + 0x004;
const RADIO_TASKS_STOP: usize = RADIO + 0x00C;
const RADIO_TASKS_DISABLE: usize = RADIO + 0x010;
const RADIO_TASKS_CCASTOP: usize = RADIO + 0x030;
const RADIO_EVENTS_READY: usize = RADIO + 0x100;
const RADIO_EVENTS_ADDRESS: usize = RADIO + 0x104;
const RADIO_EVENTS_END: usize = RADIO + 0x10C;
const RADIO_EVENTS_DISABLED: usize = RADIO + 0x110;
const RADIO_EVENTS_CRCOK: usize = RADIO + 0x130;
const RADIO_EVENTS_CRCERROR: usize = RADIO + 0x134;
const RADIO_EVENTS_FRAMESTART: usize = RADIO + 0x138;
const RADIO_EVENTS_CCAIDLE: usize = RADIO + 0x144;
const RADIO_EVENTS_CCABUSY: usize = RADIO + 0x148;
const RADIO_EVENTS_TXREADY: usize = RADIO + 0x154;
const RADIO_EVENTS_RXREADY: usize = RADIO + 0x158;
const RADIO_EVENTS_PHYEND: usize = RADIO + 0x16C;
const RADIO_SHORTS: usize = RADIO + 0x200;
const RADIO_INTENSET: usize = RADIO + 0x304;
const RADIO_INTENCLR: usize = RADIO + 0x308;
const RADIO_CRCSTATUS: usize = RADIO + 0x400;
const RADIO_PACKETPTR: usize = RADIO + 0x504;
const RADIO_FREQUENCY: usize = RADIO + 0x508;
const RADIO_TXPOWER: usize = RADIO + 0x50C;
const RADIO_MODE: usize = RADIO + 0x510;
const RADIO_PCNF0: usize = RADIO + 0x514;
const RADIO_PCNF1: usize = RADIO + 0x518;
const RADIO_CRCCNF: usize = RADIO + 0x534;
const RADIO_CRCPOLY: usize = RADIO + 0x538;
const RADIO_CRCINIT: usize = RADIO + 0x53C;
const RADIO_STATE: usize = RADIO + 0x550;
const RADIO_MODECNF0: usize = RADIO + 0x650;
const RADIO_SFD: usize = RADIO + 0x660;
const RADIO_CCACTRL: usize = RADIO + 0x66C;
const RADIO_POWER: usize = RADIO + 0xFFC;

const SHORT_END_DISABLE: u32 = 1 << 1;
const SHORT_RXREADY_CCASTART: u32 = 1 << 11;
const SHORT_CCAIDLE_TXEN: u32 = 1 << 12;
const SHORT_CCABUSY_DISABLE: u32 = 1 << 13;
const SHORT_TXREADY_START: u32 = 1 << 18;
const SHORT_RXREADY_START: u32 = 1 << 19;
const SHORT_PHYEND_DISABLE: u32 = 1 << 20;

const SHORTS_RX: u32 = SHORT_RXREADY_START | SHORT_END_DISABLE;
const SHORTS_TX_ACK: u32 = SHORT_TXREADY_START | SHORT_PHYEND_DISABLE;
const SHORTS_CCA_TX: u32 = SHORT_RXREADY_CCASTART
    | SHORT_CCABUSY_DISABLE
    | SHORT_CCAIDLE_TXEN
    | SHORT_TXREADY_START
    | SHORT_PHYEND_DISABLE;

const INT_DISABLED: u32 = 1 << 4;

const STATE_DISABLED: u32 = 0;
const STATE_RXDISABLE: u32 = 4;

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
/// MODECNF0.RU = Fast (40 µs ramp-up), DTX = B1 — as in Nordic's driver.
const MODECNF0_FAST_RAMP_UP: u32 = 1;
/// Nordic 802.15.4 driver defaults: ED mode, ED threshold 0x14,
/// correlator threshold 45, correlator limit 2.
const CCACTRL_DEFAULT: u32 = (2 << 24) | (45 << 16) | (0x14 << 8);

const TIMER0: usize = 0x4000_8000;
const TIMER0_TASKS_START: usize = TIMER0;
const TIMER0_TASKS_STOP: usize = TIMER0 + 0x004;
const TIMER0_TASKS_CLEAR: usize = TIMER0 + 0x00C;
const TIMER0_TASKS_CAPTURE3: usize = TIMER0 + 0x04C;
const TIMER0_EVENTS_COMPARE0: usize = TIMER0 + 0x140;
const TIMER0_SHORTS: usize = TIMER0 + 0x200;
const TIMER0_INTENCLR: usize = TIMER0 + 0x308;
const TIMER0_MODE: usize = TIMER0 + 0x504;
const TIMER0_BITMODE: usize = TIMER0 + 0x508;
const TIMER0_PRESCALER: usize = TIMER0 + 0x510;
const TIMER0_CC0: usize = TIMER0 + 0x540;
const TIMER0_CC1: usize = TIMER0 + 0x544;
const TIMER0_CC2: usize = TIMER0 + 0x548;
const TIMER0_CC3: usize = TIMER0 + 0x54C;

const PPI: usize = 0x4001_F000;
const PPI_CHENSET: usize = PPI + 0x504;
const PPI_CHENCLR: usize = PPI + 0x508;
/// Pre-programmed: TIMER0 COMPARE[0] → RADIO TXEN.
const PPI_CH_TIMER0_CC0_TXEN: u32 = 1 << 20;
/// Pre-programmed: RADIO ADDRESS → TIMER0 CAPTURE[1].
const PPI_CH_ADDRESS_CAPTURE1: u32 = 1 << 26;
/// Pre-programmed: RADIO END → TIMER0 CAPTURE[2].
const PPI_CH_END_CAPTURE2: u32 = 1 << 27;

const DISABLE_SPIN_LIMIT: u32 = 10_000;

#[inline(always)]
fn read(address: usize) -> u32 {
    // SAFETY: fixed, always-mapped RADIO/TIMER0/PPI registers of the
    // nRF52833/nRF52840. Ownership is established by the Embassy singletons
    // held by `NrfIsrRadioPhy`; every read-modify sequence runs inside the
    // `SHARED` critical section or in the RADIO handler.
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

fn timer_now() -> u32 {
    write(TIMER0_TASKS_CAPTURE3, 1);
    read(TIMER0_CC3)
}

// ── EasyDMA buffers ──────────────────────────────────────────

/// RX ring depth of the PHY (independent of the core's retained queue).
///
/// A router hears every neighbour relay one broadcast within a few tens of
/// milliseconds, and the task processes retained frames one at a time. On a
/// 14-neighbour network an 8-slot ring overflowed on about 18 % of received
/// frames (`NrfRadioCounters::rx_overflow`), so the ring is sized for such
/// bursts (128 B per slot).
pub const NRF_ISR_RX_SLOTS: usize = 24;
/// PHR + 127-octet PSDU. The RADIO writes the LQI over the first FCS octet.
const DMA_FRAME_LEN: usize = 128;

#[repr(C, align(4))]
struct DmaBuffers {
    rx: [[u8; DMA_FRAME_LEN]; NRF_ISR_RX_SLOTS],
    tx: [u8; DMA_FRAME_LEN],
    ack: [u8; 4],
}

struct DmaCell(UnsafeCell<DmaBuffers>);

// SAFETY: slot ownership is arbitrated through `SHARED`: the RADIO only
// writes the single uncommitted RX slot, the task only reads committed
// slots, and the TX/ACK images are written only while the RADIO is
// DISABLED inside the critical section.
unsafe impl Sync for DmaCell {}

static DMA: DmaCell = DmaCell(UnsafeCell::new(DmaBuffers {
    rx: [[0; DMA_FRAME_LEN]; NRF_ISR_RX_SLOTS],
    tx: [0; DMA_FRAME_LEN],
    ack: [0; 4],
}));

fn rx_slot(index: usize) -> *mut u8 {
    // SAFETY: in-bounds projection of the static buffer; no reference is
    // created.
    unsafe { (&raw mut (*DMA.0.get()).rx[index]).cast() }
}

fn tx_buffer() -> *mut u8 {
    // SAFETY: as for `rx_slot`.
    unsafe { (&raw mut (*DMA.0.get()).tx).cast() }
}

fn ack_buffer() -> *mut u8 {
    // SAFETY: as for `rx_slot`.
    unsafe { (&raw mut (*DMA.0.get()).ack).cast() }
}

// ── Shared ISR/task state ────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// RADIO DISABLED, nothing armed.
    Off,
    /// Receiving into the uncommitted ring slot.
    Rx,
    /// Imm-Ack armed on TIMER0 COMPARE[0] or on air.
    AckTx,
    /// CCA-gated transmission of the TX buffer.
    Tx,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TxOutcome {
    Sent,
    Busy,
}

struct Shared {
    mode: Mode,
    rx_on_when_idle: bool,
    receive_active: bool,
    sleeping: bool,
    filter: Option<PhyAddressFilter>,
    pending: PendingTable,
    head: usize,
    count: usize,
    awaiting_ack: Option<u8>,
    ack_received: Option<bool>,
    tx_outcome: Option<TxOutcome>,
    rx_end: u32,
    ack_txen_at: u32,
    ack_frame_pending: bool,
    counters: NrfRadioCounters,
}

impl Shared {
    const fn new() -> Self {
        Self {
            mode: Mode::Off,
            rx_on_when_idle: false,
            receive_active: false,
            sleeping: false,
            filter: None,
            pending: PendingTable::new(),
            head: 0,
            count: 0,
            awaiting_ack: None,
            ack_received: None,
            tx_outcome: None,
            rx_end: 0,
            ack_txen_at: 0,
            ack_frame_pending: false,
            counters: NrfRadioCounters::new(),
        }
    }

    fn listening(&self) -> bool {
        !self.sleeping
            && (self.rx_on_when_idle || self.receive_active || self.awaiting_ack.is_some())
    }

    /// Start RX into the uncommitted slot. The RADIO must be DISABLED.
    fn start_rx(&mut self) {
        let slot = (self.head + self.count) % NRF_ISR_RX_SLOTS;
        write(RADIO_PACKETPTR, rx_slot(slot) as u32);
        for event in [
            RADIO_EVENTS_READY,
            RADIO_EVENTS_RXREADY,
            RADIO_EVENTS_ADDRESS,
            RADIO_EVENTS_FRAMESTART,
            RADIO_EVENTS_END,
            RADIO_EVENTS_CRCOK,
            RADIO_EVENTS_CRCERROR,
        ] {
            write(event, 0);
        }
        clear_event(RADIO_EVENTS_DISABLED);
        write(RADIO_SHORTS, SHORTS_RX);
        self.mode = Mode::Rx;
        compiler_fence(Ordering::SeqCst);
        write(RADIO_TASKS_RXEN, 1);
    }

    /// Continue receiving or stay off. The RADIO must be DISABLED.
    fn rx_or_off(&mut self) {
        if self.listening() {
            self.start_rx();
        } else {
            self.mode = Mode::Off;
        }
    }

    /// Force DISABLED from any state and swallow the resulting event.
    fn halt(&mut self) -> bool {
        write(PPI_CHENCLR, PPI_CH_TIMER0_CC0_TXEN);
        let disabled = force_disabled();
        self.mode = Mode::Off;
        disabled
    }

    fn on_disabled(&mut self) -> bool {
        match self.mode {
            Mode::Off => false,
            Mode::Rx => self.on_rx_end(),
            Mode::AckTx => {
                write(PPI_CHENCLR, PPI_CH_TIMER0_CC0_TXEN);
                if read(RADIO_EVENTS_PHYEND) != 0 {
                    let turnaround = ack_turnaround(self.rx_end, read(TIMER0_CC1));
                    self.counters.record_ack(turnaround, self.ack_frame_pending);
                }
                self.rx_or_off();
                false
            }
            Mode::Tx => {
                let outcome = if read(RADIO_EVENTS_CCABUSY) != 0 {
                    TxOutcome::Busy
                } else if read(RADIO_EVENTS_PHYEND) != 0 {
                    TxOutcome::Sent
                } else if read(RADIO_STATE) != STATE_DISABLED {
                    // Intermediate DISABLED of the RX→TX turnaround: keep
                    // waiting for PHYEND.
                    return false;
                } else {
                    TxOutcome::Busy
                };
                match outcome {
                    TxOutcome::Sent => {
                        self.counters.tx_frames = self.counters.tx_frames.wrapping_add(1);
                    }
                    TxOutcome::Busy => {
                        self.counters.tx_busy = self.counters.tx_busy.wrapping_add(1);
                        self.awaiting_ack = None;
                    }
                }
                self.tx_outcome = Some(outcome);
                // An acknowledged transmission keeps listening for its ACK.
                self.rx_or_off();
                true
            }
        }
    }

    fn on_rx_end(&mut self) -> bool {
        if read(RADIO_EVENTS_END) == 0 {
            self.rx_or_off();
            return false;
        }
        write(RADIO_EVENTS_END, 0);
        compiler_fence(Ordering::SeqCst);
        if read(RADIO_CRCSTATUS) & 1 == 0 {
            self.counters.crc_errors = self.counters.crc_errors.wrapping_add(1);
            self.rx_or_off();
            return false;
        }
        let slot = rx_slot((self.head + self.count) % NRF_ISR_RX_SLOTS);
        // SAFETY: the uncommitted slot was just completed by EasyDMA (END,
        // RADIO now DISABLED); nothing else accesses it until it is
        // committed or reused for the next reception.
        let phr = usize::from(unsafe { slot.read() } & 0x7F);
        if phr < 5 {
            self.rx_or_off();
            return false;
        }
        // SAFETY: as above; `phr - 2 <= 125` bytes after the PHR byte.
        let frame = unsafe { core::slice::from_raw_parts(slot.add(1), phr - 2) };
        self.counters.rx_frames = self.counters.rx_frames.wrapping_add(1);
        match classify(frame, self.filter.as_ref(), &self.pending) {
            RxClass::Drop => {
                self.rx_or_off();
                false
            }
            RxClass::Ack {
                sequence,
                frame_pending,
            } => {
                let matched = self.awaiting_ack == Some(sequence);
                if matched {
                    self.awaiting_ack = None;
                    self.ack_received = Some(frame_pending);
                }
                self.rx_or_off();
                matched
            }
            RxClass::Queue { ack } => {
                if self.count >= NRF_ISR_RX_SLOTS - 1 {
                    // No slot to receive into after this one: drop it and do
                    // not acknowledge, so the sender retries.
                    self.counters.rx_overflow = self.counters.rx_overflow.wrapping_add(1);
                    self.rx_or_off();
                    return true;
                }
                self.count += 1;
                match ack {
                    Some((sequence, frame_pending))
                        if self.schedule_ack(sequence, frame_pending) => {}
                    _ => self.rx_or_off(),
                }
                true
            }
        }
    }

    /// Arm TIMER0 COMPARE[0] → TXEN for an ACK `TURNAROUND_US` after the
    /// frame end captured by PPI. The RADIO is DISABLED.
    fn schedule_ack(&mut self, sequence: u8, frame_pending: bool) -> bool {
        let rx_end = read(TIMER0_CC2);
        let Some(compare) = ack_txen_compare(rx_end, timer_now()) else {
            self.counters.acks_late = self.counters.acks_late.wrapping_add(1);
            return false;
        };
        let image = ack_dma_image(sequence, frame_pending);
        // SAFETY: the ACK image is only touched while the RADIO is DISABLED,
        // inside the critical section.
        unsafe { core::ptr::copy_nonoverlapping(image.as_ptr(), ack_buffer(), image.len()) };
        write(RADIO_PACKETPTR, ack_buffer() as u32);
        for event in [
            RADIO_EVENTS_READY,
            RADIO_EVENTS_TXREADY,
            RADIO_EVENTS_ADDRESS,
            RADIO_EVENTS_PHYEND,
            RADIO_EVENTS_END,
        ] {
            write(event, 0);
        }
        clear_event(RADIO_EVENTS_DISABLED);
        write(RADIO_SHORTS, SHORTS_TX_ACK);
        write(TIMER0_CC0, compare);
        clear_event(TIMER0_EVENTS_COMPARE0);
        compiler_fence(Ordering::SeqCst);
        write(PPI_CHENSET, PPI_CH_TIMER0_CC0_TXEN);
        self.rx_end = rx_end;
        self.ack_txen_at = compare;
        self.ack_frame_pending = frame_pending;
        self.mode = Mode::AckTx;
        // Defensive: if the compare already passed while it was being armed
        // and TXEN did not fire, abandon instead of hanging in AckTx.
        if (timer_now().wrapping_sub(compare) as i32) >= 0
            && read(RADIO_STATE) == STATE_DISABLED
            && read(RADIO_EVENTS_TXREADY) == 0
        {
            write(PPI_CHENCLR, PPI_CH_TIMER0_CC0_TXEN);
            self.counters.acks_late = self.counters.acks_late.wrapping_add(1);
            return false;
        }
        true
    }

    /// Recover from an Imm-Ack that never completed (defensive).
    fn recover_stalled_ack(&mut self) {
        const ACK_STALL_US: i32 = 2_000;
        if self.mode == Mode::AckTx
            && (timer_now().wrapping_sub(self.ack_txen_at) as i32) > ACK_STALL_US
        {
            self.counters.acks_late = self.counters.acks_late.wrapping_add(1);
            self.halt();
            self.rx_or_off();
        }
    }

    fn begin_tx(&mut self, frame: &[u8], ack: Option<u8>) -> Result<(), PhyError> {
        self.recover_stalled_ack();
        match self.mode {
            Mode::AckTx | Mode::Tx => return Err(self.busy()),
            Mode::Rx => {
                let state = read(RADIO_STATE);
                // A frame is being received, or its end is still pending in
                // the handler: the channel is busy.
                if read(RADIO_EVENTS_ADDRESS) != 0
                    || state == STATE_DISABLED
                    || state == STATE_RXDISABLE
                {
                    return Err(self.busy());
                }
            }
            Mode::Off => {}
        }
        if self.sleeping {
            return Err(PhyError::Hardware);
        }
        if !self.halt() {
            return Err(PhyError::Hardware);
        }
        let tx = tx_buffer();
        // SAFETY: RADIO is DISABLED and we are inside the critical section;
        // `frame.len() <= MAX_PHY_FRAME_LEN` (checked by the caller).
        unsafe {
            tx.write(frame.len() as u8 + 2);
            core::ptr::copy_nonoverlapping(frame.as_ptr(), tx.add(1), frame.len());
        }
        write(RADIO_PACKETPTR, tx as u32);
        for event in [
            RADIO_EVENTS_READY,
            RADIO_EVENTS_RXREADY,
            RADIO_EVENTS_CCAIDLE,
            RADIO_EVENTS_CCABUSY,
            RADIO_EVENTS_TXREADY,
            RADIO_EVENTS_ADDRESS,
            RADIO_EVENTS_PHYEND,
            RADIO_EVENTS_END,
        ] {
            write(event, 0);
        }
        clear_event(RADIO_EVENTS_DISABLED);
        write(RADIO_SHORTS, SHORTS_CCA_TX);
        self.mode = Mode::Tx;
        self.tx_outcome = None;
        self.awaiting_ack = ack;
        self.ack_received = None;
        compiler_fence(Ordering::SeqCst);
        write(RADIO_TASKS_RXEN, 1);
        Ok(())
    }

    fn busy(&mut self) -> PhyError {
        self.counters.tx_busy = self.counters.tx_busy.wrapping_add(1);
        PhyError::ChannelBusy
    }

    /// Stop listening if nothing needs the receiver any more.
    fn idle_if_unused(&mut self) {
        if self.mode == Mode::Rx && !self.listening() {
            self.halt();
        }
    }
}

/// Force the RADIO to DISABLED (bounded) and clear the DISABLED event so the
/// handler does not process it.
fn force_disabled() -> bool {
    write(RADIO_SHORTS, 0);
    if read(RADIO_STATE) != STATE_DISABLED {
        write(RADIO_TASKS_CCASTOP, 1);
        write(RADIO_TASKS_STOP, 1);
        write(RADIO_TASKS_DISABLE, 1);
        let mut spins = 0;
        while read(RADIO_STATE) != STATE_DISABLED {
            spins += 1;
            if spins >= DISABLE_SPIN_LIMIT {
                return false;
            }
            core::hint::spin_loop();
        }
    }
    clear_event(RADIO_EVENTS_DISABLED);
    // EasyDMA may have been accessing a buffer; order that before reuse.
    compiler_fence(Ordering::SeqCst);
    true
}

static SHARED: Mutex<CriticalSectionRawMutex, RefCell<Shared>> =
    Mutex::new(RefCell::new(Shared::new()));
static WAKER: AtomicWaker = AtomicWaker::new();

fn with_shared<T>(f: impl FnOnce(&mut Shared) -> T) -> T {
    SHARED.lock(|cell| f(&mut cell.borrow_mut()))
}

/// RADIO interrupt handler of [`NrfIsrRadioPhy`].
///
/// Bind it with
/// `bind_interrupts!(struct Irqs { RADIO => zigbee_mac::nrf::NrfRadioIrq; })`.
pub struct NrfRadioIrq {
    _private: (),
}

impl typelevel::Handler<typelevel::RADIO> for NrfRadioIrq {
    unsafe fn on_interrupt() {
        if read(RADIO_EVENTS_DISABLED) == 0 {
            return;
        }
        clear_event(RADIO_EVENTS_DISABLED);
        if with_shared(Shared::on_disabled) {
            WAKER.wake();
        }
    }
}

// ── PHY ──────────────────────────────────────────────────────

/// Interrupt-driven nRF radio with hardware-timed Imm-Ack (see module docs).
pub struct NrfIsrRadioPhy<'a, R: RngInstance> {
    _radio: PeripheralRef<'a, peripherals::RADIO>,
    _timer: PeripheralRef<'a, peripherals::TIMER0>,
    _txen: PeripheralRef<'a, peripherals::PPI_CH20>,
    _address: PeripheralRef<'a, peripherals::PPI_CH26>,
    _end: PeripheralRef<'a, peripherals::PPI_CH27>,
    rng: Rng<'a, R>,
    aes: NrfAesState,
}

/// Parent-role resources of the nRF router MAC: 8 queued MAC commands,
/// 8 associations in progress, and no software Frame Pending table — the
/// RADIO handler owns Frame Pending through
/// [`RadioPhy::set_ack_frame_pending`].
pub type NrfRouterRole = Parent<8, 8, 0>;

/// Retained-RX depth of the nRF router MAC.
pub const NRF_ROUTER_PENDING_RX: usize = 8;

/// Router (`ParentMacDriver`) MAC on the ISR radio PHY.
pub type NrfRouterMac<'a, R> =
    SoftMacCore<NrfIsrRadioPhy<'a, R>, NrfRouterRole, NRF_ROUTER_PENDING_RX>;

/// End-device MAC on the ISR radio PHY (hardware-timed Imm-Ack).
pub type NrfIsrSoftMac<'a, R> =
    SoftMacCore<NrfIsrRadioPhy<'a, R>, EndDevice, NRF_ROUTER_PENDING_RX>;

impl<'a, R: RngInstance> NrfIsrRadioPhy<'a, R> {
    /// Take RADIO, TIMER0 and PPI channels 20/26/27, configure IEEE 802.15.4
    /// mode and enable the RADIO interrupt at the highest priority.
    ///
    /// HFXO must already be running (`HfclkSource::ExternalXtal`).
    pub fn new(
        radio: impl Peripheral<P = peripherals::RADIO> + 'a,
        timer: impl Peripheral<P = peripherals::TIMER0> + 'a,
        txen: impl Peripheral<P = peripherals::PPI_CH20> + 'a,
        address: impl Peripheral<P = peripherals::PPI_CH26> + 'a,
        end: impl Peripheral<P = peripherals::PPI_CH27> + 'a,
        _irq: impl typelevel::Binding<typelevel::RADIO, NrfRadioIrq> + 'a,
        rng: Rng<'a, R>,
    ) -> Self {
        into_ref!(radio, timer, txen, address, end);

        typelevel::RADIO::disable();
        write(
            PPI_CHENCLR,
            PPI_CH_TIMER0_CC0_TXEN | PPI_CH_ADDRESS_CAPTURE1 | PPI_CH_END_CAPTURE2,
        );

        write(RADIO_POWER, 0);
        write(RADIO_POWER, 1);
        write(RADIO_MODE, MODE_IEEE802154);
        write(RADIO_MODECNF0, MODECNF0_FAST_RAMP_UP);
        write(RADIO_CRCCNF, CRCCNF_IEEE802154);
        write(RADIO_CRCPOLY, CRCPOLY_IEEE802154);
        write(RADIO_CRCINIT, 0);
        write(RADIO_PCNF0, PCNF0_IEEE802154);
        write(RADIO_PCNF1, PCNF1_IEEE802154);
        write(RADIO_SFD, SFD_IEEE802154);
        write(RADIO_CCACTRL, CCACTRL_DEFAULT);
        write(RADIO_TXPOWER, 0);
        write(RADIO_FREQUENCY, channel_frequency(11));
        write(RADIO_SHORTS, 0);
        write(RADIO_INTENCLR, 0xFFFF_FFFF);
        write(RADIO_INTENSET, INT_DISABLED);

        write(TIMER0_TASKS_STOP, 1);
        write(TIMER0_INTENCLR, 0xFFFF_FFFF);
        write(TIMER0_SHORTS, 0);
        write(TIMER0_MODE, 0);
        write(TIMER0_BITMODE, 3);
        write(TIMER0_PRESCALER, 4);
        write(TIMER0_TASKS_CLEAR, 1);
        write(TIMER0_TASKS_START, 1);
        write(PPI_CHENSET, PPI_CH_ADDRESS_CAPTURE1 | PPI_CH_END_CAPTURE2);

        with_shared(|shared| *shared = Shared::new());

        typelevel::RADIO::set_priority(super::embassy_nrf::interrupt::Priority::P0);
        typelevel::RADIO::unpend();
        // SAFETY: the handler is bound through `_irq` and only touches state
        // initialised above.
        unsafe { typelevel::RADIO::enable() };

        Self {
            _radio: radio,
            _timer: timer,
            _txen: txen,
            _address: address,
            _end: end,
            rng,
            aes: NrfAesState::new(),
        }
    }

    /// Consume the unique ECB token and install hardware AES after both
    /// startup known-answer tests pass.
    pub fn install_aes_engine(&mut self, token: NrfEcbToken) -> Result<(), NrfAesError> {
        self.aes.install(token)
    }

    /// Factory-programmed EUI-64 from FICR.DEVICEID[0..2].
    pub fn read_ficr_ieee() -> IeeeAddress {
        const FICR_DEVICEID0: usize = 0x1000_0060;
        const FICR_DEVICEID1: usize = 0x1000_0064;
        let mut address = [0u8; 8];
        address[0..4].copy_from_slice(&read(FICR_DEVICEID0).to_le_bytes());
        address[4..8].copy_from_slice(&read(FICR_DEVICEID1).to_le_bytes());
        address
    }

    /// One hardware-RNG word, e.g. to seed `macDSN`/`macBSN`.
    pub fn random_u32(&mut self) -> u32 {
        let mut entropy = [0u8; 4];
        self.rng.blocking_fill_bytes(&mut entropy);
        u32::from_le_bytes(entropy)
    }

    fn pib(&mut self) -> MacPib {
        let seed = self.random_u32();
        MacPib::new(Self::read_ficr_ieee(), seed as u8, (seed >> 8) as u8)
    }

    /// Build the router [`NrfRouterMac`]. Install the AES engine first.
    #[inline(never)]
    pub fn into_parent_mac(mut self) -> Result<NrfRouterMac<'a, R>, MacError> {
        let pib = self.pib();
        SoftMacCore::new_custom(self, pib)
    }

    /// Build the end-device [`NrfIsrSoftMac`]. Install the AES engine first.
    #[inline(never)]
    pub fn into_soft_mac(mut self) -> Result<NrfIsrSoftMac<'a, R>, MacError> {
        let pib = self.pib();
        SoftMacCore::new_custom(self, pib)
    }

    /// Snapshot of the RADIO handler's diagnostic counters.
    pub fn counters(&self) -> NrfRadioCounters {
        with_shared(|shared| shared.counters)
    }

    /// Wait (bounded) for an in-flight Imm-Ack, then run `f` with the
    /// receiver stopped, and resume receiving if it was running.
    fn reconfigure<T>(&mut self, f: impl FnOnce(&mut Shared) -> T) -> T {
        const ACK_DRAIN_US: u32 = 2_000;
        let started = timer_now();
        while with_shared(|shared| shared.mode == Mode::AckTx)
            && timer_now().wrapping_sub(started) < ACK_DRAIN_US
        {
            core::hint::spin_loop();
        }
        with_shared(|shared| {
            let was_receiving = matches!(shared.mode, Mode::Rx | Mode::AckTx);
            if was_receiving {
                shared.halt();
            }
            let result = f(shared);
            if was_receiving {
                shared.rx_or_off();
            }
            result
        })
    }

    async fn transmit(&mut self, frame: &[u8], ack: bool) -> Result<Option<AckResult>, PhyError> {
        /// CCA (128 µs) + ramp-up + 133 octets on air, with generous margin.
        const TX_TIMEOUT_US: u64 = 10_000;
        /// macAckWaitDuration (864 µs) plus scheduling margin.
        const ACK_WAIT_US: u64 = 1_000;

        if frame.len() > MAX_PHY_FRAME_LEN {
            return Err(PhyError::FrameTooLong);
        }
        let sequence = if ack {
            Some(*frame.get(2).ok_or(PhyError::FrameTooLong)?)
        } else {
            None
        };
        with_shared(|shared| shared.begin_tx(frame, sequence))?;
        let guard = TxGuard { armed: true };

        let outcome = poll_fn(|cx| {
            WAKER.register(cx.waker());
            match with_shared(|shared| shared.tx_outcome.take()) {
                Some(outcome) => Poll::Ready(outcome),
                None => Poll::Pending,
            }
        });
        let outcome = match select(outcome, Timer::after_micros(TX_TIMEOUT_US)).await {
            Either::First(outcome) => outcome,
            Either::Second(()) => return Err(PhyError::Hardware),
        };
        if outcome == TxOutcome::Busy {
            guard.disarm();
            return Err(PhyError::ChannelBusy);
        }
        if !ack {
            guard.disarm();
            return Ok(None);
        }

        let acked = poll_fn(|cx| {
            WAKER.register(cx.waker());
            match with_shared(|shared| shared.ack_received.take()) {
                Some(frame_pending) => Poll::Ready(frame_pending),
                None => Poll::Pending,
            }
        });
        let result = match select(acked, Timer::after_micros(ACK_WAIT_US)).await {
            Either::First(frame_pending) => Some(AckResult { frame_pending }),
            Either::Second(()) => None,
        };
        guard.disarm();
        with_shared(|shared| {
            let late = shared.ack_received.take();
            shared.awaiting_ack = None;
            let result = result.or(late.map(|frame_pending| AckResult { frame_pending }));
            if result.is_none() {
                shared.counters.tx_no_ack = shared.counters.tx_no_ack.wrapping_add(1);
            }
            shared.idle_if_unused();
            Ok(result)
        })
    }

    /// Copy the oldest retained frame into `frame`.
    fn take_frame(frame: &mut PhyRxFrame) -> Option<Result<(), PhyError>> {
        let head = with_shared(|shared| (shared.count > 0).then_some(shared.head))?;
        let slot = rx_slot(head);
        // SAFETY: committed slots are never written by EasyDMA or the
        // handler until released below.
        let result = unsafe {
            let phr = usize::from(slot.read() & 0x7F);
            let len = phr.saturating_sub(2);
            let data = core::slice::from_raw_parts(slot.add(1), len);
            let lqi = slot.add(1 + len).read();
            frame.set(data, crate::lqi::nrf_from_hardware(lqi))
        };
        with_shared(|shared| {
            shared.head = (shared.head + 1) % NRF_ISR_RX_SLOTS;
            shared.count -= 1;
        });
        Some(result)
    }
}

impl<R: RngInstance> Drop for NrfIsrRadioPhy<'_, R> {
    fn drop(&mut self) {
        typelevel::RADIO::disable();
        with_shared(|shared| {
            shared.halt();
            *shared = Shared::new();
        });
        write(PPI_CHENCLR, PPI_CH_ADDRESS_CAPTURE1 | PPI_CH_END_CAPTURE2);
        write(TIMER0_TASKS_STOP, 1);
    }
}

/// Stops the RADIO if a transmit future is dropped mid-flight, before the
/// next transmission can rewrite the TX buffer.
struct TxGuard {
    armed: bool,
}

impl TxGuard {
    fn disarm(mut self) {
        self.armed = false;
    }
}

impl Drop for TxGuard {
    fn drop(&mut self) {
        if self.armed {
            with_shared(|shared| {
                shared.awaiting_ack = None;
                shared.tx_outcome = None;
                shared.ack_received = None;
                if !shared.halt() {
                    log::error!("[nRF] radio did not reach DISABLED after cancelled TX");
                }
                shared.rx_or_off();
            });
        }
    }
}

/// Clears `receive_active` when a receive future completes or is dropped.
struct ReceiveGuard;

impl Drop for ReceiveGuard {
    fn drop(&mut self) {
        with_shared(|shared| {
            shared.receive_active = false;
            shared.idle_if_unused();
        });
    }
}

fn channel_frequency(channel: u8) -> u32 {
    u32::from(channel - 10) * 5
}

impl<R: RngInstance> RadioPhy for NrfIsrRadioPhy<'_, R> {
    /// One CCA + TX attempt with the ACK matched in the RADIO handler.
    type Tx = crate::AckWaitTx;

    /// Same values as [`super::NrfRadioPhy`] (hardware-proven end-device
    /// polling against EmberZNet and ZHA parents).
    const POLL_RESPONSE_WAIT_US: u32 = 1_500_000;
    const POLL_EXTENDED_FALLBACK: bool = true;

    fn capabilities(&self) -> PhyCapabilities {
        PhyCapabilities {
            hardware_auto_ack: true,
            hardware_address_filter: true,
            tx_power_min: -20,
            tx_power_max: 8,
        }
    }

    async fn try_transmit(&mut self, frame: &[u8]) -> Result<(), PhyError> {
        self.transmit(frame, false).await.map(|_| ())
    }

    async fn try_transmit_acknowledged(
        &mut self,
        frame: &[u8],
    ) -> Result<Option<AckResult>, PhyError> {
        self.transmit(frame, true).await
    }

    /// Imm-Acks are sent by the RADIO handler (`hardware_auto_ack`); the
    /// core never asks for a software ACK.
    async fn send_ack(&mut self, _sequence: u8, _frame_pending: bool) -> Result<(), PhyError> {
        Err(PhyError::Unsupported)
    }

    async fn receive(&mut self, timeout_us: u32, frame: &mut PhyRxFrame) -> Result<bool, PhyError> {
        let deadline = Instant::now() + Duration::from_micros(u64::from(timeout_us));
        with_shared(|shared| {
            shared.recover_stalled_ack();
            shared.receive_active = true;
            if shared.mode == Mode::Off && shared.listening() {
                shared.start_rx();
            }
        });
        let _guard = ReceiveGuard;
        loop {
            if let Some(result) = Self::take_frame(frame) {
                return result.map(|()| true);
            }
            let ready = poll_fn(|cx| {
                WAKER.register(cx.waker());
                if with_shared(|shared| shared.count > 0) {
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            });
            if let Either::Second(()) = select(ready, Timer::at(deadline)).await {
                return Ok(false);
            }
        }
    }

    fn set_channel(&mut self, channel: u8) -> Result<(), PhyError> {
        if !(11..=26).contains(&channel) {
            return Err(PhyError::InvalidChannel);
        }
        self.reconfigure(|_| write(RADIO_FREQUENCY, channel_frequency(channel)));
        Ok(())
    }

    fn set_tx_power(&mut self, dbm: i8) -> Result<(), PhyError> {
        if !matches!(
            dbm,
            -40 | -30 | -20 | -16 | -12 | -8 | -4 | 0 | 2 | 3 | 4 | 5 | 6 | 7 | 8
        ) {
            return Err(PhyError::Unsupported);
        }
        // TXPOWER is the two's-complement dBm value; it is latched at the
        // next TXEN, so no reconfiguration window is needed.
        write(RADIO_TXPOWER, u32::from(dbm as u8));
        Ok(())
    }

    async fn energy_detect(&mut self, _duration_us: u32) -> Result<u8, PhyError> {
        Err(PhyError::Unsupported)
    }

    fn set_address_filter(&mut self, filter: Option<PhyAddressFilter>) -> Result<(), PhyError> {
        with_shared(|shared| shared.filter = filter);
        Ok(())
    }

    fn set_ack_frame_pending(
        &mut self,
        source: &MacAddress,
        pending: bool,
    ) -> Result<(), PhyError> {
        // Mapped to TransactionOverflow by the parent role.
        with_shared(|shared| shared.pending.set(*source, pending)).map_err(|_| PhyError::Hardware)
    }

    fn clear_ack_frame_pending(&mut self) {
        with_shared(|shared| shared.pending.clear());
    }

    fn set_rx_on_when_idle(&mut self, enabled: bool) {
        with_shared(|shared| {
            shared.rx_on_when_idle = enabled;
            if shared.mode == Mode::Off && shared.listening() {
                shared.start_rx();
            } else {
                shared.idle_if_unused();
            }
        });
    }

    fn sleep(&mut self) -> Result<(), PhyError> {
        let disabled = self.reconfigure(|shared| {
            shared.sleeping = true;
            shared.halt()
        });
        write(TIMER0_TASKS_STOP, 1);
        if disabled {
            Ok(())
        } else {
            Err(PhyError::Hardware)
        }
    }

    fn wake(&mut self) -> Result<(), PhyError> {
        write(TIMER0_TASKS_START, 1);
        with_shared(|shared| {
            shared.sleeping = false;
            if shared.mode == Mode::Off && shared.listening() {
                shared.start_rx();
            }
        });
        Ok(())
    }
}

impl<R: RngInstance> zigbee_crypto::ForwardAesProvider for NrfIsrRadioPhy<'_, R> {
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

impl<R: RngInstance> PlatformServices for NrfIsrRadioPhy<'_, R> {
    fn monotonic_micros(&self) -> u32 {
        Instant::now().as_micros() as u32
    }

    async fn delay_micros(&mut self, duration_us: u32) {
        Timer::after_micros(u64::from(duration_us)).await;
    }

    fn fill_random(&mut self, output: &mut [u8]) -> Result<(), MacError> {
        self.rng.blocking_fill_bytes(output);
        Ok(())
    }
}
