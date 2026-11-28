//! Interrupt-driven Imm-Ack with a HAL-owned Frame Pending table
//! (`isr-ack` feature, parent/router images).
//!
//! The `FRC` RX-done interrupt examines every newly completed frame in the
//! RX buffer, in order, and
//!
//! * acknowledges a frame addressed to this node that requests an ACK
//!   ([`super::needs_ack`]), with the Frame Pending bit set only for a MAC
//!   Data Request whose source is in the source-match table armed through
//!   [`super::Radio::set_frame_pending`] (IEEE 802.15.4-2006 7.2.2.3.1 /
//!   7.5.6.3; Zigbee R22/R23 parents). The bit is decided here, before the
//!   ACK is built. An ACK to any other frame, data frames included, carries
//!   FP = 0. A Data Request acknowledged with FP = 0 is queued with that
//!   fact ([`super::RxFrame::acked_without_frame_pending`]): the child has
//!   then turned its receiver off, so the stack must not answer that poll
//!   with a frame it queued after the ACK. The bit travels with the poll
//!   itself, so any number of children is covered and nothing outlives the
//!   poll;
//! * drops ACK frames, frames for another node or PAN, and frames whose ACK
//!   can no longer be sent within the turnaround window: the sender did not
//!   get its ACK and retransmits, so the frame must not be delivered twice;
//! * moves every other frame out of BUFC into a 1 KiB queue the task reads
//!   ([`pop`]). BUFC is emptied on every interrupt, so the foreign traffic
//!   of a busy channel never accumulates while the stack is blocked
//!   elsewhere (NWK broadcast jitter, RREQ relay retries, flash writes):
//!   the queue only holds frames for this node, and when it is full a frame
//!   is dropped unacknowledged instead of overflowing the hardware buffer.
//!
//! # Sending the ACK without waiting in an interrupt
//!
//! The CPU is the radio sequencer (see the driver docs): the RSM holds
//! `TXWARM` until the driver has locked the synthesizer and ramped the PA
//! up, and `TXPD` until it has ramped the PA down. Those steps are short;
//! the waits between them are not (warm-state settling, the PA pre-stage,
//! 352 us of ACK airtime, the RX warm-up). The Imm-Ack is therefore a
//! sequence of short steps, each run by the `TIMER0` interrupt at a cycle
//! stamp (compare channel 2 of the time base, [`crate::delay::alarm`]), and
//! no handler waits through the gaps. The alarm fires [`ALARM_LEAD_US`]
//! before a step is due and the handler waits out the rest, so a step runs
//! at its stamp despite the interrupt latency:
//!
//! | step | due | work |
//! |---|---|---|
//! | `FRC` RX done | frame end + latency | classify, Frame Pending, RX off, ACK into the TX buffer, `TXEN`: RSM held in `TXWARM` |
//! | `TxSynth` | + [`WARM_SETTLE_US`] | lock the synthesizer |
//! | `TxPrestage` (xG24) | TX start - PA pre-stage - ramp - margin | PA regulators up, slices in reset |
//! | `TxRamp` | TX start - ramp - margin | start the PA ramp |
//! | `TxStart` | TX start - [`START_MARGIN_US`] | steady PA, release `TXWARM` at the TX start: the ACK goes on air |
//! | `TxEnd` | TX start + 352 us | PA ramp-down in the held `TXPD`, RSM off, RX on: RSM held in `RXWARM` |
//! | `Queue` | + [`QUEUE_AFTER_US`] | queue the frame for the stack, examine the frames behind it |
//! | `RxSynth` | RX on + [`WARM_SETTLE_US`] | lock the synthesizer for RX, release `RXWARM` |
//!
//! The TX start is [`ACK_START_US`] after the `FRC` interrupt's entry,
//! which puts the ACK's first symbol aTurnaroundTime (12 symbols, 192 us)
//! after the frame's last symbol, unless the warm-up chain takes longer
//! (the chain is fixed: `FRC` part, settle, synthesizer, PA pre-stage and
//! ramp; on the 20 dBm xG24 PA it ends a few microseconds after that
//! stamp). `TXWARM` is released by one register write, timed by the CPU
//! inside `TxStart` against the time base. No hardware event can release
//! the held state: the RSM waits for `RAC.SEQIF` to be cleared, which RAIL's
//! sequencer microcode would do and this driver does not load. The frame
//! stays at the head of BUFC until `TxEnd` has seen `TXDONE`, and is only
//! then queued for the stack: a frame whose ACK failed is never delivered.
//!
//! Every task-side radio operation runs with the `FRC` and `TIMER0` NVIC
//! lines masked (`Radio`'s operation lock), so neither interrupt sees the
//! radio mid-operation. The lock first steps an ACK in flight to its end
//! in the task (at most one ACK, about 0.6 ms). Its release hands over the
//! RX state, channel, TX power and ACK filter, and re-pends `FRC` when a
//! frame completed meanwhile. A frame that completed while the radio was
//! locked may have ended anywhere in the lock: it is aged from the lock's
//! start (decided within [`ACK_DEADLINE_US`], started within
//! [`ACK_LATEST_US`], or left unacknowledged), and its ACK starts
//! [`ACK_START_US`] after the interrupt's entry ([`ack_start`]), so never
//! before aTurnaroundTime. [`pop`], the Frame
//! Pending table and the counters only mask the lines around the access,
//! so they never wait for an ACK. The two handlers share this state
//! without further locking, so they must not preempt each other: [`init`]
//! gives `TIMER0` the `FRC` priority.
//!
//! The task only delivers queued frames, so an ACK decision always precedes
//! delivery.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicBool, Ordering};

use super::{
    AckFilter, AckStats, BUF_CMD, BUF_CMD_CLEAR, BUF_OFFSET_MASK, BUF_READOFFSET, BUF_STATUS,
    BUF_STATUS_BYTES, BUF_WRITEDATA, CLR, FP_TABLE_LEN, FRC_CMD, FRC_CMD_RXABORT, FRC_IEN, FRC_IF,
    FRC_IF_RXDONE, FRC_IF_RXOF, FRC_IF_TXABORTED, FRC_IF_TXDONE, FRC_IF_TXUF, FpAddress, FpError,
    L, MAX_FRAME, NVIC_ISER1, RAC_CMD, RAC_CMD_CLEARRXOVERFLOW, RAC_CMD_CLEARTXEN, RAC_CMD_RXDIS,
    RAC_CMD_TXDIS, RAC_CMD_TXEN, RAC_RXENSRCEN, RAC_SWRXEN, RX_TRAILER_LEN, RacState, Radio,
    SYNTH_CMD, SYNTH_CMD_STOP, WARM_SETTLE_US, Warm, apply_state, buf, chip, enter_warm, needs_ack,
    rac_state_raw, read, release_warm, rx_ram_byte, rx_ram_runs, service_sequencer, start_synth,
    tx_power_down, wait_off_releasing, write,
};
use crate::delay::{Deadline, alarm, cycles, cycles_to_us, us_to_cycles};

/// Bytes of the queue of received frames for the stack.
pub(super) const RX_QUEUE_LEN: usize = 1024;

/// Latest ACK decision after the end of a frame. The sender waits
/// macAckWaitDuration (864 us) for the ACK; past this the frame is
/// dropped unacknowledged.
const ACK_DEADLINE_US: u32 = 400;
/// Latest ACK start after the end of a frame: macAckWaitDuration (54
/// symbols) less the ACK's own 22 symbols (aTurnaroundTime +
/// aUnitBackoffPeriod). An ACK that would start later is not sent.
const ACK_LATEST_US: u32 = 512;
/// TX start (the `TXWARM` release) of the Imm-Ack after the `FRC`
/// interrupt's entry: puts the ACK's first symbol aTurnaroundTime (192 us)
/// after the frame's last symbol on the air. Calibrated with the nRF
/// sniffer: the interrupt's latency after the frame plus the RSM's and
/// modem's start after the release add about 11 us (EK2703A: ACKs
/// 191..195 us after the frame).
const ACK_START_US: u32 = 181;
/// `TxStart` is due this much before the TX start: it first sets the PA
/// steady.
const START_MARGIN_US: u32 = 2;
/// The step alarm fires this much before the step is due. A handler fetched
/// from flash reaches its step about 8 us after the compare match (BRD4187A
/// and EK2703A); it waits out the rest.
const ALARM_LEAD_US: u32 = 10;
/// `Queue` follows the end of the ACK this much later, inside the RX
/// settle time ([`WARM_SETTLE_US`]): the frame copy runs in a handler of
/// its own.
const QUEUE_AFTER_US: u32 = 15;
/// 5-byte ACK airtime: SHR, PHR and PSDU, 11 bytes.
const ACK_AIRTIME_US: u32 = 352;
/// `TxEnd` polls this often for `TXDONE` past the expected end.
const TX_END_POLL_US: u32 = 4;
/// `TXDONE` must come this long after the TX start.
const ACK_TX_TIMEOUT_US: u32 = 1_000;
/// MHR bytes the interrupt reads (the longest Data Request MHR is 23).
const HEADER_LEN: usize = 24;
/// Grace for BUFC to store a frame the FRC already reported done (the
/// interrupt waits only until the bytes are there).
const BUFC_CATCH_UP_US: u32 = 40;

/// `FRC` IRQ number per die (`chip::FRC_IRQN`: 34 on xG21/xG22, 33 on xG24).
const FRC_IRQ_BIT: u32 = 1 << (super::chip::FRC_IRQN - 32);
/// `TIMER0` IRQ (`chip::TIMER0_IRQN`: 6 on xG21, 7 on xG22, 4 on xG24).
const TIMER0_IRQ_BIT: u32 = 1 << super::chip::TIMER0_IRQN;
const NVIC_ISER0: u32 = 0xE000_E100;
const NVIC_ICER0: u32 = 0xE000_E180;
const NVIC_ICER1: u32 = 0xE000_E184;
const NVIC_ISPR1: u32 = 0xE000_E204;
const NVIC_ICPR0: u32 = 0xE000_E280;
const NVIC_IPR: u32 = 0xE000_E400;

/// Frame Pending source-match table: short and extended child addresses
/// with queued indirect data (or a pending Association Response).
#[derive(Clone, Copy)]
struct FpTable {
    /// `0xFFFF` = free.
    short: [u16; FP_TABLE_LEN],
    /// `0` = free.
    extended: [u64; FP_TABLE_LEN],
}

impl FpTable {
    const EMPTY: Self = Self {
        short: [0xFFFF; FP_TABLE_LEN],
        extended: [0; FP_TABLE_LEN],
    };

    fn set(&mut self, address: FpAddress, pending: bool) -> Result<(), FpError> {
        match address {
            FpAddress::Short(short) if short >= 0xFFFE => Err(FpError::InvalidAddress),
            FpAddress::Short(short) => table_set(&mut self.short, short, 0xFFFF, pending),
            FpAddress::Extended(ext) if ext == 0 || ext == u64::MAX => Err(FpError::InvalidAddress),
            FpAddress::Extended(ext) => table_set(&mut self.extended, ext, 0, pending),
        }
    }

    fn contains(&self, address: FpAddress) -> bool {
        match address {
            FpAddress::Short(short) => short < 0xFFFE && self.short.contains(&short),
            FpAddress::Extended(ext) => ext != 0 && self.extended.contains(&ext),
        }
    }
}

fn table_set<T: Copy + PartialEq>(
    table: &mut [T; FP_TABLE_LEN],
    address: T,
    free: T,
    pending: bool,
) -> Result<(), FpError> {
    if let Some(slot) = table.iter_mut().find(|slot| **slot == address) {
        if !pending {
            *slot = free;
        }
        return Ok(());
    }
    if !pending {
        return Ok(());
    }
    let slot = table
        .iter_mut()
        .find(|slot| **slot == free)
        .ok_or(FpError::TableFull)?;
    *slot = address;
    Ok(())
}

/// Step of the Imm-Ack in flight (see the module docs).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    Idle,
    TxSynth,
    TxPrestage,
    TxRamp,
    TxStart,
    TxEnd,
    Queue,
    RxSynth,
}

/// The frame being acknowledged, still at the head of BUFC.
#[derive(Clone, Copy)]
struct Held {
    /// BUFC read offset of its PHR.
    offset: u32,
    /// Bytes it occupies in BUFC.
    needed: u32,
    /// MPDU bytes without FCS.
    stored: u8,
    /// Its record flags once queued.
    flags: u8,
    frame_pending: bool,
}

/// The Imm-Ack in flight.
#[derive(Clone, Copy)]
struct InFlight {
    step: Step,
    /// Cycle stamp the step is due at.
    at: u32,
    /// Cycle stamp of the TX start (the actual one from `TxStart` on).
    start: u32,
    /// Cycle stamp of the PA ramp's start.
    ramp: u32,
    /// Aging base of the frame: the interrupt entry, or the start of the
    /// lock it completed in.
    base: u32,
    frame: Held,
    /// The ACK went on air (`Queue` delivers the frame).
    sent: bool,
    /// The receiver is warming up after the ACK; `RxSynth` is due at
    /// `rx_at`.
    rx_on: bool,
    rx_at: u32,
}

impl InFlight {
    const IDLE: Self = Self {
        step: Step::Idle,
        at: 0,
        start: 0,
        ramp: 0,
        base: 0,
        frame: Held {
            offset: 0,
            needed: 0,
            stored: 0,
            flags: 0,
            frame_pending: false,
        },
        sent: false,
        rx_on: false,
        rx_at: 0,
    };
}

/// State shared by the task (only while the `FRC` and `TIMER0` lines are
/// masked) and those two interrupts.
struct Shared {
    /// Operation lock nesting depth.
    depth: u8,
    /// Receiver on (from `Radio::rx_enabled` at the last lock release).
    rx_armed: bool,
    channel: u8,
    power_ddbm: i16,
    /// `VCOTUNING` cache of `channel`, `[rx, tx]`.
    fcal: [u16; 2],
    filter: AckFilter,
    /// Start of the oldest lock a frame may have completed in.
    oldest: u32,
    oldest_valid: bool,
    table: FpTable,
    stats: AckStats,
    ack: InFlight,
}

struct SharedCell(UnsafeCell<Shared>);

// SAFETY: accessed only from the `FRC` and `TIMER0` handlers (same NVIC
// priority, so they never preempt each other) and from task code that has
// masked both lines first (`lock`, `with_lines_masked`), or before the
// lines are ever enabled (`RadioToken::init`); these never overlap.
unsafe impl Sync for SharedCell {}

static SHARED: SharedCell = SharedCell(UnsafeCell::new(Shared {
    depth: 0,
    rx_armed: false,
    channel: 11,
    power_ddbm: 0,
    fcal: [0; 2],
    filter: AckFilter {
        pan_id: None,
        short_address: None,
        extended_address: None,
    },
    oldest: 0,
    oldest_valid: false,
    table: FpTable::EMPTY,
    stats: AckStats {
        isr_runs: 0,
        acks: 0,
        fp_acks: 0,
        late: 0,
        filtered: 0,
        overflows: 0,
        queue_full: 0,
        queue_full_unacked: 0,
        resyncs: 0,
        ack_tx_fail: 0,
        rx_restart_fail: 0,
        max_turnaround_us: 0,
        max_isr_us: 0,
    },
    ack: InFlight::IDLE,
}));

/// Set by the interrupt when it could not restart the receiver after an
/// ACK; read without the lock by `Radio::is_rx_on`.
static RX_LOST: AtomicBool = AtomicBool::new(false);

/// Received frames for the stack, as `[len | flags, MPDU without FCS, raw
/// RSSI]` records in a byte ring. A stored MPDU is at most 125 bytes, so
/// bit 7 of the length byte is free for [`ACKED_WITHOUT_FP`].
struct RxQueue {
    bytes: [u8; RX_QUEUE_LEN],
    /// Index of the oldest record.
    head: u16,
    /// Bytes in use.
    used: u16,
}

impl RxQueue {
    fn free(&self) -> usize {
        RX_QUEUE_LEN - usize::from(self.used)
    }

    fn put(&mut self, byte: u8) {
        let index = (usize::from(self.head) + usize::from(self.used)) % RX_QUEUE_LEN;
        self.bytes[index] = byte;
        self.used += 1;
    }

    fn take(&mut self) -> u8 {
        let byte = self.bytes[usize::from(self.head)];
        self.head = ((usize::from(self.head) + 1) % RX_QUEUE_LEN) as u16;
        self.used -= 1;
        byte
    }

    /// Append a record whose MPDU is `mpdu[0]` then `mpdu[1]` (a frame
    /// that wraps around the end of BUFC); the caller checked
    /// [`free`](Self::free).
    fn push(&mut self, mpdu: [&[u8]; 2], rssi: u8, flags: u8) {
        self.put((mpdu[0].len() + mpdu[1].len()) as u8 | flags);
        for mut run in mpdu {
            while !run.is_empty() {
                let at = (usize::from(self.head) + usize::from(self.used)) % RX_QUEUE_LEN;
                let n = run.len().min(RX_QUEUE_LEN - at);
                self.bytes[at..at + n].copy_from_slice(&run[..n]);
                self.used += n as u16;
                run = &run[n..];
            }
        }
        self.put(rssi);
    }

    /// Remove the oldest record into `out`: `(len, raw RSSI, flags)`.
    fn pop(&mut self, out: &mut [u8; MAX_FRAME]) -> Option<(usize, i8, u8)> {
        if self.used == 0 {
            return None;
        }
        let first = self.take();
        let len = usize::from(first & !RECORD_FLAGS);
        let mut copied = 0;
        while copied < len {
            let at = usize::from(self.head);
            let n = (len - copied).min(RX_QUEUE_LEN - at);
            out[copied..copied + n].copy_from_slice(&self.bytes[at..at + n]);
            self.head = ((at + n) % RX_QUEUE_LEN) as u16;
            self.used -= n as u16;
            copied += n;
        }
        Some((len, self.take() as i8, first & RECORD_FLAGS))
    }
}

/// Record flag: the frame is a MAC Data Request the interrupt acknowledged
/// with Frame Pending = 0.
const ACKED_WITHOUT_FP: u8 = 0x80;
/// Bits of a record's length byte that are flags, not length.
const RECORD_FLAGS: u8 = ACKED_WITHOUT_FP;
const _: () = assert!(MAX_FRAME as u8 & RECORD_FLAGS == 0 && MAX_FRAME < 0x80);

/// Record flags of a frame the interrupt keeps after `verdict` (its ACK, if
/// any, was sent).
fn record_flags(verdict: Verdict) -> u8 {
    match verdict {
        Verdict::Ack {
            frame_pending: false,
            poll: Some(_),
        } => ACKED_WITHOUT_FP,
        _ => 0,
    }
}

/// Bytes a queued frame of `len` MPDU bytes occupies.
const fn record_len(len: usize) -> usize {
    len + 2
}

/// Queue bytes kept free for ACK-requested frames: two maximum-length
/// records. A broadcast storm then cannot fill the queue and leave
/// unicasts to this node (child polls and data) unacknowledged.
const ACKED_RESERVE: usize = 2 * record_len(MAX_FRAME);

/// Whether a frame of `len` MPDU bytes fits into `free` queue bytes; a frame
/// without an ACK request must also leave [`ACKED_RESERVE`] free.
const fn admits(free: usize, len: usize, acked: bool) -> bool {
    let reserve = if acked { 0 } else { ACKED_RESERVE };
    free >= record_len(len) + reserve
}

struct QueueCell(UnsafeCell<RxQueue>);

// SAFETY: same discipline as `SharedCell`.
unsafe impl Sync for QueueCell {}

static QUEUE: QueueCell = QueueCell(UnsafeCell::new(RxQueue {
    bytes: [0; RX_QUEUE_LEN],
    head: 0,
    used: 0,
}));

/// # Safety
/// As [`shared`].
#[allow(clippy::mut_from_ref)]
unsafe fn queue() -> &'static mut RxQueue {
    unsafe { &mut *QUEUE.0.get() }
}

/// # Safety
/// Caller is the `FRC` or `TIMER0` handler, or task code with both lines
/// masked (or running before they are first enabled).
#[allow(clippy::mut_from_ref)]
unsafe fn shared() -> &'static mut Shared {
    unsafe { &mut *SHARED.0.get() }
}

fn mask() {
    unsafe {
        write(NVIC_ICER1, FRC_IRQ_BIT);
        write(NVIC_ICER0, TIMER0_IRQ_BIT);
    }
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

/// # Safety
/// The lock is not held (`depth == 0`).
unsafe fn unmask() {
    unsafe {
        write(NVIC_ISER0, TIMER0_IRQ_BIT);
        write(NVIC_ISER1, FRC_IRQ_BIT);
    }
}

/// `TIMER0` takes the `FRC` priority: the handlers share [`Shared`]
/// without locking, so neither may preempt the other.
pub(super) fn init() {
    unsafe {
        let priority = core::ptr::read_volatile((NVIC_IPR + chip::FRC_IRQN) as *const u8);
        core::ptr::write_volatile((NVIC_IPR + chip::TIMER0_IRQN) as *mut u8, priority);
    }
}

/// Run `f` with both lines masked, without waiting for an ACK in flight:
/// for the queue, the Frame Pending table and the counters, which the
/// steps of an ACK leave consistent between handler runs. Only masks
/// around `f`: the task never holds the operation lock across these.
fn with_lines_masked<T>(f: impl FnOnce(&mut Shared) -> T) -> T {
    mask();
    // SAFETY: both lines are masked.
    let result = f(unsafe { shared() });
    if unsafe { shared() }.depth == 0 {
        unsafe { unmask() };
    }
    result
}

/// Operation lock: masks both lines and steps an ACK in flight to its end,
/// so the task finds the radio as the last release left it (plus RX
/// restarted after the ACK).
pub(super) fn lock(radio: &mut Radio) {
    mask();
    // SAFETY: both lines are masked.
    let s = unsafe { shared() };
    if s.depth == 0 {
        if s.ack.step != Step::Idle {
            advance(s, true);
            unsafe { write(NVIC_ICPR0, TIMER0_IRQ_BIT) };
        }
        if !s.oldest_valid {
            s.oldest = cycles();
            s.oldest_valid = true;
        }
        if RX_LOST.swap(false, Ordering::Relaxed) {
            radio.rx_enabled = false;
        }
        radio.fcal[usize::from(s.channel - 11)] = s.fcal;
    }
    s.depth += 1;
}

pub(super) fn unlock(radio: &mut Radio) {
    // SAFETY: both lines are still masked.
    let s = unsafe { shared() };
    s.depth -= 1;
    if s.depth != 0 {
        return;
    }
    s.rx_armed = radio.rx_enabled;
    s.channel = radio.channel;
    s.power_ddbm = radio.power_ddbm;
    s.fcal = radio.fcal[usize::from(radio.channel - 11)];
    s.filter = radio.ack_filter;
    unsafe {
        if complete_frame_at_head() {
            write(NVIC_ISPR1, FRC_IRQ_BIT);
        } else {
            s.oldest_valid = false;
        }
        unmask();
    }
}

pub(super) fn rx_lost() -> bool {
    RX_LOST.load(Ordering::Relaxed)
}

pub(super) fn set_frame_pending(address: FpAddress, pending: bool) -> Result<(), FpError> {
    with_lines_masked(|s| s.table.set(address, pending))
}

pub(super) fn clear_frame_pending() {
    with_lines_masked(|s| s.table = FpTable::EMPTY);
}

/// Snapshot of the counters.
pub(super) fn stats() -> AckStats {
    with_lines_masked(|s| s.stats)
}

/// Next queued frame (MPDU without FCS): its length, raw RSSI and whether
/// it is a Data Request acknowledged with Frame Pending = 0.
pub(super) fn pop(out: &mut [u8; MAX_FRAME]) -> Option<(usize, i8, bool)> {
    // SAFETY: the queue is only touched with both lines masked.
    let (len, rssi, flags) = with_lines_masked(|_| unsafe { queue() }.pop(out))?;
    Some((len, rssi, flags & ACKED_WITHOUT_FP != 0))
}

/// Bytes `phr` occupies in the RX buffer (PHR, MPDU without FCS, trailer).
fn stored_len(phr: u32) -> u32 {
    1 + (phr - 2) + RX_TRAILER_LEN as u32
}

/// Whether a complete frame (or a corrupt length to resynchronise) waits
/// at the head of BUFC.
fn complete_frame_at_head() -> bool {
    let rx = buf(1);
    let available = unsafe { read(rx + BUF_STATUS) } & BUF_STATUS_BYTES;
    if available == 0 {
        return false;
    }
    let readoff = unsafe { read(rx + BUF_READOFFSET) } & BUF_OFFSET_MASK;
    let phr = u32::from(rx_ram_byte(readoff)) & 0x7F;
    phr < 5 || stored_len(phr) <= available
}

/// `FRC` interrupt body.
pub(super) fn on_frc_interrupt() {
    let entry = cycles();
    // SAFETY: this is the `FRC` handler.
    let s = unsafe { shared() };
    s.stats.isr_runs = s.stats.isr_runs.wrapping_add(1);
    let flags = unsafe { read(L.frc + FRC_IF) & read(L.frc + FRC_IEN) };
    unsafe { write(L.frc + FRC_IF + CLR, flags) };
    service_sequencer();
    // While an ACK is in flight the receiver is off and the frame holds
    // the head of BUFC; its last step examines what follows it.
    if s.ack.step == Step::Idle {
        if flags & FRC_IF_RXOF != 0 || rac_state_raw() == RacState::RxOverflow {
            // Same recovery as `Radio::service_overflow`; the receiver
            // stays enabled through `RXENSRCEN`.
            unsafe {
                write(L.frc + FRC_CMD, FRC_CMD_RXABORT);
                write(buf(1) + BUF_CMD, BUF_CMD_CLEAR);
                write(L.rac + RAC_CMD, RAC_CMD_CLEARRXOVERFLOW);
                write(L.frc + FRC_IF + CLR, FRC_IF_RXOF);
            }
            s.stats.overflows = s.stats.overflows.wrapping_add(1);
        }
        examine(s, entry, flags & FRC_IF_RXDONE != 0);
        advance(s, false);
    }
    s.oldest_valid = false;
    note_isr(s, entry);
}

/// `TIMER0` interrupt body: the steps of the ACK in flight that are due.
pub(super) fn on_timer_interrupt() {
    let entry = cycles();
    alarm::acknowledge();
    // SAFETY: this is the `TIMER0` handler.
    let s = unsafe { shared() };
    advance(s, false);
    note_isr(s, entry);
}

fn note_isr(s: &mut Shared, entry: u32) {
    let us = cycles_to_us(cycles().wrapping_sub(entry));
    if us > s.stats.max_isr_us {
        s.stats.max_isr_us = us;
    }
}

/// Whether cycle stamp `at` lies after `now` (within half the counter
/// range).
fn after(at: u32, now: u32) -> bool {
    at.wrapping_sub(now) as i32 > 0
}

/// The later of two cycle stamps.
fn later(a: u32, b: u32) -> u32 {
    if after(b, a) { b } else { a }
}

/// TX start of the ACK for a frame that ended between the cycle stamps
/// `earliest` (the start of the lock that delayed the interrupt, or its
/// entry) and `latest`: [`ACK_START_US`] after the latest end, so the ACK
/// never starts before aTurnaroundTime. `None` when that is more than
/// [`ACK_LATEST_US`] after the earliest end: the frame is then left
/// unacknowledged and the sender retries.
fn ack_start(earliest: u32, latest: u32) -> Option<u32> {
    let start = latest.wrapping_add(us_to_cycles(ACK_START_US));
    (start.wrapping_sub(earliest) <= us_to_cycles(ACK_LATEST_US)).then_some(start)
}

/// `TxRamp` of an ACK starting at `start`: the ramp ends
/// [`START_MARGIN_US`] before the TX start. `TxPrestage` (xG24) is due
/// [`chip::PA_PRESTAGE_US`] earlier.
fn ramp_at(start: u32) -> u32 {
    start.wrapping_sub(us_to_cycles(chip::PA_RAMP_LEAD_US + START_MARGIN_US))
}

/// `TxStart` of an ACK starting at `start` whose PA ramp started at cycle
/// stamp `ramp`: [`START_MARGIN_US`] before the start, or once the ramp
/// is steady if it started late. A chip without a CPU ramp (xG22) does not
/// look at `ramp`.
fn tx_start_due(ramp: u32, start: u32) -> u32 {
    let margin = start.wrapping_sub(us_to_cycles(START_MARGIN_US));
    if chip::PA_RAMP_LEAD_US == 0 {
        return margin;
    }
    later(
        ramp.wrapping_add(us_to_cycles(chip::PA_RAMP_LEAD_US)),
        margin,
    )
}

/// Run the steps that are due and arm the alarm for the next one (disarm
/// once the ACK is done). With `wait` (task, lines masked) wait for every
/// step instead, so the ACK in flight is over when this returns.
fn advance(s: &mut Shared, wait: bool) {
    while s.ack.step != Step::Idle {
        let at = s.ack.at;
        if !wait && after(at, cycles().wrapping_add(us_to_cycles(ALARM_LEAD_US))) {
            alarm::set(at.wrapping_sub(us_to_cycles(ALARM_LEAD_US)));
            return;
        }
        while after(at, cycles()) {}
        step(s);
    }
    alarm::clear();
}

/// Run the step that is due.
fn step(s: &mut Shared) {
    match s.ack.step {
        Step::Idle => {}
        Step::TxSynth => {
            if start_synth(false, &mut s.fcal[1]).is_err() {
                // As `warm_up`: release the warm state so the RSM can be
                // disabled.
                release_warm(RacState::TxWarm, Warm::Tx);
                unsafe { write(L.rac + RAC_CMD, RAC_CMD_CLEARTXEN | RAC_CMD_TXDIS) };
                radio_off();
                end_ack(s, false, cycles());
                return;
            }
            let ramp = ramp_at(s.ack.start);
            if chip::PA_PRESTAGE_US != 0 {
                s.ack.step = Step::TxPrestage;
                s.ack.at = ramp.wrapping_sub(us_to_cycles(chip::PA_PRESTAGE_US));
            } else {
                s.ack.step = Step::TxRamp;
                s.ack.at = ramp;
            }
        }
        Step::TxPrestage => {
            chip::pa_prestage();
            s.ack.step = Step::TxRamp;
            let settled = cycles().wrapping_add(us_to_cycles(chip::PA_PRESTAGE_US));
            s.ack.at = later(settled, ramp_at(s.ack.start));
        }
        Step::TxRamp => {
            s.ack.ramp = cycles();
            chip::pa_ramp_start();
            s.ack.step = Step::TxStart;
            s.ack.at = tx_start_due(s.ack.ramp, s.ack.start);
        }
        Step::TxStart => {
            chip::pa_ramp_finish(s.ack.ramp);
            while after(s.ack.start, cycles()) {}
            release_warm(RacState::TxWarm, Warm::Tx);
            let now = cycles();
            let turnaround = cycles_to_us(now.wrapping_sub(s.ack.base));
            s.stats.max_turnaround_us = s.stats.max_turnaround_us.max(turnaround);
            s.ack.start = now;
            s.ack.step = Step::TxEnd;
            s.ack.at = now.wrapping_add(us_to_cycles(ACK_AIRTIME_US));
        }
        Step::TxEnd => {
            let flags = unsafe { read(L.frc + FRC_IF) };
            let sent = if flags & FRC_IF_TXDONE != 0 {
                true
            } else if flags & (FRC_IF_TXABORTED | FRC_IF_TXUF) != 0
                || cycles().wrapping_sub(s.ack.start) >= us_to_cycles(ACK_TX_TIMEOUT_US)
            {
                false
            } else {
                s.ack.at = cycles().wrapping_add(us_to_cycles(TX_END_POLL_US));
                return;
            };
            tx_power_down();
            unsafe {
                write(
                    L.frc + FRC_IF + CLR,
                    FRC_IF_TXDONE | FRC_IF_TXABORTED | FRC_IF_TXUF,
                );
                write(L.rac + RAC_CMD, RAC_CMD_CLEARTXEN | RAC_CMD_TXDIS);
            }
            wait_off_releasing(200);
            // The frame is queued while the receiver settles, in a step of
            // its own.
            end_ack(s, sent, cycles().wrapping_add(us_to_cycles(QUEUE_AFTER_US)));
        }
        Step::Queue => {
            queue_frame(s);
            if s.ack.rx_on {
                s.ack.step = Step::RxSynth;
                s.ack.at = s.ack.rx_at;
            } else {
                s.ack.step = Step::Idle;
            }
        }
        Step::RxSynth => {
            let locked = start_synth(true, &mut s.fcal[0]);
            release_warm(RacState::RxWarm, Warm::Rx);
            s.ack.step = Step::Idle;
            if locked.is_err() {
                lose_rx(s);
            }
        }
    }
}

/// What the interrupt does with one complete frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// Deliver; no ACK needed.
    Keep,
    /// ACK frame or a frame for another node/PAN.
    Filter,
    /// Deliver after an Imm-Ack with this Frame Pending bit; `poll` is the
    /// source of a MAC Data Request.
    Ack {
        frame_pending: bool,
        poll: Option<FpAddress>,
    },
}

fn classify(header: &[u8], stored: usize, filter: &AckFilter, table: &FpTable) -> Verdict {
    if header[0] & 0x07 == 0x02 || foreign(header, filter) {
        return Verdict::Filter;
    }
    if !needs_ack(header, filter) {
        return Verdict::Keep;
    }
    let poll = data_request_source(header, stored);
    let frame_pending = poll.is_some_and(|src| table.contains(src));
    Verdict::Ack {
        frame_pending,
        poll,
    }
}

/// Examine the complete frames in BUFC, in order, until one needs an ACK:
/// that one starts the ACK and stays at the head of BUFC.
fn examine(s: &mut Shared, entry: u32, rx_done: bool) {
    let rx = buf(1);
    // SAFETY: called by the handlers or under the lock.
    let q = unsafe { queue() };
    // Frames inside the buffer at entry completed no later than `entry`
    // (or than the start of the lock that delayed this interrupt); later
    // ones no earlier than `entry` minus the BUFC catch-up.
    let readoff0 = unsafe { read(rx + BUF_READOFFSET) } & BUF_OFFSET_MASK;
    let available0 = unsafe { read(rx + BUF_STATUS) } & BUF_STATUS_BYTES;
    let old_base = if s.oldest_valid { s.oldest } else { entry };
    let mut waited = !rx_done;
    loop {
        let available = unsafe { read(rx + BUF_STATUS) } & BUF_STATUS_BYTES;
        if available == 0 {
            return;
        }
        let offset = unsafe { read(rx + BUF_READOFFSET) } & BUF_OFFSET_MASK;
        let phr = u32::from(rx_ram_byte(offset)) & 0x7F;
        if phr < 5 {
            // Shorter than an ACK: corrupt length, resynchronise.
            unsafe { write(rx + BUF_CMD, BUF_CMD_CLEAR) };
            s.stats.resyncs = s.stats.resyncs.wrapping_add(1);
            return;
        }
        let needed = stored_len(phr);
        if needed > available {
            if waited {
                return;
            }
            waited = true;
            let grace = Deadline::after_us(BUFC_CATCH_UP_US);
            while unsafe { read(rx + BUF_STATUS) } & BUF_STATUS_BYTES < needed {
                if grace.expired() {
                    break;
                }
            }
            continue;
        }
        let stored = (phr - 2) as usize;
        let mut header = [0u8; HEADER_LEN];
        let len = stored.min(HEADER_LEN);
        for (i, byte) in header.iter_mut().take(len).enumerate() {
            *byte = rx_ram_byte(offset + 1 + i as u32);
        }
        let existed = offset.wrapping_sub(readoff0) & BUF_OFFSET_MASK < available0;
        // The frame ended between `earliest` and `latest`: in the buffer at
        // entry, between the start of the lock that delayed this
        // interrupt (or the entry) and the entry; otherwise since the entry.
        let (earliest, latest) = if existed {
            (old_base, entry)
        } else {
            (entry, cycles())
        };
        let start = ack_start(earliest, latest);
        let verdict = classify(&header[..len], stored, &s.filter, &s.table);
        let room = admits(q.free(), stored, matches!(verdict, Verdict::Ack { .. }));
        let keep = match verdict {
            Verdict::Filter => {
                s.stats.filtered = s.stats.filtered.wrapping_add(1);
                false
            }
            Verdict::Keep => {
                if !room {
                    s.stats.queue_full = s.stats.queue_full.wrapping_add(1);
                }
                room
            }
            // Behind an ACK (`queue_frame`), an ACK request is always late.
            Verdict::Ack { .. }
                if s.ack.step != Step::Idle
                    || !s.rx_armed
                    || start.is_none()
                    || cycles().wrapping_sub(earliest) > us_to_cycles(ACK_DEADLINE_US) =>
            {
                s.stats.late = s.stats.late.wrapping_add(1);
                false
            }
            Verdict::Ack { .. } if !room => {
                // Unacknowledged: the sender retries once the stack has
                // read the queue.
                s.stats.queue_full_unacked = s.stats.queue_full_unacked.wrapping_add(1);
                false
            }
            Verdict::Ack { frame_pending, .. } => {
                let frame = Held {
                    offset,
                    needed,
                    stored: stored as u8,
                    flags: record_flags(verdict),
                    frame_pending,
                };
                let start = start.unwrap_or(latest);
                if !start_ack(s, header[2], frame, earliest, start) {
                    end_ack(s, false, cycles());
                }
                // The frame and the rest of BUFC now belong to the ACK.
                return;
            }
        };
        if keep {
            let rssi = rx_ram_byte(offset + 1 + stored as u32);
            q.push(rx_ram_runs(offset + 1, stored), rssi, record_flags(verdict));
        }
        // The frame is copied (or dropped): hand its bytes back to BUFC.
        unsafe { write(rx + BUF_READOFFSET, (offset + needed) & BUF_OFFSET_MASK) };
    }
}

/// Start the Imm-Ack `[FCF, seq]` for `frame` (aged from `base`, TX start
/// at `start`, see [`ack_start`]): receiver off, the ACK into the TX
/// buffer, `TXEN`; the RSM then waits in the held `TXWARM` for the
/// `TxSynth` step. `false` (RSM not off, or not in `TXWARM`) leaves the
/// radio idle.
fn start_ack(s: &mut Shared, sequence: u8, frame: Held, base: u32, start: u32) -> bool {
    s.ack = InFlight {
        step: Step::Idle,
        at: 0,
        start,
        ramp: 0,
        base,
        frame,
        ..InFlight::IDLE
    };
    unsafe {
        write(L.rac + RAC_RXENSRCEN, 0);
        write(
            L.rac + RAC_CMD,
            RAC_CMD_RXDIS | RAC_CMD_TXDIS | RAC_CMD_CLEARTXEN,
        );
    }
    if !wait_off_releasing(60) {
        unsafe { write(L.frc + FRC_CMD, FRC_CMD_RXABORT) };
        if !wait_off_releasing(100) {
            return false;
        }
    }
    let tx = buf(0);
    unsafe {
        write(tx + BUF_CMD, BUF_CMD_CLEAR);
        write(tx + BUF_WRITEDATA, 5);
        write(
            tx + BUF_WRITEDATA,
            0x02 | (u32::from(frame.frame_pending) << 4),
        );
        write(tx + BUF_WRITEDATA, 0x00);
        write(tx + BUF_WRITEDATA, u32::from(sequence));
        write(
            L.frc + FRC_IF + CLR,
            FRC_IF_TXDONE | FRC_IF_TXABORTED | FRC_IF_TXUF,
        );
    }
    apply_state(&chip::TX);
    chip::tx_state_power(s.power_ddbm);
    chip::seq_arm(Warm::Tx);
    chip::seq_arm(Warm::TxPd);
    unsafe { write(L.rac + RAC_CMD, RAC_CMD_TXEN) };
    if enter_warm(RacState::TxWarm, Warm::Tx).is_err() {
        unsafe { write(L.rac + RAC_CMD, RAC_CMD_CLEARTXEN | RAC_CMD_TXDIS) };
        radio_off();
        return false;
    }
    s.ack.step = Step::TxSynth;
    s.ack.at = cycles().wrapping_add(us_to_cycles(WARM_SETTLE_US));
    true
}

/// The ACK for the held frame is over (`sent`: it went on air) and the RSM
/// is off: restart the receiver and queue the frame in the `Queue` step at
/// `queue_at`.
fn end_ack(s: &mut Shared, sent: bool, queue_at: u32) {
    s.ack.sent = sent;
    begin_rx(s);
    s.ack.step = Step::Queue;
    s.ack.at = queue_at;
}

/// Deliver the held frame if its ACK went out (count a failure otherwise),
/// hand its bytes back to BUFC and examine the frames behind it.
fn queue_frame(s: &mut Shared) {
    let frame = s.ack.frame;
    if s.ack.sent {
        s.stats.acks = s.stats.acks.wrapping_add(1);
        if frame.frame_pending {
            s.stats.fp_acks = s.stats.fp_acks.wrapping_add(1);
        }
        let stored = usize::from(frame.stored);
        let rssi = rx_ram_byte(frame.offset + 1 + stored as u32);
        let mpdu = rx_ram_runs(frame.offset + 1, stored);
        // SAFETY: called by the handlers or under the lock. The room was
        // checked when the ACK started; only the task took frames since.
        unsafe { queue() }.push(mpdu, rssi, frame.flags);
    } else {
        s.stats.ack_tx_fail = s.stats.ack_tx_fail.wrapping_add(1);
    }
    unsafe {
        write(
            buf(1) + BUF_READOFFSET,
            (frame.offset + frame.needed) & BUF_OFFSET_MASK,
        )
    };
    examine(s, s.ack.base, false);
}

/// Stop the synthesizer and park the analog front end (`Radio::idle`).
fn radio_off() {
    unsafe {
        write(L.rac + RAC_RXENSRCEN, 0);
        write(
            L.rac + RAC_CMD,
            RAC_CMD_RXDIS | RAC_CMD_TXDIS | RAC_CMD_CLEARTXEN,
        );
    }
    wait_off_releasing(200);
    unsafe { write(L.synth + SYNTH_CMD, SYNTH_CMD_STOP) };
    apply_state(&chip::IDLE);
}

/// Re-enter RX after an ACK: the RSM waits in the held `RXWARM` for the
/// `RxSynth` step (`rx_on`, due at `rx_at`). On failure leave the radio
/// idle and report the receiver lost to the task.
fn begin_rx(s: &mut Shared) {
    s.ack.rx_on = false;
    if s.rx_armed && (rac_state_raw() == RacState::Off || wait_off_releasing(200)) {
        apply_state(&chip::RX);
        chip::seq_arm(Warm::Rx);
        unsafe { write(L.rac + RAC_RXENSRCEN, RAC_SWRXEN) };
        if enter_warm(RacState::RxWarm, Warm::Rx).is_ok() {
            s.ack.rx_on = true;
            s.ack.rx_at = cycles().wrapping_add(us_to_cycles(WARM_SETTLE_US));
            return;
        }
    }
    lose_rx(s);
}

fn lose_rx(s: &mut Shared) {
    radio_off();
    s.rx_armed = false;
    RX_LOST.store(true, Ordering::Relaxed);
    s.stats.rx_restart_fail = s.stats.rx_restart_fail.wrapping_add(1);
}

/// Whether a frame drops out as foreign: an MHR whose destination PAN or
/// address differs from the (known) local one. Beacons and other frames
/// without a destination are kept; so are 2015 frames (frame version 2),
/// whose PAN ID compression rules this check does not implement.
fn foreign(header: &[u8], filter: &AckFilter) -> bool {
    if header.len() < 3 {
        return true;
    }
    let fcf = u16::from_le_bytes([header[0], header[1]]);
    if (fcf >> 12) & 0x3 == 2 {
        return false;
    }
    let dst_mode = (fcf >> 10) & 0x3;
    if dst_mode == 0 {
        return false;
    }
    let Some(pan) = header.get(3..5) else {
        return true;
    };
    let pan = u16::from_le_bytes([pan[0], pan[1]]);
    if pan != 0xFFFF && filter.pan_id.is_some_and(|own| own != pan) {
        return true;
    }
    match dst_mode {
        2 => match header.get(5..7) {
            Some(short) => {
                let short = u16::from_le_bytes([short[0], short[1]]);
                short != 0xFFFF && filter.short_address.is_some_and(|own| own != short)
            }
            None => true,
        },
        3 => match header.get(5..13) {
            Some(ext) => {
                let mut bytes = [0u8; 8];
                bytes.copy_from_slice(ext);
                filter
                    .extended_address
                    .is_some_and(|own| own != u64::from_le_bytes(bytes))
            }
            None => true,
        },
        // Reserved addressing mode.
        _ => true,
    }
}

/// Source of a MAC Data Request command (2003/2006 frame), `None` for any
/// other frame. Mirrors the strict parse of the TLSR8258 HAL: unsecured
/// command frame with an ACK request, short or extended destination and
/// source, one PAN, and exactly the command identifier as payload.
fn data_request_source(header: &[u8], stored: usize) -> Option<FpAddress> {
    if header.len() < 3 {
        return None;
    }
    let fcf = u16::from_le_bytes([header[0], header[1]]);
    let version = (fcf >> 12) & 0x3;
    if fcf & 0x07 != 3 || fcf & (1 << 3) != 0 || fcf & (1 << 5) == 0 || version > 1 {
        return None;
    }
    let dst_len = match (fcf >> 10) & 0x3 {
        2 => 2,
        3 => 8,
        _ => return None,
    };
    let src_len = match (fcf >> 14) & 0x3 {
        2 => 2,
        3 => 8,
        _ => return None,
    };
    let dst_pan = header.get(3..5)?;
    let dst_pan = u16::from_le_bytes([dst_pan[0], dst_pan[1]]);
    let mut index = 5 + dst_len;
    if fcf & (1 << 6) == 0 {
        let src_pan = header.get(index..index + 2)?;
        if u16::from_le_bytes([src_pan[0], src_pan[1]]) != dst_pan {
            return None;
        }
        index += 2;
    }
    if dst_pan == 0xFFFF {
        return None;
    }
    let src = header.get(index..index + src_len)?;
    index += src_len;
    if stored != index + 1 || header.get(index) != Some(&0x04) {
        return None;
    }
    Some(if src_len == 2 {
        FpAddress::Short(u16::from_le_bytes([src[0], src[1]]))
    } else {
        let mut bytes = [0u8; 8];
        bytes.copy_from_slice(src);
        FpAddress::Extended(u64::from_le_bytes(bytes))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const PAN: u16 = 0x1A62;
    const OWN_SHORT: u16 = 0x0000;
    const OWN_EXT: u64 = 0x842E_14FF_FE40_AB35;
    const CHILD_EXT: u64 = 0x086B_D7FF_FEC3_9012;

    fn filter() -> AckFilter {
        AckFilter {
            pan_id: Some(PAN),
            short_address: Some(OWN_SHORT),
            extended_address: Some(OWN_EXT),
        }
    }

    /// Data Request, PAN-compressed, short dst / short src.
    fn poll_short(src: u16) -> [u8; 10] {
        let [p0, p1] = PAN.to_le_bytes();
        let [d0, d1] = OWN_SHORT.to_le_bytes();
        let [s0, s1] = src.to_le_bytes();
        [0x63, 0x88, 0x42, p0, p1, d0, d1, s0, s1, 0x04]
    }

    /// Data Request from an associating child: short dst, extended src,
    /// no PAN compression (src PAN equals dst PAN).
    fn poll_ext(src: u64, compressed: bool) -> ([u8; HEADER_LEN], usize) {
        let mut frame = [0u8; HEADER_LEN];
        let fcf: u16 = 0x0003 | 1 << 5 | (u16::from(compressed) << 6) | 2 << 10 | 3 << 14;
        frame[..2].copy_from_slice(&fcf.to_le_bytes());
        frame[2] = 0x17;
        frame[3..5].copy_from_slice(&PAN.to_le_bytes());
        frame[5..7].copy_from_slice(&OWN_SHORT.to_le_bytes());
        let mut len = 7;
        if !compressed {
            frame[7..9].copy_from_slice(&PAN.to_le_bytes());
            len = 9;
        }
        frame[len..len + 8].copy_from_slice(&src.to_le_bytes());
        frame[len + 8] = 0x04;
        (frame, len + 9)
    }

    /// The step stamps keep their order across the 32-bit counter wrap
    /// (every ~110 s), and the PA steps finish before the TX start.
    #[test]
    fn step_stamps_across_the_counter_wrap() {
        for start in [10_000u32, 3, u32::MAX - 5] {
            let ramp = ramp_at(start);
            assert!(after(start, ramp));
            // The ramp itself ends before the TX start.
            let ramp_end = ramp.wrapping_add(us_to_cycles(chip::PA_RAMP_LEAD_US));
            assert!(after(start, ramp_end));
            // A pre-stage that started late pushes the ramp, never the
            // other way round.
            let prestage = ramp.wrapping_sub(us_to_cycles(chip::PA_PRESTAGE_US));
            let late = prestage.wrapping_add(100);
            let settled = late.wrapping_add(us_to_cycles(chip::PA_PRESTAGE_US));
            assert_eq!(later(settled, ramp), settled);
            assert_eq!(later(ramp, settled), settled);
            assert_eq!(later(prestage, ramp), ramp);
        }
        assert!(!after(5, 5));
        assert!(after(2, u32::MAX - 2));
        assert!(!after(u32::MAX - 2, 2));
    }

    /// `TxStart` is due before the TX start in either half of the counter.
    /// A chip without a CPU ramp (xG22) ignores the ramp stamp: zero there
    /// once scheduled `TxStart` at stamp 0, half a counter period (about
    /// 56 s) ahead when the counter was in its upper half, with the RSM held
    /// in `TXWARM` and a task-side lock spinning on it.
    #[test]
    fn tx_start_is_due_before_the_tx_start_in_either_counter_half() {
        let margin = us_to_cycles(START_MARGIN_US);
        for start in [10_000u32, (1 << 31) + 1_000, u32::MAX - 5] {
            let due = tx_start_due(ramp_at(start), start);
            assert!(!after(due, start));
            assert!(start.wrapping_sub(due) <= margin + 1);
            // A ramp that started late delays `TxStart` until it is steady.
            let late = start.wrapping_sub(margin);
            let lead = us_to_cycles(chip::PA_RAMP_LEAD_US);
            assert_eq!(
                tx_start_due(late, start),
                later(late.wrapping_add(lead), start.wrapping_sub(margin))
            );
            if chip::PA_RAMP_LEAD_US == 0 {
                assert_eq!(tx_start_due(0, start), start.wrapping_sub(margin));
            }
        }
    }

    /// A frame that completed while the task held the radio may have ended
    /// just before the lock's release: its ACK starts [`ACK_START_US`] after
    /// the interrupt's entry, not after the lock's start (which is already
    /// past and would put the ACK on air early), and is not sent when that
    /// is later than [`ACK_LATEST_US`] after the lock's start.
    #[test]
    fn a_lock_aged_frame_is_acknowledged_from_its_latest_end() {
        let turnaround = us_to_cycles(ACK_START_US);
        for entry in [50_000u32, (1 << 31) + 1_000, 7] {
            // No lock: the frame ended at the entry.
            assert_eq!(
                ack_start(entry, entry),
                Some(entry.wrapping_add(turnaround))
            );
            // It completed during a 300 us lock.
            let lock = entry.wrapping_sub(us_to_cycles(300));
            assert_eq!(ack_start(lock, entry), Some(entry.wrapping_add(turnaround)));
            // A lock so long that the ACK would start too late for a frame
            // that ended at its start: no ACK.
            let long = entry.wrapping_sub(us_to_cycles(ACK_LATEST_US - ACK_START_US + 2));
            assert_eq!(ack_start(long, entry), None);
        }
    }

    #[test]
    fn data_request_sources() {
        let poll = poll_short(0x7C01);
        assert_eq!(
            data_request_source(&poll, poll.len()),
            Some(FpAddress::Short(0x7C01))
        );
        for compressed in [true, false] {
            let (poll, len) = poll_ext(CHILD_EXT, compressed);
            assert_eq!(
                data_request_source(&poll[..len], len),
                Some(FpAddress::Extended(CHILD_EXT))
            );
        }
    }

    #[test]
    fn non_data_requests_have_no_source() {
        let poll = poll_short(0x7C01);
        // Trailing payload byte, or a different command.
        assert_eq!(data_request_source(&poll, poll.len() + 1), None);
        let mut other = poll;
        other[9] = 0x07;
        assert_eq!(data_request_source(&other, other.len()), None);
        // Data frame with the same addressing.
        let mut data = poll;
        data[0] = 0x61;
        assert_eq!(data_request_source(&data, data.len()), None);
        // Secured, no ACK request, 2015 frame version.
        for (byte, value) in [(0, 0x6B), (0, 0x43), (1, 0xA8)] {
            let mut frame = poll;
            frame[byte] = value;
            assert_eq!(data_request_source(&frame, frame.len()), None);
        }
        // Mismatched source PAN.
        let (mut frame, len) = poll_ext(CHILD_EXT, false);
        frame[7] ^= 1;
        assert_eq!(data_request_source(&frame[..len], len), None);
        // Truncated header.
        assert_eq!(data_request_source(&poll[..6], poll.len()), None);
    }

    #[test]
    fn frame_pending_table() {
        let mut table = FpTable::EMPTY;
        assert!(!table.contains(FpAddress::Short(0x1234)));
        assert_eq!(table.set(FpAddress::Short(0x1234), true), Ok(()));
        assert_eq!(table.set(FpAddress::Short(0x1234), true), Ok(()));
        assert!(table.contains(FpAddress::Short(0x1234)));
        assert!(!table.contains(FpAddress::Extended(0x1234)));
        assert_eq!(table.set(FpAddress::Short(0x1234), false), Ok(()));
        assert!(!table.contains(FpAddress::Short(0x1234)));
        // Clearing an absent address is not an error.
        assert_eq!(table.set(FpAddress::Short(0x1234), false), Ok(()));
        for i in 0..FP_TABLE_LEN as u64 {
            assert_eq!(table.set(FpAddress::Extended(i + 1), true), Ok(()));
        }
        assert_eq!(
            table.set(FpAddress::Extended(100), true),
            Err(FpError::TableFull)
        );
        // A full table still accepts an armed address and a clear.
        assert_eq!(table.set(FpAddress::Extended(3), true), Ok(()));
        assert_eq!(table.set(FpAddress::Extended(3), false), Ok(()));
        assert_eq!(table.set(FpAddress::Extended(100), true), Ok(()));
        assert!(table.contains(FpAddress::Extended(100)));
        // Sentinels are rejected and never match.
        assert_eq!(
            table.set(FpAddress::Short(0xFFFF), true),
            Err(FpError::InvalidAddress)
        );
        assert_eq!(
            table.set(FpAddress::Short(0xFFFE), true),
            Err(FpError::InvalidAddress)
        );
        assert_eq!(
            table.set(FpAddress::Extended(0), true),
            Err(FpError::InvalidAddress)
        );
        assert!(!table.contains(FpAddress::Short(0xFFFF)));
    }

    #[test]
    fn verdicts() {
        let mut table = FpTable::EMPTY;
        let f = filter();
        let poll = poll_short(0x7C01);
        assert_eq!(
            classify(&poll, poll.len(), &f, &table),
            Verdict::Ack {
                frame_pending: false,
                poll: Some(FpAddress::Short(0x7C01)),
            }
        );
        table.set(FpAddress::Short(0x7C01), true).unwrap();
        assert_eq!(
            classify(&poll, poll.len(), &f, &table),
            Verdict::Ack {
                frame_pending: true,
                poll: Some(FpAddress::Short(0x7C01)),
            }
        );
        // A data frame from the same pending child: ACK with FP = 0.
        let mut data = [0u8; 12];
        data[..9].copy_from_slice(&poll[..9]);
        data[0] = 0x61;
        assert_eq!(
            classify(&data, data.len(), &f, &table),
            Verdict::Ack {
                frame_pending: false,
                poll: None,
            }
        );
        // Associating child, extended source armed by the association.
        let (assoc, len) = poll_ext(CHILD_EXT, false);
        table.set(FpAddress::Extended(CHILD_EXT), true).unwrap();
        assert_eq!(
            classify(&assoc[..len], len, &f, &table),
            Verdict::Ack {
                frame_pending: true,
                poll: Some(FpAddress::Extended(CHILD_EXT)),
            }
        );
        // ACK frames are dropped.
        assert_eq!(
            classify(&[0x02, 0x00, 0x42], 3, &f, &table),
            Verdict::Filter
        );
        // Broadcast data, no ACK request: delivered.
        let broadcast = [0x41, 0x88, 0x01, 0x62, 0x1A, 0xFF, 0xFF, 0x01, 0x7C, 0x08];
        assert_eq!(
            classify(&broadcast, broadcast.len(), &f, &table),
            Verdict::Keep
        );
        // Beacon (no destination): delivered.
        let beacon = [0x00, 0x80, 0x07, 0x62, 0x1A, 0x00, 0x00, 0xFF, 0xCF];
        assert_eq!(classify(&beacon, beacon.len(), &f, &table), Verdict::Keep);
        // Unicast to another node, and to another PAN: dropped.
        let mut other = poll;
        other[5] = 0x99;
        assert_eq!(classify(&other, other.len(), &f, &table), Verdict::Filter);
        let mut other_pan = poll;
        other_pan[3] ^= 0x01;
        assert_eq!(
            classify(&other_pan, other_pan.len(), &f, &table),
            Verdict::Filter
        );
        // Promiscuous (unjoined) filter keeps and never ACKs.
        let none = AckFilter::default();
        assert_eq!(classify(&other, other.len(), &none, &table), Verdict::Keep);
    }

    /// Classify `frame` and queue it as the interrupt does once its ACK
    /// (if any) was sent.
    fn interrupt_keeps(q: &mut RxQueue, frame: &[u8], f: &AckFilter, table: &FpTable) -> Verdict {
        let verdict = classify(frame, frame.len(), f, table);
        q.push([frame, &[]], 0xC4, record_flags(verdict));
        verdict
    }

    fn empty_queue() -> RxQueue {
        RxQueue {
            bytes: [0; RX_QUEUE_LEN],
            head: 0,
            used: 0,
        }
    }

    /// `(source, acked without FP)` of the next queued record.
    fn next_poll(q: &mut RxQueue) -> Option<(u16, bool)> {
        let mut out = [0u8; MAX_FRAME];
        let (len, _, flags) = q.pop(&mut out)?;
        assert_eq!(len, 10, "a short-source poll");
        Some((
            u16::from_le_bytes([out[7], out[8]]),
            flags == ACKED_WITHOUT_FP,
        ))
    }

    /// The hardware sequence of a sleepy child's secured rejoin: it polls
    /// before its parent queued the Rejoin Response (ACK with FP = 0, the
    /// child turns its receiver off), the response is queued and armed,
    /// and the child's next poll is acknowledged with FP = 1. Each poll
    /// reaches the stack with its own ACK's decision.
    #[test]
    fn each_queued_poll_carries_its_ack_decision() {
        let f = filter();
        let mut table = FpTable::EMPTY;
        let mut q = empty_queue();
        let child = FpAddress::Short(0x12D2);
        interrupt_keeps(&mut q, &poll_short(0x12D2), &f, &table);
        // The response is queued after that ACK, before the stack reads
        // the stale poll.
        table.set(child, true).unwrap();
        interrupt_keeps(&mut q, &poll_short(0x12D2), &f, &table);
        assert_eq!(next_poll(&mut q), Some((0x12D2, true)));
        assert_eq!(next_poll(&mut q), Some((0x12D2, false)));
        // Address 0 is a valid child.
        interrupt_keeps(&mut q, &poll_short(0x0000), &f, &table);
        assert_eq!(next_poll(&mut q), Some((0x0000, true)));
        // An extended-source poll (an Association Response fetch) carries
        // its decision too.
        let (ext, len) = poll_ext(CHILD_EXT, true);
        interrupt_keeps(&mut q, &ext[..len], &f, &table);
        table.set(FpAddress::Extended(CHILD_EXT), true).unwrap();
        interrupt_keeps(&mut q, &ext[..len], &f, &table);
        let mut out = [0u8; MAX_FRAME];
        assert_eq!(q.pop(&mut out), Some((len, 0xC4u8 as i8, ACKED_WITHOUT_FP)));
        assert_eq!(q.pop(&mut out), Some((len, 0xC4u8 as i8, 0)));
        // Frames that are not polls never carry the flag: a data frame
        // (ACKed with FP = 0) and a broadcast (no ACK).
        let mut data = poll_short(0x12D2);
        data[0] = 0x61;
        interrupt_keeps(&mut q, &data, &f, &table);
        let broadcast = [0x41, 0x88, 0x01, 0x62, 0x1A, 0xFF, 0xFF, 0x01, 0x7C, 0x08];
        interrupt_keeps(&mut q, &broadcast, &f, &table);
        assert_eq!(q.pop(&mut out).map(|(_, _, flags)| flags), Some(0));
        assert_eq!(q.pop(&mut out).map(|(_, _, flags)| flags), Some(0));
        assert_eq!(q.pop(&mut out), None);
    }

    /// More children than the Frame Pending table holds, sleepy children
    /// that never get data, and children that changed address: the ACK
    /// decision lives in the poll's own record, so none of them can use
    /// up shared state (a per-source record table filled with the first
    /// eight children that polled without data and lost every later one).
    #[test]
    fn every_child_keeps_its_ack_decision_beyond_the_table_size() {
        let f = filter();
        let mut table = FpTable::EMPTY;
        let mut q = empty_queue();
        let children = 0x1000u16..0x1000 + 3 * FP_TABLE_LEN as u16;
        // Every child polls without data, twice.
        for _ in 0..2 {
            for child in children.clone() {
                interrupt_keeps(&mut q, &poll_short(child), &f, &table);
            }
        }
        for _ in 0..2 {
            for child in children.clone() {
                assert_eq!(next_poll(&mut q), Some((child, true)));
            }
        }
        // Data queued for the last eight children fills the table.
        for child in children.clone().skip(2 * FP_TABLE_LEN) {
            table.set(FpAddress::Short(child), true).unwrap();
        }
        for child in children.clone() {
            interrupt_keeps(&mut q, &poll_short(child), &f, &table);
        }
        for child in children.clone() {
            let armed = child >= 0x1000 + 2 * FP_TABLE_LEN as u16;
            assert_eq!(next_poll(&mut q), Some((child, !armed)));
        }
        // Child 0x1003 rejoined as 0x7A01 and child 0x1010 left with its
        // data dropped: the new address's polls carry their own decisions,
        // and nothing of the old addresses is left to release.
        table.set(FpAddress::Short(0x1010), false).unwrap();
        interrupt_keeps(&mut q, &poll_short(0x7A01), &f, &table);
        table.set(FpAddress::Short(0x7A01), true).unwrap();
        interrupt_keeps(&mut q, &poll_short(0x7A01), &f, &table);
        assert_eq!(next_poll(&mut q), Some((0x7A01, true)));
        assert_eq!(next_poll(&mut q), Some((0x7A01, false)));
        assert_eq!(next_poll(&mut q), None);
    }

    #[test]
    fn rx_queue_wraps_in_order_and_reports_room() {
        let mut q = RxQueue {
            bytes: [0; RX_QUEUE_LEN],
            head: 0,
            used: 0,
        };
        let mut out = [0u8; MAX_FRAME];
        assert_eq!(q.pop(&mut out), None);
        // Fill with 125-byte frames (127-byte records) until no room.
        let mut pushed = 0u8;
        while q.free() >= record_len(125) {
            let frame: [u8; 125] = core::array::from_fn(|i| i as u8 ^ pushed);
            // Split like a frame that wraps around the end of BUFC.
            q.push([&frame[..40], &frame[40..]], pushed, 0);
            pushed += 1;
        }
        assert_eq!(pushed as usize, RX_QUEUE_LEN / record_len(125));
        // Drain two, then refill across the end of the ring.
        for n in 0..2u8 {
            assert_eq!(q.pop(&mut out), Some((125, n as i8, 0)));
            assert!(
                out[..125]
                    .iter()
                    .enumerate()
                    .all(|(i, &b)| b == i as u8 ^ n)
            );
        }
        q.push([&[0x41, 0x88, 0x07], &[]], 0xC4, ACKED_WITHOUT_FP);
        let mut next = 2u8;
        while next < pushed {
            assert_eq!(q.pop(&mut out), Some((125, next as i8, 0)));
            assert_eq!(out[124], 124 ^ next);
            next += 1;
        }
        assert_eq!(q.pop(&mut out), Some((3, 0xC4u8 as i8, ACKED_WITHOUT_FP)));
        assert_eq!(&out[..3], &[0x41, 0x88, 0x07]);
        assert_eq!(q.pop(&mut out), None);
        assert_eq!(q.free(), RX_QUEUE_LEN);
    }

    #[test]
    fn queue_reserves_room_for_acked_frames() {
        let max = MAX_FRAME;
        assert_eq!(ACKED_RESERVE, 2 * record_len(max));
        // Broadcasts stop at the reserve; unicasts may use it.
        let free = ACKED_RESERVE + record_len(max) - 1;
        assert!(!admits(free, max, false));
        assert!(admits(free, max, true));
        assert!(admits(free, 10, false));
        // Two maximum-length unicasts always fit after broadcasts stopped.
        let mut free = ACKED_RESERVE;
        for _ in 0..2 {
            assert!(admits(free, max, true));
            free -= record_len(max);
        }
        assert!(!admits(free, 3, true));
    }

    #[test]
    fn extended_destination_filter() {
        let f = filter();
        let mut frame = [0u8; 15];
        let fcf: u16 = 0x0003 | 1 << 5 | 1 << 6 | 3 << 10 | 3 << 14;
        frame[..2].copy_from_slice(&fcf.to_le_bytes());
        frame[3..5].copy_from_slice(&PAN.to_le_bytes());
        frame[5..13].copy_from_slice(&OWN_EXT.to_le_bytes());
        assert!(!foreign(&frame, &f));
        frame[5] ^= 1;
        assert!(foreign(&frame, &f));
        // Truncated destination.
        assert!(foreign(&frame[..9], &f));
    }
}
