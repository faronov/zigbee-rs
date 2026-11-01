//! Interrupt-driven Imm-Ack with a HAL-owned Frame Pending table
//! (`isr-ack` feature, parent/router images).
//!
//! The `FRC` RX-done interrupt examines every newly completed frame in the
//! RX buffer, in order, and
//!
//! * transmits the Imm-Ack for a frame addressed to this node that requests
//!   one ([`super::needs_ack`]), straight from the interrupt, with the Frame
//!   Pending bit set only for a MAC Data Request whose source is in the
//!   source-match table armed through [`super::Radio::set_frame_pending`]
//!   (IEEE 802.15.4-2006 7.2.2.3.1 / 7.5.6.3; Zigbee R22/R23 parents). An
//!   ACK to any other frame, data frames included, carries FP = 0;
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
//! Every task-side radio operation runs with the `FRC` NVIC line masked
//! (`Radio::locked`), so the interrupt never sees the radio mid-operation;
//! the lock release hands over the RX state, channel, TX power and ACK
//! filter, and re-pends the interrupt when a frame completed meanwhile. A
//! frame that completed while the radio was locked is aged from the start
//! of the lock, so an ACK is never sent later than [`ACK_DEADLINE_US`]
//! (plus the ACK warm-up) after the frame ended.
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
    SYNTH_CMD, SYNTH_CMD_STOP, Warm, apply_state, buf, chip, needs_ack, rac_state_raw, read,
    rx_ram_byte, service_sequencer, tx_power_down, wait_off_releasing, warm_up, write,
};
use crate::delay::{Deadline, cycles, cycles_to_us, spin_cycles, us_to_cycles};

/// Bytes of the queue of received frames for the stack.
pub(super) const RX_QUEUE_LEN: usize = 1024;

/// Latest ACK decision after the end of a frame. The ACK then needs its
/// TX warm-up (synthesizer lock); the sender waits macAckWaitDuration
/// (864 us) for the ACK's PHR, i.e. ~670 us for the ACK to start.
const ACK_DEADLINE_US: u32 = 400;
/// MHR bytes the interrupt reads (the longest Data Request MHR is 23).
const HEADER_LEN: usize = 24;
/// Grace for BUFC to store a frame the FRC already reported done.
const BUFC_CATCH_UP_US: u32 = 40;
/// 5-byte ACK airtime (352 us) with warm-down margin.
const ACK_TX_TIMEOUT_US: u32 = 1_000;

/// `FRC` is IRQ 34 on xG21 and xG22 (`FRC_PRI` is 33).
const FRC_IRQ_BIT: u32 = 1 << (34 - 32);
const NVIC_ICER1: u32 = 0xE000_E184;
const NVIC_ISPR1: u32 = 0xE000_E204;

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

/// State shared by the task (only while the `FRC` line is masked) and the
/// `FRC` interrupt.
struct Shared {
    /// `Radio::locked` nesting depth.
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
}

struct SharedCell(UnsafeCell<Shared>);

// SAFETY: accessed only from the `FRC` handler and from task code that has
// masked the `FRC` NVIC line first (`lock`), or before the line is ever
// enabled (`RadioToken::init`); the two never overlap.
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
    },
}));

/// Set by the interrupt when it could not restart the receiver after an
/// ACK; read without the lock by `Radio::is_rx_on`.
static RX_LOST: AtomicBool = AtomicBool::new(false);

/// Received frames for the stack, as `[len, MPDU without FCS, raw RSSI]`
/// records in a byte ring.
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

    /// Append a record; the caller checked [`free`](Self::free).
    fn push(&mut self, mpdu: impl Iterator<Item = u8>, len: usize, rssi: u8) {
        self.put(len as u8);
        for byte in mpdu.take(len) {
            self.put(byte);
        }
        self.put(rssi);
    }

    /// Remove the oldest record into `out`: `(len, raw RSSI)`.
    fn pop(&mut self, out: &mut [u8; MAX_FRAME]) -> Option<(usize, i8)> {
        if self.used == 0 {
            return None;
        }
        let len = usize::from(self.take());
        for slot in out.iter_mut().take(len) {
            *slot = self.take();
        }
        Some((len, self.take() as i8))
    }
}

/// Bytes a queued frame of `len` MPDU bytes occupies.
const fn record_len(len: usize) -> usize {
    len + 2
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
/// Caller is the `FRC` handler, or task code holding the lock (or running
/// before the `FRC` line is first enabled).
#[allow(clippy::mut_from_ref)]
unsafe fn shared() -> &'static mut Shared {
    unsafe { &mut *SHARED.0.get() }
}

fn mask() {
    unsafe { write(NVIC_ICER1, FRC_IRQ_BIT) };
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    }
}

pub(super) fn lock(radio: &mut Radio) {
    mask();
    // SAFETY: the `FRC` line is masked.
    let s = unsafe { shared() };
    if s.depth == 0 {
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
    // SAFETY: the `FRC` line is still masked.
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
        write(NVIC_ISER1, FRC_IRQ_BIT);
    }
}

pub(super) fn rx_lost() -> bool {
    RX_LOST.load(Ordering::Relaxed)
}

pub(super) fn set_frame_pending(address: FpAddress, pending: bool) -> Result<(), FpError> {
    // SAFETY: called under the lock.
    unsafe { shared() }.table.set(address, pending)
}

pub(super) fn clear_frame_pending() {
    // SAFETY: called under the lock.
    unsafe { shared() }.table = FpTable::EMPTY;
}

/// Snapshot of the counters. Only masks the line around the copy: the
/// task never holds the lock across an `&Radio` borrow, so the line is
/// re-enabled exactly as `unlock` would leave it.
pub(super) fn stats() -> AckStats {
    mask();
    // SAFETY: the `FRC` line is masked.
    let s = unsafe { shared() };
    let stats = s.stats;
    if s.depth == 0 {
        unsafe { write(NVIC_ISER1, FRC_IRQ_BIT) };
    }
    stats
}

/// Next queued frame (MPDU without FCS) and its raw RSSI.
pub(super) fn pop(out: &mut [u8; MAX_FRAME]) -> Option<(usize, i8)> {
    // SAFETY: called under the lock.
    unsafe { queue() }.pop(out)
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
    if flags & FRC_IF_RXOF != 0 || rac_state_raw() == RacState::RxOverflow {
        // Same recovery as `Radio::service_overflow`; the receiver stays
        // enabled through `RXENSRCEN`.
        unsafe {
            write(L.frc + FRC_CMD, FRC_CMD_RXABORT);
            write(buf(1) + BUF_CMD, BUF_CMD_CLEAR);
            write(L.rac + RAC_CMD, RAC_CMD_CLEARRXOVERFLOW);
            write(L.frc + FRC_IF + CLR, FRC_IF_RXOF);
        }
        s.stats.overflows = s.stats.overflows.wrapping_add(1);
    }
    examine(s, entry, flags & FRC_IF_RXDONE != 0);
    s.oldest_valid = false;
}

/// What the interrupt does with one complete frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Verdict {
    /// Deliver; no ACK needed.
    Keep,
    /// ACK frame or a frame for another node/PAN.
    Filter,
    /// Deliver after an Imm-Ack with this Frame Pending bit.
    Ack { frame_pending: bool },
}

fn classify(header: &[u8], stored: usize, filter: &AckFilter, table: &FpTable) -> Verdict {
    if header[0] & 0x07 == 0x02 || foreign(header, filter) {
        return Verdict::Filter;
    }
    if !needs_ack(header, filter) {
        return Verdict::Keep;
    }
    let frame_pending = data_request_source(header, stored).is_some_and(|src| table.contains(src));
    Verdict::Ack { frame_pending }
}

fn examine(s: &mut Shared, entry: u32, rx_done: bool) {
    let rx = buf(1);
    // SAFETY: this is the `FRC` handler.
    let q = unsafe { queue() };
    // Frames inside the buffer at entry completed no later than `entry`
    // (or than the start of the lock that delayed this interrupt); later
    // ones no earlier than the last receiver restart.
    let readoff0 = unsafe { read(rx + BUF_READOFFSET) } & BUF_OFFSET_MASK;
    let available0 = unsafe { read(rx + BUF_STATUS) } & BUF_STATUS_BYTES;
    let old_base = if s.oldest_valid { s.oldest } else { entry };
    let mut new_base = entry;
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
            spin_cycles(us_to_cycles(BUFC_CATCH_UP_US));
            continue;
        }
        let stored = (phr - 2) as usize;
        let mut header = [0u8; HEADER_LEN];
        let len = stored.min(HEADER_LEN);
        for (i, byte) in header.iter_mut().take(len).enumerate() {
            *byte = rx_ram_byte(offset + 1 + i as u32);
        }
        let existed = offset.wrapping_sub(readoff0) & BUF_OFFSET_MASK < available0;
        let base = if existed { old_base } else { new_base };
        let room = q.free() >= record_len(stored);
        let keep = match classify(&header[..len], stored, &s.filter, &s.table) {
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
            Verdict::Ack { .. }
                if !s.rx_armed || cycles().wrapping_sub(base) > us_to_cycles(ACK_DEADLINE_US) =>
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
            Verdict::Ack { frame_pending } => {
                let sent = send_ack(s, header[2], frame_pending, base);
                if sent {
                    s.stats.acks = s.stats.acks.wrapping_add(1);
                    if frame_pending {
                        s.stats.fp_acks = s.stats.fp_acks.wrapping_add(1);
                    }
                } else {
                    s.stats.ack_tx_fail = s.stats.ack_tx_fail.wrapping_add(1);
                }
                restart_rx(s);
                new_base = cycles();
                sent
            }
        };
        if keep {
            let mpdu = (0..stored as u32).map(|i| rx_ram_byte(offset + 1 + i));
            q.push(mpdu, stored, rx_ram_byte(offset + 1 + stored as u32));
        }
        // The frame is copied (or dropped): hand its bytes back to BUFC.
        unsafe { write(rx + BUF_READOFFSET, (offset + needed) & BUF_OFFSET_MASK) };
    }
}

/// Transmit `[FCF, seq]` Imm-Ack now. Leaves the RSM `OFF`.
fn send_ack(s: &mut Shared, sequence: u8, frame_pending: bool, base: u32) -> bool {
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
        write(tx + BUF_WRITEDATA, 0x02 | (u32::from(frame_pending) << 4));
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
    if warm_up(RacState::TxWarm, Warm::Tx, false, &mut s.fcal[1]).is_err() {
        unsafe { write(L.rac + RAC_CMD, RAC_CMD_CLEARTXEN | RAC_CMD_TXDIS) };
        radio_off();
        return false;
    }
    let turnaround = cycles_to_us(cycles().wrapping_sub(base));
    s.stats.max_turnaround_us = s.stats.max_turnaround_us.max(turnaround);
    let deadline = Deadline::after_us(ACK_TX_TIMEOUT_US);
    let sent = loop {
        service_sequencer();
        let flags = unsafe { read(L.frc + FRC_IF) };
        if flags & FRC_IF_TXDONE != 0 {
            break true;
        }
        if flags & (FRC_IF_TXABORTED | FRC_IF_TXUF) != 0 || deadline.expired() {
            break false;
        }
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
    sent
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

/// Re-enter RX search after an ACK; on failure leave the radio idle and
/// report the receiver lost to the task.
fn restart_rx(s: &mut Shared) {
    if s.rx_armed && (rac_state_raw() == RacState::Off || wait_off_releasing(200)) {
        apply_state(&chip::RX);
        chip::seq_arm(Warm::Rx);
        unsafe { write(L.rac + RAC_RXENSRCEN, RAC_SWRXEN) };
        if warm_up(RacState::RxWarm, Warm::Rx, true, &mut s.fcal[0]).is_ok() {
            return;
        }
    }
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
                frame_pending: false
            }
        );
        table.set(FpAddress::Short(0x7C01), true).unwrap();
        assert_eq!(
            classify(&poll, poll.len(), &f, &table),
            Verdict::Ack {
                frame_pending: true
            }
        );
        // A data frame from the same pending child: ACK with FP = 0.
        let mut data = [0u8; 12];
        data[..9].copy_from_slice(&poll[..9]);
        data[0] = 0x61;
        assert_eq!(
            classify(&data, data.len(), &f, &table),
            Verdict::Ack {
                frame_pending: false
            }
        );
        // Associating child, extended source armed by the association.
        let (assoc, len) = poll_ext(CHILD_EXT, false);
        table.set(FpAddress::Extended(CHILD_EXT), true).unwrap();
        assert_eq!(
            classify(&assoc[..len], len, &f, &table),
            Verdict::Ack {
                frame_pending: true
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
            q.push((0..125).map(|i| i as u8 ^ pushed), 125, pushed);
            pushed += 1;
        }
        assert_eq!(pushed as usize, RX_QUEUE_LEN / record_len(125));
        // Drain two, then refill across the end of the ring.
        for n in 0..2u8 {
            assert_eq!(q.pop(&mut out), Some((125, n as i8)));
            assert!(
                out[..125]
                    .iter()
                    .enumerate()
                    .all(|(i, &b)| b == i as u8 ^ n)
            );
        }
        q.push([0x41, 0x88, 0x07].into_iter(), 3, 0xC4);
        let mut next = 2u8;
        while next < pushed {
            assert_eq!(q.pop(&mut out), Some((125, next as i8)));
            assert_eq!(out[124], 124 ^ next);
            next += 1;
        }
        assert_eq!(q.pop(&mut out), Some((3, 0xC4u8 as i8)));
        assert_eq!(&out[..3], &[0x41, 0x88, 0x07]);
        assert_eq!(q.pop(&mut out), None);
        assert_eq!(q.free(), RX_QUEUE_LEN);
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
