//! CC2340 LRFD FIFO and IEEE operation primitives.

use super::hardware::{read16, read32, write16, write32};

pub(super) const MAX_PHY_FRAME_LEN: usize = 127;
pub(super) const MAX_MPDU_LEN: usize = MAX_PHY_FRAME_LEN - 2;

const LRFDDBELL_BASE: u32 = 0x4008_0000;
const LRFDPBE_BASE: u32 = 0x4008_1000;
const BUFRAM_BASE: u32 = 0x4009_2000;
const RXF_UNWRAPPED_BASE: u32 = 0x4009_3000;
const TXF_UNWRAPPED_BASE: u32 = 0x4009_3800;

const DBELL_RIS0: u32 = LRFDDBELL_BASE + 0x048;
const DBELL_ICLR0: u32 = LRFDDBELL_BASE + 0x054;

const PBE_API: u32 = LRFDPBE_BASE + 0x030;
const PBE_FCFG0: u32 = LRFDPBE_BASE + 0x0B4;
const PBE_FCFG1: u32 = LRFDPBE_BASE + 0x0B8;
const PBE_FCFG2: u32 = LRFDPBE_BASE + 0x0BC;
const PBE_FCFG3: u32 = LRFDPBE_BASE + 0x0C0;
const PBE_FCFG4: u32 = LRFDPBE_BASE + 0x0C4;
const PBE_FCMD: u32 = LRFDPBE_BASE + 0x1A0;
const PBE_RXFRP: u32 = LRFDPBE_BASE + 0x1AC;
const PBE_RXFSRP: u32 = LRFDPBE_BASE + 0x1B4;
const PBE_TXFWP: u32 = LRFDPBE_BASE + 0x1B8;
const PBE_RXFREADABLE: u32 = LRFDPBE_BASE + 0x1CC;
const PBE_TXFWRITABLE: u32 = LRFDPBE_BASE + 0x1D0;

// PBE IEEE RAM register map (`pbe_ieee_ram_regs.h`, SDK 9.20 / the PBE image
// this crate loads; the SDK 8.40 map is shifted by four bytes).
const PBE_IEEE_FIFOCFG: u32 = BUFRAM_BASE + 0x022;
const PBE_IEEE_EXTRABYTES: u32 = BUFRAM_BASE + 0x024;
const PBE_IEEE_RXTIMEOUT: u32 = BUFRAM_BASE + 0x02E;
const PBE_IEEE_OPCFG: u32 = BUFRAM_BASE + 0x030;
const PBE_IEEE_PIB: u32 = BUFRAM_BASE + 0x036;
const PBE_IEEE_FFTYPE: u32 = BUFRAM_BASE + 0x038;
const PBE_IEEE_NRXNOK: u32 = BUFRAM_BASE + 0x03A;
const PBE_IEEE_NRXIGNORED: u32 = BUFRAM_BASE + 0x03C;
const PBE_IEEE_NRXFIFOFULL: u32 = BUFRAM_BASE + 0x040;
const PBE_IEEE_NRXOK: u32 = BUFRAM_BASE + 0x042;
const PBE_IEEE_NTX: u32 = BUFRAM_BASE + 0x044;
const PBE_IEEE_NTXACK: u32 = BUFRAM_BASE + 0x046;
const PBE_IEEE_NRXTIMEOUT: u32 = BUFRAM_BASE + 0x048;
const PBE_IEEE_RXSTATUS: u32 = BUFRAM_BASE + 0x054;
const PBE_IEEE_PANID0: u32 = BUFRAM_BASE + 0x064;
const PBE_IEEE_SHORTADDR0: u32 = BUFRAM_BASE + 0x066;
const PBE_IEEE_EXTADDR00: u32 = BUFRAM_BASE + 0x068;
const PBE_IEEE_EXTADDR02: u32 = BUFRAM_BASE + 0x06C;
const PBE_IEEE_FFOPT0: u32 = BUFRAM_BASE + 0x070;
const PBE_IEEE_ENTRYENABLE00: u32 = BUFRAM_BASE + 0x084;
const PBE_IEEE_CFGAUTOACK: u32 = BUFRAM_BASE + 0x0A8;
const PBE_IEEE_ACKSTATUS: u32 = BUFRAM_BASE + 0x0AC;
const PBE_IEEE_CORRCNT: u32 = BUFRAM_BASE + 0x0AE;

const PBE_COMMON_ENDCAUSE: u32 = BUFRAM_BASE + 0x006;
const PBE_COMMON_FIFOCMDADD: u32 = BUFRAM_BASE + 0x008;

const FIFO_COMMAND_ADDRESS: u16 = 0x0068;
const FIFO_STATUS_ADDRESS: u16 = 0x0069;
const TX_FIFO_RESET: u32 = 0x0007;
const RX_FIFO_RESET: u32 = 0x0001;
const RX_FIFO_DISCARD: u32 = 0x0009;
const BOTH_FIFOS_RESET: u32 = 0x000B;

const IEEE_OP_TX: u32 = 0x1C;
const IEEE_OP_RX: u32 = 0x1D;
const OP_HARD_STOP: u32 = 0x01;
/// RCL RX / TX-then-RX operation: `STOP_SOFTEND | RXREPEATOK_YES |
/// RXREPEATNOK_YES | START_SYNC | SINGLE_DIS | IFSPERIOD_EN`. The PBE keeps
/// receiving after every frame and, for a TX, falls through into RX after the
/// frame (and its ACK window) instead of ending the operation.
const OPCFG_RX_CONTINUOUS: u16 = 0x0802;

// PIB.PANMODE
const PIB_PANMODE_PROMISCUOUS: u16 = 0x0000;
const PIB_PANMODE_SINGLE: u16 = 0x0001;

// FFTYPE: frame types stored in the RX FIFO, for frame versions 0/1 and 2.
// Beacon, data and MAC command; ACK frames are consumed by the PBE itself.
const FFTYPE_BEACON_DATA_MACCMD: u16 = 0x0B0B;

// FFOPT0
const FFOPT0_MAXFRAME_2006: u16 = 1 << 8;
const FFOPT0_AUTOACK_EN: u16 = 0x0004;
// AUTOPEND_DISABLE, DEFPEND = 0, PREQONLY_ANY and IMMACKSEL_PBE are all zero:
// the PBE builds every immediate ACK itself with the frame-pending bit clear,
// which is what an end device must send.

// CFGAUTOACK
const CFGAUTOACK_ACKMODE_FILTIMMACK: u16 = 0x0200;
const CFGAUTOACK_FLAGREQ_EN: u16 = 0x0100;

// ACKSTATUS (valid after `EVENT_RX_EMPTY` of a TX that expected an ACK)
const ACKSTATUS_FPSTATE: u16 = 0x0020;
const ACKSTATUS_CRCOK: u16 = 0x0010;
const ACKSTATUS_IGNORED: u16 = 0x0008;
const ACKSTATUS_OTHERFRM: u16 = 0x0004;
const ACKSTATUS_CRCERR: u16 = 0x0002;
const ACKSTATUS_SYNCTIMEOUT: u16 = 0x0001;

// RXSTATUS
const RXSTATUS_TXACKINPROGRESS: u16 = 0x0002;
const RXSTATUS_FRAMEINPROGRESS: u16 = 0x0001;

const FCFG_START_MASK: u32 = 0x01FF;
const FCFG_SIZE_MASK: u32 = 0x00FF;
const FCFG0_TX_AUTO_DEALLOCATE: u32 = 0x10;
const FCFG0_TX_AUTO_COMMIT: u32 = 0x20;
const FCFG0_RX_AUTO_DEALLOCATE: u32 = 0x01;
const FCFG0_RX_AUTO_COMMIT: u32 = 0x02;

const FIFOCFG_APPEND_CRC: u16 = 0x0400;
const FIFOCFG_APPEND_STATUS: u16 = 0x0800;
const FIFOCFG_APPEND_LQI: u16 = 0x1000;
const FIFOCFG_APPEND_RSSI: u16 = 0x4000;
const FIFOCFG_APPEND_TIMESTAMP: u16 = 0x8000;

pub(super) const EVENT_OP_DONE: u32 = 1 << 0;
pub(super) const EVENT_RX_NOK: u32 = 1 << 4;
pub(super) const EVENT_RX_IGNORED: u32 = 1 << 5;
/// ACK reception finished (ACK received, wrong frame, or `RXTIMEOUT` expired);
/// the outcome is in `ACKSTATUS`.
pub(super) const EVENT_RX_EMPTY: u32 = 1 << 6;
pub(super) const EVENT_RX_BUF_FULL: u32 = 1 << 7;
pub(super) const EVENT_RX_OK: u32 = 1 << 8;
/// The frame of a TX-then-RX operation has left the antenna.
pub(super) const EVENT_TX_DONE: u32 = 1 << 13;
pub(super) const EVENT_OP_ERROR: u32 = 1 << 15;
pub(super) const EVENT_TERMINAL: u32 = EVENT_OP_DONE | EVENT_OP_ERROR;

const MAX_FIFO_ENTRY_WORDS: usize = 36;
pub(super) const MAX_FIFO_ENTRY_BYTES: usize = MAX_FIFO_ENTRY_WORDS * 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum FifoError {
    FrameTooLong,
    NoSpace,
    MalformedEntry,
    MetadataMismatch,
    /// The PBE FIFO configuration does not append RSSI, so no per-frame link
    /// quality can be derived. See [`RxMeta::rssi`].
    MissingRssi,
    /// The PBE appended RSSI, but it is TI's "no valid reading" sentinel.
    InvalidRssi,
}

/// TI reports "RSSI not available" as `0x7F` (`+127 dBm`), a level the radio
/// cannot physically measure.
const RSSI_INVALID: i8 = 127;

/// Metadata of an RX FIFO entry whose MAC frame [`try_read_rx_frame`] wrote
/// to the caller's buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct RxMeta {
    /// MAC frame length without PHR and FCS.
    pub len: usize,
    /// Received signal strength in dBm, as appended by the PBE.
    ///
    /// This is the only calibrated per-frame quality metric this radio
    /// reports, and therefore the source of the stack's normalized LQI (see
    /// [`crate::lqi::from_rssi_dbm`]). Decoding fails rather than inventing a
    /// value when it is absent or invalid.
    pub rssi: i8,
    /// Raw modem LQI byte appended by the PBE when `FIFOCFG.APPEND_LQI` is set.
    ///
    /// **Diagnostic only.** On CC2340 this is a modem/frequency-offset
    /// estimate whose scale and polarity are undocumented; it is *not* an
    /// IEEE 802.15.4 LQI and must never be reported as one, placed in a
    /// `PanDescriptor`, or used for NWK link cost. `None` when the FIFO is not
    /// configured to append it.
    pub modem_lqi: Option<u8>,
}

#[inline(always)]
pub(super) fn events() -> u32 {
    read32(DBELL_RIS0)
}

#[inline(always)]
pub(super) fn clear_events(mask: u32) {
    write32(DBELL_ICLR0, mask);
}

#[inline(always)]
pub(super) fn end_cause() -> u8 {
    read16(PBE_COMMON_ENDCAUSE) as u8
}

/// Whether the RX FIFO still holds at least the header of another entry.
///
/// `EVENT_RX_OK` announces "one or more frames arrived", not "exactly one".
/// After consuming an entry the event may only be cleared once the FIFO has
/// actually been drained, or a queued frame would sit unread until the next
/// reception re-raised the event.
#[inline(always)]
pub(super) fn rx_data_pending() -> bool {
    (read32(PBE_RXFREADABLE) as usize) >= 4
}

pub(super) fn reset_tx() {
    write32(PBE_FCMD, TX_FIFO_RESET);
    let fifo_config = read32(PBE_FCFG0);
    write32(
        PBE_FCFG0,
        (fifo_config & !FCFG0_TX_AUTO_DEALLOCATE) | FCFG0_TX_AUTO_COMMIT,
    );
}

pub(super) fn reset_rx() {
    write32(PBE_FCMD, RX_FIFO_RESET);
    let fifo_config = read32(PBE_FCFG0);
    write32(
        PBE_FCFG0,
        fifo_config & !(FCFG0_RX_AUTO_COMMIT | FCFG0_RX_AUTO_DEALLOCATE),
    );
    // RCL `LRF_prepareRxFifo()` + `LRF_setRxFifoEffSz(unlimited)`: after the
    // reset SRP is first written to 0 ("no space available"), then written to
    // RP. Writing SRP with an unchanged value moves it one full round, which
    // is what deallocates the FIFO (full space). A single SRP=RP write right
    // after reset leaves zero writable space, so every received frame ended
    // with ENDCAUSE = ERR_RXF and surfaced as `FifoOverflow`.
    write_fifo_pointer(PBE_RXFSRP, 0);
    write_fifo_pointer(PBE_RXFSRP, read32(PBE_RXFRP));
}

pub(super) fn reset_both() {
    write32(PBE_FCMD, BOTH_FIFOS_RESET);
}

/// Drop RX FIFO bytes the PBE wrote but never committed (a frame cut short
/// by a hard stop). Committed, still unread frames are kept. Only while the
/// PBE is stopped.
pub(super) fn discard_uncommitted_rx() {
    write32(PBE_FCMD, RX_FIFO_DISCARD);
}

/// Local addressing used by the PBE's IEEE 802.15.4 frame filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FrameFilter {
    pub pan_id: u16,
    pub short_addr: u16,
    /// EUI-64 in over-the-air (little-endian) byte order.
    pub ext_addr: [u8; 8],
    pub promiscuous: bool,
}

/// PBE register values for one [`FrameFilter`], as RCL's
/// `RCL_Handler_Ieee_RxTx` setup writes them for a single PAN.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct FilterRegisters {
    pub pib: u16,
    pub pan_id: u16,
    pub short_addr: u16,
    pub ext_addr_low: u32,
    pub ext_addr_high: u32,
    pub ffopt0: u16,
    pub fftype: u16,
}

pub(super) const fn filter_registers(filter: &FrameFilter) -> FilterRegisters {
    let ext = filter.ext_addr;
    FilterRegisters {
        pib: if filter.promiscuous {
            PIB_PANMODE_PROMISCUOUS
        } else {
            PIB_PANMODE_SINGLE
        },
        pan_id: filter.pan_id,
        short_addr: filter.short_addr,
        ext_addr_low: u32::from_le_bytes([ext[0], ext[1], ext[2], ext[3]]),
        ext_addr_high: u32::from_le_bytes([ext[4], ext[5], ext[6], ext[7]]),
        // RCL `RCL_CmdIeee_AutoAck_ImmAckNoAutoPend`, maxFrameVersion 1.
        ffopt0: FFOPT0_MAXFRAME_2006 | FFOPT0_AUTOACK_EN,
        fftype: FFTYPE_BEACON_DATA_MACCMD,
    }
}

/// Program frame filtering and hardware auto-ACK. Only while the PBE is
/// stopped; the PBE latches the PAN configuration at operation start.
pub(super) fn configure_frame_filter(filter: &FrameFilter) {
    let registers = filter_registers(filter);
    write16(PBE_IEEE_PIB, registers.pib);
    write16(PBE_IEEE_PANID0, registers.pan_id);
    write16(PBE_IEEE_SHORTADDR0, registers.short_addr);
    write32(PBE_IEEE_EXTADDR00, registers.ext_addr_low);
    write32(PBE_IEEE_EXTADDR02, registers.ext_addr_high);
    write16(PBE_IEEE_FFOPT0, registers.ffopt0);
    // No source-match table: no per-child frame-pending bits.
    for index in 0..4 {
        write16(PBE_IEEE_ENTRYENABLE00 + index * 2, 0);
    }
    write16(PBE_IEEE_FFTYPE, registers.fftype);
}

/// Continuous RX (RCL RX setup / `restartRx`).
pub(super) fn prepare_rx() {
    write16(PBE_IEEE_OPCFG, OPCFG_RX_CONTINUOUS);
    write16(PBE_IEEE_RXTIMEOUT, 0);
    write16(PBE_IEEE_CFGAUTOACK, 0);
}

/// `CFGAUTOACK` for a TX that expects an immediate ACK with `sequence`.
pub(super) const fn ack_request_config(sequence: u8) -> u16 {
    CFGAUTOACK_ACKMODE_FILTIMMACK | CFGAUTOACK_FLAGREQ_EN | sequence as u16
}

/// TX that falls through into continuous RX (RCL TX with an RX action).
///
/// With `ack = Some((sequence, timeout))` the PBE itself listens for the
/// immediate ACK right after the frame, `timeout` in 0.25 us units from the
/// end of the TX frame to ACK sync, and reports it via `EVENT_RX_EMPTY` and
/// [`ack_status`]. Without, `EVENT_TX_DONE` marks the end of the frame.
pub(super) fn prepare_tx_then_rx(ack: Option<(u8, u16)>) {
    write16(PBE_IEEE_OPCFG, OPCFG_RX_CONTINUOUS);
    match ack {
        Some((sequence, timeout)) => {
            write16(PBE_IEEE_RXTIMEOUT, timeout);
            write16(PBE_IEEE_CFGAUTOACK, ack_request_config(sequence));
        }
        None => {
            write16(PBE_IEEE_RXTIMEOUT, 0);
            write16(PBE_IEEE_CFGAUTOACK, 0);
        }
    }
}

/// Outcome of the PBE's ACK reception for the last TX.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum AckStatus {
    /// Matching ACK with a good CRC; `frame_pending` is its FP bit.
    Received { frame_pending: bool },
    /// No ACK sync within `RXTIMEOUT`.
    Timeout,
    /// Something was received but it was not our ACK (other frame, bad CRC,
    /// wrong sequence number).
    RxError,
    /// `ACKSTATUS` carried no known outcome.
    Unknown(u16),
}

pub(super) const fn decode_ack_status(status: u16) -> AckStatus {
    if status & ACKSTATUS_SYNCTIMEOUT != 0 {
        AckStatus::Timeout
    } else if status & ACKSTATUS_CRCOK != 0 {
        AckStatus::Received {
            frame_pending: status & ACKSTATUS_FPSTATE != 0,
        }
    } else if status & (ACKSTATUS_IGNORED | ACKSTATUS_OTHERFRM | ACKSTATUS_CRCERR) != 0 {
        AckStatus::RxError
    } else {
        AckStatus::Unknown(status)
    }
}

pub(super) fn ack_status() -> AckStatus {
    decode_ack_status(read16(PBE_IEEE_ACKSTATUS))
}

/// Whether the PBE is in the middle of receiving a frame or sending the
/// auto-ACK for one; a hard stop now would lose the frame or the ACK.
pub(super) fn rx_busy() -> bool {
    read16(PBE_IEEE_RXSTATUS) & (RXSTATUS_FRAMEINPROGRESS | RXSTATUS_TXACKINPROGRESS) != 0
}

/// Correlation tops the PBE counted in the last 128 us (RCL `CORRCNT`,
/// "counter of MDMPROG events to be used for CCA mode 2").
pub(super) fn correlation_count() -> u16 {
    read16(PBE_IEEE_CORRCNT) & 0x00FF
}

/// PBE IEEE statistics counters. They are cleared by the PBE at every
/// operation start, so they describe the current RX/TX operation only.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct PbeCounters {
    pub rx_ok: u16,
    pub rx_nok: u16,
    pub rx_ignored: u16,
    pub rx_fifo_full: u16,
    pub rx_timeout: u16,
    pub tx: u16,
    pub tx_ack: u16,
}

pub(super) fn pbe_counters() -> PbeCounters {
    PbeCounters {
        rx_ok: read16(PBE_IEEE_NRXOK),
        rx_nok: read16(PBE_IEEE_NRXNOK),
        rx_ignored: read16(PBE_IEEE_NRXIGNORED),
        rx_fifo_full: read16(PBE_IEEE_NRXFIFOFULL),
        rx_timeout: read16(PBE_IEEE_NRXTIMEOUT),
        tx: read16(PBE_IEEE_NTX),
        tx_ack: read16(PBE_IEEE_NTXACK),
    }
}

#[inline(always)]
pub(super) fn start_tx() {
    write32(PBE_API, IEEE_OP_TX);
}

#[inline(always)]
pub(super) fn start_rx() {
    write32(PBE_API, IEEE_OP_RX);
}

#[inline(always)]
pub(super) fn hard_stop() {
    write32(PBE_API, OP_HARD_STOP);
}

pub(super) fn write_tx_frame(frame: &[u8]) -> Result<(), FifoError> {
    let mut words = [0u32; MAX_FIFO_ENTRY_WORDS];
    let word_count = encode_tx_entry(frame, &mut words)?;
    let byte_count = word_count * 4;

    if read32(PBE_TXFWRITABLE) < byte_count as u32 {
        return Err(FifoError::NoSpace);
    }

    let fifo_start = (read32(PBE_FCFG1) & FCFG_START_MASK) << 2;
    let fifo_size = (read32(PBE_FCFG2) & FCFG_SIZE_MASK) << 2;
    let write_pointer = read32(PBE_TXFWP) & !3;
    if fifo_size == 0 || byte_count as u32 > fifo_size {
        return Err(FifoError::NoSpace);
    }

    for (index, word) in words[..word_count].iter().copied().enumerate() {
        write32(
            TXF_UNWRAPPED_BASE + fifo_start + write_pointer + index as u32 * 4,
            word,
        );
    }

    let next_pointer = (write_pointer + byte_count as u32) % fifo_size;
    write_fifo_pointer(PBE_TXFWP, next_pointer);
    Ok(())
}

/// Consume the oldest complete RX FIFO entry and write its MAC frame
/// (without PHR and FCS) to `data`, straight from the FIFO.
///
/// The entry is released even when it does not decode; `data` is then
/// unspecified.
pub(super) fn try_read_rx_frame(
    data: &mut [u8; MAX_MPDU_LEN],
) -> Result<Option<RxMeta>, FifoError> {
    let readable = read32(PBE_RXFREADABLE) as usize;
    if readable < 4 {
        return Ok(None);
    }

    let fifo_start = (read32(PBE_FCFG3) & FCFG_START_MASK) << 2;
    let fifo_size = (read32(PBE_FCFG4) & FCFG_SIZE_MASK) << 2;
    let read_pointer = read32(PBE_RXFRP) & !3;
    if fifo_size == 0 {
        return Err(FifoError::MalformedEntry);
    }

    let first_word = read32(BUFRAM_BASE + fifo_start + read_pointer);
    let entry_length = (first_word & 0xFFFF) as usize;
    let byte_count = padded_entry_len(entry_length);
    if byte_count > MAX_FIFO_ENTRY_BYTES {
        return Err(FifoError::MalformedEntry);
    }
    if readable < byte_count {
        return Ok(None);
    }

    let entry = RXF_UNWRAPPED_BASE + fifo_start + read_pointer;
    let fifo_config = read16(PBE_IEEE_FIFOCFG);
    let extra_bytes = read16(PBE_IEEE_EXTRABYTES) as usize;
    let decoded = decode_rx_entry(
        |index| (read32(entry + (index & !3) as u32) >> ((index % 4) * 8)) as u8,
        byte_count,
        fifo_config,
        extra_bytes,
        data,
    );

    let next_pointer = (read_pointer + byte_count as u32) % fifo_size;
    // RCL `LRF_discardRxFifoWords()`: advance RP and SRP together so the
    // consumed space is released and the FIFO stays deallocated.
    write_fifo_pointers(PBE_RXFRP, PBE_RXFSRP, next_pointer);
    decoded.map(Some)
}

/// Leading MAC frame bytes [`peek_rx_head`] returns: frame control, sequence
/// number, destination PAN and destination short address.
pub(super) const RX_HEAD_PREFIX_LEN: usize = 7;
/// Entry words [`peek_rx_head`] reads; covers the prefix for `num_pad <= 4`.
const RX_HEAD_WORDS: usize = 4;

/// The oldest complete RX FIFO entry, inspected without consuming it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct RxHead {
    /// FIFO bytes the entry occupies.
    byte_count: u32,
    prefix: [u8; RX_HEAD_PREFIX_LEN],
    prefix_len: u8,
}

impl RxHead {
    /// The first (at most [`RX_HEAD_PREFIX_LEN`]) MAC frame bytes.
    pub fn prefix(&self) -> &[u8] {
        &self.prefix[..usize::from(self.prefix_len)]
    }
}

/// Free RX FIFO bytes; `None` when the FIFO is not configured.
pub(super) fn rx_free_bytes() -> Option<usize> {
    let fifo_size = ((read32(PBE_FCFG4) & FCFG_SIZE_MASK) << 2) as usize;
    if fifo_size == 0 {
        return None;
    }
    Some(fifo_size.saturating_sub(read32(PBE_RXFREADABLE) as usize))
}

/// Inspect the oldest committed RX FIFO entry without consuming it.
///
/// `None` when no complete entry is committed, or when the entry is not a
/// well-formed frame; [`try_read_rx_frame`] then consumes and classifies it.
pub(super) fn peek_rx_head() -> Option<RxHead> {
    let readable = read32(PBE_RXFREADABLE) as usize;
    if readable < 4 {
        return None;
    }
    let fifo_start = (read32(PBE_FCFG3) & FCFG_START_MASK) << 2;
    let fifo_size = (read32(PBE_FCFG4) & FCFG_SIZE_MASK) << 2;
    let read_pointer = read32(PBE_RXFRP) & !3;
    if fifo_size == 0 {
        return None;
    }

    let first_word = read32(BUFRAM_BASE + fifo_start + read_pointer);
    let byte_count = padded_entry_len((first_word & 0xFFFF) as usize);
    if byte_count > MAX_FIFO_ENTRY_BYTES || readable < byte_count {
        return None;
    }
    let mut words = [0u32; RX_HEAD_WORDS];
    let word_count = (byte_count / 4).min(RX_HEAD_WORDS);
    for (index, word) in words[..word_count].iter_mut().enumerate() {
        *word = read32(RXF_UNWRAPPED_BASE + fifo_start + read_pointer + index as u32 * 4);
    }
    decode_rx_head(&words[..word_count], byte_count)
}

/// Release the entry [`peek_rx_head`] returned, without reading it.
pub(super) fn discard_rx_head(head: &RxHead) {
    let fifo_size = (read32(PBE_FCFG4) & FCFG_SIZE_MASK) << 2;
    if fifo_size == 0 {
        return;
    }
    let read_pointer = read32(PBE_RXFRP) & !3;
    let next_pointer = (read_pointer + head.byte_count) % fifo_size;
    // Same release as `try_read_rx_frame`.
    write_fifo_pointers(PBE_RXFRP, PBE_RXFSRP, next_pointer);
}

/// Decode the frame prefix of an RX entry; `words` holds its first words.
fn decode_rx_head(words: &[u32], byte_count: usize) -> Option<RxHead> {
    let entry_bytes = (words.len() * 4).min(byte_count);
    let packet_start = 3 + usize::from(entry_byte(words, 2));
    if packet_start >= entry_bytes {
        return None;
    }
    let phy_length = usize::from(entry_byte(words, packet_start));
    if !(2..=MAX_PHY_FRAME_LEN).contains(&phy_length) {
        return None;
    }
    let prefix_len = (phy_length - 2).min(RX_HEAD_PREFIX_LEN);
    if packet_start + 1 + prefix_len > entry_bytes {
        return None;
    }
    let mut prefix = [0u8; RX_HEAD_PREFIX_LEN];
    for (index, byte) in prefix[..prefix_len].iter_mut().enumerate() {
        *byte = entry_byte(words, packet_start + 1 + index);
    }
    Some(RxHead {
        byte_count: byte_count as u32,
        prefix,
        prefix_len: prefix_len as u8,
    })
}

fn encode_tx_entry(frame: &[u8], words: &mut [u32]) -> Result<usize, FifoError> {
    if frame.is_empty() || frame.len() > MAX_MPDU_LEN {
        return Err(FifoError::FrameTooLong);
    }

    let entry_length = frame.len() + 2;
    let byte_count = padded_entry_len(entry_length);
    let word_count = byte_count / 4;
    if word_count > words.len() {
        return Err(FifoError::NoSpace);
    }

    words[..word_count].fill(0);
    set_entry_byte(words, 0, entry_length as u8);
    set_entry_byte(words, 1, (entry_length >> 8) as u8);
    set_entry_byte(words, 2, 0);
    set_entry_byte(words, 3, (frame.len() + 2) as u8);
    for (index, byte) in frame.iter().copied().enumerate() {
        set_entry_byte(words, 4 + index, byte);
    }

    Ok(word_count)
}

/// Decode an RX FIFO entry of `entry_bytes` readable bytes, given byte by
/// byte by `byte`, and write its MAC frame to `data`.
fn decode_rx_entry(
    byte: impl Fn(usize) -> u8,
    entry_bytes: usize,
    fifo_config: u16,
    extra_bytes: usize,
    data: &mut [u8; MAX_MPDU_LEN],
) -> Result<RxMeta, FifoError> {
    if entry_bytes < 4 {
        return Err(FifoError::MalformedEntry);
    }

    let entry_length = byte(0) as usize | ((byte(1) as usize) << 8);
    let byte_count = padded_entry_len(entry_length);
    if byte_count > entry_bytes || entry_length < 2 {
        return Err(FifoError::MalformedEntry);
    }

    let expected_extra_bytes = appended_byte_count(fifo_config);
    if extra_bytes != expected_extra_bytes || extra_bytes > entry_length {
        return Err(FifoError::MetadataMismatch);
    }

    let num_pad = byte(2) as usize;
    let packet_start = 3usize
        .checked_add(num_pad)
        .ok_or(FifoError::MalformedEntry)?;
    let metadata_start = 2 + entry_length - extra_bytes;
    if packet_start >= metadata_start {
        return Err(FifoError::MalformedEntry);
    }

    let phy_length = byte(packet_start) as usize;
    if !(2..=MAX_PHY_FRAME_LEN).contains(&phy_length) {
        return Err(FifoError::MalformedEntry);
    }
    let frame_length = phy_length - 2;
    let frame_start = packet_start + 1;
    let frame_end = frame_start
        .checked_add(frame_length)
        .ok_or(FifoError::MalformedEntry)?;
    if frame_end > metadata_start {
        return Err(FifoError::MalformedEntry);
    }

    for (index, out) in data[..frame_length].iter_mut().enumerate() {
        *out = byte(frame_start + index);
    }

    let mut metadata_offset = metadata_start;
    if fifo_config & FIFOCFG_APPEND_CRC != 0 {
        metadata_offset += 2;
    }
    if fifo_config & FIFOCFG_APPEND_STATUS != 0 {
        metadata_offset += 1;
    }
    // Raw modem estimate: consumed only to keep the metadata offset aligned and
    // to expose as diagnostics. Never used as the stack's LQI.
    let modem_lqi = if fifo_config & FIFOCFG_APPEND_LQI != 0 {
        let value = byte(metadata_offset);
        metadata_offset += 1;
        Some(value)
    } else {
        None
    };
    if fifo_config & FIFOCFG_APPEND_RSSI == 0 {
        // Without RSSI there is no defensible link quality for this frame.
        // Reporting a success-shaped `lqi = 0` would look like a real, very
        // bad link and would poison NWK link cost and parent selection.
        return Err(FifoError::MissingRssi);
    }
    let rssi = byte(metadata_offset) as i8;
    if rssi == RSSI_INVALID {
        return Err(FifoError::InvalidRssi);
    }

    Ok(RxMeta {
        len: frame_length,
        rssi,
        modem_lqi,
    })
}

const fn padded_entry_len(entry_length: usize) -> usize {
    (entry_length + 5) & !3
}

const fn appended_byte_count(fifo_config: u16) -> usize {
    let mut count = 0;
    if fifo_config & FIFOCFG_APPEND_CRC != 0 {
        count += 2;
    }
    if fifo_config & FIFOCFG_APPEND_STATUS != 0 {
        count += 1;
    }
    if fifo_config & FIFOCFG_APPEND_LQI != 0 {
        count += 1;
    }
    if fifo_config & FIFOCFG_APPEND_RSSI != 0 {
        count += 1;
    }
    if fifo_config & FIFOCFG_APPEND_TIMESTAMP != 0 {
        count += 4;
    }
    count
}

fn entry_byte(words: &[u32], index: usize) -> u8 {
    ((words[index / 4] >> ((index % 4) * 8)) & 0xFF) as u8
}

fn set_entry_byte(words: &mut [u32], index: usize, value: u8) {
    let shift = (index % 4) * 8;
    let word = &mut words[index / 4];
    *word = (*word & !(0xFF << shift)) | ((value as u32) << shift);
}

fn write_fifo_pointers(register0: u32, register1: u32, value: u32) {
    let interrupt_state = disable_interrupts();
    write16(PBE_COMMON_FIFOCMDADD, FIFO_STATUS_ADDRESS);
    let _ = read16(PBE_COMMON_FIFOCMDADD);
    let _ = read16(PBE_COMMON_FIFOCMDADD);
    write32(register0, value);
    write32(register1, value);
    write16(PBE_COMMON_FIFOCMDADD, FIFO_COMMAND_ADDRESS);
    restore_interrupts(interrupt_state);
}

fn write_fifo_pointer(register: u32, value: u32) {
    let interrupt_state = disable_interrupts();
    write16(PBE_COMMON_FIFOCMDADD, FIFO_STATUS_ADDRESS);
    let _ = read16(PBE_COMMON_FIFOCMDADD);
    let _ = read16(PBE_COMMON_FIFOCMDADD);
    write32(register, value);
    write16(PBE_COMMON_FIFOCMDADD, FIFO_COMMAND_ADDRESS);
    restore_interrupts(interrupt_state);
}

#[cfg(target_arch = "arm")]
fn disable_interrupts() -> u32 {
    let state: u32;
    unsafe {
        core::arch::asm!(
            "mrs {state}, PRIMASK",
            "cpsid i",
            state = out(reg) state,
            options(nomem, nostack, preserves_flags)
        );
    }
    state
}

#[cfg(not(target_arch = "arm"))]
const fn disable_interrupts() -> u32 {
    0
}

#[cfg(target_arch = "arm")]
fn restore_interrupts(state: u32) {
    if state & 1 == 0 {
        unsafe {
            core::arch::asm!("cpsie i", options(nomem, nostack, preserves_flags));
        }
    }
}

#[cfg(not(target_arch = "arm"))]
const fn restore_interrupts(_state: u32) {}

#[cfg(test)]
mod tests {
    use super::*;

    const TEST_FIFO_CONFIG: u16 =
        FIFOCFG_APPEND_LQI | FIFOCFG_APPEND_RSSI | FIFOCFG_APPEND_TIMESTAMP;

    struct Decoded {
        data: [u8; MAX_MPDU_LEN],
        len: usize,
        rssi: i8,
        modem_lqi: Option<u8>,
    }

    /// [`decode_rx_entry`] over a word buffer, as the FIFO window is read.
    fn decode(words: &[u32], fifo_config: u16, extra_bytes: usize) -> Result<Decoded, FifoError> {
        let mut data = [0u8; MAX_MPDU_LEN];
        let meta = decode_rx_entry(
            |index| entry_byte(words, index),
            words.len() * 4,
            fifo_config,
            extra_bytes,
            &mut data,
        )?;
        Ok(Decoded {
            data,
            len: meta.len,
            rssi: meta.rssi,
            modem_lqi: meta.modem_lqi,
        })
    }

    /// Build an RX FIFO entry for `frame` with `metadata` appended verbatim.
    fn rx_entry(frame: &[u8], metadata: &[u8]) -> ([u32; MAX_FIFO_ENTRY_WORDS], usize) {
        let entry_length = 1 + 1 + frame.len() + metadata.len();
        let mut words = [0u32; MAX_FIFO_ENTRY_WORDS];

        set_entry_byte(&mut words, 0, entry_length as u8);
        set_entry_byte(&mut words, 1, (entry_length >> 8) as u8);
        set_entry_byte(&mut words, 2, 0);
        set_entry_byte(&mut words, 3, (frame.len() + 2) as u8);
        for (index, byte) in frame.iter().copied().enumerate() {
            set_entry_byte(&mut words, 4 + index, byte);
        }

        let metadata_start = 2 + entry_length - metadata.len();
        for (index, byte) in metadata.iter().copied().enumerate() {
            set_entry_byte(&mut words, metadata_start + index, byte);
        }

        (words, padded_entry_len(entry_length) / 4)
    }

    #[test]
    fn encodes_tx_data_entry_with_phr_and_padding() {
        let frame = [0x61, 0x88, 0x42, 0xAA, 0xBB];
        let mut words = [0u32; MAX_FIFO_ENTRY_WORDS];

        let word_count = encode_tx_entry(&frame, &mut words).unwrap();

        assert_eq!(word_count, 3);
        assert_eq!(entry_byte(&words, 0), 7);
        assert_eq!(entry_byte(&words, 1), 0);
        assert_eq!(entry_byte(&words, 2), 0);
        assert_eq!(entry_byte(&words, 3), 7);
        for (index, expected) in frame.iter().copied().enumerate() {
            assert_eq!(entry_byte(&words, 4 + index), expected);
        }
        assert_eq!(entry_byte(&words, 9), 0);
        assert_eq!(entry_byte(&words, 10), 0);
        assert_eq!(entry_byte(&words, 11), 0);
    }

    #[test]
    fn decodes_rx_entry_and_strips_phr_fcs_and_metadata() {
        let frame = [0x61, 0x88, 0x42, 0xAA, 0xBB];
        // LQI, RSSI, then the 4-byte timestamp.
        let metadata = [201, (-42i8) as u8, 0x11, 0x22, 0x33, 0x44];
        let (words, word_count) = rx_entry(&frame, &metadata);

        let decoded = decode(&words[..word_count], TEST_FIFO_CONFIG, metadata.len())
            .expect("entry with LQI, RSSI and timestamp should decode");

        assert_eq!(decoded.len, frame.len());
        assert_eq!(&decoded.data[..decoded.len], &frame);
        assert_eq!(decoded.rssi, -42);
        assert_eq!(decoded.modem_lqi, Some(201));
    }

    #[test]
    fn rx_head_exposes_frame_prefix_without_metadata() {
        let metadata = [201, (-42i8) as u8, 0x11, 0x22, 0x33, 0x44];

        // Immediate ACK: three frame bytes, all of them in the prefix.
        let (words, word_count) = rx_entry(&[0x12, 0x00, 0x42], &metadata);
        let head = decode_rx_head(&words[..word_count.min(RX_HEAD_WORDS)], word_count * 4)
            .expect("ACK entry");
        assert_eq!(head.prefix(), &[0x12, 0x00, 0x42]);
        assert_eq!(head.byte_count as usize, word_count * 4);

        // Data frame: only the addressing prefix is copied.
        let frame = [0x41, 0x88, 0x07, 0x34, 0x12, 0xFF, 0xFF, 0x00, 0x00, 0xAA];
        let (words, word_count) = rx_entry(&frame, &metadata);
        let head = decode_rx_head(&words[..RX_HEAD_WORDS], word_count * 4).expect("data entry");
        assert_eq!(head.prefix(), &frame[..RX_HEAD_PREFIX_LEN]);
        assert_eq!(head.byte_count as usize, word_count * 4);
    }

    #[test]
    fn rx_head_rejects_malformed_entries() {
        let (mut words, word_count) = rx_entry(&[0x12, 0x00, 0x42], &[0, 0]);
        set_entry_byte(&mut words, 3, 1); // PHY length below the FCS size
        assert_eq!(decode_rx_head(&words[..word_count], word_count * 4), None);

        let (mut words, word_count) = rx_entry(&[0x12, 0x00, 0x42], &[0, 0]);
        set_entry_byte(&mut words, 2, 40); // padding beyond the entry
        assert_eq!(decode_rx_head(&words[..word_count], word_count * 4), None);
    }

    /// Metadata is positioned by FIFO configuration order, not by luck: with
    /// STATUS enabled every following field shifts by one byte.
    #[test]
    fn honours_metadata_ordering_with_status_lqi_rssi_and_timestamp() {
        const CONFIG: u16 = FIFOCFG_APPEND_STATUS
            | FIFOCFG_APPEND_LQI
            | FIFOCFG_APPEND_RSSI
            | FIFOCFG_APPEND_TIMESTAMP;

        let frame = [0x61, 0x88, 0x07];
        // STATUS, LQI, RSSI, timestamp[4].
        let metadata = [0x5A, 0x99, (-70i8) as u8, 0xDE, 0xAD, 0xBE, 0xEF];
        let (words, word_count) = rx_entry(&frame, &metadata);

        let decoded = decode(&words[..word_count], CONFIG, metadata.len())
            .expect("status/LQI/RSSI/timestamp entry should decode");

        assert_eq!(&decoded.data[..decoded.len], &frame);
        // Not the status byte and not the timestamp byte.
        assert_eq!(decoded.modem_lqi, Some(0x99));
        assert_eq!(decoded.rssi, -70);
    }

    /// The appended modem LQI is optional; RSSI alone is enough to produce a
    /// frame, because the stack's LQI comes from RSSI.
    #[test]
    fn decodes_without_append_lqi_and_reports_no_modem_lqi() {
        const CONFIG: u16 = FIFOCFG_APPEND_RSSI | FIFOCFG_APPEND_TIMESTAMP;

        let frame = [0x61, 0x88, 0x01, 0x02];
        let metadata = [(-55i8) as u8, 0x11, 0x22, 0x33, 0x44];
        let (words, word_count) = rx_entry(&frame, &metadata);

        let decoded = decode(&words[..word_count], CONFIG, metadata.len())
            .expect("RSSI alone must be sufficient to decode a frame");

        assert_eq!(&decoded.data[..decoded.len], &frame);
        assert_eq!(decoded.modem_lqi, None);
        assert_eq!(decoded.rssi, -55);
        // And the stack still gets a real, usable link quality.
        assert_eq!(
            crate::lqi::from_rssi_dbm(decoded.rssi),
            crate::lqi::from_rssi_dbm(-55)
        );
    }

    /// Without RSSI there is nothing to derive LQI from. Failing loudly beats
    /// returning a frame carrying a fabricated `lqi = 0`, which is
    /// indistinguishable from a genuinely dead link.
    #[test]
    fn rejects_entries_without_appended_rssi() {
        const CONFIG: u16 = FIFOCFG_APPEND_LQI | FIFOCFG_APPEND_TIMESTAMP;

        let frame = [0x61, 0x88, 0x03];
        let metadata = [0x7B, 0x11, 0x22, 0x33, 0x44];
        let (words, word_count) = rx_entry(&frame, &metadata);

        assert_eq!(
            decode(&words[..word_count], CONFIG, metadata.len()).err(),
            Some(FifoError::MissingRssi)
        );
    }

    #[test]
    fn rejects_ti_invalid_rssi_sentinel() {
        let frame = [0x61, 0x88, 0x04];
        let metadata = [180, RSSI_INVALID as u8, 0x11, 0x22, 0x33, 0x44];
        let (words, word_count) = rx_entry(&frame, &metadata);

        assert_eq!(
            decode(&words[..word_count], TEST_FIFO_CONFIG, metadata.len()).err(),
            Some(FifoError::InvalidRssi)
        );
    }

    /// The regression this change fixes: the CC2340 modem LQI byte is a raw
    /// frequency-offset estimate on an undocumented scale. It must not become
    /// the stack's IEEE LQI, which is RSSI-derived and independent of it.
    #[test]
    fn modem_lqi_is_diagnostic_and_never_the_stack_lqi() {
        let frame = [0x61, 0x88, 0x05];
        // A weak link (-90 dBm) that the modem happens to report as 0xFF.
        let metadata = [0xFF, (-90i8) as u8, 0x00, 0x00, 0x00, 0x00];
        let (words, word_count) = rx_entry(&frame, &metadata);

        let decoded = decode(&words[..word_count], TEST_FIFO_CONFIG, metadata.len())
            .expect("entry should decode");
        let stack_lqi = crate::lqi::from_rssi_dbm(decoded.rssi);

        assert_eq!(decoded.modem_lqi, Some(0xFF));
        // The raw byte would claim a perfect link; the real one is poor.
        assert_ne!(stack_lqi, 0xFF);
        assert!(
            stack_lqi < 64,
            "weak RSSI must yield a low LQI, got {stack_lqi}"
        );

        // And two frames with identical RSSI but opposite modem bytes are
        // indistinguishable to the stack.
        let (other_words, other_count) =
            rx_entry(&frame, &[0x00, (-90i8) as u8, 0x00, 0x00, 0x00, 0x00]);
        let other = decode(&other_words[..other_count], TEST_FIFO_CONFIG, 6).unwrap();
        assert_eq!(other.modem_lqi, Some(0x00));
        assert_eq!(crate::lqi::from_rssi_dbm(other.rssi), stack_lqi);
    }

    #[test]
    fn rejects_inconsistent_metadata_configuration() {
        let words = [0x0005_0003, 0];
        assert_eq!(
            decode(&words, TEST_FIFO_CONFIG, 2).err(),
            Some(FifoError::MetadataMismatch)
        );
    }

    #[test]
    fn filter_registers_match_rcl_single_pan_auto_ack_setup() {
        let registers = filter_registers(&FrameFilter {
            pan_id: 0xDFE9,
            short_addr: 0xA661,
            // Over-the-air order of EUI 00:12:4b:00:30:d2:99:c8.
            ext_addr: [0xC8, 0x99, 0xD2, 0x30, 0x00, 0x4B, 0x12, 0x00],
            promiscuous: false,
        });

        assert_eq!(registers.pib, PIB_PANMODE_SINGLE);
        assert_eq!(registers.pan_id, 0xDFE9);
        assert_eq!(registers.short_addr, 0xA661);
        // RCL writes `(uint32_t) localExtAddr` and `localExtAddr >> 32` of the
        // little-endian 64-bit address.
        assert_eq!(registers.ext_addr_low, 0x30D2_99C8);
        assert_eq!(registers.ext_addr_high, 0x0012_4B00);
        // maxFrameVersion 1, auto-ACK by the PBE, no auto-pend, FP = 0.
        assert_eq!(registers.ffopt0, 0x0104);
        // Beacon + data + MAC command for both frame-version groups, no ACKs.
        assert_eq!(registers.fftype, 0x0B0B);
    }

    #[test]
    fn promiscuous_filter_disables_pan_matching() {
        let registers = filter_registers(&FrameFilter {
            pan_id: 0xFFFF,
            short_addr: 0xFFFF,
            ext_addr: [1, 2, 3, 4, 5, 6, 7, 8],
            promiscuous: true,
        });
        assert_eq!(registers.pib, PIB_PANMODE_PROMISCUOUS);
    }

    #[test]
    fn ack_request_config_matches_rcl_immediate_ack() {
        // ACKMODE_FILTIMMACK | FLAGREQ_EN | expected sequence number.
        assert_eq!(ack_request_config(0x00), 0x0300);
        assert_eq!(ack_request_config(0xA5), 0x03A5);
    }

    #[test]
    fn decodes_ack_status_like_rcl() {
        assert_eq!(decode_ack_status(0x0001), AckStatus::Timeout);
        assert_eq!(
            decode_ack_status(0x0010),
            AckStatus::Received {
                frame_pending: false
            }
        );
        assert_eq!(
            decode_ack_status(0x0030),
            AckStatus::Received {
                frame_pending: true
            }
        );
        assert_eq!(decode_ack_status(0x0002), AckStatus::RxError);
        assert_eq!(decode_ack_status(0x0004), AckStatus::RxError);
        assert_eq!(decode_ack_status(0x0008), AckStatus::RxError);
        assert_eq!(decode_ack_status(0x0000), AckStatus::Unknown(0));
        // A sync timeout wins over stale bits, as in RCL.
        assert_eq!(decode_ack_status(0x0011), AckStatus::Timeout);
    }

    #[test]
    fn pads_data_entries_to_word_boundary() {
        assert_eq!(padded_entry_len(0), 4);
        assert_eq!(padded_entry_len(2), 4);
        assert_eq!(padded_entry_len(3), 8);
        assert_eq!(padded_entry_len(133), 136);
    }
}
