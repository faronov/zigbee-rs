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

const PBE_IEEE_FIFOCFG: u32 = BUFRAM_BASE + 0x022;
const PBE_IEEE_EXTRABYTES: u32 = BUFRAM_BASE + 0x024;
const PBE_IEEE_RXTIMEOUT: u32 = BUFRAM_BASE + 0x02E;
const PBE_IEEE_OPCFG: u32 = BUFRAM_BASE + 0x030;
const PBE_IEEE_PIB: u32 = BUFRAM_BASE + 0x036;
const PBE_IEEE_CFGAUTOACK: u32 = BUFRAM_BASE + 0x0A8;

const PBE_COMMON_ENDCAUSE: u32 = BUFRAM_BASE + 0x006;
const PBE_COMMON_FIFOCMDADD: u32 = BUFRAM_BASE + 0x008;

const FIFO_COMMAND_ADDRESS: u16 = 0x0068;
const FIFO_STATUS_ADDRESS: u16 = 0x0069;
const TX_FIFO_RESET: u32 = 0x0007;
const RX_FIFO_RESET: u32 = 0x0001;
const BOTH_FIFOS_RESET: u32 = 0x000B;

const IEEE_OP_TX: u32 = 0x1C;
const IEEE_OP_RX: u32 = 0x1D;
const OP_HARD_STOP: u32 = 0x01;
const OPCFG_TX_SINGLE: u16 = 0x0804;
const OPCFG_RX_CONTINUOUS: u16 = 0x0802;

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
pub(super) const EVENT_RX_BUF_FULL: u32 = 1 << 7;
pub(super) const EVENT_RX_OK: u32 = 1 << 8;
pub(super) const EVENT_OP_ERROR: u32 = 1 << 15;
pub(super) const EVENT_TERMINAL: u32 = EVENT_OP_DONE | EVENT_OP_ERROR;

const MAX_FIFO_ENTRY_WORDS: usize = 36;
const MAX_FIFO_ENTRY_BYTES: usize = MAX_FIFO_ENTRY_WORDS * 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum FifoError {
    FrameTooLong,
    NoSpace,
    MalformedEntry,
    MetadataMismatch,
    /// The PBE FIFO configuration does not append RSSI, so no per-frame link
    /// quality can be derived. See [`ReceivedFrame::rssi`].
    MissingRssi,
    /// The PBE appended RSSI, but it is TI's "no valid reading" sentinel.
    InvalidRssi,
}

/// TI reports "RSSI not available" as `0x7F` (`+127 dBm`), a level the radio
/// cannot physically measure.
const RSSI_INVALID: i8 = 127;

pub(super) struct ReceivedFrame {
    pub data: [u8; MAX_PHY_FRAME_LEN],
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
    write_fifo_pointer(PBE_RXFSRP, read32(PBE_RXFRP));
}

pub(super) fn reset_both() {
    write32(PBE_FCMD, BOTH_FIFOS_RESET);
}

pub(super) fn prepare_tx() {
    write16(PBE_IEEE_OPCFG, OPCFG_TX_SINGLE);
    write16(PBE_IEEE_RXTIMEOUT, 0);
    write16(PBE_IEEE_CFGAUTOACK, 0);
}

pub(super) fn prepare_promiscuous_rx() {
    write16(PBE_IEEE_OPCFG, OPCFG_RX_CONTINUOUS);
    write16(PBE_IEEE_RXTIMEOUT, 0);
    write16(PBE_IEEE_PIB, 0);
    write16(PBE_IEEE_CFGAUTOACK, 0);
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

pub(super) fn try_read_rx_frame() -> Result<Option<ReceivedFrame>, FifoError> {
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

    let word_count = byte_count / 4;
    let mut words = [0u32; MAX_FIFO_ENTRY_WORDS];
    for (index, word) in words[..word_count].iter_mut().enumerate() {
        *word = read32(RXF_UNWRAPPED_BASE + fifo_start + read_pointer + index as u32 * 4);
    }

    let next_pointer = (read_pointer + byte_count as u32) % fifo_size;
    write_fifo_pointer(PBE_RXFRP, next_pointer);

    let fifo_config = read16(PBE_IEEE_FIFOCFG);
    let extra_bytes = read16(PBE_IEEE_EXTRABYTES) as usize;
    decode_rx_entry(&words[..word_count], fifo_config, extra_bytes).map(Some)
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

fn decode_rx_entry(
    words: &[u32],
    fifo_config: u16,
    extra_bytes: usize,
) -> Result<ReceivedFrame, FifoError> {
    if words.is_empty() {
        return Err(FifoError::MalformedEntry);
    }

    let entry_length = entry_byte(words, 0) as usize | ((entry_byte(words, 1) as usize) << 8);
    let byte_count = padded_entry_len(entry_length);
    if byte_count > words.len() * 4 || entry_length < 2 {
        return Err(FifoError::MalformedEntry);
    }

    let expected_extra_bytes = appended_byte_count(fifo_config);
    if extra_bytes != expected_extra_bytes || extra_bytes > entry_length {
        return Err(FifoError::MetadataMismatch);
    }

    let num_pad = entry_byte(words, 2) as usize;
    let packet_start = 3usize
        .checked_add(num_pad)
        .ok_or(FifoError::MalformedEntry)?;
    let metadata_start = 2 + entry_length - extra_bytes;
    if packet_start >= metadata_start {
        return Err(FifoError::MalformedEntry);
    }

    let phy_length = entry_byte(words, packet_start) as usize;
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

    let mut data = [0u8; MAX_PHY_FRAME_LEN];
    for (index, byte) in data[..frame_length].iter_mut().enumerate() {
        *byte = entry_byte(words, frame_start + index);
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
        let value = entry_byte(words, metadata_offset);
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
    let rssi = entry_byte(words, metadata_offset) as i8;
    if rssi == RSSI_INVALID {
        return Err(FifoError::InvalidRssi);
    }

    Ok(ReceivedFrame {
        data,
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

        let decoded = decode_rx_entry(&words[..word_count], TEST_FIFO_CONFIG, metadata.len())
            .expect("entry with LQI, RSSI and timestamp should decode");

        assert_eq!(decoded.len, frame.len());
        assert_eq!(&decoded.data[..decoded.len], &frame);
        assert_eq!(decoded.rssi, -42);
        assert_eq!(decoded.modem_lqi, Some(201));
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

        let decoded = decode_rx_entry(&words[..word_count], CONFIG, metadata.len())
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

        let decoded = decode_rx_entry(&words[..word_count], CONFIG, metadata.len())
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
            decode_rx_entry(&words[..word_count], CONFIG, metadata.len()).err(),
            Some(FifoError::MissingRssi)
        );
    }

    #[test]
    fn rejects_ti_invalid_rssi_sentinel() {
        let frame = [0x61, 0x88, 0x04];
        let metadata = [180, RSSI_INVALID as u8, 0x11, 0x22, 0x33, 0x44];
        let (words, word_count) = rx_entry(&frame, &metadata);

        assert_eq!(
            decode_rx_entry(&words[..word_count], TEST_FIFO_CONFIG, metadata.len()).err(),
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

        let decoded = decode_rx_entry(&words[..word_count], TEST_FIFO_CONFIG, metadata.len())
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
        let other = decode_rx_entry(&other_words[..other_count], TEST_FIFO_CONFIG, 6).unwrap();
        assert_eq!(other.modem_lqi, Some(0x00));
        assert_eq!(crate::lqi::from_rssi_dbm(other.rssi), stack_lqi);
    }

    #[test]
    fn rejects_inconsistent_metadata_configuration() {
        let words = [0x0005_0003, 0];
        assert_eq!(
            decode_rx_entry(&words, TEST_FIFO_CONFIG, 2).err(),
            Some(FifoError::MetadataMismatch)
        );
    }

    #[test]
    fn pads_data_entries_to_word_boundary() {
        assert_eq!(padded_entry_len(0), 4);
        assert_eq!(padded_entry_len(2), 4);
        assert_eq!(padded_entry_len(3), 8);
        assert_eq!(padded_entry_len(133), 136);
    }
}
