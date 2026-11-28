//! Bounded source table for Frame Pending in hardware auto-ACKs.
//!
//! A parent whose radio sends the Imm-Ack itself (hardware auto-ACK) has to
//! decide the ACK's Frame Pending bit in the receive interrupt, before the
//! turnaround elapses. [`AckPendingTable`] is the heap-free set of armed child
//! identities that a PHY consults there; [`data_request_source`] extracts the
//! exact child identity from a received MAC Data Request.
//!
//! The parent MAC core arms an entry through
//! [`RadioPhy::set_ack_frame_pending`](crate::phy::RadioPhy::set_ack_frame_pending)
//! with exactly one owner: the PHY. The core's software pending queue for
//! this ACK path stays at `PENDING = 0`.
//!
//! [`PollAckLog`] carries each ACK's decision to the frame it answered, for a
//! PHY whose interrupt cannot mark the queued frame itself
//! ([`PhyRxFrame::acked_without_frame_pending`](crate::PhyRxFrame::acked_without_frame_pending)).

use zigbee_types::{MacAddress, PanId, ShortAddress};

/// MAC command identifier of a Data Request (IEEE 802.15.4-2015 §7.5.5).
const CMD_DATA_REQUEST: u8 = 0x04;

/// Why an entry could not be armed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AckPendingError {
    /// Broadcast PAN, broadcast/reserved short address or all-ones EUI-64.
    InvalidAddress,
    /// Every slot is armed; the caller must report a transaction overflow.
    TableFull,
}

/// Fixed-capacity set of child addresses whose Data Requests are acknowledged
/// with Frame Pending = 1.
///
/// Entries are kept in a contiguous prefix; lookup is a bounded linear scan
/// of at most `N` entries, which keeps the interrupt-time cost fixed.
#[derive(Debug, Clone)]
pub struct AckPendingTable<const N: usize> {
    entries: [MacAddress; N],
    len: usize,
}

const EMPTY: MacAddress = MacAddress::Short(PanId(0), ShortAddress(0));

impl<const N: usize> AckPendingTable<N> {
    pub const fn new() -> Self {
        Self {
            entries: [EMPTY; N],
            len: 0,
        }
    }

    /// Number of armed sources.
    pub const fn len(&self) -> usize {
        self.len
    }

    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Arm (`pending`) or disarm one source. Disarming an unknown source
    /// succeeds; arming an already armed source is idempotent.
    pub fn set(&mut self, address: MacAddress, pending: bool) -> Result<(), AckPendingError> {
        if !valid_address(&address) {
            return Err(AckPendingError::InvalidAddress);
        }
        match (self.position(&address), pending) {
            (Some(_), true) | (None, false) => Ok(()),
            (Some(index), false) => {
                self.len -= 1;
                self.entries[index] = self.entries[self.len];
                self.entries[self.len] = EMPTY;
                Ok(())
            }
            (None, true) => {
                let slot = self
                    .entries
                    .get_mut(self.len)
                    .ok_or(AckPendingError::TableFull)?;
                *slot = address;
                self.len += 1;
                Ok(())
            }
        }
    }

    /// Whether `address` is armed.
    pub fn contains(&self, address: &MacAddress) -> bool {
        self.position(address).is_some()
    }

    /// Frame Pending for the ACK of `frame` (MHR + payload, no FCS): true
    /// only for a valid, unsecured Data Request from an armed source.
    pub fn frame_pending(&self, frame: &[u8]) -> bool {
        data_request_source(frame).is_some_and(|source| self.contains(&source))
    }

    /// Disarm every source (MLME-RESET).
    pub fn clear(&mut self) {
        self.entries = [EMPTY; N];
        self.len = 0;
    }

    fn position(&self, address: &MacAddress) -> Option<usize> {
        self.entries[..self.len]
            .iter()
            .position(|entry| entry == address)
    }
}

impl<const N: usize> Default for AckPendingTable<N> {
    fn default() -> Self {
        Self::new()
    }
}

fn valid_address(address: &MacAddress) -> bool {
    match *address {
        MacAddress::Short(pan, short) => pan != PanId::BROADCAST && short.0 < 0xFFF8,
        MacAddress::Extended(pan, ieee) => pan != PanId::BROADCAST && ieee != [0xFF; 8],
    }
}

/// Exact source identity of a valid, unsecured, ACK-requesting MAC Data
/// Request (`frame` is MHR + payload without FCS). The source PAN is the
/// destination PAN under PAN-ID compression and must otherwise equal it.
pub fn data_request_source(frame: &[u8]) -> Option<MacAddress> {
    let fc = u16::from_le_bytes([*frame.first()?, *frame.get(1)?]);
    let frame_type = fc & 0x07;
    let security = fc & (1 << 3) != 0;
    let ack_request = fc & (1 << 5) != 0;
    if frame_type != 0x03 || security || !ack_request {
        return None;
    }
    let dst_mode = (fc >> 10) & 0x03;
    let src_mode = (fc >> 14) & 0x03;
    if !matches!(dst_mode, 2 | 3) || !matches!(src_mode, 2 | 3) {
        return None;
    }

    let mut offset = 3;
    let dst_pan = read_u16(frame, offset)?;
    offset += 2 + if dst_mode == 2 { 2 } else { 8 };
    let src_pan = if fc & (1 << 6) != 0 {
        dst_pan
    } else {
        let pan = read_u16(frame, offset)?;
        offset += 2;
        pan
    };
    if src_pan != dst_pan {
        return None;
    }
    let pan = PanId(src_pan);
    let source = if src_mode == 2 {
        let short = read_u16(frame, offset)?;
        offset += 2;
        MacAddress::Short(pan, ShortAddress(short))
    } else {
        let mut ieee = [0u8; 8];
        ieee.copy_from_slice(frame.get(offset..offset + 8)?);
        offset += 8;
        MacAddress::Extended(pan, ieee)
    };
    if frame.len() != offset + 1 || frame[offset] != CMD_DATA_REQUEST || !valid_address(&source) {
        return None;
    }
    Some(source)
}

/// The Frame Pending decisions of the auto-ACKs to received Data Requests,
/// in reception order, until the task dequeues the polls they answered.
///
/// The interrupt [`record`](Self::record)s every acknowledged Data Request;
/// the task [`take`](Self::take)s the decision when the same frame (source
/// and sequence number) comes out of the radio's receive queue. Entries
/// ahead of the match belong to frames the radio's queue dropped and are
/// discarded with it. Nothing is kept per child, so any number of children
/// is covered; `N` only has to cover the polls one receive queue can hold.
/// When it is exceeded the oldest decision is overwritten and its poll
/// reports `None` (the decision is unknown).
#[derive(Debug, Clone)]
pub struct PollAckLog<const N: usize> {
    entries: [(MacAddress, u8, bool); N],
    /// Index of the oldest entry.
    head: usize,
    len: usize,
}

impl<const N: usize> PollAckLog<N> {
    pub const fn new() -> Self {
        Self {
            entries: [(EMPTY, 0, false); N],
            head: 0,
            len: 0,
        }
    }

    /// The ACK to the Data Request `sequence` from `source` carried
    /// `frame_pending` (interrupt side).
    pub fn record(&mut self, source: MacAddress, sequence: u8, frame_pending: bool) {
        if N == 0 {
            return;
        }
        if self.len == N {
            self.head = (self.head + 1) % N;
            self.len -= 1;
        }
        self.entries[(self.head + self.len) % N] = (source, sequence, frame_pending);
        self.len += 1;
    }

    /// The Frame Pending bit of the ACK to the Data Request `sequence` from
    /// `source`, now out of the receive queue; `None` if it was not
    /// recorded, was overwritten, or cannot be told apart: a later record of
    /// the same poll (a MAC retransmission after a lost ACK) with the other
    /// decision, which this frame is if the queue dropped the first copy.
    pub fn take(&mut self, source: &MacAddress, sequence: u8) -> Option<bool> {
        let is_poll = |i: usize| {
            let (address, seq, _) = &self.entries[(self.head + i) % N];
            address == source && *seq == sequence
        };
        let skip = (0..self.len).find(|&i| is_poll(i))?;
        let (_, _, frame_pending) = self.entries[(self.head + skip) % N];
        let ambiguous = (skip + 1..self.len)
            .any(|i| is_poll(i) && self.entries[(self.head + i) % N].2 != frame_pending);
        self.head = (self.head + skip + 1) % N;
        self.len -= skip + 1;
        (!ambiguous).then_some(frame_pending)
    }

    /// Forget every decision (MLME-RESET).
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

impl<const N: usize> Default for PollAckLog<N> {
    fn default() -> Self {
        Self::new()
    }
}

fn read_u16(frame: &[u8], offset: usize) -> Option<u16> {
    Some(u16::from_le_bytes([
        *frame.get(offset)?,
        *frame.get(offset + 1)?,
    ]))
}

#[cfg(test)]
mod tests {
    use super::*;

    const PAN: u16 = 0xDFE9;
    const CHILD_EXT: [u8; 8] = [0x7C, 0xB9, 0x4C, 0x61, 0x92, 0x3A, 0x00, 0x00];

    fn short(address: u16) -> MacAddress {
        MacAddress::Short(PanId(PAN), ShortAddress(address))
    }

    /// Data Request, PAN-ID compressed, dst short parent, src short child.
    fn request_short(seq: u8, parent: u16, child: u16) -> [u8; 10] {
        let [p0, p1] = PAN.to_le_bytes();
        let [d0, d1] = parent.to_le_bytes();
        let [s0, s1] = child.to_le_bytes();
        [0x63, 0x88, seq, p0, p1, d0, d1, s0, s1, CMD_DATA_REQUEST]
    }

    /// Data Request during association: dst short parent, src EUI-64.
    fn request_extended(parent: u16, child: [u8; 8]) -> [u8; 16] {
        let [p0, p1] = PAN.to_le_bytes();
        let [d0, d1] = parent.to_le_bytes();
        let mut frame = [0u8; 16];
        frame[..7].copy_from_slice(&[0x63, 0xC8, 7, p0, p1, d0, d1]);
        frame[7..15].copy_from_slice(&child);
        frame[15] = CMD_DATA_REQUEST;
        frame
    }

    #[test]
    fn matches_only_armed_data_request_source() {
        let mut table = AckPendingTable::<4>::new();
        table.set(short(0x3344), true).unwrap();

        assert!(table.frame_pending(&request_short(1, 0x0001, 0x3344)));
        assert!(!table.frame_pending(&request_short(2, 0x0001, 0x3345)));

        // Data frame from the same source: never Frame Pending.
        let mut data = request_short(3, 0x0001, 0x3344);
        data[0] = 0x61;
        assert!(!table.frame_pending(&data));

        table.set(short(0x3344), false).unwrap();
        assert!(!table.frame_pending(&request_short(1, 0x0001, 0x3344)));
        assert!(table.is_empty());
    }

    #[test]
    fn extended_source_and_pan_must_match() {
        let mut table = AckPendingTable::<4>::new();
        table
            .set(MacAddress::Extended(PanId(PAN), CHILD_EXT), true)
            .unwrap();
        assert!(table.frame_pending(&request_extended(0x0001, CHILD_EXT)));

        let mut other = CHILD_EXT;
        other[0] ^= 1;
        assert!(!table.frame_pending(&request_extended(0x0001, other)));

        // Same short address on another PAN is a different source.
        table.set(short(0x1111), true).unwrap();
        let mut foreign = request_short(1, 0x0001, 0x1111);
        foreign[3] ^= 1;
        assert!(!table.frame_pending(&foreign));
    }

    #[test]
    fn rejects_secured_unacknowledged_and_malformed_requests() {
        let request = request_short(1, 0x0001, 0x3344);
        assert_eq!(data_request_source(&request), Some(short(0x3344)));

        let mut secured = request;
        secured[0] |= 1 << 3;
        assert_eq!(data_request_source(&secured), None);

        let mut no_ack = request;
        no_ack[0] &= !(1 << 5);
        assert_eq!(data_request_source(&no_ack), None);

        let mut other_command = request;
        other_command[9] = 0x01;
        assert_eq!(data_request_source(&other_command), None);

        // FCS (or RSSI/LQI) left attached: not an exact Data Request.
        let mut with_fcs = [0u8; 12];
        with_fcs[..10].copy_from_slice(&request);
        assert_eq!(data_request_source(&with_fcs), None);

        for len in 0..request.len() {
            assert_eq!(data_request_source(&request[..len]), None);
        }
    }

    #[test]
    fn association_request_never_matches() {
        // Association Request from the BL702 -> TLSR8258 parent capture:
        // source PAN 0xFFFF and command ID 0x01.
        let association_request = [
            0x23, 0xC8, 0x9F, 0xE9, 0xDF, 0xDF, 0xF8, 0xFF, 0xFF, 0x7C, 0xB9, 0x4C, 0x61, 0x92,
            0x3A, 0x00, 0x00, 0x01, 0x80,
        ];
        assert_eq!(data_request_source(&association_request), None);
        let mut table = AckPendingTable::<4>::new();
        table
            .set(MacAddress::Extended(PanId(PAN), CHILD_EXT), true)
            .unwrap();
        assert!(!table.frame_pending(&association_request));
    }

    #[test]
    fn bounded_validated_and_clearable() {
        let mut table = AckPendingTable::<3>::new();
        for address in 0..3 {
            table.set(short(address), true).unwrap();
        }
        // Re-arming an armed source does not consume a slot.
        table.set(short(1), true).unwrap();
        assert_eq!(table.set(short(9), true), Err(AckPendingError::TableFull));
        assert_eq!(table.len(), 3);

        // Removing from the middle keeps the remaining entries.
        table.set(short(1), false).unwrap();
        assert!(table.contains(&short(0)) && table.contains(&short(2)));
        assert!(!table.contains(&short(1)));
        table.set(short(9), true).unwrap();

        assert_eq!(
            table.set(short(0xFFFF), true),
            Err(AckPendingError::InvalidAddress)
        );
        assert_eq!(
            table.set(MacAddress::Short(PanId::BROADCAST, ShortAddress(1)), true),
            Err(AckPendingError::InvalidAddress)
        );
        assert_eq!(
            table.set(MacAddress::Extended(PanId(PAN), [0xFF; 8]), true),
            Err(AckPendingError::InvalidAddress)
        );

        table.clear();
        assert!(table.is_empty());
        assert!(!table.frame_pending(&request_short(1, 0x0001, 0x0000)));
    }

    #[test]
    fn poll_ack_log_matches_each_poll_with_its_own_ack() {
        let mut log = PollAckLog::<4>::new();
        // Polls of more children than the log holds, each taken in turn:
        // nothing accumulates per child.
        for child in 0x1000u16..0x1040 {
            log.record(short(child), child as u8, false);
            assert_eq!(log.take(&short(child), child as u8), Some(false));
        }
        // Two polls queued before the task reads them: the stale one
        // (FP = 0) and the next one (FP = 1, the response armed meanwhile).
        log.record(short(0x12D2), 7, false);
        log.record(short(0x12D2), 8, true);
        assert_eq!(log.take(&short(0x12D2), 7), Some(false));
        assert_eq!(log.take(&short(0x12D2), 8), Some(true));
        // A MAC retransmission (same sequence) with the same decision is
        // matched in order.
        log.record(short(0x12D2), 9, true);
        log.record(short(0x12D2), 9, true);
        assert_eq!(log.take(&short(0x12D2), 9), Some(true));
        assert_eq!(log.take(&short(0x12D2), 9), Some(true));
        assert_eq!(log.take(&short(0x12D2), 9), None);
    }

    /// A retransmitted poll whose two ACKs differ: the first copy cannot be
    /// told from the second when the radio queue may have dropped the first,
    /// so it is unknown; the second (the last record) is not.
    #[test]
    fn poll_ack_log_never_guesses_between_retransmissions() {
        let mut log = PollAckLog::<4>::new();
        // Both copies dequeued.
        log.record(short(0x12D2), 9, false);
        log.record(short(0x12D2), 9, true);
        assert_eq!(log.take(&short(0x12D2), 9), None);
        assert_eq!(log.take(&short(0x12D2), 9), Some(true));
        // The first copy was dropped by the radio queue: the one dequeued
        // (the retransmission, FP = 1) is not given the first ACK's FP = 0.
        log.record(short(0x12D2), 10, false);
        log.record(short(0x12D2), 10, true);
        log.record(short(0x0003), 3, false);
        assert_eq!(log.take(&short(0x12D2), 10), None);
        // The lingering record is discarded with the next poll's match.
        assert_eq!(log.take(&short(0x0003), 3), Some(false));
        assert_eq!(log.take(&short(0x12D2), 10), None);
    }

    #[test]
    fn poll_ack_log_discards_polls_the_queue_dropped_and_bounds_itself() {
        let mut log = PollAckLog::<4>::new();
        let ext = MacAddress::Extended(PanId(PAN), CHILD_EXT);
        log.record(short(0x0001), 1, false); // dropped by the radio queue
        log.record(ext, 2, false);
        log.record(short(0x0003), 3, true);
        // A frame without a record (not a poll, or not acknowledged) takes
        // nothing.
        assert_eq!(log.take(&short(0x0009), 1), None);
        assert_eq!(log.take(&ext, 2), Some(false));
        // The dropped poll's decision went with it.
        assert_eq!(log.take(&short(0x0001), 1), None);
        assert_eq!(log.take(&short(0x0003), 3), Some(true));
        // Full: the oldest decision is overwritten, its poll is unknown.
        for seq in 10..15 {
            log.record(short(0x0004), seq, false);
        }
        assert_eq!(log.take(&short(0x0004), 10), None);
        assert_eq!(log.take(&short(0x0004), 11), Some(false));
        log.clear();
        assert_eq!(log.take(&short(0x0004), 12), None);
        // An empty log never panics.
        let mut none = PollAckLog::<0>::new();
        none.record(short(0x0001), 1, false);
        assert_eq!(none.take(&short(0x0001), 1), None);
    }
}
