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
}
