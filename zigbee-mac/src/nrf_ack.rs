//! Register-independent receive classification and Imm-Ack scheduling for
//! the nRF52 ISR radio PHY.
//!
//! The RADIO interrupt handler in `nrf::isr_phy` decides, for every frame
//! with a valid FCS, whether to retain it for the shared `SoftMacCore`,
//! whether to acknowledge it, and with which Frame Pending bit, before the
//! 192 µs aTurnaroundTime elapses. Those decisions live here as pure
//! functions so they are host-tested against the same rules the core applies
//! in software (`SoftMacCore::accepts_destination` /
//! `SoftMacCore::is_exact_destination`).

use crate::frames::{build_ack, parse_mac_addresses};
use crate::phy::PhyAddressFilter;
use zigbee_types::{MacAddress, PanId};

/// aTurnaroundTime: last bit of the received frame to first bit of the ACK.
pub const TURNAROUND_US: u32 = 192;
/// RADIO TXEN to TXREADY with MODECNF0.RU = Fast.
pub const TX_RAMP_UP_US: u32 = 40;
/// Latency from the last bit of a received frame on air to the RADIO END
/// event that captures `rx_end` (Nordic's `RX_PHYEND_EVENT_LATENCY_US`).
pub const RX_END_LATENCY_US: u32 = 23;
/// Preamble (4 octets) + SFD (1 octet) at 32 µs per octet. The RADIO ADDRESS
/// event marks the end of the SHR.
pub const SHR_US: u32 = 160;
/// Minimum distance between "now" and the TXEN compare that the ISR accepts.
/// Below this the PPI compare may already have passed while it was being
/// armed, so the ACK is abandoned instead of being sent at an undefined time.
pub const MIN_SCHEDULE_MARGIN_US: u32 = 3;
/// TIMER ticks (1 MHz) from the captured END event to the TXEN trigger, as in
/// Nordic's `nrf_802154_trx_transmit_ack`: the END event already lags the
/// frame's last bit by [`RX_END_LATENCY_US`].
pub const ACK_TXEN_DELAY_US: u32 = TURNAROUND_US - TX_RAMP_UP_US - RX_END_LATENCY_US;

/// Hardware Frame Pending source-match capacity.
///
/// The parent role arms one entry per child with queued indirect data and
/// one per association in progress. A full table is reported to the core,
/// which maps it to `TransactionOverflow`; it is never silently truncated.
pub const NRF_ACK_PENDING_CAPACITY: usize = 16;

/// What the RADIO ISR does with one CRC-valid frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RxClass {
    /// Not for this node: drop it and restart RX.
    Drop,
    /// An Imm-Ack frame: matched against the outstanding transmission, never
    /// queued for the MAC.
    Ack { sequence: u8, frame_pending: bool },
    /// Retain the frame for the MAC. `ack` is `Some(frame_pending)` when an
    /// Imm-Ack must be transmitted.
    Queue { ack: Option<(u8, bool)> },
}

/// Source-address table that selects the Frame Pending bit of outgoing ACKs.
#[derive(Debug, Default)]
pub struct PendingTable {
    entries: heapless::Vec<MacAddress, NRF_ACK_PENDING_CAPACITY>,
}

/// The pending table already holds [`NRF_ACK_PENDING_CAPACITY`] entries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PendingTableFull;

impl PendingTable {
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
        }
    }

    /// Arm (`pending = true`) or disarm one source address. Idempotent.
    pub fn set(&mut self, source: MacAddress, pending: bool) -> Result<(), PendingTableFull> {
        let index = self.entries.iter().position(|entry| *entry == source);
        match (index, pending) {
            (None, true) => self.entries.push(source).map_err(|_| PendingTableFull),
            (Some(index), false) => {
                self.entries.swap_remove(index);
                Ok(())
            }
            _ => Ok(()),
        }
    }

    pub fn contains(&self, source: &MacAddress) -> bool {
        self.entries.iter().any(|entry| entry == source)
    }

    pub fn clear(&mut self) {
        self.entries.clear();
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// Same rule as `SoftMacCore::accepts_destination` with promiscuous off.
fn accepts(filter: &PhyAddressFilter, destination: &MacAddress) -> bool {
    match destination {
        MacAddress::Short(pan_id, address) => {
            (*pan_id == filter.pan_id || *pan_id == PanId::BROADCAST)
                && (*address == filter.short_address || address.0 == 0xFFFF)
        }
        MacAddress::Extended(pan_id, address) => {
            (*pan_id == filter.pan_id || *pan_id == PanId::BROADCAST)
                && *address == filter.extended_address
        }
    }
}

/// Same rule as `SoftMacCore::is_exact_destination`.
fn is_exact(filter: &PhyAddressFilter, destination: &MacAddress) -> bool {
    match destination {
        MacAddress::Short(pan_id, address) => {
            address.0 != 0xFFFF && *pan_id == filter.pan_id && *address == filter.short_address
        }
        MacAddress::Extended(pan_id, address) => {
            *pan_id == filter.pan_id && *address == filter.extended_address
        }
    }
}

/// Classify one received PSDU (FCS already stripped and verified).
///
/// * `filter == None` is promiscuous mode: everything except ACKs is
///   retained and nothing is acknowledged.
/// * Beacons are always retained (the core consumes them during scans).
/// * Data frames and MAC commands are retained when the destination is
///   accepted, and acknowledged when they request an ACK and name this node
///   exactly. Frame Pending is set only on ACKs of MAC commands (Data
///   Request) whose source is armed in `pending`, exactly as the software
///   parent role does.
pub fn classify(
    frame: &[u8],
    filter: Option<&PhyAddressFilter>,
    pending: &PendingTable,
) -> RxClass {
    if frame.len() < 3 {
        return RxClass::Drop;
    }
    let kind = frame[0] & 0x07;
    if kind == 0x02 {
        return RxClass::Ack {
            sequence: frame[2],
            frame_pending: frame[0] & (1 << 4) != 0,
        };
    }
    let Some(filter) = filter else {
        return RxClass::Queue { ack: None };
    };
    if kind == 0x00 {
        return RxClass::Queue { ack: None };
    }
    let command = kind == 0x03;
    if kind != 0x01 && !command {
        return RxClass::Drop;
    }
    let (source, destination, payload_offset, _) = parse_mac_addresses(frame);
    if payload_offset < 3 || payload_offset > frame.len() || !accepts(filter, &destination) {
        return RxClass::Drop;
    }
    let ack = (frame[0] & (1 << 5) != 0 && is_exact(filter, &destination))
        .then(|| (frame[2], command && pending.contains(&source)));
    RxClass::Queue { ack }
}

/// RADIO EasyDMA image of an Imm-Ack: PHR (PSDU + FCS length) then the PSDU.
pub const fn ack_dma_image(sequence: u8, frame_pending: bool) -> [u8; 4] {
    let psdu = build_ack(sequence, frame_pending);
    [psdu.len() as u8 + 2, psdu[0], psdu[1], psdu[2]]
}

/// TIMER compare value that triggers TXEN so the ACK's first bit leaves
/// [`TURNAROUND_US`] after the received frame's last bit (`rx_end` minus
/// [`RX_END_LATENCY_US`]), or `None` when `now` is already
/// too close to (or past) it. Wrapping 32-bit arithmetic: valid across the
/// 71-minute free-running TIMER wrap.
pub const fn ack_txen_compare(rx_end: u32, now: u32) -> Option<u32> {
    let compare = rx_end.wrapping_add(ACK_TXEN_DELAY_US);
    let lead = compare.wrapping_sub(now) as i32;
    if lead >= MIN_SCHEDULE_MARGIN_US as i32 && lead <= ACK_TXEN_DELAY_US as i32 {
        Some(compare)
    } else {
        None
    }
}

/// On-chip turnaround estimate (µs) from the received frame's last bit to
/// the first ACK bit, given the TIMER captures of the received frame's END
/// event and the ACK's ADDRESS (end-of-SHR) event. It depends on the modelled
/// END latency and ADDRESS timing, so it shows jitter and late scheduling but
/// is not an on-air measurement.
pub const fn ack_turnaround(rx_end: u32, ack_address: u32) -> u32 {
    ack_address
        .wrapping_sub(rx_end)
        .wrapping_sub(SHR_US)
        .wrapping_add(RX_END_LATENCY_US)
}

/// Diagnostic counters maintained by the RADIO ISR.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct NrfRadioCounters {
    /// Frames received with a valid FCS.
    pub rx_frames: u32,
    /// Frames dropped on FCS failure.
    pub crc_errors: u32,
    /// Frames for us dropped (and not acknowledged) because the RX ring was
    /// full.
    pub rx_overflow: u32,
    /// Imm-Acks transmitted from the ISR.
    pub acks_sent: u32,
    /// Imm-Acks with Frame Pending set.
    pub acks_frame_pending: u32,
    /// Imm-Acks abandoned because the ISR ran too late to meet the turnaround.
    pub acks_late: u32,
    /// Shortest / longest measured turnaround (µs) of transmitted ACKs.
    pub ack_turnaround_min_us: u32,
    pub ack_turnaround_max_us: u32,
    /// Transmissions started (after CCA).
    pub tx_frames: u32,
    /// Transmit attempts refused because CCA (or an in-progress reception)
    /// found the channel busy.
    pub tx_busy: u32,
    /// Acknowledged transmissions whose ACK did not arrive in time.
    pub tx_no_ack: u32,
}

impl NrfRadioCounters {
    pub const fn new() -> Self {
        Self {
            rx_frames: 0,
            crc_errors: 0,
            rx_overflow: 0,
            acks_sent: 0,
            acks_frame_pending: 0,
            acks_late: 0,
            ack_turnaround_min_us: u32::MAX,
            ack_turnaround_max_us: 0,
            tx_frames: 0,
            tx_busy: 0,
            tx_no_ack: 0,
        }
    }

    pub fn record_ack(&mut self, turnaround_us: u32, frame_pending: bool) {
        self.acks_sent = self.acks_sent.wrapping_add(1);
        if frame_pending {
            self.acks_frame_pending = self.acks_frame_pending.wrapping_add(1);
        }
        self.ack_turnaround_min_us = self.ack_turnaround_min_us.min(turnaround_us);
        self.ack_turnaround_max_us = self.ack_turnaround_max_us.max(turnaround_us);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_types::ShortAddress;

    const PAN: PanId = PanId(0xDFE9);
    const IEEE: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];

    fn filter() -> PhyAddressFilter {
        PhyAddressFilter {
            pan_id: PAN,
            short_address: ShortAddress(0x1234),
            extended_address: IEEE,
        }
    }

    /// Data request (command 0x04), short dst 0x1234, short src `src`,
    /// PAN-ID compression, AR set.
    fn data_request(src: u16, seq: u8) -> [u8; 10] {
        let [d0, d1] = 0x1234u16.to_le_bytes();
        let [s0, s1] = src.to_le_bytes();
        let [p0, p1] = PAN.0.to_le_bytes();
        [0x63, 0x88, seq, p0, p1, d0, d1, s0, s1, 0x04]
    }

    #[test]
    fn acks_exact_data_request_with_pending_source() {
        let mut pending = PendingTable::new();
        pending
            .set(MacAddress::Short(PAN, ShortAddress(0x5678)), true)
            .unwrap();
        assert_eq!(
            classify(&data_request(0x5678, 7), Some(&filter()), &pending),
            RxClass::Queue {
                ack: Some((7, true))
            }
        );
        assert_eq!(
            classify(&data_request(0x9999, 8), Some(&filter()), &pending),
            RxClass::Queue {
                ack: Some((8, false))
            }
        );
    }

    #[test]
    fn extended_source_matches_extended_entry() {
        let mut pending = PendingTable::new();
        let child = [9, 9, 9, 9, 9, 9, 9, 9];
        pending.set(MacAddress::Extended(PAN, child), true).unwrap();
        // Command, AR, PAN compression, dst short, src extended.
        let mut frame = [0u8; 16];
        frame[..9].copy_from_slice(&[0x63, 0xC8, 3, 0xE9, 0xDF, 0x34, 0x12, 9, 9]);
        frame[9..15].copy_from_slice(&[9; 6]);
        frame[15] = 0x04;
        assert_eq!(
            classify(&frame, Some(&filter()), &pending),
            RxClass::Queue {
                ack: Some((3, true))
            }
        );
    }

    #[test]
    fn frame_pending_only_on_commands() {
        let mut pending = PendingTable::new();
        pending
            .set(MacAddress::Short(PAN, ShortAddress(0x5678)), true)
            .unwrap();
        let mut frame = data_request(0x5678, 1);
        frame[0] = 0x61; // data frame, AR
        assert_eq!(
            classify(&frame, Some(&filter()), &pending),
            RxClass::Queue {
                ack: Some((1, false))
            }
        );
    }

    #[test]
    fn broadcasts_are_queued_but_never_acknowledged() {
        let pending = PendingTable::new();
        let frame = [0x41, 0x88, 5, 0xE9, 0xDF, 0xFF, 0xFF, 0x00, 0x00, 0xAA];
        assert_eq!(
            classify(&frame, Some(&filter()), &pending),
            RxClass::Queue { ack: None }
        );
    }

    #[test]
    fn frames_for_other_nodes_are_dropped() {
        let pending = PendingTable::new();
        let mut frame = data_request(0x5678, 1);
        frame[5] = 0x99;
        assert_eq!(classify(&frame, Some(&filter()), &pending), RxClass::Drop);
        let mut other_pan = data_request(0x5678, 1);
        other_pan[3] = 0x00;
        assert_eq!(
            classify(&other_pan, Some(&filter()), &pending),
            RxClass::Drop
        );
    }

    #[test]
    fn no_ack_without_ack_request() {
        let pending = PendingTable::new();
        let mut frame = data_request(0x5678, 1);
        frame[0] = 0x43;
        assert_eq!(
            classify(&frame, Some(&filter()), &pending),
            RxClass::Queue { ack: None }
        );
    }

    #[test]
    fn beacon_request_to_broadcast_pan_is_queued() {
        let pending = PendingTable::new();
        let frame = [0x03, 0x08, 9, 0xFF, 0xFF, 0xFF, 0xFF, 0x07];
        assert_eq!(
            classify(&frame, Some(&filter()), &pending),
            RxClass::Queue { ack: None }
        );
    }

    #[test]
    fn beacons_and_promiscuous_are_retained_without_ack() {
        let pending = PendingTable::new();
        let beacon = [0x00, 0x80, 1, 0xE9, 0xDF, 0x00, 0x00, 0xFF, 0xCF];
        let other = data_request(0x5678, 1);
        assert_eq!(
            classify(&beacon, Some(&filter()), &pending),
            RxClass::Queue { ack: None }
        );
        assert_eq!(
            classify(&other, None, &pending),
            RxClass::Queue { ack: None }
        );
    }

    #[test]
    fn ack_frames_are_reported_not_queued() {
        let pending = PendingTable::new();
        assert_eq!(
            classify(&[0x12, 0x00, 0x44], Some(&filter()), &pending),
            RxClass::Ack {
                sequence: 0x44,
                frame_pending: true
            }
        );
        assert_eq!(classify(&[0x41, 0x88], None, &pending), RxClass::Drop);
    }

    #[test]
    fn pending_table_is_bounded_and_idempotent() {
        let mut table = PendingTable::new();
        for short in 0..NRF_ACK_PENDING_CAPACITY as u16 {
            let entry = MacAddress::Short(PAN, ShortAddress(short));
            table.set(entry, true).unwrap();
            table.set(entry, true).unwrap();
        }
        assert_eq!(table.len(), NRF_ACK_PENDING_CAPACITY);
        assert_eq!(
            table.set(MacAddress::Short(PAN, ShortAddress(0xAAAA)), true),
            Err(PendingTableFull)
        );
        table
            .set(MacAddress::Short(PAN, ShortAddress(0)), false)
            .unwrap();
        table
            .set(MacAddress::Short(PAN, ShortAddress(0)), false)
            .unwrap();
        assert_eq!(table.len(), NRF_ACK_PENDING_CAPACITY - 1);
        table.clear();
        assert!(table.is_empty());
    }

    #[test]
    fn ack_image_has_phr_and_frame_pending() {
        assert_eq!(ack_dma_image(0x42, false), [5, 0x02, 0x00, 0x42]);
        assert_eq!(ack_dma_image(0x42, true), [5, 0x12, 0x00, 0x42]);
    }

    #[test]
    fn txen_compare_meets_turnaround_or_refuses() {
        assert_eq!(ACK_TXEN_DELAY_US, 129);
        assert_eq!(ack_txen_compare(1000, 1000), Some(1129));
        assert_eq!(ack_txen_compare(1000, 1125), Some(1129));
        assert_eq!(ack_txen_compare(1000, 1127), None);
        assert_eq!(ack_txen_compare(1000, 1200), None);
        // Captured END before the 32-bit wrap, ISR after it.
        let rx_end = u32::MAX - 10;
        assert_eq!(ack_txen_compare(rx_end, 5), Some(118));
        // `now` before `rx_end` is a corrupt capture: refuse.
        assert_eq!(ack_txen_compare(1000, 900), None);
    }

    #[test]
    fn turnaround_measurement_subtracts_shr_and_end_latency() {
        // TXEN at END + 129, 40 µs ramp-up, 160 µs SHR: 192 µs on air.
        let address = 1000 + ACK_TXEN_DELAY_US + TX_RAMP_UP_US + SHR_US;
        assert_eq!(ack_turnaround(1000, address), 192);
        assert_eq!(ack_turnaround(u32::MAX - 1, 329 - 2), 192);
    }

    #[test]
    fn counters_track_min_max() {
        let mut counters = NrfRadioCounters::new();
        counters.record_ack(193, false);
        counters.record_ack(191, true);
        assert_eq!(counters.acks_sent, 2);
        assert_eq!(counters.acks_frame_pending, 1);
        assert_eq!(counters.ack_turnaround_min_us, 191);
        assert_eq!(counters.ack_turnaround_max_us, 193);
    }
}
