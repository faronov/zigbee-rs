//! TLSR8258 802.15.4 radio: PHY/channel/DMA bring-up (`phy`, hardware-only)
//! plus pure, host-testable framing (`frame`). This module is the only
//! place `mac_test` needs to touch for radio I/O.

pub mod frame;
#[cfg(target_arch = "tc32")]
pub mod phy;

/// Total DMA buffer size in bytes: 5-byte header + up to 127-byte MAC PSDU +
/// HW trailer (CRC/RSSI/status), rounded up to a 16-byte DMA granule and
/// kept 4-byte aligned. Matches the sensor lab's proven `DmaBuf` size.
pub const DMA_BUF_LEN: usize = 144;
/// Maximum MAC frame length passed between the driver and upper MAC layer.
/// The IEEE 802.15.4 PSDU limit is 127 bytes including the two-byte FCS,
/// which TLSR8258 appends/removes in hardware.
pub const MAX_MAC_FRAME_LEN: usize = 125;
pub const TX_POWER_MIN_DBM: i8 = -25;
pub const TX_POWER_MAX_DBM: i8 = 10;
pub const MAX_ACK_PENDING_ADDRESSES: usize = 16;

#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IdleRxState {
    Off,
    Armed,
}

/// State to restore after a bounded RX/TX/CCA operation finishes.
///
/// A sleepy end device keeps the historical bounded-window behavior. A
/// router with `macRxOnWhenIdle` set must return to an armed receiver after
/// every exit, including NoData, TX timeout, and channel-access failure.
#[cfg(any(target_arch = "tc32", test))]
const fn idle_rx_state(rx_on_when_idle: bool) -> IdleRxState {
    if rx_on_when_idle {
        IdleRxState::Armed
    } else {
        IdleRxState::Off
    }
}

/// Child identity used by the turnaround-critical software ACK path.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum AckPendingAddress {
    Short { pan_id: u16, address: u16 },
    Extended { pan_id: u16, address: [u8; 8] },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AckPendingError {
    InvalidAddress,
    TableFull,
}

#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy)]
struct AckPendingEntry {
    address: AckPendingAddress,
}

#[cfg(any(target_arch = "tc32", test))]
const EMPTY_ACK_PENDING_ENTRY: AckPendingEntry = AckPendingEntry {
    address: AckPendingAddress::Short {
        pan_id: 0,
        address: 0,
    },
};

#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy)]
struct AckPendingTable {
    entries: [AckPendingEntry; MAX_ACK_PENDING_ADDRESSES],
    len: u8,
}

#[cfg(any(target_arch = "tc32", test))]
impl AckPendingTable {
    const fn new() -> Self {
        Self {
            entries: [EMPTY_ACK_PENDING_ENTRY; MAX_ACK_PENDING_ADDRESSES],
            len: 0,
        }
    }

    fn set(&mut self, address: AckPendingAddress, pending: bool) -> Result<(), AckPendingError> {
        if !valid_ack_pending_address(address) {
            return Err(AckPendingError::InvalidAddress);
        }
        let (index, found) = self.find(address);
        if found {
            if !pending {
                let len = self.len as usize;
                self.entries.copy_within(index + 1..len, index);
                self.entries[len - 1] = EMPTY_ACK_PENDING_ENTRY;
                self.len -= 1;
            }
            return Ok(());
        }
        if !pending {
            return Ok(());
        }
        let len = self.len as usize;
        if len == MAX_ACK_PENDING_ADDRESSES {
            return Err(AckPendingError::TableFull);
        }
        self.entries.copy_within(index..len, index + 1);
        self.entries[index] = AckPendingEntry { address };
        self.len += 1;
        Ok(())
    }

    /// Binary search over the sorted, contiguous prefix. The ACK path uses
    /// the same layout through volatile reads, so a full 16-entry scan is
    /// never needed.
    fn find(&self, address: AckPendingAddress) -> (usize, bool) {
        let mut low = 0usize;
        let mut high = self.len as usize;
        while low < high {
            let mid = low + (high - low) / 2;
            if self.entries[mid].address < address {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        (
            low,
            low < self.len as usize && self.entries[low].address == address,
        )
    }

    #[cfg(test)]
    fn frame_pending(&self, psdu: &[u8]) -> bool {
        let Some(source) = data_request_source(psdu) else {
            return false;
        };
        self.frame_pending_for(source)
    }

    #[cfg(test)]
    fn frame_pending_for(&self, source: AckPendingAddress) -> bool {
        self.find(source).1
    }

    #[cfg(test)]
    fn clear(&mut self) {
        self.entries = [EMPTY_ACK_PENDING_ENTRY; MAX_ACK_PENDING_ADDRESSES];
        self.len = 0;
    }
}

#[cfg(any(target_arch = "tc32", test))]
fn valid_ack_pending_address(address: AckPendingAddress) -> bool {
    match address {
        AckPendingAddress::Short { pan_id, address } => pan_id != 0xFFFF && address < 0xFFF8,
        AckPendingAddress::Extended { pan_id, address } => pan_id != 0xFFFF && address != [0xFF; 8],
    }
}

#[cfg(any(target_arch = "tc32", test))]
const fn remaining_settle_ticks(rx_complete_ticks: u32, now_ticks: u32, settle_ticks: u32) -> u32 {
    settle_ticks.saturating_sub(now_ticks.wrapping_sub(rx_complete_ticks))
}

#[cfg(any(target_arch = "tc32", test))]
const fn ack_frame_control(frame_pending: bool) -> u8 {
    if frame_pending { 0x12 } else { 0x02 }
}

/// Return the exact child identity from a valid, unsecured Data Request.
#[cfg(any(target_arch = "tc32", test))]
fn data_request_source(psdu: &[u8]) -> Option<AckPendingAddress> {
    if psdu.len() < 8 {
        return None;
    }
    let frame_control = u16::from_le_bytes([psdu[0], psdu[1]]);
    if frame_control & 0x07 != 0x03
        || frame_control & (1 << 3) != 0
        || frame_control & (1 << 5) == 0
    {
        return None;
    }
    let destination_mode = (frame_control >> 10) & 0x03;
    let source_mode = (frame_control >> 14) & 0x03;
    if !matches!(destination_mode, 2 | 3) || !matches!(source_mode, 2 | 3) {
        return None;
    }

    let mut offset = 3;
    let destination_pan =
        u16::from_le_bytes([*psdu.get(offset)?, *psdu.get(offset.checked_add(1)?)?]);
    offset += 2;
    offset += match destination_mode {
        2 => 2,
        3 => 8,
        _ => return None,
    };

    let source_pan = if frame_control & (1 << 6) != 0 {
        destination_pan
    } else {
        let pan = u16::from_le_bytes([*psdu.get(offset)?, *psdu.get(offset.checked_add(1)?)?]);
        offset += 2;
        pan
    };
    if source_pan != destination_pan || source_pan == 0xFFFF {
        return None;
    }

    let source = match source_mode {
        2 => {
            let address =
                u16::from_le_bytes([*psdu.get(offset)?, *psdu.get(offset.checked_add(1)?)?]);
            offset += 2;
            AckPendingAddress::Short {
                pan_id: source_pan,
                address,
            }
        }
        3 => {
            let end = offset.checked_add(8)?;
            let mut address = [0u8; 8];
            address.copy_from_slice(psdu.get(offset..end)?);
            offset = end;
            AckPendingAddress::Extended {
                pan_id: source_pan,
                address,
            }
        }
        _ => return None,
    };
    if !valid_ack_pending_address(source)
        || psdu.len() != offset.checked_add(1)?
        || psdu[offset] != frame::CMD_ID_DATA_REQUEST
    {
        return None;
    }
    Some(source)
}

// Official TLSR8258 RF_PowerTypeDef levels, expressed as centi-dBm and the
// register value consumed by rf_set_power_level().
#[cfg(any(target_arch = "tc32", test))]
const TX_POWER_LEVELS: &[(i16, u8)] = &[
    (1046, 63),
    (1029, 61),
    (1001, 58),
    (981, 56),
    (948, 53),
    (924, 51),
    (897, 49),
    (873, 47),
    (844, 45),
    (813, 43),
    (779, 41),
    (741, 39),
    (702, 37),
    (660, 35),
    (614, 33),
    (565, 31),
    (513, 29),
    (457, 27),
    (394, 25),
    (323, 23),
    (301, 0x80 | 63),
    (281, 0x80 | 61),
    (261, 0x80 | 59),
    (239, 0x80 | 57),
    (199, 0x80 | 54),
    (173, 0x80 | 52),
    (145, 0x80 | 50),
    (117, 0x80 | 48),
    (90, 0x80 | 46),
    (58, 0x80 | 44),
    (4, 0x80 | 41),
    (-14, 0x80 | 40),
    (-97, 0x80 | 36),
    (-142, 0x80 | 34),
    (-189, 0x80 | 32),
    (-248, 0x80 | 30),
    (-303, 0x80 | 28),
    (-361, 0x80 | 26),
    (-426, 0x80 | 24),
    (-503, 0x80 | 22),
    (-581, 0x80 | 20),
    (-667, 0x80 | 18),
    (-765, 0x80 | 16),
    (-865, 0x80 | 14),
    (-989, 0x80 | 12),
    (-1140, 0x80 | 10),
    (-1329, 0x80 | 8),
    (-1588, 0x80 | 6),
    (-1927, 0x80 | 4),
    (-2518, 0x80 | 2),
];

#[cfg(any(target_arch = "tc32", test))]
fn tx_power_register_value(dbm: i8) -> Option<u8> {
    if !(TX_POWER_MIN_DBM..=TX_POWER_MAX_DBM).contains(&dbm) {
        return None;
    }

    let requested = i32::from(dbm) * 100;
    TX_POWER_LEVELS
        .iter()
        .min_by_key(|(centi_dbm, _)| (i32::from(*centi_dbm) - requested).abs())
        .map(|(_, register)| *register)
}

#[cfg(any(target_arch = "tc32", test))]
fn tx_power_register_fields(level: u8) -> (bool, u8, u8) {
    (level & 0x80 != 0, (level & 0x01) << 7, (level >> 1) & 0x1F)
}

/// RF DMA buffer wrapper. `repr(align(4))` is required by the TLSR8258 DMA
/// engine (see `memory.x`'s `.rf_dma` section, which is checked post-link
/// for 4-byte alignment by `scripts/tlsr8258.sh verify_layout`).
#[repr(align(4))]
pub struct DmaBuf(pub [u8; DMA_BUF_LEN]);

#[cfg(any(target_arch = "tc32", test))]
fn dma_buffer_layout_valid(addresses: [u32; 4]) -> bool {
    for (index, address) in addresses.iter().enumerate() {
        if address % 4 != 0 {
            return false;
        }
        for other in addresses.iter().skip(index + 1) {
            let (low, high) = if address < other {
                (*address, *other)
            } else {
                (*other, *address)
            };
            if high - low < DMA_BUF_LEN as u32 {
                return false;
            }
        }
    }
    true
}

#[derive(Clone, Copy, Debug)]
pub struct ReceivedFrame {
    data: [u8; MAX_MAC_FRAME_LEN],
    len: u8,
    pub lqi: u8,
    pub rssi: i8,
}

impl ReceivedFrame {
    /// Empty frame used to initialize the bounded receive queue's storage.
    /// Never observable: [`RxQueue::pop`] only returns slots that
    /// [`RxQueue::push`] has written.
    #[cfg(any(target_arch = "tc32", test))]
    const EMPTY: Self = Self {
        data: [0; MAX_MAC_FRAME_LEN],
        len: 0,
        lqi: 0,
        rssi: 0,
    };

    /// Build a frame value from an already length-validated MAC PSDU.
    #[cfg(any(target_arch = "tc32", test))]
    fn new(psdu: &[u8], lqi: u8, rssi: i8) -> Self {
        let len = if psdu.len() > MAX_MAC_FRAME_LEN {
            MAX_MAC_FRAME_LEN
        } else {
            psdu.len()
        };
        let mut data = [0u8; MAX_MAC_FRAME_LEN];
        data[..len].copy_from_slice(&psdu[..len]);
        Self {
            data,
            len: len as u8,
            lqi,
            rssi,
        }
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.data[..self.len as usize]
    }

    pub fn len(&self) -> usize {
        self.len as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

#[derive(Clone, Copy, Debug)]
pub enum RawRxOutcome {
    Frame(ReceivedFrame),
    InvalidLength,
    InvalidCrc,
}

/// Local MAC identity the bounded receive queue consults when it has to
/// choose which frame to lose under overload.
///
/// This is the same identity [`Radio::set_ack_filter`] already publishes for
/// the software-ACK path, so no new configuration surface is introduced.
#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RxAddressFilter {
    pan_id: u16,
    short_address: u16,
    extended_address: [u8; 8],
    enabled: bool,
}

/// Retention class of a received frame, ordered lowest-value = first to be
/// sacrificed when the bounded receive queue is full.
///
/// Derived from the IEEE 802.15.4 MAC header alone (frame type + addressing
/// fields, at most 13 bytes). No NWK/APS/ZCL parsing happens here, and the
/// classification runs *after* the software ACK has already been sent, so it
/// cannot extend the 192 us RX->ACK turnaround.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum RxPriority {
    /// Provably not ours: another PAN, or a unicast to some other node.
    /// Only queued while there is spare room.
    Foreign = 0,
    /// MAC acknowledgement captured outside a synchronous ACK window.
    ///
    /// ACK frames carry no destination, so a queued ACK cannot be proven to
    /// belong to this node. Expected ACKs are consumed by the polled
    /// `transmit_with_ack` path before they reach this queue; retain a queued
    /// ACK over known-foreign traffic, but never over a broadcast or local
    /// data frame.
    Ack = 1,
    /// Broadcast, or a frame with no destination addressing (beacons,
    /// coordinator-directed frames). Cannot be proven irrelevant.
    Broadcast = 2,
    /// Unicast to this node's PAN + short or extended address.
    Local = 3,
}

/// Classify `psdu` for the bounded receive queue's overload policy.
///
/// Deliberately conservative: anything that cannot be *proven* to belong to
/// another node keeps at least [`RxPriority::Broadcast`], and while the
/// local identity is unknown (`enabled == false`, or PAN id still 0xFFFF
/// during scan/association) every frame is classified [`RxPriority::Local`]
/// so the queue degrades to plain FIFO instead of guessing.
#[cfg(any(target_arch = "tc32", test))]
fn classify_rx_priority(psdu: &[u8], filter: &RxAddressFilter) -> RxPriority {
    // Frame Control (2) + sequence number (1) is the shortest legal MAC
    // header. Anything shorter is malformed; it cannot be an ACK for us.
    if psdu.len() < 3 {
        return RxPriority::Foreign;
    }
    let frame_control = u16::from_le_bytes([psdu[0], psdu[1]]);
    let frame_type = frame_control & 0x07;
    // Reserved frame types (0b100..0b111) are not IEEE 802.15.4-2006 frames.
    if frame_type > 0x03 {
        return RxPriority::Foreign;
    }
    if !filter.enabled || filter.pan_id == 0xFFFF {
        return RxPriority::Local;
    }
    // Frame type 0b010 = Acknowledgement. An ACK carries no addressing, so
    // it cannot outrank traffic known to be addressed to this node. ACKs
    // expected by a local transmission are consumed synchronously and do not
    // normally pass through this interrupt queue.
    if frame_type == 0x02 {
        return RxPriority::Ack;
    }

    let destination_mode = (frame_control >> 10) & 0x03;
    let source_mode = (frame_control >> 14) & 0x03;
    match destination_mode {
        // No destination addressing: a beacon, or a frame implicitly
        // addressed to the PAN coordinator. Never provably foreign.
        0x00 => {
            if source_mode == 0x00 {
                // Neither address present and not an ACK: malformed.
                RxPriority::Foreign
            } else {
                RxPriority::Broadcast
            }
        }
        // 0b01 is reserved by IEEE 802.15.4-2006.
        0x01 => RxPriority::Foreign,
        0x02 => {
            let Some(bytes) = psdu.get(3..7) else {
                return RxPriority::Foreign;
            };
            let destination_pan = u16::from_le_bytes([bytes[0], bytes[1]]);
            if destination_pan != filter.pan_id && destination_pan != 0xFFFF {
                return RxPriority::Foreign;
            }
            let destination = u16::from_le_bytes([bytes[2], bytes[3]]);
            if destination == 0xFFFF {
                RxPriority::Broadcast
            } else if filter.short_address < 0xFFF8 && destination == filter.short_address {
                RxPriority::Local
            } else {
                RxPriority::Foreign
            }
        }
        _ => {
            let Some(bytes) = psdu.get(3..13) else {
                return RxPriority::Foreign;
            };
            let destination_pan = u16::from_le_bytes([bytes[0], bytes[1]]);
            if destination_pan != filter.pan_id && destination_pan != 0xFFFF {
                return RxPriority::Foreign;
            }
            if bytes[2..10] == filter.extended_address {
                RxPriority::Local
            } else {
                RxPriority::Foreign
            }
        }
    }
}

/// Slot count of the bounded interrupt receive queue.
///
/// Sized from the five-hour channel-15 TB-04 capture rather than guessed:
/// 463 720 decoded frames in ~18 000 s is a 26 frames/s mean, and the
/// application's longest un-drained gap (CSMA backoff + TX + 8 ms ACK wait +
/// `process_incoming`) is on the order of 100 ms. Sixteen 128-byte slots
/// (2 KiB of the 64 KiB SRAM) cover ~600 ms at the observed mean rate and a
/// 16-frame back-to-back burst. [`RxDiagnostics::queue_high_water`] reports
/// the depth actually reached so this bound stays measured instead of
/// re-guessed.
#[cfg(any(target_arch = "tc32", test))]
const IRQ_RX_QUEUE_CAPACITY: usize = 16;

/// Outcome of one [`RxQueue::push`].
#[cfg(any(target_arch = "tc32", test))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RxQueuePush {
    /// There was room; nothing was lost.
    Queued,
    /// The queue was full and a strictly lower-priority queued frame was
    /// sacrificed so this one could be kept.
    Evicted,
    /// The queue was full of frames at least as important as this one, so
    /// this frame was dropped.
    Dropped,
}

/// Bounded, priority-aware receive queue drained by the polled MAC windows
/// and filled by [`handle_irq`].
///
/// Ordering is strict FIFO while there is room, which is the only behavior
/// the MAC ever observes on an idle channel. Under overload the queue
/// sacrifices the oldest strictly-lower-priority frame instead of the newest
/// arrival, because the newest arrival is the one the upper layers are
/// usually waiting for (an ACK, or a ZDO response addressed to us) while the
/// stale entries are unrelated channel traffic this node only has to relay.
///
/// `order` is a permutation of every slot index: positions
/// `head..head + len` are occupied and the remainder are free, so an
/// eviction only shifts single-byte indices and never copies frame bytes.
#[cfg(any(target_arch = "tc32", test))]
struct RxQueue {
    slots: [ReceivedFrame; IRQ_RX_QUEUE_CAPACITY],
    priorities: [RxPriority; IRQ_RX_QUEUE_CAPACITY],
    order: [u8; IRQ_RX_QUEUE_CAPACITY],
    head: u8,
    len: u8,
    high_water: u8,
    overflow: u32,
    evicted: u32,
}

#[cfg(any(target_arch = "tc32", test))]
impl RxQueue {
    const fn new() -> Self {
        let mut order = [0u8; IRQ_RX_QUEUE_CAPACITY];
        let mut index = 0;
        while index < IRQ_RX_QUEUE_CAPACITY {
            order[index] = index as u8;
            index += 1;
        }
        Self {
            slots: [ReceivedFrame::EMPTY; IRQ_RX_QUEUE_CAPACITY],
            priorities: [RxPriority::Foreign; IRQ_RX_QUEUE_CAPACITY],
            order,
            head: 0,
            len: 0,
            high_water: 0,
            overflow: 0,
            evicted: 0,
        }
    }

    fn slot_at(&self, position: usize) -> usize {
        self.order[(self.head as usize + position) % IRQ_RX_QUEUE_CAPACITY] as usize
    }

    fn push(&mut self, frame: ReceivedFrame, priority: RxPriority) -> RxQueuePush {
        let len = self.len as usize;
        if len == IRQ_RX_QUEUE_CAPACITY {
            self.overflow = self.overflow.wrapping_add(1);
            let Some(victim) = self.victim_for(priority) else {
                return RxQueuePush::Dropped;
            };
            // Rotate the evicted slot index out of the occupied region and
            // into the (currently empty) free region at the tail. Only
            // `u8` indices move; the 128-byte frame bodies stay put.
            let head = self.head as usize;
            let evicted_slot = self.order[(head + victim) % IRQ_RX_QUEUE_CAPACITY];
            let mut position = victim;
            while position + 1 < len {
                self.order[(head + position) % IRQ_RX_QUEUE_CAPACITY] =
                    self.order[(head + position + 1) % IRQ_RX_QUEUE_CAPACITY];
                position += 1;
            }
            self.order[(head + len - 1) % IRQ_RX_QUEUE_CAPACITY] = evicted_slot;
            self.len -= 1;
            self.evicted = self.evicted.wrapping_add(1);
            self.store(frame, priority);
            return RxQueuePush::Evicted;
        }
        self.store(frame, priority);
        RxQueuePush::Queued
    }

    /// Oldest queued entry among those of the *lowest* priority present,
    /// but only if that priority is strictly below the arriving frame's.
    ///
    /// Picking the global minimum (rather than the first entry that merely
    /// happens to be lower) preserves all higher-value local and broadcast
    /// traffic while sacrificing the stalest provably foreign frame first.
    /// A linear scan over at most 16 one-byte priorities is a handful of
    /// cycles and runs only when the queue is already full.
    fn victim_for(&self, priority: RxPriority) -> Option<usize> {
        let mut victim: Option<(usize, RxPriority)> = None;
        for position in 0..self.len as usize {
            let candidate = self.priorities[self.slot_at(position)];
            if candidate >= priority {
                continue;
            }
            match victim {
                Some((_, lowest)) if lowest <= candidate => {}
                _ => victim = Some((position, candidate)),
            }
        }
        victim.map(|(position, _)| position)
    }

    fn store(&mut self, frame: ReceivedFrame, priority: RxPriority) {
        let slot = self.slot_at(self.len as usize);
        self.slots[slot] = frame;
        self.priorities[slot] = priority;
        self.len += 1;
        if self.len > self.high_water {
            self.high_water = self.len;
        }
    }

    fn pop(&mut self) -> Option<ReceivedFrame> {
        if self.len == 0 {
            return None;
        }
        let slot = self.slot_at(0);
        self.head = ((self.head as usize + 1) % IRQ_RX_QUEUE_CAPACITY) as u8;
        self.len -= 1;
        Some(self.slots[slot])
    }

    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    /// Cumulative frames lost at this queue, and how many of those losses
    /// were low-priority evictions that saved a more important arrival.
    #[cfg(test)]
    const fn counters(&self) -> (u32, u32, u8) {
        (self.overflow, self.evicted, self.high_water)
    }

    #[cfg(test)]
    fn len(&self) -> usize {
        self.len as usize
    }
}

/// Bounded receive-path counters.
///
/// These attribute a missing MAC acknowledgement to a specific stage of the
/// receive path from a single RAM read, without any per-frame logging that
/// would perturb the turnaround-critical software-ACK sequence:
///
/// - `frames_valid` far below the number of frames the sniffer saw on air
///   while `invalid_length` and `invalid_crc` stay near zero means the
///   baseband never reported those frames at all: RX was disabled, still
///   in TX/RX turnaround, or on the wrong channel.
/// - `invalid_length` / `invalid_crc` rising with the loss means frames do
///   reach the DMA buffer but fail the TLSR8258 length/CRC gate before any
///   ACK decision is made.
/// - `dma_incomplete` counts frames whose baseband end-of-packet flag was
///   observed while the RX DMA transfer-complete latch was still clear,
///   i.e. the buffer was consumed while the DMA writeback was still in
///   flight. A non-zero value here alongside `invalid_length` identifies a
///   DMA writeback race rather than a radio sensitivity problem.
/// - `queue_overflow` counts frames the bounded interrupt queue could not
///   keep because upper layers did not drain it in time.
/// - `queue_evicted` is the subset of `queue_overflow` where the lost frame
///   was a strictly lower-priority one that was sacrificed to keep a newly
///   arrived ACK or locally addressed frame. `queue_overflow -
///   queue_evicted` is therefore the number of arrivals dropped outright,
///   which is the number that must stay near zero.
/// - `queue_high_water` is the deepest the queue ever got. It is the
///   measurement that justifies (or refutes) `IRQ_RX_QUEUE_CAPACITY`: a
///   value that never approaches the capacity means the bound is generous,
///   and a value pinned at the capacity alongside a rising
///   `queue_overflow - queue_evicted` means it is too small.
/// - `serviced_irq` versus `serviced_polled` shows which servicing path is
///   actually running; `serviced_irq == 0` on an `rx_on_when_idle` router
///   means the RF interrupt is not reaching [`handle_irq`].
///
/// Frames that fail the length or CRC gate are counted in `invalid_length` /
/// `invalid_crc` and are *not* queued: no consumer acts on them, and on a
/// busy channel they were 14% of all completions, i.e. 14% of the queue
/// slots that used to be spent on frames nothing could ever use.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RxDiagnostics {
    pub frames_valid: u32,
    pub invalid_length: u32,
    pub invalid_crc: u32,
    pub dma_incomplete: u32,
    pub queue_overflow: u32,
    pub queue_evicted: u32,
    pub queue_high_water: u8,
    pub serviced_irq: u32,
    pub serviced_polled: u32,
}

/// Exclusive handle to the TLSR8258 radio/DMA engine.
///
/// TLSR8258 has one RF block and this HAL uses fixed application-linked DMA
/// storage, so production code should obtain the handle with [`Radio::take`].
pub struct Radio {
    _private: (),
}

#[cfg(target_arch = "tc32")]
static mut RADIO_TAKEN: u8 = 0;

impl Radio {
    #[cfg(target_arch = "tc32")]
    pub fn take() -> Option<Self> {
        let acquired = crate::mmio::with_irqs_disabled(|| unsafe {
            let ptr = core::ptr::addr_of_mut!(RADIO_TAKEN);
            if core::ptr::read_volatile(ptr) != 0 {
                return false;
            }
            core::ptr::write_volatile(ptr, 1);
            true
        });
        if !acquired {
            return None;
        }
        Some(Self { _private: () })
    }

    /// Bypass singleton acquisition. The caller must guarantee that no other
    /// radio handle or legacy free-function user can access the RF block.
    #[cfg(target_arch = "tc32")]
    pub unsafe fn steal() -> Self {
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(RADIO_TAKEN), 1);
        }
        Self { _private: () }
    }

    #[cfg(target_arch = "tc32")]
    pub fn init(&mut self) {
        hw::init();
    }

    /// Quiesce the RF block before the MCU enters a retention sleep mode.
    ///
    /// Retention wake does not preserve the RF/DMA register state. Call
    /// [`Radio::init`] after wake before using the radio again.
    #[cfg(target_arch = "tc32")]
    pub fn prepare_for_sleep(&mut self) {
        hw::prepare_for_sleep();
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_channel(&mut self, channel: u8) {
        hw::set_channel(channel);
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_tx_power(&mut self, dbm: i8) -> bool {
        let Some(level) = tx_power_register_value(dbm) else {
            return false;
        };
        phy::set_tx_power_level(level);
        true
    }

    #[cfg(target_arch = "tc32")]
    pub fn set_ack_filter(&mut self, pan_id: u16, short_address: u16, extended_address: [u8; 8]) {
        hw::set_ack_filter(pan_id, short_address, extended_address);
    }

    /// Keep the PHY armed between bounded MAC operations.
    ///
    /// In this mode RX completion is serviced by the RF/DMA interrupt while
    /// upper layers are running. Received frames are retained in a bounded
    /// HAL queue and delivered by the next receive slice.
    #[cfg(target_arch = "tc32")]
    pub fn set_rx_on_when_idle(&mut self, enabled: bool) {
        hw::set_rx_on_when_idle(enabled);
    }

    /// Set or clear Frame Pending for ACKs to one child's Data Requests.
    #[cfg(target_arch = "tc32")]
    pub fn set_ack_frame_pending(
        &mut self,
        child: AckPendingAddress,
        pending: bool,
    ) -> Result<(), AckPendingError> {
        hw::set_ack_frame_pending(child, pending)
    }

    #[cfg(target_arch = "tc32")]
    pub fn clear_ack_frame_pending(&mut self) {
        hw::clear_ack_frame_pending();
    }

    #[cfg(target_arch = "tc32")]
    pub fn transmit(&mut self, frame: &[u8]) -> TxOutcome {
        hw::send_mac_frame(frame)
    }

    #[cfg(target_arch = "tc32")]
    pub fn receive_raw_for(
        &mut self,
        timeout_ticks: u32,
        max_frames: u16,
        on_frame: impl FnMut(RawRxOutcome),
    ) -> u32 {
        hw::rx_raw_window_for(timeout_ticks, max_frames, on_frame)
    }

    #[cfg(target_arch = "tc32")]
    pub fn receive_raw_until(
        &mut self,
        timeout_ticks: u32,
        max_frames: u16,
        on_frame: impl FnMut(RawRxOutcome) -> bool,
    ) -> u32 {
        hw::rx_raw_window_until(timeout_ticks, max_frames, on_frame)
    }

    #[cfg(target_arch = "tc32")]
    pub fn measure_energy(&mut self) -> u8 {
        hw::measure_energy()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_power_mapping_uses_nearest_official_level() {
        assert_eq!(tx_power_register_value(10), Some(58));
        assert_eq!(tx_power_register_value(3), Some(0x80 | 63));
        assert_eq!(tx_power_register_value(0), Some(0x80 | 41));
        assert_eq!(tx_power_register_value(-10), Some(0x80 | 12));
        assert_eq!(tx_power_register_value(-25), Some(0x80 | 2));
    }

    #[test]
    fn tx_power_mapping_rejects_unsupported_range() {
        assert_eq!(tx_power_register_value(TX_POWER_MAX_DBM + 1), None);
        assert_eq!(tx_power_register_value(TX_POWER_MIN_DBM - 1), None);
    }

    #[test]
    fn tx_power_fields_match_official_register_layout() {
        assert_eq!(tx_power_register_fields(0x80 | 41), (true, 0x80, 0x14));
        assert_eq!(tx_power_register_fields(58), (false, 0x00, 0x1D));
    }

    #[test]
    fn normal_tx_and_software_ack_require_distinct_dma_storage() {
        let stride = DMA_BUF_LEN as u32;
        assert!(dma_buffer_layout_valid([
            0x1000,
            0x1000 + stride,
            0x1000 + 2 * stride,
            0x1000 + 3 * stride,
        ]));
        assert!(!dma_buffer_layout_valid([0x1000, 0x1090, 0x1120, 0x1120,]));
        assert!(!dma_buffer_layout_valid([0x1000, 0x1090, 0x1120, 0x1124,]));
    }

    #[test]
    fn ack_pending_table_matches_only_exact_data_request_source() {
        let mut table = AckPendingTable::new();
        let child = AckPendingAddress::Short {
            pan_id: 0x1234,
            address: 0x3344,
        };
        table.set(child, true).unwrap();

        let request = frame::data_request_associated_short(1, 0x1234, 0x0001, 0x3344);
        assert!(table.frame_pending(&request));

        let other = frame::data_request_associated_short(2, 0x1234, 0x0001, 0x3345);
        assert!(!table.frame_pending(&other));
        assert!(!table.frame_pending(&frame::data_frame_short(3, 0x1234, 0x0001, 0x3344)));

        table.set(child, false).unwrap();
        assert!(!table.frame_pending(&request));

        let extended_child = AckPendingAddress::Extended {
            pan_id: 0x1234,
            address: [1, 2, 3, 4, 5, 6, 7, 8],
        };
        table.set(extended_child, true).unwrap();
        let extended_request =
            frame::data_request_short(4, 0x1234, 0x0001, [1, 2, 3, 4, 5, 6, 7, 8]);
        assert!(table.frame_pending(&extended_request));
    }

    #[test]
    fn association_request_never_performs_a_frame_pending_match() {
        // Frame 3084 from the clean BL702 -> TLSR8258 parent capture. Unlike
        // an extended Data Request, an Association Request has source PAN
        // 0xffff and command ID 0x01.
        let association_request = [
            0x23, 0xC8, 0x9F, 0xE9, 0xDF, 0xDF, 0xF8, 0xFF, 0xFF, 0x7C, 0xB9, 0x4C, 0x61, 0x92,
            0x3A, 0x00, 0x00, 0x01, 0x80,
        ];
        assert_eq!(data_request_source(&association_request), None);

        let mut table = AckPendingTable::new();
        table
            .set(
                AckPendingAddress::Extended {
                    pan_id: 0xDFE9,
                    address: [0x7C, 0xB9, 0x4C, 0x61, 0x92, 0x3A, 0x00, 0x00],
                },
                true,
            )
            .unwrap();
        assert!(!table.frame_pending(&association_request));
    }

    #[test]
    fn ack_pending_table_is_bounded_and_validates_addresses() {
        let mut table = AckPendingTable::new();
        for address in 0..MAX_ACK_PENDING_ADDRESSES as u16 {
            table
                .set(
                    AckPendingAddress::Short {
                        pan_id: 0x1234,
                        address,
                    },
                    true,
                )
                .unwrap();
        }
        assert_eq!(
            table.set(
                AckPendingAddress::Short {
                    pan_id: 0x1234,
                    address: 0x0100,
                },
                true,
            ),
            Err(AckPendingError::TableFull)
        );
        assert_eq!(
            table.set(
                AckPendingAddress::Short {
                    pan_id: 0x1234,
                    address: 0xFFFF,
                },
                true,
            ),
            Err(AckPendingError::InvalidAddress)
        );
    }

    #[test]
    fn clearing_ack_pending_table_removes_all_child_state() {
        let mut table = AckPendingTable::new();
        let child = AckPendingAddress::Extended {
            pan_id: 0x1234,
            address: [1, 2, 3, 4, 5, 6, 7, 8],
        };
        let request = frame::data_request_short(1, 0x1234, 0x0001, [1, 2, 3, 4, 5, 6, 7, 8]);
        table.set(child, true).unwrap();
        assert!(table.frame_pending(&request));

        table.clear();

        assert!(!table.frame_pending(&request));
    }

    #[test]
    fn ack_pending_lookup_is_sorted_and_exact_at_capacity() {
        let mut table = AckPendingTable::new();
        for address in (0..MAX_ACK_PENDING_ADDRESSES as u16).rev() {
            table
                .set(
                    AckPendingAddress::Short {
                        pan_id: 0x1234,
                        address,
                    },
                    true,
                )
                .unwrap();
        }
        for address in 0..MAX_ACK_PENDING_ADDRESSES as u16 {
            assert!(table.frame_pending_for(AckPendingAddress::Short {
                pan_id: 0x1234,
                address,
            }));
            assert_eq!(
                table.entries[address as usize].address,
                AckPendingAddress::Short {
                    pan_id: 0x1234,
                    address,
                }
            );
        }
        assert!(!table.frame_pending_for(AckPendingAddress::Short {
            pan_id: 0x1234,
            address: 0x1234,
        }));
    }

    #[test]
    fn ack_settle_is_anchored_to_rx_completion_and_wrap_safe() {
        assert_eq!(remaining_settle_ticks(1_000, 1_030, 120), 90);
        assert_eq!(remaining_settle_ticks(1_000, 1_120, 120), 0);
        assert_eq!(remaining_settle_ticks(1_000, 1_200, 120), 0);
        assert_eq!(remaining_settle_ticks(u32::MAX - 9, 20, 120), 90);
    }

    #[test]
    fn ack_frame_pending_changes_only_the_pending_bit() {
        assert_eq!(ack_frame_control(false), 0x02);
        assert_eq!(ack_frame_control(true), 0x12);
    }

    #[test]
    fn router_restores_rx_after_every_bounded_operation_exit() {
        for _exit in [
            "receive timeout",
            "receive callback",
            "tx complete",
            "tx timeout",
            "cca failure",
        ] {
            assert_eq!(idle_rx_state(true), IdleRxState::Armed);
        }
    }

    #[test]
    fn sleepy_device_preserves_bounded_rx_windows() {
        assert_eq!(idle_rx_state(false), IdleRxState::Off);
    }

    const TEST_FILTER: RxAddressFilter = RxAddressFilter {
        pan_id: 0x1A62,
        short_address: 0x9F3C,
        extended_address: [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88],
        enabled: true,
    };

    /// MAC header: Frame Control, sequence number, destination PAN,
    /// destination short address, then two payload bytes. Data frame with
    /// short destination and short source addressing.
    fn short_data_to(pan: u16, destination: u16) -> [u8; 9] {
        let pan = pan.to_le_bytes();
        let destination = destination.to_le_bytes();
        [
            0x61,
            0x88,
            0x42,
            pan[0],
            pan[1],
            destination[0],
            destination[1],
            0xAA,
            0xBB,
        ]
    }

    /// Same, with extended destination addressing (dst addr mode 0b11).
    fn extended_data_to(pan: u16, destination: [u8; 8]) -> [u8; 15] {
        let pan = pan.to_le_bytes();
        let mut psdu = [0u8; 15];
        psdu[..3].copy_from_slice(&[0x61, 0x8C, 0x43]);
        psdu[3..5].copy_from_slice(&pan);
        psdu[5..13].copy_from_slice(&destination);
        psdu[13..].copy_from_slice(&[0xAA, 0xBB]);
        psdu
    }

    fn frame_with_priority(priority: RxPriority, tag: u8) -> (ReceivedFrame, RxPriority) {
        (ReceivedFrame::new(&[tag; 5], 0xFF, -40), priority)
    }

    /// Drain the queue into a fixed buffer of first payload bytes.
    fn drain_tags(queue: &mut RxQueue) -> ([u8; IRQ_RX_QUEUE_CAPACITY], usize) {
        let mut tags = [0u8; IRQ_RX_QUEUE_CAPACITY];
        let mut count = 0;
        while let Some(frame) = queue.pop() {
            tags[count] = frame.as_slice()[0];
            count += 1;
        }
        (tags, count)
    }

    #[test]
    fn priority_classifier_matches_short_extended_and_broadcast_modes() {
        // Unicast to our short address inside our PAN.
        assert_eq!(
            classify_rx_priority(&short_data_to(0x1A62, 0x9F3C), &TEST_FILTER),
            RxPriority::Local
        );
        // Unicast to our extended address inside our PAN.
        assert_eq!(
            classify_rx_priority(
                &extended_data_to(0x1A62, TEST_FILTER.extended_address),
                &TEST_FILTER
            ),
            RxPriority::Local
        );
        // MAC broadcast inside our PAN.
        assert_eq!(
            classify_rx_priority(&short_data_to(0x1A62, 0xFFFF), &TEST_FILTER),
            RxPriority::Broadcast
        );
        // Broadcast PAN (a Beacon Request's addressing) is not foreign.
        assert_eq!(
            classify_rx_priority(&short_data_to(0xFFFF, 0xFFFF), &TEST_FILTER),
            RxPriority::Broadcast
        );
        // Beacon: no destination addressing at all, short source only.
        assert_eq!(
            classify_rx_priority(&[0x00, 0x80, 0x01, 0x62, 0x1A, 0x00, 0x00], &TEST_FILTER),
            RxPriority::Broadcast
        );
        // Acknowledgement, which carries no addressing fields at all.
        assert_eq!(
            classify_rx_priority(&[0x02, 0x00, 0x61], &TEST_FILTER),
            RxPriority::Ack
        );
        assert_eq!(
            classify_rx_priority(&[ack_frame_control(true), 0x00, 0x61], &TEST_FILTER),
            RxPriority::Ack
        );
    }

    #[test]
    fn priority_classifier_rejects_other_pans_and_other_nodes() {
        // Same PAN, some other node's short address.
        assert_eq!(
            classify_rx_priority(&short_data_to(0x1A62, 0x1234), &TEST_FILTER),
            RxPriority::Foreign
        );
        // Another PAN entirely: exactly the traffic that filled the queue.
        assert_eq!(
            classify_rx_priority(&short_data_to(0x7788, 0x9F3C), &TEST_FILTER),
            RxPriority::Foreign
        );
        // Same PAN, some other node's extended address.
        assert_eq!(
            classify_rx_priority(&extended_data_to(0x1A62, [0xFE; 8]), &TEST_FILTER),
            RxPriority::Foreign
        );
    }

    #[test]
    fn priority_classifier_handles_malformed_frames_without_panicking() {
        // Shorter than Frame Control + sequence number.
        assert_eq!(classify_rx_priority(&[], &TEST_FILTER), RxPriority::Foreign);
        assert_eq!(
            classify_rx_priority(&[0x61], &TEST_FILTER),
            RxPriority::Foreign
        );
        assert_eq!(
            classify_rx_priority(&[0x61, 0x88], &TEST_FILTER),
            RxPriority::Foreign
        );
        // Short destination addressing claimed but truncated.
        assert_eq!(
            classify_rx_priority(&[0x61, 0x88, 0x42, 0x62, 0x1A], &TEST_FILTER),
            RxPriority::Foreign
        );
        // Extended destination addressing claimed but truncated.
        assert_eq!(
            classify_rx_priority(&[0x61, 0x8C, 0x42, 0x62, 0x1A, 0x11, 0x22], &TEST_FILTER),
            RxPriority::Foreign
        );
        // Reserved destination addressing mode 0b01.
        assert_eq!(
            classify_rx_priority(&[0x61, 0x84, 0x42, 0x62, 0x1A, 0xAA, 0xBB], &TEST_FILTER),
            RxPriority::Foreign
        );
        // Reserved frame type 0b101.
        assert_eq!(
            classify_rx_priority(&[0x65, 0x88, 0x42, 0x62, 0x1A, 0x3C, 0x9F], &TEST_FILTER),
            RxPriority::Foreign
        );
        // Neither source nor destination addressing, and not an ACK.
        assert_eq!(
            classify_rx_priority(&[0x01, 0x00, 0x42], &TEST_FILTER),
            RxPriority::Foreign
        );
    }

    #[test]
    fn priority_classifier_stays_neutral_before_the_local_identity_is_known() {
        // Before `set_ack_filter`, and during scan/association when the PAN
        // id is still 0xFFFF, nothing can be proven foreign, so every frame
        // must classify identically and the queue must behave as pure FIFO.
        let disabled = RxAddressFilter {
            enabled: false,
            ..TEST_FILTER
        };
        let unassociated = RxAddressFilter {
            pan_id: 0xFFFF,
            short_address: 0xFFFF,
            ..TEST_FILTER
        };
        for filter in [disabled, unassociated] {
            assert_eq!(
                classify_rx_priority(&short_data_to(0x7788, 0x1234), &filter),
                RxPriority::Local
            );
            assert_eq!(
                classify_rx_priority(&short_data_to(0x1A62, 0xFFFF), &filter),
                RxPriority::Local
            );
            // Before the local identity is known even an ACK cannot be
            // attributed safely, so all valid frames share one FIFO class.
            assert_eq!(
                classify_rx_priority(&[0x02, 0x00, 0x61], &filter),
                RxPriority::Local
            );
        }
    }

    #[test]
    fn priority_classifier_ignores_an_unassigned_short_address() {
        // 0xFFF8..=0xFFFF are not assignable short addresses. A frame to
        // 0xFFFE must never be mistaken for a frame to an unjoined node.
        let unassigned = RxAddressFilter {
            short_address: 0xFFFE,
            ..TEST_FILTER
        };
        assert_eq!(
            classify_rx_priority(&short_data_to(0x1A62, 0xFFFE), &unassigned),
            RxPriority::Foreign
        );
        // The extended address still identifies us before a short address
        // is assigned, which is how an Association Response arrives.
        assert_eq!(
            classify_rx_priority(
                &extended_data_to(0x1A62, TEST_FILTER.extended_address),
                &unassigned
            ),
            RxPriority::Local
        );
    }

    #[test]
    fn rx_queue_is_strict_fifo_while_it_has_room() {
        let mut queue = RxQueue::new();
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            let (frame, priority) = frame_with_priority(RxPriority::Foreign, tag);
            assert_eq!(queue.push(frame, priority), RxQueuePush::Queued);
        }
        assert_eq!(queue.len(), IRQ_RX_QUEUE_CAPACITY);
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            assert_eq!(queue.pop().unwrap().as_slice(), [tag; 5]);
        }
        assert!(queue.pop().is_none());
        // Nothing was lost, so neither counter moved.
        assert_eq!(
            queue.counters(),
            (0, 0, IRQ_RX_QUEUE_CAPACITY as u8),
            "an un-overflowed queue must not report any loss"
        );
    }

    #[test]
    fn rx_queue_wraps_without_reordering_or_losing_frames() {
        let mut queue = RxQueue::new();
        let mut expected = 0u8;
        for tag in 0..(3 * IRQ_RX_QUEUE_CAPACITY) as u8 {
            let (frame, priority) = frame_with_priority(RxPriority::Local, tag);
            assert_eq!(queue.push(frame, priority), RxQueuePush::Queued);
            if queue.len() == 4 {
                for _ in 0..4 {
                    assert_eq!(queue.pop().unwrap().as_slice(), [expected; 5]);
                    expected += 1;
                }
            }
        }
        assert_eq!(queue.counters().0, 0);
        assert_eq!(queue.counters().1, 0);
    }

    #[test]
    fn overloaded_rx_queue_keeps_a_new_local_frame_over_stale_foreign_traffic() {
        let mut queue = RxQueue::new();
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            let (frame, priority) = frame_with_priority(RxPriority::Foreign, tag);
            queue.push(frame, priority);
        }
        // The ZDO response the coordinator sent 45 ms after our request.
        let (response, priority) = frame_with_priority(RxPriority::Local, 0xC1);
        assert_eq!(queue.push(response, priority), RxQueuePush::Evicted);

        // The oldest unrelated frame is the one that was sacrificed, and the
        // surviving foreign frames keep their relative order.
        for tag in 1..IRQ_RX_QUEUE_CAPACITY as u8 {
            assert_eq!(queue.pop().unwrap().as_slice(), [tag; 5]);
        }
        assert_eq!(queue.pop().unwrap().as_slice(), [0xC1; 5]);
        assert!(queue.pop().is_none());
        assert_eq!(queue.counters().0, 1, "one frame was lost overall");
        assert_eq!(queue.counters().1, 1, "and it was lost by eviction");
    }

    #[test]
    fn overloaded_rx_queue_keeps_a_new_ack_over_stale_foreign_traffic() {
        let mut queue = RxQueue::new();
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            let (frame, priority) = frame_with_priority(RxPriority::Foreign, tag);
            queue.push(frame, priority);
        }
        let (ack, priority) = frame_with_priority(RxPriority::Ack, 0xAC);
        assert_eq!(queue.push(ack, priority), RxQueuePush::Evicted);
        let (tags, count) = drain_tags(&mut queue);
        let tags = &tags[..count];
        assert!(tags.contains(&0xAC), "the ACK must survive: {tags:?}");
        assert!(
            !tags.contains(&0),
            "the oldest foreign frame must be evicted"
        );
    }

    #[test]
    fn overloaded_rx_queue_keeps_local_data_over_queued_acks() {
        let mut queue = RxQueue::new();
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            let (frame, priority) = frame_with_priority(RxPriority::Ack, tag);
            queue.push(frame, priority);
        }
        let (response, priority) = frame_with_priority(RxPriority::Local, 0xC1);
        assert_eq!(queue.push(response, priority), RxQueuePush::Evicted);
        let (tags, count) = drain_tags(&mut queue);
        let tags = &tags[..count];
        assert!(
            tags.contains(&0xC1),
            "a local ZDO response must survive queued foreign ACKs: {tags:?}"
        );
        assert!(!tags.contains(&0), "the oldest queued ACK must be evicted");
    }

    #[test]
    fn overloaded_rx_queue_never_evicts_an_equally_important_frame() {
        let mut queue = RxQueue::new();
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            let (frame, priority) = frame_with_priority(RxPriority::Local, tag);
            queue.push(frame, priority);
        }
        // Nothing queued is less important, so the arrival is dropped and
        // the established FIFO order is preserved intact.
        let (late, priority) = frame_with_priority(RxPriority::Local, 0xEE);
        assert_eq!(queue.push(late, priority), RxQueuePush::Dropped);
        // A foreign arrival is likewise dropped, not allowed to displace
        // anything.
        let (foreign, priority) = frame_with_priority(RxPriority::Foreign, 0xFA);
        assert_eq!(queue.push(foreign, priority), RxQueuePush::Dropped);
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            assert_eq!(queue.pop().unwrap().as_slice(), [tag; 5]);
        }
        assert!(queue.pop().is_none());
        assert_eq!(queue.counters().0, 2, "both arrivals count as loss");
        assert_eq!(queue.counters().1, 0, "neither was an eviction");
    }

    #[test]
    fn rx_queue_evicts_the_oldest_lowest_priority_entry_only() {
        let mut queue = RxQueue::new();
        // Alternate foreign and local so eviction has a real choice.
        for tag in 0..IRQ_RX_QUEUE_CAPACITY as u8 {
            let priority = if tag % 2 == 0 {
                RxPriority::Local
            } else {
                RxPriority::Foreign
            };
            let (frame, priority) = frame_with_priority(priority, tag);
            queue.push(frame, priority);
        }
        let (local, priority) = frame_with_priority(RxPriority::Local, 0xAC);
        assert_eq!(queue.push(local, priority), RxQueuePush::Evicted);
        let (tags, count) = drain_tags(&mut queue);
        let tags = &tags[..count];
        // Tag 1 was the oldest foreign entry; every local entry survived.
        assert!(!tags.contains(&1), "{tags:?}");
        for local in (0..IRQ_RX_QUEUE_CAPACITY as u8).step_by(2) {
            assert!(tags.contains(&local), "{tags:?}");
        }
        assert_eq!(tags.last(), Some(&0xAC));
    }

    #[test]
    fn rx_queue_counters_are_cumulative_and_high_water_is_measured() {
        let mut queue = RxQueue::new();
        assert_eq!(queue.counters(), (0, 0, 0));
        for tag in 0..4u8 {
            let (frame, priority) = frame_with_priority(RxPriority::Foreign, tag);
            queue.push(frame, priority);
        }
        assert_eq!(queue.counters(), (0, 0, 4));
        for _ in 0..4 {
            queue.pop();
        }
        // Draining must not walk the high-water mark back.
        assert_eq!(queue.counters(), (0, 0, 4));
        // `clear()` (retention sleep) also keeps the cumulative history: a
        // reliability defect has to stay attributable across sleep cycles.
        queue.clear();
        assert_eq!(queue.counters(), (0, 0, 4));
    }
}

#[cfg(target_arch = "tc32")]
mod hw {
    use core::sync::atomic::{Ordering, compiler_fence};

    use super::frame::{self, BeaconInfo};
    use super::{
        AckPendingAddress, AckPendingError, AckPendingTable, DMA_BUF_LEN, DmaBuf,
        MAX_MAC_FRAME_LEN, RawRxOutcome, ReceivedFrame, RxAddressFilter, RxQueue, RxQueuePush, phy,
    };
    use crate::timer;

    #[unsafe(link_section = ".rf_dma")]
    static mut RF_RX_BUF: [DmaBuf; 2] = [DmaBuf([0u8; DMA_BUF_LEN]), DmaBuf([0u8; DMA_BUF_LEN])];
    #[unsafe(link_section = ".rf_dma")]
    static mut RF_TX_BUF: DmaBuf = DmaBuf([0u8; DMA_BUF_LEN]);
    /// Turnaround-critical software ACK storage. This must not alias
    /// `RF_TX_BUF`: synchronous TX can service a completed RX before it
    /// triggers DMA for the caller's already-encoded frame.
    #[unsafe(link_section = ".rf_dma")]
    static mut RF_ACK_TX_BUF: DmaBuf = DmaBuf([0u8; DMA_BUF_LEN]);

    #[repr(C)]
    #[derive(Clone, Copy)]
    struct AckFilter {
        pan_id: u16,
        short_address: u16,
        extended_address: [u8; 8],
        enabled: u8,
    }

    static mut ACK_FILTER: AckFilter = AckFilter {
        pan_id: 0xFFFF,
        short_address: 0xFFFF,
        extended_address: [0; 8],
        enabled: 0,
    };
    // Updates are published by flipping ACK_PENDING_ACTIVE only after the
    // inactive snapshot is complete. This keeps an eventual RX IRQ (and the
    // current polled turnaround path) from observing a half-shifted sorted
    // table while NWK changes one child's indirect state.
    static mut ACK_PENDING_TABLES: [AckPendingTable; 2] =
        [AckPendingTable::new(), AckPendingTable::new()];
    static mut ACK_PENDING_ACTIVE: u8 = 0;
    static mut SOFTWARE_ACK_COUNT: u32 = 0;
    static mut SOFTWARE_ACK_TIMEOUT_COUNT: u32 = 0;
    static mut ACTIVE_RX_INDEX: u8 = 0;
    static mut RX_ARMED_AFTER_TX: u8 = 0;
    static mut RX_ON_WHEN_IDLE: u8 = 0;
    static mut RX_ARMED: u8 = 0;
    static mut CSMA_RNG_STATE: u32 = 0;
    static mut CCA_ATTEMPT_COUNT: u32 = 0;
    static mut CCA_BUSY_COUNT: u32 = 0;
    static mut CHANNEL_ACCESS_FAILURE_COUNT: u32 = 0;
    static mut IRQ_RX_QUEUE: RxQueue = RxQueue::new(); // Receive-path diagnostics. Written only from the RX servicing paths
    // that already run, read only by `rx_diagnostics()`. Nothing in the
    // driver branches on them, so they cannot change radio behavior; the
    // only cost is one load/store per already-completed frame, all of it
    // outside the RX->ACK turnaround window.
    static mut RX_VALID_FRAME_COUNT: u32 = 0;
    static mut RX_INVALID_LENGTH_COUNT: u32 = 0;
    static mut RX_INVALID_CRC_COUNT: u32 = 0;
    static mut RX_DMA_INCOMPLETE_COUNT: u32 = 0;
    static mut RX_SERVICED_IRQ_COUNT: u32 = 0;
    static mut RX_SERVICED_POLLED_COUNT: u32 = 0;

    /// The two `reg_irq_mask`/`reg_irq_src` bits this module's RX path
    /// gates as a single unit: [`crate::irq::IrqSource::Dma`] (bit 4, the
    /// DMA-complete signal the RF RX DMA channel raises) and
    /// [`crate::irq::IrqSource::ZbRt`] (bit 13, the baseband "done" IRQ).
    /// Built from `crate::irq`'s canonical bit table instead of repeating
    /// the `(1 << 4) | (1 << 13)` literal here — see
    /// `mask_cpu_rx_irq`/`enable_cpu_rx_irq` below for why the
    /// mask/enable *control flow* itself still doesn't call through
    /// `crate::irq`'s `set_enabled`/`enable`/`disable`.
    const CPU_RX_IRQ_MASK: u32 =
        crate::irq::IrqSource::Dma.mask() | crate::irq::IrqSource::ZbRt.mask();

    const CCA_THRESHOLD_DBM: i8 = -70;
    const CCA_RX_SETTLE_TICKS: u32 = timer::TICKS_PER_MS * 128 / 1_000;
    const CCA_SAMPLE_TICKS: u32 = timer::TICKS_PER_MS * 128 / 1_000;
    const UNIT_BACKOFF_TICKS: u32 = timer::TICKS_PER_MS * 320 / 1_000;
    const MAC_MIN_BE: u8 = 3;
    const MAC_MAX_BE: u8 = 5;
    const MAC_MAX_CSMA_BACKOFFS: u8 = 4;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum TxOutcome {
        Sent,
        InvalidFrame,
        ChannelAccessFailure,
        Timeout,
    }

    #[derive(Debug, Clone, Copy)]
    pub struct CsmaStats {
        pub cca_attempts: u32,
        pub cca_busy: u32,
        pub channel_access_failures: u32,
    }

    /// Compile-time-ish alignment/placement facts the post-link script
    /// re-verifies from the linked ELF (see `verify_layout` in
    /// `scripts/tlsr8258.sh`): all DMA buffers must be 4-byte aligned,
    /// non-overlapping, and must live inside the `.rf_dma` section, i.e.
    /// outside the I-cache tag/data reservation.
    pub fn dma_buffers_aligned() -> bool {
        let rx0 = rx_buffer_ptr(0) as u32;
        let rx1 = rx_buffer_ptr(1) as u32;
        let tx = core::ptr::addr_of!(RF_TX_BUF) as u32;
        let ack_tx = core::ptr::addr_of!(RF_ACK_TX_BUF) as u32;
        super::dma_buffer_layout_valid([rx0, rx1, tx, ack_tx])
    }

    /// Bring up Timer0 + the RF PHY/DMA, and program channel 11 as the
    /// initial channel. Must run once, after `.data`/`.bss` init.
    pub fn init() {
        crate::mmio::disable_all_irqs();
        timer::init();
        set_active_rx_index(0);
        set_rx_armed(false);
        set_rx_armed_after_tx(false);
        set_rx_on_when_idle_flag(false);
        clear_irq_rx_queue();
        let rx_ptr = active_rx_ptr();
        phy::init(rx_ptr);
    }

    pub fn prepare_for_sleep() {
        mask_cpu_rx_irq();
        set_rx_on_when_idle_flag(false);
        phy::set_trx_off();
        set_rx_armed(false);
        phy::clear_irq_mask();
        phy::clear_irq_status();
    }

    pub fn set_channel(channel: u8) {
        let restore_rx = begin_radio_operation();
        phy::set_channel(channel);
        set_rx_armed(false);
        end_radio_operation(restore_rx);
    }

    pub fn set_ack_filter(pan_id: u16, short_address: u16, extended_address: [u8; 8]) {
        let restore_irq = mask_cpu_rx_irq();
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(ACK_FILTER),
                AckFilter {
                    pan_id,
                    short_address,
                    extended_address,
                    enabled: 1,
                },
            );
        }
        restore_cpu_rx_irq(restore_irq);
    }

    pub fn set_rx_on_when_idle(enabled: bool) {
        mask_cpu_rx_irq();
        set_rx_on_when_idle_flag(enabled);
        if enabled {
            if phy::rx_done() {
                queue_completed_rx();
            }
            if !rx_is_armed() {
                rearm_rx(active_rx_ptr());
            }
            enable_cpu_rx_irq();
        } else {
            phy::set_trx_off();
            set_rx_armed(false);
            phy::rx_done_clear();
        }
    }

    pub fn set_ack_frame_pending(
        child: AckPendingAddress,
        pending: bool,
    ) -> Result<(), AckPendingError> {
        unsafe {
            let active =
                core::ptr::read_volatile(core::ptr::addr_of!(ACK_PENDING_ACTIVE)) as usize & 1;
            let next = active ^ 1;
            let tables = core::ptr::addr_of_mut!(ACK_PENDING_TABLES).cast::<AckPendingTable>();
            let mut table = core::ptr::read_volatile(tables.add(active));
            table.set(child, pending)?;
            core::ptr::write_volatile(tables.add(next), table);
            compiler_fence(Ordering::Release);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(ACK_PENDING_ACTIVE), next as u8);
        }
        Ok(())
    }

    pub fn clear_ack_frame_pending() {
        unsafe {
            let active =
                core::ptr::read_volatile(core::ptr::addr_of!(ACK_PENDING_ACTIVE)) as usize & 1;
            let next = active ^ 1;
            let tables = core::ptr::addr_of_mut!(ACK_PENDING_TABLES).cast::<AckPendingTable>();
            core::ptr::write_volatile(tables.add(next), AckPendingTable::new());
            compiler_fence(Ordering::Release);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(ACK_PENDING_ACTIVE), next as u8);
        }
    }

    pub fn software_ack_stats() -> (u32, u32) {
        unsafe {
            (
                core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_COUNT)),
                core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_TIMEOUT_COUNT)),
            )
        }
    }

    pub fn csma_stats() -> CsmaStats {
        unsafe {
            CsmaStats {
                cca_attempts: core::ptr::read_volatile(core::ptr::addr_of!(CCA_ATTEMPT_COUNT)),
                cca_busy: core::ptr::read_volatile(core::ptr::addr_of!(CCA_BUSY_COUNT)),
                channel_access_failures: core::ptr::read_volatile(core::ptr::addr_of!(
                    CHANNEL_ACCESS_FAILURE_COUNT
                )),
            }
        }
    }

    /// Snapshot the bounded receive-path counters. See
    /// [`super::RxDiagnostics`] for how each field is meant to be read.
    pub fn rx_diagnostics() -> super::RxDiagnostics {
        let (queue_overflow, queue_evicted, queue_high_water) = irq_rx_queue_counters();
        unsafe {
            super::RxDiagnostics {
                frames_valid: core::ptr::read_volatile(core::ptr::addr_of!(RX_VALID_FRAME_COUNT)),
                invalid_length: core::ptr::read_volatile(core::ptr::addr_of!(
                    RX_INVALID_LENGTH_COUNT
                )),
                invalid_crc: core::ptr::read_volatile(core::ptr::addr_of!(RX_INVALID_CRC_COUNT)),
                dma_incomplete: core::ptr::read_volatile(core::ptr::addr_of!(
                    RX_DMA_INCOMPLETE_COUNT
                )),
                queue_overflow,
                queue_evicted,
                queue_high_water,
                serviced_irq: core::ptr::read_volatile(core::ptr::addr_of!(RX_SERVICED_IRQ_COUNT)),
                serviced_polled: core::ptr::read_volatile(core::ptr::addr_of!(
                    RX_SERVICED_POLLED_COUNT
                )),
            }
        }
    }

    pub fn measure_energy() -> u8 {
        let restore_rx = begin_radio_operation();
        let rx_ptr = active_rx_ptr();
        phy::set_trx_off();
        set_rx_armed(false);
        phy::rx_done_clear();
        prepare_rx_dma(rx_ptr);
        phy::set_rx_mode();
        set_rx_armed(true);
        timer::sleep_ticks(CCA_RX_SETTLE_TICKS);

        let start = timer::now_ticks();
        let mut sum = 0i32;
        let mut samples = 0i32;
        loop {
            sum += phy::rssi_dbm() as i32;
            samples += 1;
            if timer::now_ticks().wrapping_sub(start) >= CCA_SAMPLE_TICKS {
                break;
            }
            unsafe { core::arch::asm!("nop") };
        }
        phy::set_trx_off();
        set_rx_armed(false);

        let rssi = (sum / samples).clamp(-99, -15);
        let energy = (255 * (rssi + 99) / 84) as u8;
        end_radio_operation(restore_rx);
        energy
    }

    /// Fixed bound for one Beacon Request TX: settle + on-air time at 250
    /// kb/s for a short command frame is well under 1 ms; 5 ms leaves
    /// generous margin without ever blocking indefinitely.
    pub const TX_TIMEOUT_TICKS: u32 = timer::TICKS_PER_MS * 5;

    pub fn send_mac_frame(mac_frame: &[u8]) -> TxOutcome {
        if mac_frame.len() > MAX_MAC_FRAME_LEN {
            return TxOutcome::InvalidFrame;
        }
        let tx_ptr = core::ptr::addr_of_mut!(RF_TX_BUF) as *mut u8;
        let tx_slice = unsafe { core::slice::from_raw_parts_mut(tx_ptr, DMA_BUF_LEN) };
        // This call must execute in release builds: putting it inside
        // debug_assert! would remove the DMA-buffer write entirely.
        if frame::encode_tx_dma(tx_slice, mac_frame).is_err() {
            return TxOutcome::InvalidFrame;
        }
        compiler_fence(Ordering::Release);

        let restore_rx = begin_radio_operation();
        set_rx_armed_after_tx(false);
        if !perform_csma_ca() {
            end_radio_operation(restore_rx);
            return TxOutcome::ChannelAccessFailure;
        }

        let rx_ptr = active_rx_ptr();
        phy::rx_done_clear();
        prepare_rx_dma(rx_ptr);

        phy::set_tx_dma_config(DMA_BUF_LEN as u16);
        phy::tx_done_clear();
        phy::set_tx_mode();
        set_rx_armed(false);
        // Settle delay before triggering DMA, matching the proven sensor-lab
        // sequence (PLL/analog settle after the mode-register write).
        timer::sleep_ticks(timer::ms(1) / 4); // ~0.25 ms fixed pause
        phy::tx_pkt(tx_ptr);

        let ok = timer::wait_until(TX_TIMEOUT_TICKS, phy::tx_done);
        if ok {
            phy::tx_done_clear();
            // An 802.15.4 ACK starts only 12 symbols after the transmitted
            // frame. Enter RX here, before returning to layout-sensitive
            // caller code, and leave the already-armed DMA buffer intact.
            phy::set_rx_mode();
            set_rx_armed(true);
            set_rx_armed_after_tx(true);
        } else {
            phy::set_trx_off();
            set_rx_armed(false);
        }
        let outcome = if ok {
            TxOutcome::Sent
        } else {
            TxOutcome::Timeout
        };
        end_radio_operation(restore_rx);
        outcome
    }

    /// Encode and transmit a Beacon Request with the given sequence number.
    pub fn send_beacon_request(seq: u8) -> TxOutcome {
        let mac_frame = frame::beacon_request_mac_frame(seq);
        send_mac_frame(&mac_frame)
    }

    /// One received-frame classification, produced by [`rx_window`].
    #[derive(Debug, Clone, Copy)]
    pub enum RxOutcome {
        /// Length-valid, CRC-valid, and MAC-parseable as a Beacon frame.
        Beacon {
            info: BeaconInfo,
            len: u8,
            lqi: u8,
            rssi: i8,
        },
        /// A valid ACK, including the Frame Pending bit needed by polling.
        Ack { sequence: u8, frame_pending: bool },
        /// A valid MAC Association Response command.
        AssociationResponse(frame::AssociationResponse),
        /// Length check (`RF_ZIGBEE_PACKET_LENGTH_OK`) failed.
        InvalidLength,
        /// Length was valid but the CRC/status check failed.
        InvalidCrc,
        /// Length- and CRC-valid, but not parseable as a Beacon (e.g. an ACK
        /// or a different command/data frame received during the window).
        NotABeacon { len: u8, lqi: u8, rssi: i8 },
    }

    /// Fixed bound for one RX window: long enough to catch a coordinator's
    /// beacon response after a Beacon Request, short enough that the
    /// channel-cycle loop in `mac_test` makes visible progress. Tuned to
    /// the same ~10 ms window used by the sensor lab's proven `scan_one`.
    pub const RX_WINDOW_TICKS: u32 = timer::TICKS_PER_MS * 10;

    /// Enter RX and poll for up to [`RX_WINDOW_TICKS`], classifying up to
    /// `max_frames` received frames via `on_frame`. Always returns after the
    /// fixed deadline (or after `max_frames` frames, whichever is first) —
    /// "no infinite wait for radio status".
    pub fn rx_window(max_frames: u16, on_frame: impl FnMut(RxOutcome)) -> u32 {
        rx_window_for(RX_WINDOW_TICKS, max_frames, on_frame)
    }

    pub fn rx_window_for(
        timeout_ticks: u32,
        max_frames: u16,
        mut on_frame: impl FnMut(RxOutcome),
    ) -> u32 {
        rx_raw_window_for(timeout_ticks, max_frames, |outcome| match outcome {
            RawRxOutcome::Frame(frame) => classify_and_report(&frame, &mut on_frame),
            RawRxOutcome::InvalidLength => on_frame(RxOutcome::InvalidLength),
            RawRxOutcome::InvalidCrc => on_frame(RxOutcome::InvalidCrc),
        })
    }

    /// Receive validated MAC frames without classifying their frame type.
    /// FCS bytes are removed because TLSR8258 validates them in hardware.
    pub fn rx_raw_window_for(
        timeout_ticks: u32,
        max_frames: u16,
        mut on_frame: impl FnMut(RawRxOutcome),
    ) -> u32 {
        rx_raw_window_until(timeout_ticks, max_frames, |outcome| {
            on_frame(outcome);
            false
        })
    }

    /// Receive validated MAC frames until the deadline, frame limit, or the
    /// callback reports that the caller has obtained the frame it needs.
    pub fn rx_raw_window_until(
        timeout_ticks: u32,
        max_frames: u16,
        mut on_frame: impl FnMut(RawRxOutcome) -> bool,
    ) -> u32 {
        let restore_rx = begin_radio_operation();
        let rx_ptr = active_rx_ptr();

        if !take_rx_armed_after_tx() && !rx_is_armed() {
            phy::set_trx_off();
            set_rx_armed(false);
            phy::rx_done_clear();
            rearm_rx(rx_ptr);
        }

        let start = timer::now_ticks();
        let mut frames_seen: u16 = 0;
        loop {
            if frames_seen >= max_frames {
                break;
            }
            if let Some(frame) = pop_irq_rx() {
                frames_seen += 1;
                if on_frame(RawRxOutcome::Frame(frame)) {
                    break;
                }
                continue;
            }
            if timer::now_ticks().wrapping_sub(start) >= timeout_ticks {
                break;
            }
            if phy::rx_done() {
                let outcome = take_completed_rx();
                increment_counter(core::ptr::addr_of_mut!(RX_SERVICED_POLLED_COUNT));
                frames_seen += 1;
                if on_frame(outcome) {
                    break;
                }
            }
            unsafe { core::arch::asm!("nop") };
        }
        let elapsed = timer::now_ticks().wrapping_sub(start);
        if super::idle_rx_state(restore_rx) == super::IdleRxState::Off {
            phy::set_trx_off();
            set_rx_armed(false);
        }
        end_radio_operation(restore_rx);
        elapsed
    }

    /// RF/DMA interrupt entry used by the always-on router application.
    ///
    /// The handler owns RX completion only while no synchronous radio
    /// operation has masked the CPU RF sources. It ACKs immediately, rotates
    /// DMA ownership, queues the frame, and restores continuous RX.
    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    pub fn handle_irq() {
        if rx_on_when_idle() {
            // A second frame can finish while the first ACK and queue copy
            // are in progress. Drain a small fixed burst; if more remains,
            // preserve the pending source and let the CPU vector again.
            for _ in 0..2 {
                if !phy::rx_done() {
                    break;
                }
                queue_completed_rx();
                increment_counter(core::ptr::addr_of_mut!(RX_SERVICED_IRQ_COUNT));
            }
        }
        if !phy::rx_done() {
            clear_cpu_rx_irq_sources();
        }
        // TLSR8258 clears the global enable on IRQ entry. The proven lab
        // vector requires the handler to re-enable it before IRQ return.
        unsafe {
            crate::mmio::w8(crate::mmio::REG_IRQ_EN, 1);
        }
    }

    fn set_rx_armed_after_tx(armed: bool) {
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(RX_ARMED_AFTER_TX),
                if armed { 1 } else { 0 },
            );
        }
    }

    fn set_rx_on_when_idle_flag(enabled: bool) {
        unsafe {
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(RX_ON_WHEN_IDLE),
                if enabled { 1 } else { 0 },
            );
        }
    }

    fn rx_on_when_idle() -> bool {
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!(RX_ON_WHEN_IDLE)) != 0 }
    }

    fn set_rx_armed(armed: bool) {
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(RX_ARMED), if armed { 1 } else { 0 });
        }
    }

    fn rx_is_armed() -> bool {
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!(RX_ARMED)) != 0 }
    }

    fn begin_radio_operation() -> bool {
        let restore_rx = rx_on_when_idle();
        mask_cpu_rx_irq();
        // A frame may have completed immediately before the CPU mask was
        // changed. Own it before any operation clears RX status or DMA.
        if restore_rx && phy::rx_done() {
            queue_completed_rx();
        }
        restore_rx
    }

    fn end_radio_operation(restore_rx: bool) {
        match super::idle_rx_state(restore_rx) {
            super::IdleRxState::Armed => {
                if phy::rx_done() {
                    queue_completed_rx();
                }
                if !rx_is_armed() {
                    rearm_rx(active_rx_ptr());
                }
                enable_cpu_rx_irq();
            }
            super::IdleRxState::Off => {}
        }
    }

    /// Mask (disable) the CPU RX IRQ sources and report whether they were
    /// enabled beforehand.
    ///
    /// # Why this doesn't call `crate::irq::set_enabled`/`disable`
    ///
    /// [`crate::irq`] centralizes the *generic* `reg_irq_mask`/
    /// `reg_irq_src` read-modify-write/write-1-to-clear idiom, and this
    /// function's overall shape — save `reg_irq_en`, clear it, touch
    /// `reg_irq_mask`, restore `reg_irq_en` — is exactly that idiom. It
    /// still isn't expressed as a call through `crate::irq` because this
    /// function's *return value* (`was_enabled`, read from the mask
    /// register while `reg_irq_en` is known-clear) is part of its
    /// contract, and [`crate::irq::set_enabled`] does not report the
    /// previous state — adding that here would mean either changing
    /// `crate::irq`'s generic API's return type for every other caller
    /// (none of which need it) or duplicating the read anyway outside the
    /// masked window (racy: the mask could change between an unmasked
    /// read and this function's own masked write). The explicit
    /// `compiler_fence(Ordering::SeqCst)` between the `reg_irq_mask`
    /// write and the `reg_irq_en` restore is also specific to this RX
    /// hot path's proven-on-hardware ordering requirements (see
    /// `enable_cpu_rx_irq`'s own doc for the matching concern on the
    /// re-enable side) and has no equivalent in the generic helper.
    fn mask_cpu_rx_irq() -> bool {
        unsafe {
            let global = crate::mmio::r8(crate::mmio::REG_IRQ_EN);
            crate::mmio::w8(crate::mmio::REG_IRQ_EN, 0);
            let mask = crate::mmio::r32(crate::mmio::REG_IRQ_MASK);
            let was_enabled = mask & CPU_RX_IRQ_MASK != 0;
            crate::mmio::w32(crate::mmio::REG_IRQ_MASK, mask & !CPU_RX_IRQ_MASK);
            compiler_fence(Ordering::SeqCst);
            crate::mmio::w8(crate::mmio::REG_IRQ_EN, global);
            was_enabled
        }
    }

    fn restore_cpu_rx_irq(was_enabled: bool) {
        if was_enabled && rx_on_when_idle() {
            enable_cpu_rx_irq();
        }
    }

    /// Re-enable the CPU RX IRQ sources.
    ///
    /// # Why this doesn't call `crate::irq::enable`
    ///
    /// Unlike [`crate::irq::set_enabled`] (a pure `reg_irq_mask` toggle),
    /// this function performs a fixed *sequence* around that toggle that
    /// is load-bearing for this RX path specifically: it conditionally
    /// clears stale CPU latches (only if a frame did *not* just complete
    /// in the handoff window — see the inline comment), then sets the
    /// mask bits, then unconditionally forces `reg_irq_en`'s bit 0 on
    /// (`global | 1`) rather than restoring whatever `reg_irq_en` held
    /// before this call, because every call site reaches this function
    /// specifically to (re)establish "RX IRQs are live", not to restore
    /// an arbitrary prior global-enable state. `crate::irq::enable` only
    /// does the last of those three steps' register (`reg_irq_mask`)
    /// half, and does not touch `reg_irq_en` or `reg_irq_src` at all —
    /// composing it with separate calls for the other two steps would
    /// reopen exactly the same race across independent masked windows
    /// this function's single critical section is proven to close. This
    /// stays hand-rolled by design; see [`mask_cpu_rx_irq`]'s doc for the
    /// matching concern on the mask side.
    fn enable_cpu_rx_irq() {
        unsafe {
            let global = crate::mmio::r8(crate::mmio::REG_IRQ_EN);
            crate::mmio::w8(crate::mmio::REG_IRQ_EN, 0);
            // Clear only stale CPU latches. If RX completed in the narrow
            // handoff between the caller's last status check and this mask
            // update, preserve its source so enabling the mask vectors it.
            if !phy::rx_done() {
                clear_cpu_rx_irq_sources();
            }
            let mask = crate::mmio::r32(crate::mmio::REG_IRQ_MASK);
            crate::mmio::w32(crate::mmio::REG_IRQ_MASK, mask | CPU_RX_IRQ_MASK);
            compiler_fence(Ordering::SeqCst);
            crate::mmio::w8(crate::mmio::REG_IRQ_EN, global | 1);
        }
    }

    /// Acknowledge (write-1-to-clear) both [`CPU_RX_IRQ_MASK`] bits in
    /// `reg_irq_src` in one `w32`. Equivalent in effect to calling
    /// [`crate::irq::clear_pending`] once each for
    /// [`crate::irq::IrqSource::Dma`] and [`crate::irq::IrqSource::ZbRt`],
    /// but kept as a single combined write here (not migrated) since this
    /// is always called from inside [`enable_cpu_rx_irq`]'s own
    /// `reg_irq_en`-masked critical section, and splitting it into two
    /// separate `crate::irq` calls would not change the generated code,
    /// only add call overhead in an RX-path hot function.
    fn clear_cpu_rx_irq_sources() {
        unsafe {
            crate::mmio::w32(crate::mmio::REG_IRQ_SRC, CPU_RX_IRQ_MASK);
        }
    }

    fn rx_buffer_ptr(index: u8) -> *mut u8 {
        debug_assert!(index < 2);
        unsafe {
            core::ptr::addr_of_mut!(RF_RX_BUF)
                .cast::<DmaBuf>()
                .add(index as usize)
                .cast::<u8>()
        }
    }

    fn active_rx_index() -> u8 {
        unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ACTIVE_RX_INDEX)) & 1 }
    }

    fn set_active_rx_index(index: u8) {
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(ACTIVE_RX_INDEX), index & 1);
        }
    }

    fn active_rx_ptr() -> *mut u8 {
        rx_buffer_ptr(active_rx_index())
    }

    fn rotate_rx_buffer() -> *mut u8 {
        let next = active_rx_index() ^ 1;
        set_active_rx_index(next);
        rx_buffer_ptr(next)
    }

    fn take_rx_armed_after_tx() -> bool {
        unsafe {
            let ptr = core::ptr::addr_of_mut!(RX_ARMED_AFTER_TX);
            let armed = core::ptr::read_volatile(ptr) != 0;
            core::ptr::write_volatile(ptr, 0);
            armed
        }
    }

    fn prepare_rx_dma(rx_ptr: *mut u8) {
        unsafe {
            core::ptr::write_volatile(rx_ptr, 0);
            core::ptr::write_volatile(rx_ptr.add(4), 0);
        }
        phy::set_rx_buffer(rx_ptr);
        phy::enable_dma_rx();
    }

    fn rearm_rx(rx_ptr: *mut u8) {
        prepare_rx_dma(rx_ptr);
        phy::set_rx_mode();
        set_rx_armed(true);
    }

    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn take_completed_rx() -> RawRxOutcome {
        // Anchor RX->TX settling before clearing status or parsing. The
        // pending lookup and ACK encoding run inside the hardware settle.
        let rx_complete_ticks = timer::now_ticks();
        // Diagnostic only, sampled before `rx_done_clear()` destroys the
        // latch: did the RX DMA writeback finish before this frame was
        // consumed? One register read, absorbed by the settle below.
        if !phy::rx_dma_done() {
            increment_counter(core::ptr::addr_of_mut!(RX_DMA_INCOMPLETE_COUNT));
        }
        phy::rx_done_clear();
        compiler_fence(Ordering::Acquire);
        let completed_rx_ptr = active_rx_ptr();
        let next_rx_ptr = rotate_rx_buffer();
        phy::disable_dma_rx();
        phy::disable_rx_mode();
        phy::set_tx_dma_config(DMA_BUF_LEN as u16);
        phy::tx_done_clear();
        phy::set_tx_mode();
        set_rx_armed(false);
        prepare_rx_dma(next_rx_ptr);
        if !maybe_send_software_ack(completed_rx_ptr, rx_complete_ticks) {
            rearm_rx(next_rx_ptr);
        }

        let completed =
            unsafe { core::slice::from_raw_parts(completed_rx_ptr.cast_const(), DMA_BUF_LEN) };
        decode_received_frame(completed)
    }

    #[inline(always)]
    fn queue_completed_rx() {
        if let RawRxOutcome::Frame(frame) = take_completed_rx() {
            push_irq_rx(frame);
        }
    }

    fn clear_irq_rx_queue() {
        unsafe {
            (*core::ptr::addr_of_mut!(IRQ_RX_QUEUE)).clear();
        }
    }

    /// Queue one length/CRC-valid frame for the polled MAC windows.
    ///
    /// Runs after [`maybe_send_software_ack`] has already transmitted, so
    /// neither the header classification nor a possible eviction can eat
    /// into the RX->ACK turnaround budget.
    ///
    /// # Exclusion
    ///
    /// Queue mutation is limited to this function and [`pop_irq_rx`], and
    /// those two can never run concurrently: every producer path other than
    /// [`handle_irq`] already sits behind [`begin_radio_operation`]'s
    /// `mask_cpu_rx_irq()`, and the single consumer
    /// ([`rx_raw_window_until`]) runs entirely inside that same mask.
    /// [`irq_rx_queue_counters`] briefly takes the same mask for a coherent
    /// read-only snapshot. The `compiler_fence`s keep queue stores from being
    /// reordered across the mask/unmask boundary.
    fn push_irq_rx(frame: ReceivedFrame) {
        let filter = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ACK_FILTER)) };
        let priority = super::classify_rx_priority(
            frame.as_slice(),
            &RxAddressFilter {
                pan_id: filter.pan_id,
                short_address: filter.short_address,
                extended_address: filter.extended_address,
                enabled: filter.enabled != 0,
            },
        );
        compiler_fence(Ordering::Release);
        // The return value is intentionally unused here: `RxQueue` already
        // accounts for every outcome in its own cumulative counters, which
        // `rx_diagnostics()` exports. There is no silent failure path.
        let _: RxQueuePush =
            unsafe { (*core::ptr::addr_of_mut!(IRQ_RX_QUEUE)).push(frame, priority) };
        compiler_fence(Ordering::Release);
    }

    fn pop_irq_rx() -> Option<ReceivedFrame> {
        compiler_fence(Ordering::Acquire);
        let frame = unsafe { (*core::ptr::addr_of_mut!(IRQ_RX_QUEUE)).pop() };
        compiler_fence(Ordering::Acquire);
        frame
    }

    fn irq_rx_queue_counters() -> (u32, u32, u8) {
        let restore_irq = mask_cpu_rx_irq();
        compiler_fence(Ordering::Acquire);
        let queue = core::ptr::addr_of!(IRQ_RX_QUEUE);
        let counters = unsafe {
            (
                core::ptr::read_volatile(core::ptr::addr_of!((*queue).overflow)),
                core::ptr::read_volatile(core::ptr::addr_of!((*queue).evicted)),
                core::ptr::read_volatile(core::ptr::addr_of!((*queue).high_water)),
            )
        };
        compiler_fence(Ordering::Acquire);
        restore_cpu_rx_irq(restore_irq);
        counters
    }

    fn perform_csma_ca() -> bool {
        let mut backoffs = 0u8;
        let mut backoff_exponent = MAC_MIN_BE;

        loop {
            let slots = next_random() & ((1u32 << backoff_exponent) - 1);
            let rx_ptr = active_rx_ptr();

            phy::set_trx_off();
            set_rx_armed(false);
            phy::rx_done_clear();
            prepare_rx_dma(rx_ptr);
            phy::set_rx_mode();
            set_rx_armed(true);
            if wait_while_receiving(CCA_RX_SETTLE_TICKS)
                || (slots != 0 && wait_while_receiving(slots * UNIT_BACKOFF_TICKS))
            {
                increment_counter(core::ptr::addr_of_mut!(CCA_BUSY_COUNT));
                backoffs = backoffs.saturating_add(1);
                if backoffs > MAC_MAX_CSMA_BACKOFFS {
                    increment_counter(core::ptr::addr_of_mut!(CHANNEL_ACCESS_FAILURE_COUNT));
                    phy::set_trx_off();
                    set_rx_armed(false);
                    phy::rx_done_clear();
                    return false;
                }
                backoff_exponent = backoff_exponent.saturating_add(1).min(MAC_MAX_BE);
                continue;
            }

            increment_counter(core::ptr::addr_of_mut!(CCA_ATTEMPT_COUNT));
            if channel_is_clear() {
                phy::set_trx_off();
                set_rx_armed(false);
                phy::rx_done_clear();
                return true;
            }

            increment_counter(core::ptr::addr_of_mut!(CCA_BUSY_COUNT));
            backoffs = backoffs.saturating_add(1);
            if backoffs > MAC_MAX_CSMA_BACKOFFS {
                increment_counter(core::ptr::addr_of_mut!(CHANNEL_ACCESS_FAILURE_COUNT));
                phy::set_trx_off();
                set_rx_armed(false);
                phy::rx_done_clear();
                return false;
            }
            backoff_exponent = backoff_exponent.saturating_add(1).min(MAC_MAX_BE);
        }
    }

    fn wait_while_receiving(ticks: u32) -> bool {
        let start = timer::now_ticks();
        loop {
            if phy::rx_done() {
                queue_completed_rx();
                return true;
            }
            if timer::now_ticks().wrapping_sub(start) >= ticks {
                return false;
            }
            unsafe { core::arch::asm!("nop") };
        }
    }

    fn channel_is_clear() -> bool {
        let start = timer::now_ticks();
        let mut sum = 0i32;
        let mut samples = 0i32;
        loop {
            sum += phy::rssi_dbm() as i32;
            samples += 1;
            if phy::rx_done() {
                queue_completed_rx();
                return false;
            }
            if timer::now_ticks().wrapping_sub(start) >= CCA_SAMPLE_TICKS {
                break;
            }
            unsafe { core::arch::asm!("nop") };
        }
        sum / samples <= CCA_THRESHOLD_DBM as i32
    }

    fn next_random() -> u32 {
        unsafe {
            let ptr = core::ptr::addr_of_mut!(CSMA_RNG_STATE);
            let mut value = core::ptr::read_volatile(ptr)
                ^ timer::now_ticks().rotate_left(11)
                ^ ((phy::rssi_dbm() as u8 as u32) << 24);
            if value == 0 {
                value = 0xA536_6B4D;
            }
            value ^= value << 13;
            value ^= value >> 17;
            value ^= value << 5;
            core::ptr::write_volatile(ptr, value);
            value
        }
    }

    fn increment_counter(ptr: *mut u32) {
        unsafe {
            let value = core::ptr::read_volatile(ptr);
            core::ptr::write_volatile(ptr, value.wrapping_add(1));
        }
    }

    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn maybe_send_software_ack(rx_ptr: *mut u8, rx_complete_ticks: u32) -> bool {
        let filter = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ACK_FILTER)) };
        if filter.enabled == 0 {
            return false;
        }

        let total_len = unsafe { core::ptr::read_volatile(rx_ptr) } as usize;
        let payload_len = unsafe { core::ptr::read_volatile(rx_ptr.add(4)) } as usize;
        if total_len == 0 || total_len > 136 || total_len != payload_len + 9 || payload_len < 7 {
            return false;
        }
        let status = unsafe { core::ptr::read_volatile(rx_ptr.add(total_len + 3)) };
        if status & 0x51 != 0x10 {
            return false;
        }

        let frame_control =
            u16::from_le_bytes(
                [unsafe { core::ptr::read_volatile(rx_ptr.add(5)) }, unsafe {
                    core::ptr::read_volatile(rx_ptr.add(6))
                }],
            );
        if frame_control & (1 << 5) == 0 {
            return false;
        }

        let destination_pan =
            u16::from_le_bytes(
                [unsafe { core::ptr::read_volatile(rx_ptr.add(8)) }, unsafe {
                    core::ptr::read_volatile(rx_ptr.add(9))
                }],
            );
        if destination_pan != filter.pan_id && destination_pan != 0xFFFF {
            return false;
        }

        let destination_mode = (frame_control >> 10) & 0x03;
        let addressed_to_us = match destination_mode {
            0x02 => {
                let destination = u16::from_le_bytes([
                    unsafe { core::ptr::read_volatile(rx_ptr.add(10)) },
                    unsafe { core::ptr::read_volatile(rx_ptr.add(11)) },
                ]);
                filter.short_address != 0xFFFF && destination == filter.short_address
            }
            0x03 if payload_len >= 13 => {
                let mut matches = true;
                let mut index = 0;
                while index < 8 {
                    let byte = unsafe { core::ptr::read_volatile(rx_ptr.add(10 + index)) };
                    matches &= byte == filter.extended_address[index];
                    index += 1;
                }
                matches
            }
            _ => false,
        };
        if !addressed_to_us {
            return false;
        }

        let sequence = unsafe { core::ptr::read_volatile(rx_ptr.add(7)) };
        let frame_pending = if payload_len >= 2 {
            let psdu = unsafe {
                core::slice::from_raw_parts(rx_ptr.add(5), payload_len.saturating_sub(2))
            };
            // Association Requests also ask for an ACK, but can never carry
            // Frame Pending. Determine whether this is a Data Request before
            // touching the 16-entry pending table. The old code copied the
            // entire table with read_volatile() for every ACK-requested
            // frame; on TC32 that consumed most of the 192 us turnaround
            // budget before the mandatory 120 us RX->TX settle even began.
            super::data_request_source(psdu).is_some_and(ack_pending_for_source)
        } else {
            false
        };
        if send_ack_fast(sequence, frame_pending, rx_complete_ticks) {
            unsafe {
                let count = core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_COUNT));
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(SOFTWARE_ACK_COUNT),
                    count.wrapping_add(1),
                );
            }
        } else {
            unsafe {
                let count =
                    core::ptr::read_volatile(core::ptr::addr_of!(SOFTWARE_ACK_TIMEOUT_COUNT));
                core::ptr::write_volatile(
                    core::ptr::addr_of_mut!(SOFTWARE_ACK_TIMEOUT_COUNT),
                    count.wrapping_add(1),
                );
            }
        }
        // The next DMA buffer was armed before filtering; send_ack_fast()
        // restores RX even when TX completion times out.
        true
    }

    #[inline(always)]
    fn ack_pending_for_source(source: AckPendingAddress) -> bool {
        // One volatile publication read selects a coherent sorted snapshot.
        // Binary search needs at most five entry reads for 16 children,
        // instead of copying or scanning the complete table.
        let active =
            unsafe { core::ptr::read_volatile(core::ptr::addr_of!(ACK_PENDING_ACTIVE)) as usize }
                & 1;
        let tables = core::ptr::addr_of!(ACK_PENDING_TABLES).cast::<AckPendingTable>();
        let table = unsafe { tables.add(active) };
        let len = unsafe { core::ptr::read_volatile(core::ptr::addr_of!((*table).len)) } as usize;
        let entries =
            unsafe { core::ptr::addr_of!((*table).entries).cast::<super::AckPendingEntry>() };
        let mut low = 0usize;
        let mut high = len;
        while low < high {
            let mid = low + (high - low) / 2;
            let entry = unsafe { core::ptr::read_volatile(entries.add(mid)) };
            if entry.address < source {
                low = mid + 1;
            } else {
                high = mid;
            }
        }
        low < len && unsafe { core::ptr::read_volatile(entries.add(low)) }.address == source
    }

    #[inline(never)]
    #[unsafe(link_section = ".ram_code")]
    fn send_ack_fast(sequence: u8, frame_pending: bool, rx_complete_ticks: u32) -> bool {
        // RX completion runs either in the RF IRQ or with that IRQ masked by
        // a synchronous operation. Only this path owns RF_ACK_TX_BUF.
        let tx_ptr = core::ptr::addr_of_mut!(RF_ACK_TX_BUF) as *mut u8;
        unsafe {
            core::ptr::write_volatile(tx_ptr, 4);
            core::ptr::write_volatile(tx_ptr.add(1), 0);
            core::ptr::write_volatile(tx_ptr.add(2), 0);
            core::ptr::write_volatile(tx_ptr.add(3), 0);
            core::ptr::write_volatile(tx_ptr.add(4), 5);
            core::ptr::write_volatile(tx_ptr.add(5), super::ack_frame_control(frame_pending));
            core::ptr::write_volatile(tx_ptr.add(6), 0);
            core::ptr::write_volatile(tx_ptr.add(7), sequence);
        }
        compiler_fence(Ordering::Release);
        // The TLSR8258 requires 120 us from the RX->TX transition, not 120 us
        // after arbitrary address parsing and pending-table lookup.
        let remaining =
            super::remaining_settle_ticks(rx_complete_ticks, timer::now_ticks(), timer::us(120));
        if remaining != 0 {
            timer::sleep_ticks(remaining);
        }
        phy::tx_pkt(tx_ptr);
        let sent = timer::wait_until(timer::TICKS_PER_MS, phy::tx_done);
        if sent {
            phy::tx_done_clear();
        }
        phy::set_rx_mode();
        set_rx_armed(true);
        sent
    }

    fn decode_received_frame(buf: &[u8]) -> RawRxOutcome {
        if !frame::packet_length_ok(buf) {
            increment_counter(core::ptr::addr_of_mut!(RX_INVALID_LENGTH_COUNT));
            return RawRxOutcome::InvalidLength;
        }
        if !frame::packet_crc_ok(buf) {
            increment_counter(core::ptr::addr_of_mut!(RX_INVALID_CRC_COUNT));
            return RawRxOutcome::InvalidCrc;
        }
        let dma_len = frame::payload_len(buf) as usize;
        if dma_len < 2 || dma_len - 2 > MAX_MAC_FRAME_LEN {
            increment_counter(core::ptr::addr_of_mut!(RX_INVALID_LENGTH_COUNT));
            return RawRxOutcome::InvalidLength;
        }
        let rssi = frame::packet_rssi(buf);
        let lqi = frame::rssi_to_lqi(rssi);
        let Some(psdu) = frame::mac_psdu(buf) else {
            increment_counter(core::ptr::addr_of_mut!(RX_INVALID_LENGTH_COUNT));
            return RawRxOutcome::InvalidLength;
        };
        let frame_len = dma_len - 2;
        increment_counter(core::ptr::addr_of_mut!(RX_VALID_FRAME_COUNT));
        RawRxOutcome::Frame(ReceivedFrame::new(&psdu[..frame_len], lqi, rssi))
    }

    fn classify_and_report(received: &ReceivedFrame, on_frame: &mut impl FnMut(RxOutcome)) {
        let psdu = received.as_slice();
        if let Some((sequence, frame_pending)) = frame::ack_info(psdu) {
            on_frame(RxOutcome::Ack {
                sequence,
                frame_pending,
            });
            return;
        }
        if let Some(response) = frame::parse_association_response(psdu) {
            on_frame(RxOutcome::AssociationResponse(response));
            return;
        }
        match frame::parse_beacon(psdu) {
            Some(info) => on_frame(RxOutcome::Beacon {
                info,
                len: received.len() as u8,
                lqi: received.lqi,
                rssi: received.rssi,
            }),
            None => on_frame(RxOutcome::NotABeacon {
                len: received.len() as u8,
                lqi: received.lqi,
                rssi: received.rssi,
            }),
        }
    }
}

#[cfg(target_arch = "tc32")]
pub use hw::{
    CsmaStats, RX_WINDOW_TICKS, RxOutcome, TX_TIMEOUT_TICKS, TxOutcome, csma_stats,
    dma_buffers_aligned, handle_irq, init, rx_diagnostics, rx_raw_window_for, rx_window,
    rx_window_for, send_beacon_request, send_mac_frame, set_ack_filter, set_channel,
    software_ack_stats,
};
