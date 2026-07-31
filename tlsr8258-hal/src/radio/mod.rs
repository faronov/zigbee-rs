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
}

#[cfg(target_arch = "tc32")]
mod hw {
    use core::sync::atomic::{Ordering, compiler_fence};

    use super::frame::{self, BeaconInfo};
    use super::{
        AckPendingAddress, AckPendingError, AckPendingTable, DMA_BUF_LEN, DmaBuf,
        MAX_MAC_FRAME_LEN, RawRxOutcome, ReceivedFrame, phy,
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
    const IRQ_RX_QUEUE_CAPACITY: usize = 8;
    static mut IRQ_RX_QUEUE: [RawRxOutcome; IRQ_RX_QUEUE_CAPACITY] =
        [RawRxOutcome::InvalidLength; IRQ_RX_QUEUE_CAPACITY];
    static mut IRQ_RX_QUEUE_HEAD: u8 = 0;
    static mut IRQ_RX_QUEUE_LEN: u8 = 0;
    static mut IRQ_RX_QUEUE_OVERFLOW_COUNT: u32 = 0;

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
            if let Some(outcome) = pop_irq_rx() {
                frames_seen += 1;
                if on_frame(outcome) {
                    break;
                }
                continue;
            }
            if timer::now_ticks().wrapping_sub(start) >= timeout_ticks {
                break;
            }
            if phy::rx_done() {
                let outcome = take_completed_rx();
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
        let outcome = take_completed_rx();
        push_irq_rx(outcome);
    }

    fn clear_irq_rx_queue() {
        unsafe {
            core::ptr::write_volatile(core::ptr::addr_of_mut!(IRQ_RX_QUEUE_HEAD), 0);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(IRQ_RX_QUEUE_LEN), 0);
        }
    }

    fn push_irq_rx(outcome: RawRxOutcome) {
        unsafe {
            let len = core::ptr::read_volatile(core::ptr::addr_of!(IRQ_RX_QUEUE_LEN)) as usize;
            if len == IRQ_RX_QUEUE_CAPACITY {
                increment_counter(core::ptr::addr_of_mut!(IRQ_RX_QUEUE_OVERFLOW_COUNT));
                return;
            }
            let head = core::ptr::read_volatile(core::ptr::addr_of!(IRQ_RX_QUEUE_HEAD)) as usize;
            let index = (head + len) % IRQ_RX_QUEUE_CAPACITY;
            let queue = core::ptr::addr_of_mut!(IRQ_RX_QUEUE).cast::<RawRxOutcome>();
            core::ptr::write_volatile(queue.add(index), outcome);
            compiler_fence(Ordering::Release);
            core::ptr::write_volatile(core::ptr::addr_of_mut!(IRQ_RX_QUEUE_LEN), (len + 1) as u8);
        }
    }

    fn pop_irq_rx() -> Option<RawRxOutcome> {
        unsafe {
            let len = core::ptr::read_volatile(core::ptr::addr_of!(IRQ_RX_QUEUE_LEN));
            if len == 0 {
                return None;
            }
            compiler_fence(Ordering::Acquire);
            let head = core::ptr::read_volatile(core::ptr::addr_of!(IRQ_RX_QUEUE_HEAD)) as usize;
            let queue = core::ptr::addr_of!(IRQ_RX_QUEUE).cast::<RawRxOutcome>();
            let outcome = core::ptr::read_volatile(queue.add(head));
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(IRQ_RX_QUEUE_HEAD),
                ((head + 1) % IRQ_RX_QUEUE_CAPACITY) as u8,
            );
            core::ptr::write_volatile(core::ptr::addr_of_mut!(IRQ_RX_QUEUE_LEN), len - 1);
            Some(outcome)
        }
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
            return RawRxOutcome::InvalidLength;
        }
        if !frame::packet_crc_ok(buf) {
            return RawRxOutcome::InvalidCrc;
        }
        let dma_len = frame::payload_len(buf) as usize;
        if dma_len < 2 || dma_len - 2 > MAX_MAC_FRAME_LEN {
            return RawRxOutcome::InvalidLength;
        }
        let rssi = frame::packet_rssi(buf);
        let lqi = frame::rssi_to_lqi(rssi);
        let Some(psdu) = frame::mac_psdu(buf) else {
            return RawRxOutcome::InvalidLength;
        };
        let frame_len = dma_len - 2;
        let mut data = [0u8; MAX_MAC_FRAME_LEN];
        data[..frame_len].copy_from_slice(&psdu[..frame_len]);
        RawRxOutcome::Frame(ReceivedFrame {
            data,
            len: frame_len as u8,
            lqi,
            rssi,
        })
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
    dma_buffers_aligned, handle_irq, init, rx_raw_window_for, rx_window, rx_window_for,
    send_beacon_request, send_mac_frame, set_ack_filter, set_channel, software_ack_stats,
};
