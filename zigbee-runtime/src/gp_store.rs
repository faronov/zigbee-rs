//! Durable Green Power Proxy Table.
//!
//! GP 1.1 requires the Proxy Table, including each GPD's security frame
//! counter, to survive a reboot. This module is the opt-in persistence for
//! the runtime's [`zigbee_gp::proxy::Proxy`]:
//!
//! - [`PersistentGpProxyTable`] is a bounded snapshot of every Proxy Table
//!   entry in the GP 1.1 Proxy Table attribute encoding (Table 39), bound to
//!   the network's extended PAN ID so a snapshot is never applied to another
//!   network;
//! - [`GpProxyJournal`] is an atomic two-sector flash journal of those
//!   snapshots (CRC, commit marker, read-back verification, generation
//!   ordering; an interrupted write or erase leaves the previous snapshot).
//!   A change confined to frame counters is appended as 32-byte counter
//!   records after the last full 1 KiB record instead of a new full record;
//! - [`ZigbeeDevice::restore_gp_proxy_table`],
//!   [`ZigbeeDevice::save_gp_proxy_table_if_dirty`] and
//!   [`ZigbeeDevice::clear_persisted_gp_proxy_table`] are the lifecycle.
//!
//! With `green-power-sink` the same snapshot and journal also carry the
//! combo's Sink Table (see [`GpProxyJournal`]); journals written without it
//! load unchanged.
//!
//! The proxy marks its table dirty when an entry changes and, for incoming
//! GPD frame counters, only every
//! [`ProxyConfig::counter_persist_stride`](zigbee_gp::proxy::ProxyConfig)
//! frames, which bounds flash wear. On restore it skips each protected
//! entry's counter ahead by `stride - 1`, the most it can have accepted
//! since the last write, so no frame seen before a power loss is accepted
//! again; the price is up to `stride - 1` new frames per GPD rejected as
//! stale after a reboot. The only remaining window is a frame accepted
//! after its write was triggered but before the journal write in the same
//! router loop pass completed.

use embedded_storage::nor_flash::NorFlash;
use zigbee_gp::proxy::{MAX_ENTRY_ENCODED_LEN, ProxyEntry};
use zigbee_mac::MacDriver;
use zigbee_types::IeeeAddress;

use crate::ZigbeeDevice;
use crate::green_power::GP_PROXY_TABLE_ENTRIES;
#[cfg(feature = "green-power-sink")]
use crate::green_power::GP_SINK_TABLE_ENTRIES;
use crate::role::DeviceRole;
use crate::security_journal::crc32;
#[cfg(feature = "green-power-sink")]
use zigbee_gp::sink::{MAX_SINK_ENTRY_ENCODED_LEN, SinkEntry};

/// Largest encoded snapshot payload: an entry count, then every entry as a
/// length byte followed by its Proxy Table attribute encoding.
pub const MAX_ENCODED_GP_PROXY_TABLE_LEN: usize =
    1 + GP_PROXY_TABLE_ENTRIES * (1 + MAX_ENTRY_ENCODED_LEN);

/// Largest encoded Sink Table payload: an entry count, then every entry as a
/// length byte followed by its Sink Table attribute encoding.
#[cfg(feature = "green-power-sink")]
pub const MAX_ENCODED_GP_SINK_TABLE_LEN: usize =
    1 + GP_SINK_TABLE_ENTRIES * (1 + MAX_SINK_ENTRY_ENCODED_LEN);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GpStoreError {
    /// Flash access failed or the configured geometry is unusable.
    Hardware,
    /// A stored snapshot is malformed or the live table cannot be encoded.
    Corrupt,
    /// The stored table does not fit the live Proxy Table.
    Full,
    /// The 32-bit journal generation is exhausted.
    GenerationExhausted,
    /// A non-empty snapshot belongs to another network.
    ForeignNetwork,
}

/// Bounded, network-bound snapshot of a Proxy Table.
#[derive(Clone, PartialEq, Eq)]
pub struct PersistentGpProxyTable {
    extended_pan_id: IeeeAddress,
    len: usize,
    payload: [u8; MAX_ENCODED_GP_PROXY_TABLE_LEN],
    /// Bit `i`: entry `i`'s frame counter was written exactly
    /// ([`zigbee_gp::counter`]); clear restores it batched.
    exact: u32,
    /// The combo's Sink Table, same layout (count, then length-prefixed
    /// Sink Table attribute entries).
    #[cfg(feature = "green-power-sink")]
    sink_len: usize,
    #[cfg(feature = "green-power-sink")]
    sink_payload: [u8; SINK_PAYLOAD_CAPACITY],
    /// Exact-counter bits of the Sink Table entries.
    #[cfg(feature = "green-power-sink")]
    sink_exact: u32,
}

impl core::fmt::Debug for PersistentGpProxyTable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // The payload carries GPD keys: never print it.
        let mut f = f.debug_struct("PersistentGpProxyTable");
        f.field("extended_pan_id", &self.extended_pan_id)
            .field("entries", &self.payload[0]);
        #[cfg(feature = "green-power-sink")]
        f.field("sink_entries", &self.sink_payload[0]);
        f.finish_non_exhaustive()
    }
}

impl Default for PersistentGpProxyTable {
    fn default() -> Self {
        Self::empty([0; 8])
    }
}

impl PersistentGpProxyTable {
    /// A snapshot holding no entries.
    pub const fn empty(extended_pan_id: IeeeAddress) -> Self {
        Self {
            extended_pan_id,
            len: 1,
            payload: [0; MAX_ENCODED_GP_PROXY_TABLE_LEN],
            exact: 0,
            #[cfg(feature = "green-power-sink")]
            sink_len: 1,
            #[cfg(feature = "green-power-sink")]
            sink_payload: [0; SINK_PAYLOAD_CAPACITY],
            #[cfg(feature = "green-power-sink")]
            sink_exact: 0,
        }
    }

    /// Encode `entries` for the network `extended_pan_id`.
    pub fn capture(
        extended_pan_id: IeeeAddress,
        entries: &[ProxyEntry],
    ) -> Result<Self, GpStoreError> {
        let mut snapshot = Self::empty(extended_pan_id);
        if entries.len() > GP_PROXY_TABLE_ENTRIES {
            return Err(GpStoreError::Full);
        }
        snapshot.payload[0] = entries.len() as u8;
        let mut at = 1;
        for (index, entry) in entries.iter().enumerate() {
            let out = &mut snapshot.payload[at + 1..at + 1 + MAX_ENTRY_ENCODED_LEN];
            let n = entry.encode(out).map_err(|_| GpStoreError::Corrupt)?;
            snapshot.payload[at] = n as u8;
            snapshot.exact |= u32::from(entry.counter_exact()) << index;
            at += 1 + n;
        }
        snapshot.len = at;
        Ok(snapshot)
    }

    /// Rebuild a snapshot from its stored parts, validating every entry.
    pub fn from_parts(extended_pan_id: IeeeAddress, payload: &[u8]) -> Result<Self, GpStoreError> {
        Self::validate_payload(payload)?;
        let mut snapshot = Self::empty(extended_pan_id);
        snapshot.set_validated(extended_pan_id, payload);
        Ok(snapshot)
    }

    /// Validate a stored payload in place, without a snapshot-sized copy.
    fn validate_payload(payload: &[u8]) -> Result<(), GpStoreError> {
        if payload.is_empty() || payload.len() > MAX_ENCODED_GP_PROXY_TABLE_LEN {
            return Err(GpStoreError::Corrupt);
        }
        let mut count = 0usize;
        for entry in Self::entries_in(payload) {
            entry?;
            count += 1;
        }
        if count != payload[0] as usize || count > GP_PROXY_TABLE_ENTRIES {
            return Err(GpStoreError::Corrupt);
        }
        Ok(())
    }

    /// Overwrite with a payload [`validate_payload`](Self::validate_payload)
    /// accepted.
    fn set_validated(&mut self, extended_pan_id: IeeeAddress, payload: &[u8]) {
        self.extended_pan_id = extended_pan_id;
        self.payload[..payload.len()].copy_from_slice(payload);
        self.len = payload.len();
        self.exact = 0;
    }

    /// Exact-counter bits of the Proxy Table entries (bit `i`: entry `i`).
    pub fn exact_counters(&self) -> u32 {
        self.exact
    }

    /// Whether any Proxy Table (or Sink Table) entry has an exact counter.
    pub fn has_exact_counters(&self) -> bool {
        #[cfg(feature = "green-power-sink")]
        if self.sink_exact != 0 {
            return true;
        }
        self.exact != 0
    }

    pub fn extended_pan_id(&self) -> IeeeAddress {
        self.extended_pan_id
    }

    /// Number of stored entries.
    pub fn len(&self) -> usize {
        self.payload[0] as usize
    }

    /// Whether the snapshot holds no Proxy Table (and, with
    /// `green-power-sink`, no Sink Table) entry.
    pub fn is_empty(&self) -> bool {
        #[cfg(feature = "green-power-sink")]
        if self.sink_len() != 0 {
            return false;
        }
        self.len() == 0
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.len]
    }

    /// Decode the stored entries. Trailing bytes or a malformed entry yield
    /// [`GpStoreError::Corrupt`].
    pub fn entries(&self) -> impl Iterator<Item = Result<ProxyEntry, GpStoreError>> + '_ {
        Self::entries_in(self.payload())
            .enumerate()
            .map(|(index, entry)| entry.map(|e| e.with_counter_exact(self.exact >> index & 1 != 0)))
    }

    /// Decode the entries of a non-empty encoded payload.
    fn entries_in(payload: &[u8]) -> impl Iterator<Item = Result<ProxyEntry, GpStoreError>> + '_ {
        let mut at = 1usize;
        let mut left = payload[0] as usize;
        let mut failed = false;
        core::iter::from_fn(move || {
            if failed {
                return None;
            }
            if left == 0 {
                if at != payload.len() {
                    failed = true;
                    return Some(Err(GpStoreError::Corrupt));
                }
                return None;
            }
            left -= 1;
            let decoded = (|| {
                let n = *payload.get(at).ok_or(GpStoreError::Corrupt)? as usize;
                let bytes = payload
                    .get(at + 1..at + 1 + n)
                    .ok_or(GpStoreError::Corrupt)?;
                let (entry, used) = ProxyEntry::decode(bytes).map_err(|_| GpStoreError::Corrupt)?;
                if used != n {
                    return Err(GpStoreError::Corrupt);
                }
                at += 1 + n;
                Ok(entry)
            })();
            failed = decoded.is_err();
            Some(decoded)
        })
    }
}

#[cfg(feature = "green-power-sink")]
impl PersistentGpProxyTable {
    /// Add the Sink Table `entries` to a snapshot.
    pub fn with_sink(mut self, entries: &[SinkEntry]) -> Result<Self, GpStoreError> {
        if entries.len() > GP_SINK_TABLE_ENTRIES {
            return Err(GpStoreError::Full);
        }
        self.sink_payload[0] = entries.len() as u8;
        self.sink_exact = 0;
        let mut at = 1;
        for (index, entry) in entries.iter().enumerate() {
            let out = &mut self.sink_payload[at + 1..at + 1 + MAX_SINK_ENTRY_ENCODED_LEN];
            let n = entry.encode(out).map_err(|_| GpStoreError::Corrupt)?;
            self.sink_payload[at] = n as u8;
            self.sink_exact |= u32::from(entry.counter_exact()) << index;
            at += 1 + n;
        }
        self.sink_len = at;
        Ok(self)
    }

    /// Number of stored Sink Table entries.
    pub fn sink_len(&self) -> usize {
        self.sink_payload[0] as usize
    }

    pub fn sink_payload(&self) -> &[u8] {
        &self.sink_payload[..self.sink_len]
    }

    /// Decode the stored Sink Table entries.
    pub fn sink_entries(&self) -> impl Iterator<Item = Result<SinkEntry, GpStoreError>> + '_ {
        Self::sink_entries_in(self.sink_payload())
            .enumerate()
            .map(|(index, entry)| {
                entry.map(|e| e.with_counter_exact(self.sink_exact >> index & 1 != 0))
            })
    }

    /// Exact-counter bits of the Sink Table entries.
    pub fn sink_exact_counters(&self) -> u32 {
        self.sink_exact
    }

    /// Decode the entries of a non-empty encoded Sink Table payload.
    fn sink_entries_in(
        payload: &[u8],
    ) -> impl Iterator<Item = Result<SinkEntry, GpStoreError>> + '_ {
        let mut at = 1usize;
        let mut left = payload[0] as usize;
        let mut failed = false;
        core::iter::from_fn(move || {
            if failed {
                return None;
            }
            if left == 0 {
                if at != payload.len() {
                    failed = true;
                    return Some(Err(GpStoreError::Corrupt));
                }
                return None;
            }
            left -= 1;
            let decoded = (|| {
                let n = *payload.get(at).ok_or(GpStoreError::Corrupt)? as usize;
                let bytes = payload
                    .get(at + 1..at + 1 + n)
                    .ok_or(GpStoreError::Corrupt)?;
                let (entry, used) = SinkEntry::decode(bytes).map_err(|_| GpStoreError::Corrupt)?;
                if used != n {
                    return Err(GpStoreError::Corrupt);
                }
                at += 1 + n;
                Ok(entry)
            })();
            failed = decoded.is_err();
            Some(decoded)
        })
    }

    /// Validate a stored Sink Table payload in place.
    fn validate_sink_payload(payload: &[u8]) -> Result<(), GpStoreError> {
        if payload.is_empty() || payload.len() > MAX_ENCODED_GP_SINK_TABLE_LEN {
            return Err(GpStoreError::Corrupt);
        }
        let mut count = 0usize;
        for entry in Self::sink_entries_in(payload) {
            entry?;
            count += 1;
        }
        if count != payload[0] as usize || count > GP_SINK_TABLE_ENTRIES {
            return Err(GpStoreError::Corrupt);
        }
        Ok(())
    }

    fn set_sink_validated(&mut self, payload: &[u8]) {
        self.sink_payload[..payload.len()].copy_from_slice(payload);
        self.sink_len = payload.len();
        self.sink_exact = 0;
    }
}

/// Durable storage for one [`PersistentGpProxyTable`].
pub trait GpProxyStore {
    /// Load the newest snapshot into `snapshot`; `Ok(false)` leaves it
    /// untouched when nothing is stored.
    fn load_into(&mut self, snapshot: &mut PersistentGpProxyTable) -> Result<bool, GpStoreError>;
    fn store(&mut self, snapshot: &PersistentGpProxyTable) -> Result<(), GpStoreError>;
}

/// Volatile store for tests and products without a GP partition.
#[derive(Default)]
pub struct RamGpProxyStore {
    snapshot: Option<PersistentGpProxyTable>,
    pub writes: usize,
}

impl RamGpProxyStore {
    pub const fn new() -> Self {
        Self {
            snapshot: None,
            writes: 0,
        }
    }
}

impl GpProxyStore for RamGpProxyStore {
    fn load_into(&mut self, snapshot: &mut PersistentGpProxyTable) -> Result<bool, GpStoreError> {
        match &self.snapshot {
            Some(stored) => {
                snapshot.clone_from(stored);
                Ok(true)
            }
            None => Ok(false),
        }
    }

    fn store(&mut self, snapshot: &PersistentGpProxyTable) -> Result<(), GpStoreError> {
        self.snapshot = Some(snapshot.clone());
        self.writes += 1;
        Ok(())
    }
}

/// Erase-unit size assumed for each journal sector.
pub const GP_PROXY_JOURNAL_SECTOR_SIZE: usize = 4096;
/// Size of one GP Proxy Table journal slot.
pub const GP_PROXY_JOURNAL_SLOT_SIZE: usize = 1024;
/// Size of one compact frame-counter record.
pub const GP_COUNTER_RECORD_SIZE: usize = 32;

const RECORD_MAGIC: [u8; 4] = *b"ZBGP";
const RECORD_VERSION: u8 = 1;
const RECORD_EPID_OFFSET: usize = 12;
const RECORD_PAYLOAD_OFFSET: usize = 20;
const RECORD_CRC_OFFSET: usize = GP_PROXY_JOURNAL_SLOT_SIZE - 12;
const RECORD_COMMIT_OFFSET: usize = GP_PROXY_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

// Compact counter record: magic, version, entry index, 2 reserved bytes,
// base generation, GPD ID tag, frame counter, CRC32 of bytes 0..20, then
// the commit marker at 24..28; 28..32 stay erased.
const COUNTER_MAGIC: [u8; 4] = *b"ZBGC";
/// Sink Table record magic (`green-power-sink`). A sink record has the
/// full-record layout and immediately precedes, in the same sector, the
/// Proxy Table record of the same generation.
#[cfg(feature = "green-power-sink")]
const SINK_RECORD_MAGIC: [u8; 4] = *b"ZBGS";
/// Byte 7 of a Proxy Table record (zero, and not checked, in the original
/// format): set when a sink record of the same generation precedes it. A
/// lone sink record left by an interrupted pair write is never paired with a
/// later Proxy Table record that reuses its generation.
#[cfg(feature = "green-power-sink")]
const RECORD_PAIRED_WITH_SINK: u8 = 0x01;
/// Counter records of Sink Table entries carry this bit in the entry index;
/// firmware without the sink ignores them (index beyond its table).
#[cfg(feature = "green-power-sink")]
const SINK_COUNTER_INDEX: u8 = 0x80;
const COUNTER_VERSION: u8 = 1;
/// Counter record byte 6 (written zero, and never read, before adaptive
/// persistence): flags of the stored counter.
const COUNTER_FLAGS_OFFSET: usize = 6;
/// The counter was written exactly: restore it as is, not skipped ahead.
/// Firmware that ignores the byte restores it batched, which is safe.
const COUNTER_FLAG_EXACT: u8 = 0x01;
const COUNTER_CRC_OFFSET: usize = 20;

/// `bits` with bit `index` set to `value`.
const fn set_bit(bits: u32, index: usize, value: bool) -> u32 {
    if value {
        bits | 1 << index
    } else {
        bits & !(1 << index)
    }
}
const COUNTER_COMMIT_OFFSET: usize = 24;

const _: () = assert!(RECORD_PAYLOAD_OFFSET + MAX_ENCODED_GP_PROXY_TABLE_LEN <= RECORD_CRC_OFFSET);
const _: () = assert!(GP_PROXY_JOURNAL_SLOT_SIZE.is_multiple_of(GP_COUNTER_RECORD_SIZE));
const _: () = assert!(GP_PROXY_TABLE_ENTRIES <= u8::MAX as usize);
const _: () = assert!(GP_PROXY_TABLE_ENTRIES <= u32::BITS as usize);
#[cfg(feature = "green-power-sink")]
const _: () = assert!(RECORD_PAYLOAD_OFFSET + MAX_ENCODED_GP_SINK_TABLE_LEN <= RECORD_CRC_OFFSET);
#[cfg(feature = "green-power-sink")]
const _: () = assert!(GP_SINK_TABLE_ENTRIES < SINK_COUNTER_INDEX as usize);
#[cfg(feature = "green-power-sink")]
const _: () = assert!(GP_SINK_TABLE_ENTRIES <= u32::BITS as usize);
#[cfg(feature = "green-power-sink")]
const _: () = assert!(GP_PROXY_TABLE_ENTRIES < SINK_COUNTER_INDEX as usize);

/// Where each entry's GPD ID tag and frame counter sit in a snapshot
/// payload. A zero counter offset means the entry carries no counter.
#[cfg(not(feature = "green-power-sink"))]
struct PayloadLayout {
    entries: usize,
    tags: [u32; GP_PROXY_TABLE_ENTRIES],
    counter_at: [u16; GP_PROXY_TABLE_ENTRIES],
}

#[cfg(not(feature = "green-power-sink"))]
impl PayloadLayout {
    /// Lay out a payload [`PersistentGpProxyTable::validate_payload`]
    /// accepted.
    fn of(payload: &[u8]) -> Result<Self, GpStoreError> {
        let mut layout = Self {
            entries: payload[0] as usize,
            tags: [0; GP_PROXY_TABLE_ENTRIES],
            counter_at: [0; GP_PROXY_TABLE_ENTRIES],
        };
        if layout.entries > GP_PROXY_TABLE_ENTRIES {
            return Err(GpStoreError::Corrupt);
        }
        let mut at = 1usize;
        for index in 0..layout.entries {
            let n = *payload.get(at).ok_or(GpStoreError::Corrupt)? as usize;
            let entry = payload
                .get(at + 1..at + 1 + n)
                .ok_or(GpStoreError::Corrupt)?;
            let (id, counter) =
                ProxyEntry::encoded_layout(entry).map_err(|_| GpStoreError::Corrupt)?;
            layout.tags[index] = crc32(&entry[id]);
            if let Some(counter) = counter {
                layout.counter_at[index] = (at + 1 + counter) as u16;
            }
            at += 1 + n;
        }
        Ok(layout)
    }

    fn counter(&self, payload: &[u8], index: usize) -> Option<u32> {
        let at = self.counter_at[index] as usize;
        (at != 0).then(|| {
            u32::from_le_bytes([
                payload[at],
                payload[at + 1],
                payload[at + 2],
                payload[at + 3],
            ])
        })
    }

    /// Whether payload byte `at` belongs to a frame counter.
    fn in_counter(&self, at: usize) -> bool {
        self.counter_at[..self.entries]
            .iter()
            .any(|&c| c != 0 && (c as usize..c as usize + 4).contains(&at))
    }
}

/// With `green-power-sink` one layout type serves both tables: where each
/// entry's GPD ID tag and frame counter sit in a Proxy Table or Sink Table
/// payload (zero counter offset: no counter).
#[cfg(feature = "green-power-sink")]
struct EntryLayout {
    entries: usize,
    tags: [u32; LAYOUT_ENTRIES],
    counter_at: [u16; LAYOUT_ENTRIES],
}

#[cfg(feature = "green-power-sink")]
const LAYOUT_ENTRIES: usize = if GP_PROXY_TABLE_ENTRIES > GP_SINK_TABLE_ENTRIES {
    GP_PROXY_TABLE_ENTRIES
} else {
    GP_SINK_TABLE_ENTRIES
};

#[cfg(feature = "green-power-sink")]
type PayloadLayout = EntryLayout;

/// Locates the GPD ID and frame counter of one encoded entry.
#[cfg(feature = "green-power-sink")]
type LocateEntry = fn(&[u8]) -> zigbee_gp::Result<(core::ops::Range<usize>, Option<usize>)>;

#[cfg(feature = "green-power-sink")]
impl EntryLayout {
    /// Lay out a Proxy Table payload.
    fn of(payload: &[u8]) -> Result<Self, GpStoreError> {
        Self::walk(payload, GP_PROXY_TABLE_ENTRIES, ProxyEntry::encoded_layout)
    }

    /// Lay out a Sink Table payload.
    fn of_sink(payload: &[u8]) -> Result<Self, GpStoreError> {
        Self::walk(payload, GP_SINK_TABLE_ENTRIES, SinkEntry::encoded_layout)
    }

    #[inline(never)]
    fn walk(payload: &[u8], max: usize, locate: LocateEntry) -> Result<Self, GpStoreError> {
        let mut layout = Self {
            entries: *payload.first().ok_or(GpStoreError::Corrupt)? as usize,
            tags: [0; LAYOUT_ENTRIES],
            counter_at: [0; LAYOUT_ENTRIES],
        };
        if layout.entries > max {
            return Err(GpStoreError::Corrupt);
        }
        let mut at = 1usize;
        for index in 0..layout.entries {
            let n = *payload.get(at).ok_or(GpStoreError::Corrupt)? as usize;
            let entry = payload
                .get(at + 1..at + 1 + n)
                .ok_or(GpStoreError::Corrupt)?;
            let (id, counter) = locate(entry).map_err(|_| GpStoreError::Corrupt)?;
            layout.tags[index] = crc32(&entry[id]);
            if let Some(counter) = counter {
                layout.counter_at[index] = (at + 1 + counter) as u16;
            }
            at += 1 + n;
        }
        Ok(layout)
    }

    fn counter(&self, payload: &[u8], index: usize) -> Option<u32> {
        let at = self.counter_at[index] as usize;
        (at != 0).then(|| {
            u32::from_le_bytes([
                payload[at],
                payload[at + 1],
                payload[at + 2],
                payload[at + 3],
            ])
        })
    }

    /// Whether payload byte `at` belongs to a frame counter.
    fn in_counter(&self, at: usize) -> bool {
        self.counter_at[..self.entries]
            .iter()
            .any(|&c| c != 0 && (c as usize..c as usize + 4).contains(&at))
    }
}

/// The Sink Table record of the newest full record.
#[cfg(feature = "green-power-sink")]
#[derive(Clone, Copy)]
struct SinkBase {
    /// Slot of the sink record (the Proxy Table record's slot - 1).
    slot: u8,
    /// Encoded Sink Table payload length.
    len: u16,
    /// Effective frame counter of each Sink Table entry.
    counters: [u32; GP_SINK_TABLE_ENTRIES],
    /// Effective exact-counter bit of each Sink Table entry.
    exact: u32,
}

/// The newest full record and the counter records appended after it.
#[derive(Clone, Copy)]
struct Base {
    generation: u32,
    sector: u8,
    slot: u8,
    /// Encoded payload length of the full record.
    len: u16,
    /// Sector offset of the next erased counter record unit.
    append: u16,
    /// Effective frame counter of each entry: the full record's value
    /// overlaid by the newest counter record for that entry.
    counters: [u32; GP_PROXY_TABLE_ENTRIES],
    /// Effective exact-counter bit of each entry (a full record: none).
    exact: u32,
    /// The Sink Table record paired with this full record, if any.
    #[cfg(feature = "green-power-sink")]
    sink: Option<SinkBase>,
}

/// Atomic two-sector journal of GP Proxy Table snapshots.
///
/// A structural change appends a full CRC-protected record (1 KiB slot) to
/// the next erased slot of the current sector, writing its commit marker
/// last and verifying the whole slot by read-back. A full sector rolls over
/// by erasing the other sector, so an interrupted erase or write always
/// leaves the previous snapshot.
///
/// A snapshot that differs from the stored one only in frame counters is
/// written as compact 32-byte counter records appended after the full
/// record, one per changed entry, each bound to the full record's
/// generation and to its GPD by a tag; the newest one wins on load. Only
/// when the sector has no room for them is a full record written. An
/// unchanged snapshot is not written at all. Journals holding only full
/// records (the original format) load unchanged; firmware without counter
/// record support ignores them and restores the full record's counters.
///
/// `green-power-sink`: a snapshot with Sink Table entries is written as two
/// consecutive slots of one sector, a `ZBGS` sink record (same layout as the
/// full record) and then the `ZBGP` Proxy Table record of the same
/// generation, flagged as paired in its (formerly zero, unchecked) byte 7;
/// its commit marker commits both. Load pairs a flagged newest `ZBGP` record
/// with the `ZBGS` record of the same generation in the slot before it;
/// anything else (no flag, a torn pair, a journal written without the sink)
/// means an empty Sink Table. Sink entry frame counters use counter
/// records with bit 7 set in the entry index. A snapshot without sink
/// entries is written exactly as without the feature. Firmware without the
/// sink ignores `ZBGS` slots and sink counter records, so its Proxy Table
/// still loads.
pub struct GpProxyJournal<S, const SECTOR_SIZE: usize = GP_PROXY_JOURNAL_SECTOR_SIZE> {
    storage: S,
    sectors: [u32; 2],
    /// Newest committed full record, once scanned.
    current: Option<Base>,
    scanned: bool,
}

impl<S: NorFlash, const SECTOR_SIZE: usize> GpProxyJournal<S, SECTOR_SIZE> {
    const SLOTS_PER_SECTOR: usize = SECTOR_SIZE / GP_PROXY_JOURNAL_SLOT_SIZE;
    /// Whether the flash can program counter records (body, then commit).
    const COUNTER_RECORDS: bool = COUNTER_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
        && RECORD_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
        && GP_COUNTER_RECORD_SIZE.is_multiple_of(S::READ_SIZE);

    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        assert!(SECTOR_SIZE >= GP_PROXY_JOURNAL_SLOT_SIZE);
        assert!(SECTOR_SIZE.is_multiple_of(GP_PROXY_JOURNAL_SLOT_SIZE));
        assert!(SECTOR_SIZE <= u16::MAX as usize);
        Self {
            storage,
            sectors: [first_sector, second_sector],
            current: None,
            scanned: false,
        }
    }

    pub fn into_storage(self) -> S {
        self.storage
    }

    fn geometry_ok(&self) -> bool {
        self.sectors[0].abs_diff(self.sectors[1]) >= SECTOR_SIZE as u32
            && S::READ_SIZE != 0
            && S::WRITE_SIZE != 0
            && S::ERASE_SIZE != 0
            && GP_PROXY_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
            && RECORD_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
            && RECORD_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            && SECTOR_SIZE.is_multiple_of(S::ERASE_SIZE)
            && self.sectors.iter().all(|sector| {
                (*sector as usize).is_multiple_of(S::ERASE_SIZE)
                    && (*sector as usize)
                        .checked_add(SECTOR_SIZE)
                        .is_some_and(|end| end <= self.storage.capacity())
            })
    }

    fn slot_address(&self, sector: usize, slot: usize) -> u32 {
        self.sectors[sector] + (slot * GP_PROXY_JOURNAL_SLOT_SIZE) as u32
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        record: &mut [u8; GP_PROXY_JOURNAL_SLOT_SIZE],
    ) -> Result<(), GpStoreError> {
        let address = self.slot_address(sector, slot);
        self.storage
            .read(address, record)
            .map_err(|_| GpStoreError::Hardware)
    }

    /// Generation and payload length of a committed, CRC-valid record.
    #[cfg(not(feature = "green-power-sink"))]
    fn header(record: &[u8; GP_PROXY_JOURNAL_SLOT_SIZE]) -> Option<(u32, usize)> {
        if record[0..4] != RECORD_MAGIC
            || record[4] != RECORD_VERSION
            || record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if len == 0 || len > MAX_ENCODED_GP_PROXY_TABLE_LEN {
            return None;
        }
        let crc = u32::from_le_bytes([
            record[RECORD_CRC_OFFSET],
            record[RECORD_CRC_OFFSET + 1],
            record[RECORD_CRC_OFFSET + 2],
            record[RECORD_CRC_OFFSET + 3],
        ]);
        if crc32(&record[..RECORD_CRC_OFFSET]) != crc {
            return None;
        }
        Some((
            u32::from_le_bytes([record[8], record[9], record[10], record[11]]),
            len,
        ))
    }

    /// Entry index, GPD ID tag, frame counter and exact flag of a committed,
    /// CRC-valid counter record bound to the full record `generation`.
    fn counter_record(unit: &[u8], generation: u32) -> Option<(usize, u32, u32, bool)> {
        let word =
            |at: usize| u32::from_le_bytes([unit[at], unit[at + 1], unit[at + 2], unit[at + 3]]);
        (unit[0..4] == COUNTER_MAGIC
            && unit[4] == COUNTER_VERSION
            && unit[COUNTER_COMMIT_OFFSET..COUNTER_COMMIT_OFFSET + 4] == RECORD_COMMIT
            && crc32(&unit[..COUNTER_CRC_OFFSET]) == word(COUNTER_CRC_OFFSET)
            && word(8) == generation)
            .then(|| {
                (
                    unit[5] as usize,
                    word(12),
                    word(16),
                    unit[COUNTER_FLAGS_OFFSET] & COUNTER_FLAG_EXACT != 0,
                )
            })
    }

    /// Scan both sectors, copying the newest valid snapshot, with the
    /// newest counter record of each entry applied, into `snapshot` (when
    /// given). Equal generations prefer scan order.
    fn scan(
        &mut self,
        mut snapshot: Option<&mut PersistentGpProxyTable>,
    ) -> Result<bool, GpStoreError> {
        if !self.geometry_ok() {
            return Err(GpStoreError::Hardware);
        }
        let mut record = [0u8; GP_PROXY_JOURNAL_SLOT_SIZE];
        let mut best: Option<(u32, usize, usize, usize)> = None;
        for sector in 0..2 {
            for slot in 0..Self::SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some((generation, len)) = Self::header(&record) else {
                    continue;
                };
                if best.is_some_and(|(best_generation, ..)| generation <= best_generation) {
                    continue;
                }
                let payload = &record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + len];
                if PersistentGpProxyTable::validate_payload(payload).is_err() {
                    continue;
                }
                best = Some((generation, sector, slot, len));
            }
        }
        self.current = None;
        self.scanned = false;
        let Some((generation, sector, slot, len)) = best else {
            self.scanned = true;
            return Ok(false);
        };

        // The sink record of this generation sits in the slot before it.
        #[cfg(feature = "green-power-sink")]
        let (mut sink, sink_layout) =
            self.scan_sink(sector, slot, generation, &mut record, &mut snapshot)?;
        self.read_slot(sector, slot, &mut record)?;
        let payload = &record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + len];
        let layout = PayloadLayout::of(payload)?;
        let mut base = Base {
            generation,
            sector: sector as u8,
            slot: slot as u8,
            len: len as u16,
            append: SECTOR_SIZE as u16,
            counters: [0; GP_PROXY_TABLE_ENTRIES],
            exact: 0,
            #[cfg(feature = "green-power-sink")]
            sink: None,
        };
        for index in 0..layout.entries {
            base.counters[index] = layout.counter(payload, index).unwrap_or(0);
        }
        if let Some(snapshot) = snapshot.as_deref_mut() {
            let mut extended_pan_id = [0u8; 8];
            extended_pan_id.copy_from_slice(&record[RECORD_EPID_OFFSET..RECORD_EPID_OFFSET + 8]);
            snapshot.set_validated(extended_pan_id, payload);
        }

        if Self::COUNTER_RECORDS {
            'walk: for unit_slot in slot + 1..Self::SLOTS_PER_SECTOR {
                self.read_slot(sector, unit_slot, &mut record)?;
                for (unit_index, unit) in record.chunks_exact(GP_COUNTER_RECORD_SIZE).enumerate() {
                    if unit.iter().all(|byte| *byte == 0xFF) {
                        base.append = (unit_slot * GP_PROXY_JOURNAL_SLOT_SIZE
                            + unit_index * GP_COUNTER_RECORD_SIZE)
                            as u16;
                        break 'walk;
                    }
                    // Torn or foreign units are skipped.
                    if let Some((index, tag, counter, exact)) =
                        Self::counter_record(unit, generation)
                        && index < layout.entries
                        && layout.counter_at[index] != 0
                        && layout.tags[index] == tag
                    {
                        base.counters[index] = counter;
                        base.exact = set_bit(base.exact, index, exact);
                    }
                    #[cfg(feature = "green-power-sink")]
                    if let (Some(sink), Some((index, tag, counter, exact))) =
                        (sink.as_mut(), Self::counter_record(unit, generation))
                        && index & SINK_COUNTER_INDEX as usize != 0
                    {
                        let index = index & !(SINK_COUNTER_INDEX as usize);
                        if index < sink_layout.entries
                            && sink_layout.counter_at[index] != 0
                            && sink_layout.tags[index] == tag
                        {
                            sink.counters[index] = counter;
                            sink.exact = set_bit(sink.exact, index, exact);
                        }
                    }
                }
            }
            if let Some(snapshot) = snapshot {
                snapshot.exact = base.exact;
                for index in 0..layout.entries {
                    let at = layout.counter_at[index] as usize;
                    if at != 0 {
                        snapshot.payload[at..at + 4]
                            .copy_from_slice(&base.counters[index].to_le_bytes());
                    }
                }
                #[cfg(feature = "green-power-sink")]
                if let Some(sink) = &sink {
                    snapshot.sink_exact = sink.exact;
                    for index in 0..sink_layout.entries {
                        let at = sink_layout.counter_at[index] as usize;
                        if at != 0 {
                            snapshot.sink_payload[at..at + 4]
                                .copy_from_slice(&sink.counters[index].to_le_bytes());
                        }
                    }
                }
            }
        }
        #[cfg(feature = "green-power-sink")]
        {
            base.sink = sink;
        }
        self.current = Some(base);
        self.scanned = true;
        Ok(true)
    }

    /// First fully erased slot of `sector` from slot `from`.
    fn first_erased_slot(
        &mut self,
        sector: usize,
        from: usize,
    ) -> Result<Option<usize>, GpStoreError> {
        let mut record = [0u8; GP_PROXY_JOURNAL_SLOT_SIZE];
        for slot in from..Self::SLOTS_PER_SECTOR {
            self.read_slot(sector, slot, &mut record)?;
            if record.iter().all(|byte| *byte == 0xFF) {
                return Ok(Some(slot));
            }
        }
        Ok(None)
    }

    #[cfg(not(feature = "green-power-sink"))]
    fn write_record(
        &mut self,
        sector: usize,
        slot: usize,
        generation: u32,
        snapshot: &PersistentGpProxyTable,
    ) -> Result<(), GpStoreError> {
        let payload = snapshot.payload();
        let mut record = [0xFFu8; GP_PROXY_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = RECORD_VERSION;
        record[5..7].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        record[RECORD_EPID_OFFSET..RECORD_EPID_OFFSET + 8]
            .copy_from_slice(&snapshot.extended_pan_id);
        record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + payload.len()]
            .copy_from_slice(payload);
        let crc = crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.slot_address(sector, slot);
        self.storage
            .write(address, &record[..RECORD_COMMIT_OFFSET])
            .map_err(|_| GpStoreError::Hardware)?;
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + RECORD_COMMIT_OFFSET as u32,
                &record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4],
            )
            .map_err(|_| GpStoreError::Hardware)?;

        // Verify in place, reusing the record buffer's comparison window.
        let mut chunk = [0u8; 64];
        for (index, expected) in record.chunks(chunk.len()).enumerate() {
            let at = address + (index * chunk.len()) as u32;
            let window = &mut chunk[..expected.len()];
            self.storage
                .read(at, window)
                .map_err(|_| GpStoreError::Hardware)?;
            if window != expected {
                return Err(GpStoreError::Hardware);
            }
        }
        Ok(())
    }

    /// Whether the stored full record equals `snapshot` in everything but
    /// the frame counters (network, length and every other byte).
    #[cfg(not(feature = "green-power-sink"))]
    fn same_structure(
        &mut self,
        base: &Base,
        snapshot: &PersistentGpProxyTable,
        layout: &PayloadLayout,
    ) -> Result<bool, GpStoreError> {
        let payload = snapshot.payload();
        if base.len as usize != payload.len() {
            return Ok(false);
        }
        let address = self.slot_address(base.sector as usize, base.slot as usize);
        let mut epid = [0u8; 8];
        self.storage
            .read(address + RECORD_EPID_OFFSET as u32, &mut epid)
            .map_err(|_| GpStoreError::Hardware)?;
        if epid != snapshot.extended_pan_id {
            return Ok(false);
        }
        let mut chunk = [0u8; 64];
        for (index, expected) in payload.chunks(chunk.len()).enumerate() {
            let start = index * chunk.len();
            let window = &mut chunk[..expected.len()];
            self.storage
                .read(address + (RECORD_PAYLOAD_OFFSET + start) as u32, window)
                .map_err(|_| GpStoreError::Hardware)?;
            let differs = window
                .iter()
                .zip(expected)
                .enumerate()
                .any(|(i, (stored, new))| stored != new && !layout.in_counter(start + i));
            if differs {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Append one counter record at `base.append`, verified by read-back.
    /// `Ok(false)`: the unit is not erased, write a full record instead.
    fn write_counter_record(
        &mut self,
        base: &Base,
        index: usize,
        tag: u32,
        (counter, exact): (u32, bool),
    ) -> Result<bool, GpStoreError> {
        let address = self.sectors[base.sector as usize] + base.append as u32;
        let mut unit = [0u8; GP_COUNTER_RECORD_SIZE];
        self.storage
            .read(address, &mut unit)
            .map_err(|_| GpStoreError::Hardware)?;
        if unit.iter().any(|byte| *byte != 0xFF) {
            return Ok(false);
        }
        unit[0..4].copy_from_slice(&COUNTER_MAGIC);
        unit[4] = COUNTER_VERSION;
        unit[5] = index as u8;
        unit[COUNTER_FLAGS_OFFSET] = if exact { COUNTER_FLAG_EXACT } else { 0 };
        unit[7] = 0;
        unit[8..12].copy_from_slice(&base.generation.to_le_bytes());
        unit[12..16].copy_from_slice(&tag.to_le_bytes());
        unit[16..20].copy_from_slice(&counter.to_le_bytes());
        let crc = crc32(&unit[..COUNTER_CRC_OFFSET]);
        unit[COUNTER_CRC_OFFSET..COUNTER_COMMIT_OFFSET].copy_from_slice(&crc.to_le_bytes());
        self.storage
            .write(address, &unit[..COUNTER_COMMIT_OFFSET])
            .map_err(|_| GpStoreError::Hardware)?;
        unit[COUNTER_COMMIT_OFFSET..COUNTER_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + COUNTER_COMMIT_OFFSET as u32,
                &unit[COUNTER_COMMIT_OFFSET..COUNTER_COMMIT_OFFSET + 4],
            )
            .map_err(|_| GpStoreError::Hardware)?;
        let mut check = [0u8; GP_COUNTER_RECORD_SIZE];
        self.storage
            .read(address, &mut check)
            .map_err(|_| GpStoreError::Hardware)?;
        if check != unit {
            return Err(GpStoreError::Hardware);
        }
        Ok(true)
    }

    /// Persist a counter-only change (a counter or its exact flag) as
    /// counter records. `Ok(false)` when a full record is needed
    /// (structural change or no room).
    fn commit_counters(&mut self, snapshot: &PersistentGpProxyTable) -> Result<bool, GpStoreError> {
        let Some(mut base) = self.current else {
            return Ok(false);
        };
        if !Self::COUNTER_RECORDS {
            return Ok(false);
        }
        let payload = snapshot.payload();
        let layout = PayloadLayout::of(payload)?;
        if !self.same_structure(&base, snapshot, &layout)? {
            return Ok(false);
        }
        #[cfg(feature = "green-power-sink")]
        let sink_layout = EntryLayout::of_sink(snapshot.sink_payload())?;
        #[cfg(feature = "green-power-sink")]
        if !self.same_sink_structure(&base, snapshot, &sink_layout)? {
            return Ok(false);
        }
        // (counter, exact) of entry `index` when it differs from `stored`.
        let changed_at = |layout: &PayloadLayout,
                          payload: &[u8],
                          exact: u32,
                          index: usize,
                          counters: &[u32],
                          stored_exact: u32|
         -> Option<(u32, bool)> {
            let counter = layout.counter(payload, index)?;
            let flag = exact >> index & 1 != 0;
            (counter != counters[index] || flag != (stored_exact >> index & 1 != 0))
                .then_some((counter, flag))
        };
        let changed = (0..layout.entries)
            .filter(|&index| {
                changed_at(
                    &layout,
                    payload,
                    snapshot.exact,
                    index,
                    &base.counters,
                    base.exact,
                )
                .is_some()
            })
            .count();
        #[cfg(feature = "green-power-sink")]
        let changed = changed
            + base.sink.map_or(0, |sink| {
                (0..sink_layout.entries)
                    .filter(|&index| {
                        changed_at(
                            &sink_layout,
                            snapshot.sink_payload(),
                            snapshot.sink_exact,
                            index,
                            &sink.counters,
                            sink.exact,
                        )
                        .is_some()
                    })
                    .count()
            });
        if base.append as usize + changed * GP_COUNTER_RECORD_SIZE > SECTOR_SIZE {
            return Ok(false);
        }
        for index in 0..layout.entries {
            let Some(value) = changed_at(
                &layout,
                payload,
                snapshot.exact,
                index,
                &base.counters,
                base.exact,
            ) else {
                continue;
            };
            if !self.write_counter_record(&base, index, layout.tags[index], value)? {
                return Ok(false);
            }
            base.counters[index] = value.0;
            base.exact = set_bit(base.exact, index, value.1);
            base.append += GP_COUNTER_RECORD_SIZE as u16;
            self.current = Some(base);
        }
        #[cfg(feature = "green-power-sink")]
        if let Some(mut sink) = base.sink {
            let sink_payload = snapshot.sink_payload();
            for index in 0..sink_layout.entries {
                let Some(value) = changed_at(
                    &sink_layout,
                    sink_payload,
                    snapshot.sink_exact,
                    index,
                    &sink.counters,
                    sink.exact,
                ) else {
                    continue;
                };
                let unit_index = SINK_COUNTER_INDEX as usize | index;
                if !self.write_counter_record(&base, unit_index, sink_layout.tags[index], value)? {
                    return Ok(false);
                }
                sink.counters[index] = value.0;
                sink.exact = set_bit(sink.exact, index, value.1);
                base.append += GP_COUNTER_RECORD_SIZE as u16;
                base.sink = Some(sink);
                self.current = Some(base);
            }
        }
        Ok(true)
    }

    fn erase_sector(&mut self, sector: usize) -> Result<(), GpStoreError> {
        let start = self.sectors[sector];
        self.storage
            .erase(start, start + SECTOR_SIZE as u32)
            .map_err(|_| GpStoreError::Hardware)
    }

    fn commit(&mut self, snapshot: &PersistentGpProxyTable) -> Result<(), GpStoreError> {
        if !self.scanned {
            self.scan(None)?;
        }
        if self.commit_counters(snapshot)? {
            return Ok(());
        }
        // A full record restores every counter batched; exact ones are
        // re-asserted by counter records right after it, so leave room for
        // them in the same sector.
        let reassert = snapshot.has_exact_counters() && Self::SLOTS_PER_SECTOR > 1;
        self.commit_full(snapshot, reassert)?;
        if reassert {
            // No room (or a failure) leaves them batched, which is safe.
            self.commit_counters(snapshot)?;
        }
        Ok(())
    }

    fn commit_full(
        &mut self,
        snapshot: &PersistentGpProxyTable,
        reassert: bool,
    ) -> Result<(), GpStoreError> {
        #[cfg(feature = "green-power-sink")]
        if snapshot.sink_len() != 0 {
            return self.commit_with_sink(snapshot, reassert);
        }
        let (generation, sector, slot) = match self.current {
            Some(base) => {
                let generation = base
                    .generation
                    .checked_add(1)
                    .ok_or(GpStoreError::GenerationExhausted)?;
                let sector = base.sector as usize;
                match self.first_erased_slot(sector, base.slot as usize + 1)? {
                    Some(slot) if !reassert || slot + 1 < Self::SLOTS_PER_SECTOR => {
                        (generation, sector, slot)
                    }
                    _ => {
                        let target = 1 - sector;
                        self.erase_sector(target)?;
                        (generation, target, 0)
                    }
                }
            }
            None => {
                self.erase_sector(0)?;
                (0, 0, 0)
            }
        };
        self.write_record(sector, slot, generation, snapshot)?;
        let payload = snapshot.payload();
        let layout = PayloadLayout::of(payload)?;
        let mut base = Base {
            generation,
            sector: sector as u8,
            slot: slot as u8,
            len: payload.len() as u16,
            append: ((slot + 1) * GP_PROXY_JOURNAL_SLOT_SIZE) as u16,
            counters: [0; GP_PROXY_TABLE_ENTRIES],
            exact: 0,
            #[cfg(feature = "green-power-sink")]
            sink: None,
        };
        for index in 0..layout.entries {
            base.counters[index] = layout.counter(payload, index).unwrap_or(0);
        }
        self.current = Some(base);
        Ok(())
    }
}

#[cfg(feature = "green-power-sink")]
impl<S: NorFlash, const SECTOR_SIZE: usize> GpProxyJournal<S, SECTOR_SIZE> {
    /// Generation and payload length of a committed, CRC-valid record with
    /// `magic` and a payload of at most `max_len` bytes.
    #[inline(never)]
    fn record_header(
        record: &[u8; GP_PROXY_JOURNAL_SLOT_SIZE],
        magic: [u8; 4],
        max_len: usize,
    ) -> Option<(u32, usize)> {
        if record[0..4] != magic
            || record[4] != RECORD_VERSION
            || record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if len == 0 || len > max_len {
            return None;
        }
        let crc = u32::from_le_bytes([
            record[RECORD_CRC_OFFSET],
            record[RECORD_CRC_OFFSET + 1],
            record[RECORD_CRC_OFFSET + 2],
            record[RECORD_CRC_OFFSET + 3],
        ]);
        if crc32(&record[..RECORD_CRC_OFFSET]) != crc {
            return None;
        }
        Some((
            u32::from_le_bytes([record[8], record[9], record[10], record[11]]),
            len,
        ))
    }

    /// Generation and payload length of a committed, CRC-valid Proxy Table
    /// record.
    fn header(record: &[u8; GP_PROXY_JOURNAL_SLOT_SIZE]) -> Option<(u32, usize)> {
        Self::record_header(record, RECORD_MAGIC, MAX_ENCODED_GP_PROXY_TABLE_LEN)
    }

    /// Generation and payload length of a committed, CRC-valid sink record.
    fn sink_header(record: &[u8; GP_PROXY_JOURNAL_SLOT_SIZE]) -> Option<(u32, usize)> {
        Self::record_header(record, SINK_RECORD_MAGIC, MAX_ENCODED_GP_SINK_TABLE_LEN)
    }

    /// A full Proxy Table record on its own (no sink record before it).
    fn write_record(
        &mut self,
        sector: usize,
        slot: usize,
        generation: u32,
        snapshot: &PersistentGpProxyTable,
    ) -> Result<(), GpStoreError> {
        let epid = snapshot.extended_pan_id;
        self.write_pair_record(
            sector,
            slot,
            (RECORD_MAGIC, 0),
            generation,
            &epid,
            (snapshot.payload(), &[]),
        )
    }

    /// Whether the stored Proxy Table record equals `snapshot` in everything
    /// but the frame counters.
    fn same_structure(
        &mut self,
        base: &Base,
        snapshot: &PersistentGpProxyTable,
        layout: &PayloadLayout,
    ) -> Result<bool, GpStoreError> {
        let address = self.slot_address(base.sector as usize, base.slot as usize);
        let mut epid = [0u8; 8];
        self.storage
            .read(address + RECORD_EPID_OFFSET as u32, &mut epid)
            .map_err(|_| GpStoreError::Hardware)?;
        if epid != snapshot.extended_pan_id {
            return Ok(false);
        }
        self.same_payload(address, base.len, snapshot.payload(), layout)
    }

    /// Whether the record at `address` holds `payload` (stored length
    /// `stored_len`) up to the frame counter bytes of `layout`.
    #[inline(never)]
    fn same_payload(
        &mut self,
        address: u32,
        stored_len: u16,
        payload: &[u8],
        layout: &EntryLayout,
    ) -> Result<bool, GpStoreError> {
        if stored_len as usize != payload.len() {
            return Ok(false);
        }
        let mut chunk = [0u8; 64];
        for (index, expected) in payload.chunks(chunk.len()).enumerate() {
            let start = index * chunk.len();
            let window = &mut chunk[..expected.len()];
            self.storage
                .read(address + (RECORD_PAYLOAD_OFFSET + start) as u32, window)
                .map_err(|_| GpStoreError::Hardware)?;
            let differs = window
                .iter()
                .zip(expected)
                .enumerate()
                .any(|(i, (stored, new))| stored != new && !layout.in_counter(start + i));
            if differs {
                return Ok(false);
            }
        }
        Ok(true)
    }

    /// Load the sink record paired with the Proxy Table record at
    /// (`sector`, `slot`) of `generation`, into `snapshot` when given.
    fn scan_sink(
        &mut self,
        sector: usize,
        slot: usize,
        generation: u32,
        record: &mut [u8; GP_PROXY_JOURNAL_SLOT_SIZE],
        snapshot: &mut Option<&mut PersistentGpProxyTable>,
    ) -> Result<(Option<SinkBase>, EntryLayout), GpStoreError> {
        let empty = EntryLayout {
            entries: 0,
            tags: [0; LAYOUT_ENTRIES],
            counter_at: [0; LAYOUT_ENTRIES],
        };
        if let Some(snapshot) = snapshot.as_deref_mut() {
            snapshot.set_sink_validated(&[0]);
            #[cfg(feature = "green-power-translation-table")]
            snapshot.set_translation_tail(&[]);
        }
        if slot == 0 {
            return Ok((None, empty));
        }
        self.read_slot(sector, slot, record)?;
        if record[7] != RECORD_PAIRED_WITH_SINK {
            return Ok((None, empty));
        }
        self.read_slot(sector, slot - 1, record)?;
        let Some((sink_generation, len)) = Self::sink_header(record) else {
            return Ok((None, empty));
        };
        let payload = &record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + len];
        if sink_generation != generation
            || PersistentGpProxyTable::validate_sink_payload(payload).is_err()
        {
            return Ok((None, empty));
        }
        let layout = EntryLayout::of_sink(payload)?;
        let mut sink = SinkBase {
            slot: (slot - 1) as u8,
            len: len as u16,
            counters: [0; GP_SINK_TABLE_ENTRIES],
            exact: 0,
        };
        for index in 0..layout.entries {
            sink.counters[index] = layout.counter(payload, index).unwrap_or(0);
        }
        if let Some(snapshot) = snapshot.as_deref_mut() {
            snapshot.set_sink_validated(payload);
            #[cfg(feature = "green-power-translation-table")]
            snapshot.set_translation_tail(translations::stored_tail(record, len));
        }
        Ok((Some(sink), layout))
    }

    /// Whether the stored sink record equals the snapshot's Sink Table in
    /// everything but the frame counters (no sink record: an empty table).
    fn same_sink_structure(
        &mut self,
        base: &Base,
        snapshot: &PersistentGpProxyTable,
        layout: &EntryLayout,
    ) -> Result<bool, GpStoreError> {
        let Some(sink) = base.sink else {
            return Ok(snapshot.sink_len() == 0);
        };
        let address = self.slot_address(base.sector as usize, sink.slot as usize);
        #[cfg(feature = "green-power-translation-table")]
        if !self.same_translation_tail(address, sink.len, snapshot)? {
            return Ok(false);
        }
        self.same_payload(address, sink.len, snapshot.sink_payload(), layout)
    }

    /// Write one record of a pair (full-record layout; `kind` is the magic
    /// and byte 7), commit marker last, verified by read-back. `tail`
    /// follows the payload inside the CRC-covered record (a sink record's
    /// translation section); the header length counts the payload only.
    fn write_pair_record(
        &mut self,
        sector: usize,
        slot: usize,
        (magic, flags): ([u8; 4], u8),
        generation: u32,
        extended_pan_id: &IeeeAddress,
        (payload, tail): (&[u8], &[u8]),
    ) -> Result<(), GpStoreError> {
        let mut record = [0xFFu8; GP_PROXY_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&magic);
        record[4] = RECORD_VERSION;
        record[5..7].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        record[7] = flags;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        record[RECORD_EPID_OFFSET..RECORD_EPID_OFFSET + 8].copy_from_slice(extended_pan_id);
        record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + payload.len()]
            .copy_from_slice(payload);
        let tail_at = RECORD_PAYLOAD_OFFSET + payload.len();
        record[tail_at..tail_at + tail.len()].copy_from_slice(tail);
        let crc = crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        let address = self.slot_address(sector, slot);
        self.storage
            .write(address, &record[..RECORD_COMMIT_OFFSET])
            .map_err(|_| GpStoreError::Hardware)?;
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + RECORD_COMMIT_OFFSET as u32,
                &record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4],
            )
            .map_err(|_| GpStoreError::Hardware)?;
        let mut chunk = [0u8; 64];
        for (index, expected) in record.chunks(chunk.len()).enumerate() {
            let at = address + (index * chunk.len()) as u32;
            let window = &mut chunk[..expected.len()];
            self.storage
                .read(at, window)
                .map_err(|_| GpStoreError::Hardware)?;
            if window != expected {
                return Err(GpStoreError::Hardware);
            }
        }
        Ok(())
    }

    /// First slot `s >= from` of `sector` with slots `s` and `s + 1` erased.
    fn erased_slot_pair(
        &mut self,
        sector: usize,
        from: usize,
    ) -> Result<Option<usize>, GpStoreError> {
        let mut slot = from;
        while slot + 1 < Self::SLOTS_PER_SECTOR {
            let Some(first) = self.first_erased_slot(sector, slot)? else {
                return Ok(None);
            };
            if first + 1 >= Self::SLOTS_PER_SECTOR {
                return Ok(None);
            }
            if self.first_erased_slot(sector, first + 1)? == Some(first + 1) {
                return Ok(Some(first));
            }
            slot = first + 1;
        }
        Ok(None)
    }

    /// Full commit of a snapshot with Sink Table entries: sink record, then
    /// the Proxy Table record of the same generation in the next slot.
    fn commit_with_sink(
        &mut self,
        snapshot: &PersistentGpProxyTable,
        reassert: bool,
    ) -> Result<(), GpStoreError> {
        if Self::SLOTS_PER_SECTOR < 2 {
            return Err(GpStoreError::Hardware);
        }
        let (generation, sector, slot) = match self.current {
            Some(base) => {
                let generation = base
                    .generation
                    .checked_add(1)
                    .ok_or(GpStoreError::GenerationExhausted)?;
                let sector = base.sector as usize;
                match self.erased_slot_pair(sector, base.slot as usize + 1)? {
                    Some(slot) if !reassert || slot + 2 < Self::SLOTS_PER_SECTOR => {
                        (generation, sector, slot)
                    }
                    _ => {
                        let target = 1 - sector;
                        self.erase_sector(target)?;
                        (generation, target, 0)
                    }
                }
            }
            None => {
                self.erase_sector(0)?;
                (0, 0, 0)
            }
        };
        let epid = snapshot.extended_pan_id;
        self.write_pair_record(
            sector,
            slot,
            (SINK_RECORD_MAGIC, sink_record_flags(snapshot)),
            generation,
            &epid,
            (snapshot.sink_payload(), sink_record_tail(snapshot)),
        )?;
        self.write_pair_record(
            sector,
            slot + 1,
            (RECORD_MAGIC, RECORD_PAIRED_WITH_SINK),
            generation,
            &epid,
            (snapshot.payload(), &[]),
        )?;
        let payload = snapshot.payload();
        let layout = PayloadLayout::of(payload)?;
        let sink_payload = snapshot.sink_payload();
        let sink_layout = EntryLayout::of_sink(sink_payload)?;
        let mut base = Base {
            generation,
            sector: sector as u8,
            slot: (slot + 1) as u8,
            len: payload.len() as u16,
            append: ((slot + 2) * GP_PROXY_JOURNAL_SLOT_SIZE) as u16,
            counters: [0; GP_PROXY_TABLE_ENTRIES],
            exact: 0,
            sink: Some(SinkBase {
                slot: slot as u8,
                len: sink_payload.len() as u16,
                counters: [0; GP_SINK_TABLE_ENTRIES],
                exact: 0,
            }),
        };
        for index in 0..layout.entries {
            base.counters[index] = layout.counter(payload, index).unwrap_or(0);
        }
        if let Some(sink) = base.sink.as_mut() {
            for index in 0..sink_layout.entries {
                sink.counters[index] = sink_layout.counter(sink_payload, index).unwrap_or(0);
            }
        }
        self.current = Some(base);
        Ok(())
    }
}

/// Sink record byte 7: the record carries the translation section after
/// the Sink Table payload (`green-power-translation-table`). Zero, and not
/// checked, in journals written before it.
#[cfg(feature = "green-power-sink")]
const SINK_RECORD_HAS_TRANSLATIONS: u8 = 0x02;
/// Capacity of [`PersistentGpProxyTable`]'s Sink Table buffer: the payload
/// and, with `green-power-translation-table`, the translation section
/// behind it.
#[cfg(feature = "green-power-sink")]
const SINK_PAYLOAD_CAPACITY: usize = MAX_ENCODED_GP_SINK_TABLE_LEN + SINK_TAIL_CAPACITY;
#[cfg(all(
    feature = "green-power-sink",
    not(feature = "green-power-translation-table")
))]
const SINK_TAIL_CAPACITY: usize = 0;
#[cfg(feature = "green-power-translation-table")]
const SINK_TAIL_CAPACITY: usize = translations::MAX_TAIL_LEN;
#[cfg(feature = "green-power-translation-table")]
const _: () = assert!(
    RECORD_PAYLOAD_OFFSET + MAX_ENCODED_GP_SINK_TABLE_LEN + SINK_TAIL_CAPACITY <= RECORD_CRC_OFFSET
);

/// Byte 7 of a snapshot's sink record.
#[cfg(feature = "green-power-sink")]
fn sink_record_flags(snapshot: &PersistentGpProxyTable) -> u8 {
    if sink_record_tail(snapshot).is_empty() {
        0
    } else {
        SINK_RECORD_HAS_TRANSLATIONS
    }
}

/// The translation section written after a snapshot's Sink Table payload.
#[cfg(all(
    feature = "green-power-sink",
    not(feature = "green-power-translation-table")
))]
fn sink_record_tail(_: &PersistentGpProxyTable) -> &[u8] {
    &[]
}

#[cfg(feature = "green-power-translation-table")]
fn sink_record_tail(snapshot: &PersistentGpProxyTable) -> &[u8] {
    snapshot.translation_tail()
}

impl<S: NorFlash, const SECTOR_SIZE: usize> GpProxyStore for GpProxyJournal<S, SECTOR_SIZE> {
    fn load_into(&mut self, snapshot: &mut PersistentGpProxyTable) -> Result<bool, GpStoreError> {
        self.scan(Some(snapshot))
    }

    fn store(&mut self, snapshot: &PersistentGpProxyTable) -> Result<(), GpStoreError> {
        let result = self.commit(snapshot);
        if result.is_err() {
            // Rescan before the next attempt instead of trusting the cache.
            self.scanned = false;
            self.current = None;
        }
        result
    }
}

impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// Restore the Proxy Table (and, with `green-power-sink`, the Sink Table)
    /// for the joined network and re-establish the GPEP group membership they
    /// require. Returns the number of restored entries of both tables.
    ///
    /// Call once after the device restored its network state. An empty or
    /// missing snapshot leaves an empty table. A non-empty snapshot from
    /// another extended PAN ID is rejected with
    /// [`GpStoreError::ForeignNetwork`]; the caller then clears it with
    /// [`clear_persisted_gp_proxy_table`](Self::clear_persisted_gp_proxy_table).
    pub fn restore_gp_proxy_table<S: GpProxyStore + ?Sized>(
        &mut self,
        store: &mut S,
    ) -> Result<usize, GpStoreError> {
        let mut snapshot = PersistentGpProxyTable::default();
        self.green_power.clear();
        if !store.load_into(&mut snapshot)? || snapshot.is_empty() {
            self.gp_take_dirty();
            return Ok(0);
        }
        let extended_pan_id = self.bdb.zdo().nwk().nib().extended_pan_id;
        if snapshot.extended_pan_id() != extended_pan_id {
            self.gp_take_dirty();
            return Err(GpStoreError::ForeignNetwork);
        }
        let mut restored = 0;
        for entry in snapshot.entries() {
            let entry = entry?;
            self.gp_proxy_mut()
                .restore(entry)
                .map_err(|_| GpStoreError::Full)?;
            restored += 1;
        }
        #[cfg(feature = "green-power-sink")]
        for entry in snapshot.sink_entries() {
            let entry = entry?;
            self.gp_sink_mut()
                .restore(entry)
                .map_err(|_| GpStoreError::Full)?;
            restored += 1;
        }
        #[cfg(feature = "green-power-translation-table")]
        snapshot.restore_translations(self.gp_sink_mut())?;
        self.gp_restore_groups();
        self.gp_take_dirty();
        Ok(restored)
    }

    /// Persist the Proxy Table when the proxy reported a change.
    ///
    /// Returns `Ok(true)` when a snapshot was written. A failed write keeps
    /// the table dirty so the next call retries it.
    pub fn save_gp_proxy_table_if_dirty<S: GpProxyStore + ?Sized>(
        &mut self,
        store: &mut S,
    ) -> Result<bool, GpStoreError> {
        if !self.gp_take_dirty() {
            return Ok(false);
        }
        let extended_pan_id = self.bdb.zdo().nwk().nib().extended_pan_id;
        #[cfg(not(feature = "green-power-sink"))]
        let result = PersistentGpProxyTable::capture(extended_pan_id, self.gp_proxy().entries())
            .and_then(|snapshot| store.store(&snapshot));
        #[cfg(all(
            feature = "green-power-sink",
            not(feature = "green-power-translation-table")
        ))]
        let result = PersistentGpProxyTable::capture(extended_pan_id, self.gp_proxy().entries())
            .and_then(|snapshot| snapshot.with_sink(self.gp_sink().entries()))
            .and_then(|snapshot| store.store(&snapshot));
        #[cfg(feature = "green-power-translation-table")]
        let result = PersistentGpProxyTable::capture(extended_pan_id, self.gp_proxy().entries())
            .and_then(|snapshot| snapshot.with_sink(self.gp_sink().entries()))
            .and_then(|mut snapshot| {
                snapshot.set_translations(self.gp_sink())?;
                store.store(&snapshot)
            });
        if result.is_err() {
            self.gp_mark_dirty();
        }
        result.map(|()| true)
    }

    /// Clear both the live and the durable Proxy Table.
    pub fn clear_persisted_gp_proxy_table<S: GpProxyStore + ?Sized>(
        &mut self,
        store: &mut S,
    ) -> Result<(), GpStoreError> {
        self.green_power.clear();
        store.store(&PersistentGpProxyTable::empty([0; 8]))?;
        self.gp_take_dirty();
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use zigbee_gp::types::{GpdId, KeyType, SecurityLevel};

    const EPID: IeeeAddress = [0x33; 8];
    const SECTOR: usize = GP_PROXY_JOURNAL_SECTOR_SIZE;

    fn entry(src_id: u32, counter: u32) -> ProxyEntry {
        let mut entry = ProxyEntry::new(GpdId::SrcId(src_id));
        entry.derived_group = true;
        entry.security_level = SecurityLevel::Encrypted;
        entry.key_type = KeyType::IndividualKey;
        entry.key = Some([0xA5; 16]);
        entry.frame_counter = counter;
        entry.groupcast_radius = 0xFF;
        entry
    }

    /// `entry` with a structural (non-counter) difference.
    fn entry_radius(src_id: u32, counter: u32, radius: u8) -> ProxyEntry {
        let mut entry = entry(src_id, counter);
        entry.groupcast_radius = radius;
        entry
    }

    fn capture(entries: &[ProxyEntry]) -> PersistentGpProxyTable {
        PersistentGpProxyTable::capture(EPID, entries).unwrap()
    }

    #[derive(Debug)]
    pub(crate) struct FlashError;

    impl embedded_storage::nor_flash::NorFlashError for FlashError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    pub(crate) struct MockFlash {
        data: [u8; 2 * SECTOR],
        /// Successful program operations left before every write fails.
        programs_left: Option<usize>,
        erases: usize,
    }

    impl MockFlash {
        pub(crate) fn new() -> Self {
            Self {
                data: [0xFF; 2 * SECTOR],
                programs_left: None,
                erases: 0,
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = FlashError;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError> {
            let start = offset as usize;
            bytes.copy_from_slice(
                self.data
                    .get(start..start + bytes.len())
                    .ok_or(FlashError)?,
            );
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = SECTOR;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
            self.erases += 1;
            self.data
                .get_mut(from as usize..to as usize)
                .ok_or(FlashError)?
                .fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
            if let Some(left) = self.programs_left.as_mut() {
                if *left == 0 {
                    return Err(FlashError);
                }
                *left -= 1;
            }
            let start = offset as usize;
            let target = self
                .data
                .get_mut(start..start + bytes.len())
                .ok_or(FlashError)?;
            for (cell, byte) in target.iter_mut().zip(bytes) {
                // NOR programming only clears bits.
                *cell &= *byte;
            }
            Ok(())
        }
    }

    fn journal(flash: MockFlash) -> GpProxyJournal<MockFlash> {
        GpProxyJournal::new(flash, 0, SECTOR as u32)
    }

    fn load(journal: &mut GpProxyJournal<MockFlash>) -> Option<PersistentGpProxyTable> {
        let mut snapshot = PersistentGpProxyTable::default();
        journal
            .load_into(&mut snapshot)
            .unwrap()
            .then_some(snapshot)
    }

    fn counters(snapshot: &PersistentGpProxyTable) -> heapless::Vec<u32, 8> {
        snapshot
            .entries()
            .map(|entry| entry.unwrap().frame_counter)
            .collect()
    }

    fn exact_entry(src_id: u32, counter: u32) -> ProxyEntry {
        entry(src_id, counter).with_counter_exact(true)
    }

    fn exact_bits(snapshot: &PersistentGpProxyTable) -> heapless::Vec<bool, 8> {
        snapshot
            .entries()
            .map(|entry| entry.unwrap().counter_exact())
            .collect()
    }

    /// The exact-counter flag survives a full record (re-asserted by a
    /// counter record right after it), counter records and a reboot; a
    /// flag-only change is a counter record, not a full record.
    #[test]
    fn exact_counter_flags_round_trip_through_the_journal() {
        let mut journal = journal(MockFlash::new());
        let first = capture(&[exact_entry(1, 10), entry(2, 20)]);
        assert_eq!(first.exact_counters(), 0b01);
        journal.store(&first).unwrap();
        // Full record in slot 0, then one re-assert counter record.
        let slot = GP_PROXY_JOURNAL_SLOT_SIZE;
        assert_eq!(journal.storage.data[slot..slot + 4], *b"ZBGC");
        assert_eq!(
            journal.storage.data[slot + COUNTER_FLAGS_OFFSET],
            COUNTER_FLAG_EXACT
        );
        assert!(
            journal.storage.data[slot + GP_COUNTER_RECORD_SIZE..2 * slot]
                .iter()
                .all(|b| *b == 0xFF)
        );
        let mut rebooted =
            GpProxyJournal::<_, SECTOR>::new(journal.into_storage(), 0, SECTOR as u32);
        let loaded = load(&mut rebooted).unwrap();
        assert_eq!(exact_bits(&loaded).as_slice(), &[true, false]);
        assert_eq!(counters(&loaded).as_slice(), &[10, 20]);

        // The switch is pressed (exact, counter record); the sensor leaves
        // nothing to write; then the switch falls back to batched: a
        // flag-only change is one more counter record.
        rebooted
            .store(&capture(&[exact_entry(1, 11), entry(2, 20)]))
            .unwrap();
        rebooted
            .store(&capture(&[entry(1, 11), entry(2, 20)]))
            .unwrap();
        let data = &rebooted.storage.data;
        assert_eq!(data[slot + 2 * GP_COUNTER_RECORD_SIZE..][..4], *b"ZBGC");
        assert_eq!(
            data[slot + 2 * GP_COUNTER_RECORD_SIZE + COUNTER_FLAGS_OFFSET],
            0
        );
        assert_eq!(rebooted.storage.erases, 1);
        let mut again = GpProxyJournal::<_, SECTOR>::new(rebooted.into_storage(), 0, SECTOR as u32);
        let loaded = load(&mut again).unwrap();
        assert_eq!(exact_bits(&loaded).as_slice(), &[false, false]);
        assert_eq!(counters(&loaded).as_slice(), &[11, 20]);
        // Unchanged: nothing written.
        let before = again.storage.data;
        again
            .store(&capture(&[entry(1, 11), entry(2, 20)]))
            .unwrap();
        assert_eq!(again.storage.data, before);
    }

    /// A journal without exact flags (the original format), and a torn
    /// re-assert record after a full record, both restore batched, which
    /// is always safe.
    #[test]
    fn missing_or_torn_exact_flags_restore_batched() {
        let legacy = capture(&[entry(1, 10)]);
        let mut flash = MockFlash::new();
        flash.data[..1024].copy_from_slice(&legacy_record(4, &legacy));
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(
            exact_bits(&load(&mut journal).unwrap()).as_slice(),
            &[false]
        );

        // Power fails after the full record's commit, before the re-assert
        // record's commit marker.
        let mut flash = MockFlash::new();
        // Erase, then: full record body + commit, re-assert body only.
        flash.programs_left = Some(3);
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let _ = journal.store(&capture(&[exact_entry(1, 10)]));
        let mut flash = journal.into_storage();
        flash.programs_left = None;
        let mut rebooted = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let loaded = load(&mut rebooted).unwrap();
        assert_eq!(counters(&loaded).as_slice(), &[10]);
        assert_eq!(exact_bits(&loaded).as_slice(), &[false]);
    }

    /// A full record never lands in a sector's last slot while an exact
    /// flag must follow it: the journal rolls over instead.
    #[test]
    fn full_record_leaves_room_for_the_exact_flags() {
        let mut journal = journal(MockFlash::new());
        // Fill slots 0..=2 with structural changes of a batched table.
        for radius in 0..3 {
            journal
                .store(&capture(&[entry_radius(1, 10, radius)]))
                .unwrap();
        }
        assert_eq!(journal.current.unwrap().slot, 2);
        // Slot 3 is the last one: an exact entry rolls over to sector 1.
        journal
            .store(&capture(&[exact_entry(1, 10).clone()]).clone())
            .unwrap();
        let mut changed = exact_entry(1, 10);
        changed.groupcast_radius = 9;
        journal.store(&capture(&[changed])).unwrap();
        let base = journal.current.unwrap();
        assert_eq!(base.sector, 1);
        assert_eq!(base.exact, 0b1);
        let mut rebooted =
            GpProxyJournal::<_, SECTOR>::new(journal.into_storage(), 0, SECTOR as u32);
        assert_eq!(
            exact_bits(&load(&mut rebooted).unwrap()).as_slice(),
            &[true]
        );
    }

    #[test]
    fn snapshot_round_trips_every_entry() {
        let entries = [entry(0x1111_1111, 7), entry(0x2222_2222, 9)];
        let snapshot = PersistentGpProxyTable::capture(EPID, &entries).unwrap();
        let parsed = PersistentGpProxyTable::from_parts(EPID, snapshot.payload()).unwrap();
        assert_eq!(parsed.len(), 2);
        let decoded: heapless::Vec<ProxyEntry, 2> = parsed.entries().map(Result::unwrap).collect();
        assert_eq!(decoded[0].gpd_id, GpdId::SrcId(0x1111_1111));
        assert_eq!(decoded[1].key, Some([0xA5; 16]));
        assert_eq!(decoded[1].frame_counter, 9);
    }

    #[test]
    fn maximal_table_fits_one_slot() {
        let entries: heapless::Vec<ProxyEntry, GP_PROXY_TABLE_ENTRIES> = (0
            ..GP_PROXY_TABLE_ENTRIES as u32)
            .map(|i| entry(0x1000 + i, i))
            .collect();
        let snapshot = PersistentGpProxyTable::capture(EPID, &entries).unwrap();
        assert_eq!(snapshot.len(), GP_PROXY_TABLE_ENTRIES);
        let mut journal = journal(MockFlash::new());
        journal.store(&snapshot).unwrap();
        assert_eq!(load(&mut journal).unwrap(), snapshot);
    }

    #[test]
    fn malformed_payload_is_rejected() {
        let snapshot = PersistentGpProxyTable::capture(EPID, &[entry(1, 1)]).unwrap();
        let mut bytes = [0u8; MAX_ENCODED_GP_PROXY_TABLE_LEN];
        let len = snapshot.payload().len();
        bytes[..len].copy_from_slice(snapshot.payload());
        // Count claims a second entry that is not there.
        bytes[0] = 2;
        assert_eq!(
            PersistentGpProxyTable::from_parts(EPID, &bytes[..len]),
            Err(GpStoreError::Corrupt)
        );
        // Trailing garbage after the declared entries.
        bytes[0] = 1;
        assert_eq!(
            PersistentGpProxyTable::from_parts(EPID, &bytes[..len + 1]),
            Err(GpStoreError::Corrupt)
        );
        // Entry length byte disagreeing with the entry encoding.
        bytes[1] -= 1;
        assert_eq!(
            PersistentGpProxyTable::from_parts(EPID, &bytes[..len]),
            Err(GpStoreError::Corrupt)
        );
    }

    #[test]
    fn erased_flash_loads_nothing() {
        let mut journal = journal(MockFlash::new());
        assert!(load(&mut journal).is_none());
    }

    #[test]
    fn newest_generation_wins_across_rollover() {
        let mut journal = journal(MockFlash::new());
        let slots = 2 * (SECTOR / GP_PROXY_JOURNAL_SLOT_SIZE) + 3;
        for counter in 0..slots as u32 {
            // A structural change each time: one full record per store.
            journal
                .store(&capture(&[entry_radius(1, counter, counter as u8)]))
                .unwrap();
        }
        // A fresh journal over the same flash finds the newest record.
        let flash = journal.into_storage();
        assert!(flash.erases >= 2);
        let mut reopened = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let loaded = load(&mut reopened).unwrap();
        assert_eq!(counters(&loaded).as_slice(), &[slots as u32 - 1]);
    }

    #[test]
    fn interrupted_write_keeps_previous_snapshot() {
        let mut journal = journal(MockFlash::new());
        let first = PersistentGpProxyTable::capture(EPID, &[entry(1, 5)]).unwrap();
        journal.store(&first).unwrap();
        let mut flash = journal.into_storage();
        // The body is programmed, the commit marker is not.
        flash.programs_left = Some(1);
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let second = capture(&[entry_radius(1, 6, 7)]);
        assert_eq!(journal.store(&second), Err(GpStoreError::Hardware));
        let mut flash = journal.into_storage();
        flash.programs_left = None;
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(load(&mut journal).unwrap(), first);
        // The torn slot is skipped by the next commit.
        journal.store(&second).unwrap();
        assert_eq!(load(&mut journal).unwrap(), second);
    }

    #[test]
    fn corrupted_crc_falls_back_to_older_record() {
        let mut journal = journal(MockFlash::new());
        let first = PersistentGpProxyTable::capture(EPID, &[entry(1, 5)]).unwrap();
        let second = capture(&[entry_radius(1, 6, 7)]);
        journal.store(&first).unwrap();
        journal.store(&second).unwrap();
        let mut flash = journal.into_storage();
        // Flip a payload bit of the second slot.
        flash.data[GP_PROXY_JOURNAL_SLOT_SIZE + RECORD_PAYLOAD_OFFSET + 4] ^= 0x01;
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(load(&mut journal).unwrap(), first);
    }

    #[test]
    fn exhausted_generation_is_reported() {
        let mut journal = journal(MockFlash::new());
        let snapshot = PersistentGpProxyTable::capture(EPID, &[entry(1, 5)]).unwrap();
        journal.store(&snapshot).unwrap();
        let mut flash = journal.into_storage();
        // Rewrite the record with the last generation and a valid CRC.
        let mut record = [0u8; GP_PROXY_JOURNAL_SLOT_SIZE];
        record.copy_from_slice(&flash.data[..GP_PROXY_JOURNAL_SLOT_SIZE]);
        record[8..12].copy_from_slice(&u32::MAX.to_le_bytes());
        let crc = crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        flash.data[..GP_PROXY_JOURNAL_SLOT_SIZE].copy_from_slice(&record);
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(
            journal.store(&capture(&[entry_radius(1, 5, 7)])),
            Err(GpStoreError::GenerationExhausted)
        );
        // The stored record is still readable.
        assert_eq!(load(&mut journal).unwrap(), snapshot);
        // Counter records do not consume generations.
        journal.store(&capture(&[entry(1, 70)])).unwrap();
        assert_eq!(counters(&load(&mut journal).unwrap()).as_slice(), &[70]);
    }

    /// Byte offset of counter record `n` after a full record in slot 0.
    const fn unit(n: usize) -> usize {
        GP_PROXY_JOURNAL_SLOT_SIZE + n * GP_COUNTER_RECORD_SIZE
    }

    /// A full record exactly as the original (counter-record-less) journal
    /// wrote it, built from the format description rather than the code.
    fn legacy_record(generation: u32, snapshot: &PersistentGpProxyTable) -> [u8; 1024] {
        let payload = snapshot.payload();
        let mut record = [0xFFu8; 1024];
        record[0..4].copy_from_slice(b"ZBGP");
        record[4] = 1;
        record[5..7].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        record[12..20].copy_from_slice(&snapshot.extended_pan_id());
        record[20..20 + payload.len()].copy_from_slice(payload);
        let crc = crc32(&record[..1012]);
        record[1012..1016].copy_from_slice(&crc.to_le_bytes());
        record[1016..1020].copy_from_slice(b"CMIT");
        record
    }

    #[test]
    fn counter_updates_append_compact_records_until_the_sector_is_full() {
        let mut journal = journal(MockFlash::new());
        journal
            .store(&capture(&[entry(1, 100), entry(2, 200)]))
            .unwrap();
        let mut full = [0u8; GP_PROXY_JOURNAL_SLOT_SIZE];
        full.copy_from_slice(&journal.storage.data[..GP_PROXY_JOURNAL_SLOT_SIZE]);
        let per_sector = (SECTOR - GP_PROXY_JOURNAL_SLOT_SIZE) / GP_COUNTER_RECORD_SIZE;
        assert_eq!(per_sector, 96);
        for step in 1..=per_sector as u32 {
            journal
                .store(&capture(&[entry(1, 100 + 64 * step), entry(2, 200)]))
                .unwrap();
        }
        let flash = journal.into_storage();
        // No erase beyond the first, the full record is untouched and only
        // the changed entry got records.
        assert_eq!(flash.erases, 1);
        assert_eq!(flash.data[..GP_PROXY_JOURNAL_SLOT_SIZE], full);
        assert_eq!(flash.data[unit(0)..unit(0) + 4], *b"ZBGC");
        assert_eq!(flash.data[unit(0) + 5], 0, "entry index");
        assert!(flash.data[SECTOR..].iter().all(|b| *b == 0xFF));
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let last = 100 + 64 * per_sector as u32;
        assert_eq!(
            counters(&load(&mut journal).unwrap()).as_slice(),
            &[last, 200]
        );
        // The next update has no room: a full record in the other sector.
        journal
            .store(&capture(&[entry(1, last + 64), entry(2, 200)]))
            .unwrap();
        assert_eq!(journal.storage.erases, 2);
        assert_eq!(journal.storage.data[SECTOR..SECTOR + 4], *b"ZBGP");
        let flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(
            counters(&load(&mut journal).unwrap()).as_slice(),
            &[last + 64, 200]
        );
    }

    #[test]
    fn unchanged_snapshot_is_not_written_again() {
        let mut journal = journal(MockFlash::new());
        let snapshot = capture(&[entry(1, 100)]);
        journal.store(&snapshot).unwrap();
        let before = journal.storage.data;
        journal.store(&snapshot).unwrap();
        // Also after a reboot, and with counter records applied.
        let flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        journal.store(&snapshot).unwrap();
        assert_eq!(journal.storage.data, before);
        let moved = capture(&[entry(1, 164)]);
        journal.store(&moved).unwrap();
        let before = journal.storage.data;
        let flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        journal.store(&moved).unwrap();
        assert_eq!(journal.storage.data, before);
    }

    #[test]
    fn structural_change_writes_a_full_record_and_drops_older_counter_records() {
        let mut journal = journal(MockFlash::new());
        journal.store(&capture(&[entry(1, 100)])).unwrap();
        for counter in [164, 228, 292] {
            journal.store(&capture(&[entry(1, counter)])).unwrap();
        }
        // A sink re-pairs the GPD with a lower counter and a new radius.
        journal.store(&capture(&[entry_radius(1, 150, 9)])).unwrap();
        // Counter records filled part of slot 1: the full record is in slot 2.
        assert_eq!(
            journal.storage.data[2 * GP_PROXY_JOURNAL_SLOT_SIZE..][..4],
            *b"ZBGP"
        );
        let flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let loaded = load(&mut journal).unwrap();
        assert_eq!(counters(&loaded).as_slice(), &[150]);
        assert_eq!(
            loaded.entries().next().unwrap().unwrap().groupcast_radius,
            9
        );
        // New counter records bind to the new full record.
        journal.store(&capture(&[entry_radius(1, 214, 9)])).unwrap();
        assert_eq!(
            journal.storage.data[3 * GP_PROXY_JOURNAL_SLOT_SIZE..][..4],
            *b"ZBGC"
        );
        let flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(counters(&load(&mut journal).unwrap()).as_slice(), &[214]);
    }

    #[test]
    fn torn_or_corrupt_counter_records_are_skipped() {
        let mut journal = journal(MockFlash::new());
        journal.store(&capture(&[entry(1, 100)])).unwrap();
        journal.store(&capture(&[entry(1, 164)])).unwrap();
        // Power loss after the body, before the commit marker.
        journal.storage.programs_left = Some(1);
        assert_eq!(
            journal.store(&capture(&[entry(1, 228)])),
            Err(GpStoreError::Hardware)
        );
        let mut flash = journal.into_storage();
        flash.programs_left = None;
        assert_ne!(flash.data[unit(1)], 0xFF, "torn record body programmed");
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(counters(&load(&mut journal).unwrap()).as_slice(), &[164]);
        // The retry appends after the torn unit.
        journal.store(&capture(&[entry(1, 228)])).unwrap();
        assert_eq!(journal.storage.data[unit(2)..unit(2) + 4], *b"ZBGC");
        let mut flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(counters(&load(&mut journal).unwrap()).as_slice(), &[228]);
        // A bit flip in the newest record falls back to the previous one.
        flash = journal.into_storage();
        flash.data[unit(2) + 16] ^= 0x01;
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(counters(&load(&mut journal).unwrap()).as_slice(), &[164]);
    }

    #[test]
    fn counter_records_for_another_generation_index_or_gpd_are_ignored() {
        let mut journal = journal(MockFlash::new());
        journal
            .store(&capture(&[entry(1, 100), entry(2, 200)]))
            .unwrap();
        let mut base = journal.current.unwrap();
        // Wrong GPD tag for entry 0.
        let tag = crc32(&1u32.to_le_bytes());
        assert!(
            journal
                .write_counter_record(&base, 0, tag ^ 1, (999, false))
                .unwrap()
        );
        base.append += GP_COUNTER_RECORD_SIZE as u16;
        // Entry index past the table.
        assert!(
            journal
                .write_counter_record(&base, 2, tag, (999, false))
                .unwrap()
        );
        base.append += GP_COUNTER_RECORD_SIZE as u16;
        // Older generation.
        base.generation = base.generation.wrapping_sub(1);
        assert!(
            journal
                .write_counter_record(&base, 0, tag, (999, false))
                .unwrap()
        );
        base.append += GP_COUNTER_RECORD_SIZE as u16;
        // The matching record still applies.
        base.generation = base.generation.wrapping_add(1);
        assert!(
            journal
                .write_counter_record(&base, 0, tag, (333, false))
                .unwrap()
        );
        let flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(
            counters(&load(&mut journal).unwrap()).as_slice(),
            &[333, 200]
        );
    }

    #[test]
    fn legacy_full_record_journal_loads_and_takes_counter_records() {
        // A journal as written before counter records existed (both the
        // ZBT-1 and the H2-Zero GP partitions hold this format): an older
        // generation in sector 0, the two newest in sector 1.
        let old = capture(&[entry(1, 10)]);
        let current = capture(&[entry(1, 74), entry_radius(2, 5, 3)]);
        let mut flash = MockFlash::new();
        flash.data[..1024].copy_from_slice(&legacy_record(3, &old));
        flash.data[SECTOR..SECTOR + 1024].copy_from_slice(&legacy_record(4, &old));
        flash.data[SECTOR + 1024..SECTOR + 2048].copy_from_slice(&legacy_record(5, &current));
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(load(&mut journal).unwrap(), current);
        // A counter-only update appends a compact record after it.
        journal
            .store(&capture(&[entry(1, 138), entry_radius(2, 5, 3)]))
            .unwrap();
        let at = SECTOR + 2 * GP_PROXY_JOURNAL_SLOT_SIZE;
        assert_eq!(journal.storage.data[at..at + 4], *b"ZBGC");
        assert_eq!(journal.storage.erases, 0);
        // The full record is untouched, so older firmware still restores
        // it (with its counter), and its slot search skips the used slot.
        assert_eq!(
            journal.storage.data[SECTOR + 1024..SECTOR + 2048],
            legacy_record(5, &current)
        );
        assert_eq!(journal.first_erased_slot(1, 0).unwrap(), Some(3));
        let flash = journal.into_storage();
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(counters(&load(&mut journal).unwrap()).as_slice(), &[138, 5]);
        // Without the compact record (older firmware's view) the full
        // record's counter remains.
        let mut flash = journal.into_storage();
        flash.data[at..at + GP_COUNTER_RECORD_SIZE].fill(0);
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(counters(&load(&mut journal).unwrap()).as_slice(), &[74, 5]);
    }

    /// End to end: a paired proxy persists through counter records, reboots
    /// from them with the counter skipped ahead, and rejects every frame it
    /// accepted before the reboot.
    #[test]
    fn reboot_from_counter_records_skips_ahead_and_rejects_replays() {
        use zigbee_gp::cluster::{GppGpdLink, Pairing, PairingSink, client_cmd};
        use zigbee_gp::frame::{FrameType, GpdfParams, parse, parse_mac};
        use zigbee_gp::proxy::{
            DEFAULT_COUNTER_PERSIST_STRIDE, DropReason, GpdfOutcome, Proxy, ProxyConfig,
        };
        use zigbee_gp::security::build_gpdf;
        use zigbee_gp::types::{CommunicationMode, cmd};

        type P = Proxy<GP_PROXY_TABLE_ENTRIES, 4>;
        const SRC: u32 = 0x293C_9879;
        const KEY: [u8; 16] = [0x5C; 16];
        const S: u32 = DEFAULT_COUNTER_PERSIST_STRIDE;
        // Batched persistence only (exact writes are covered separately).
        const BATCHED: ProxyConfig = ProxyConfig {
            exact_counter_burst: 0,
            ..ProxyConfig::DEFAULT
        };
        let mut aes = zigbee_mac::mock::MockMac::new([0; 8]);

        let mut gpdf = |p: &mut P, fc: u32, now_ms: u32| {
            let params = GpdfParams {
                frame_type: FrameType::Data,
                auto_commissioning: false,
                src_id: Some(SRC),
                security_level: SecurityLevel::Encrypted,
                individual_key: false,
                rx_after_tx: false,
                to_gpd: false,
                frame_counter: fc,
            };
            let mut out = [0u8; 64];
            let n = build_gpdf(
                &mut aes,
                Some(&KEY),
                &params,
                fc as u8,
                cmd::OFF,
                &[],
                &mut out,
            )
            .unwrap();
            let mac = parse_mac(&out[..n]).unwrap();
            let frame = parse(&out[mac.payload_offset..n], fc as u8, None).unwrap();
            let outcome = p.on_gpdf(&mut aes, &frame, GppGpdLink(0), now_ms);
            // Drain the tunnelled notification.
            while p.poll(now_ms + 200).is_some() {}
            outcome
        };
        let mut journal = journal(MockFlash::new());
        let persist = |p: &mut P, journal: &mut GpProxyJournal<MockFlash>| {
            if p.take_dirty() {
                journal.store(&capture(p.entries())).unwrap();
            }
        };

        let mut p = P::new(BATCHED);
        p.set_own_nwk(0x8C23);
        let pairing = Pairing {
            gpd_id: GpdId::SrcId(SRC),
            add_sink: true,
            remove_gpd: false,
            communication_mode: CommunicationMode::CommissionedGroupcast,
            gpd_fixed: false,
            mac_seq_cap: true,
            security_level: SecurityLevel::Encrypted,
            key_type: KeyType::GpdGroupKey,
            sink: PairingSink::Group(0x0B84),
            device_id: Some(0x02),
            frame_counter: Some(1000),
            key: Some(KEY),
            assigned_alias: None,
            groupcast_radius: None,
        };
        let mut buf = [0u8; 64];
        let n = pairing.encode(&mut buf).unwrap();
        p.on_client_command(0, client_cmd::PAIRING, &buf[..n], 0, &mut None)
            .unwrap();
        persist(&mut p, &mut journal);

        let last = 1000 + 3 * S + 5;
        let mut now = 0;
        for fc in 1001..=last {
            now += 1_000;
            assert_eq!(gpdf(&mut p, fc, now), GpdfOutcome::Notification, "fc {fc}");
            persist(&mut p, &mut journal);
        }
        // The pairing's full record; a second one at the first stride,
        // because the first forwards also set InRange and FirstToForward;
        // then counter records only, no rollover.
        let mut flash = journal.into_storage();
        assert_eq!(flash.erases, 1);
        let slot = GP_PROXY_JOURNAL_SLOT_SIZE;
        assert_eq!(flash.data[slot..slot + 4], *b"ZBGP");
        let records = 2 * slot..2 * slot + 2 * GP_COUNTER_RECORD_SIZE;
        assert!(
            records
                .clone()
                .step_by(GP_COUNTER_RECORD_SIZE)
                .all(|at| flash.data[at..at + 4] == *b"ZBGC")
        );
        assert!(flash.data[records.end..].iter().all(|b| *b == 0xFF));
        flash.programs_left = Some(0);

        // Power loss and reboot.
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let snapshot = load(&mut journal).unwrap();
        let mut p = P::new(BATCHED);
        p.set_own_nwk(0x8C23);
        for entry in snapshot.entries() {
            p.restore(entry.unwrap()).unwrap();
        }
        let floor = 1000 + 3 * S + S - 1;
        assert_eq!(p.entries()[0].frame_counter, floor);
        for fc in [1000, 1001, 1000 + 3 * S, last, floor] {
            now += 10_000;
            assert_eq!(
                gpdf(&mut p, fc, now),
                GpdfOutcome::Dropped(DropReason::Stale),
                "replayed fc {fc}"
            );
        }
        assert!(!p.take_dirty());
        now += 10_000;
        assert_eq!(gpdf(&mut p, floor + 1, now), GpdfOutcome::Notification);
    }

    #[test]
    fn unusable_geometry_is_a_hardware_error() {
        let mut journal = GpProxyJournal::<_, SECTOR>::new(MockFlash::new(), 0, 0);
        let mut snapshot = PersistentGpProxyTable::default();
        assert_eq!(
            journal.load_into(&mut snapshot),
            Err(GpStoreError::Hardware)
        );
    }

    /// A sink-less build ignores sink (`ZBGS`) records: the Proxy Table
    /// record after one loads, takes counter records and new records.
    #[test]
    fn records_of_another_kind_are_skipped() {
        let proxy = capture(&[entry(1, 10)]);
        let mut sink_like = legacy_record(7, &proxy);
        sink_like[0..4].copy_from_slice(b"ZBGS");
        let crc = crc32(&sink_like[..1012]);
        sink_like[1012..1016].copy_from_slice(&crc.to_le_bytes());
        let mut flash = MockFlash::new();
        flash.data[..1024].copy_from_slice(&sink_like);
        flash.data[1024..2048].copy_from_slice(&legacy_record(7, &proxy));
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(load(&mut journal).unwrap().payload(), proxy.payload());
        journal.store(&capture(&[entry(1, 74)])).unwrap();
        journal.store(&capture(&[entry_radius(1, 74, 3)])).unwrap();
        let flash = journal.into_storage();
        assert_eq!(flash.data[..1024], sink_like);
        let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        assert_eq!(
            load(&mut journal).unwrap().payload(),
            capture(&[entry_radius(1, 74, 3)]).payload()
        );
    }

    #[cfg(feature = "green-power-sink")]
    mod sink {
        use super::*;

        fn sink_entry(src_id: u32, counter: u32) -> SinkEntry {
            let mut entry = SinkEntry::new(GpdId::SrcId(src_id));
            entry.security_level = SecurityLevel::Encrypted;
            entry.key_type = KeyType::IndividualKey;
            entry.key = Some([0x5A; 16]);
            entry.frame_counter = counter;
            entry.device_id = 0x02;
            entry
        }

        fn combo(proxy: &[ProxyEntry], sink: &[SinkEntry]) -> PersistentGpProxyTable {
            capture(proxy).with_sink(sink).unwrap()
        }

        fn sink_counters(snapshot: &PersistentGpProxyTable) -> heapless::Vec<u32, 8> {
            snapshot
                .sink_entries()
                .map(|entry| entry.unwrap().frame_counter)
                .collect()
        }

        /// Sink entries carry their own exact flags (bit 7 index records).
        #[test]
        fn sink_exact_counter_flags_round_trip() {
            let mut journal = journal(MockFlash::new());
            let switch = sink_entry(2, 20).with_counter_exact(true);
            journal
                .store(&combo(
                    &[entry(1, 10)],
                    &[switch.clone(), sink_entry(3, 30)],
                ))
                .unwrap();
            let mut rebooted =
                GpProxyJournal::<_, SECTOR>::new(journal.into_storage(), 0, SECTOR as u32);
            let loaded = load(&mut rebooted).unwrap();
            let flags: heapless::Vec<bool, 8> = loaded
                .sink_entries()
                .map(|e| e.unwrap().counter_exact())
                .collect();
            assert_eq!(flags.as_slice(), &[true, false]);
            assert_eq!(exact_bits(&loaded).as_slice(), &[false]);
            // A press: one sink counter record, still exact.
            let mut pressed = switch.clone();
            pressed.frame_counter = 21;
            rebooted
                .store(&combo(&[entry(1, 10)], &[pressed, sink_entry(3, 30)]))
                .unwrap();
            let mut again =
                GpProxyJournal::<_, SECTOR>::new(rebooted.into_storage(), 0, SECTOR as u32);
            let loaded = load(&mut again).unwrap();
            assert_eq!(sink_counters(&loaded).as_slice(), &[21, 30]);
            assert!(
                loaded
                    .sink_entries()
                    .next()
                    .unwrap()
                    .unwrap()
                    .counter_exact()
            );
        }

        #[test]
        fn sink_snapshot_round_trips_and_counts_as_content() {
            let snapshot = combo(&[], &[sink_entry(1, 7), sink_entry(2, 9)]);
            assert!(!snapshot.is_empty());
            assert_eq!(snapshot.len(), 0);
            assert_eq!(snapshot.sink_len(), 2);
            assert_eq!(sink_counters(&snapshot).as_slice(), &[7, 9]);
            assert!(PersistentGpProxyTable::default().is_empty());
            let full: heapless::Vec<SinkEntry, GP_SINK_TABLE_ENTRIES> = (0..GP_SINK_TABLE_ENTRIES
                as u32)
                .map(|i| sink_entry(i + 1, i))
                .collect();
            let over: heapless::Vec<SinkEntry, { GP_SINK_TABLE_ENTRIES + 1 }> =
                full.iter().cloned().chain([sink_entry(0x99, 9)]).collect();
            assert_eq!(
                capture(&[]).with_sink(&over).err(),
                Some(GpStoreError::Full)
            );
            let _ = capture(&[]).with_sink(&full).unwrap();
        }

        /// Pair layout: sink record then the Proxy Table record of the same
        /// generation; load restores both; a legacy journal loads with an
        /// empty Sink Table and the next sink commit writes the pair.
        #[test]
        fn journal_keeps_proxy_and_sink_tables() {
            let legacy = capture(&[entry(1, 10)]);
            let mut flash = MockFlash::new();
            flash.data[..1024].copy_from_slice(&legacy_record(4, &legacy));
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            let loaded = load(&mut journal).unwrap();
            assert_eq!(loaded.payload(), legacy.payload());
            assert_eq!(loaded.sink_len(), 0);

            let both = combo(&[entry(1, 10)], &[sink_entry(2, 20)]);
            journal.store(&both).unwrap();
            let data = &journal.storage.data;
            assert_eq!(data[1024..1028], *b"ZBGS");
            assert_eq!(data[2048..2052], *b"ZBGP");
            assert_eq!(data[1024 + 8..1024 + 12], 5u32.to_le_bytes());
            assert_eq!(data[2048 + 8..2048 + 12], 5u32.to_le_bytes());
            let flash = journal.into_storage();
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            assert_eq!(load(&mut journal).unwrap(), both);

            // A sink counter change is a compact record with the sink bit.
            let moved = combo(&[entry(1, 10)], &[sink_entry(2, 84)]);
            journal.store(&moved).unwrap();
            let at = 3 * GP_PROXY_JOURNAL_SLOT_SIZE;
            assert_eq!(journal.storage.data[at..at + 4], *b"ZBGC");
            assert_eq!(journal.storage.data[at + 5], 0x80);
            assert_eq!(journal.storage.erases, 0);
            let flash = journal.into_storage();
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            let loaded = load(&mut journal).unwrap();
            assert_eq!(sink_counters(&loaded).as_slice(), &[84]);
            assert_eq!(counters(&loaded).as_slice(), &[10]);
            // Unchanged: nothing written.
            let before = journal.storage.data;
            journal.store(&moved).unwrap();
            assert_eq!(journal.storage.data, before);
        }

        #[test]
        fn torn_pair_keeps_the_previous_tables() {
            let first = combo(&[entry(1, 10)], &[sink_entry(2, 20)]);
            let mut journal = journal(MockFlash::new());
            journal.store(&first).unwrap();
            // Power fails after the sink record, before the proxy record.
            let next = combo(&[entry(1, 10)], &[sink_entry(2, 20), sink_entry(3, 1)]);
            journal.storage.programs_left = Some(2);
            assert_eq!(journal.store(&next), Err(GpStoreError::Hardware));
            let mut flash = journal.into_storage();
            flash.programs_left = None;
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            assert_eq!(load(&mut journal).unwrap(), first);
            // The retry pairs records of the same (reused) generation.
            journal.store(&next).unwrap();
            let flash = journal.into_storage();
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            assert_eq!(load(&mut journal).unwrap(), next);
        }

        #[test]
        fn pairs_roll_over_and_an_emptied_sink_table_drops_its_record() {
            let mut journal = journal(MockFlash::new());
            let mut last = PersistentGpProxyTable::default();
            for radius in 0..6u8 {
                last = combo(&[entry_radius(1, 10, radius)], &[sink_entry(2, 20)]);
                journal.store(&last).unwrap();
            }
            assert!(journal.storage.erases >= 2);
            let flash = journal.into_storage();
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            assert_eq!(load(&mut journal).unwrap(), last);
            // Removing every sink entry writes a lone Proxy Table record;
            // the older sink record before it is not paired.
            let proxy_only = capture(&[entry_radius(1, 10, 9)]);
            journal.store(&proxy_only).unwrap();
            let flash = journal.into_storage();
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            let loaded = load(&mut journal).unwrap();
            assert_eq!(loaded.payload(), proxy_only.payload());
            assert_eq!(loaded.sink_len(), 0);
        }

        /// A sink record left by an interrupted pair is not paired with the
        /// next (proxy-only) record, which reuses its generation.
        #[test]
        fn orphan_sink_record_is_not_resurrected() {
            let first = combo(&[entry(1, 10)], &[sink_entry(2, 20)]);
            let mut journal = journal(MockFlash::new());
            journal.store(&first).unwrap();
            journal.storage.programs_left = Some(2);
            let attempted = combo(&[entry(1, 10)], &[sink_entry(2, 20), sink_entry(3, 1)]);
            assert_eq!(journal.store(&attempted), Err(GpStoreError::Hardware));
            journal.storage.programs_left = None;
            let proxy_only = capture(&[entry(1, 10)]);
            journal.store(&proxy_only).unwrap();
            // The orphan (generation 1) sits right before the new record
            // (generation 1).
            assert_eq!(journal.storage.data[2048..2052], *b"ZBGS");
            assert_eq!(journal.storage.data[3072..3076], *b"ZBGP");
            let flash = journal.into_storage();
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            let loaded = load(&mut journal).unwrap();
            assert_eq!(loaded.payload(), proxy_only.payload());
            assert_eq!(loaded.sink_len(), 0);
        }

        #[test]
        fn sink_record_of_another_generation_is_not_paired() {
            let both = combo(&[entry(1, 10)], &[sink_entry(2, 20)]);
            let mut journal = journal(MockFlash::new());
            journal.store(&both).unwrap();
            // Corrupt the sink record's CRC: the proxy record still loads.
            journal.storage.data[100] ^= 1;
            let flash = journal.into_storage();
            let mut journal = GpProxyJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            let loaded = load(&mut journal).unwrap();
            assert_eq!(loaded.payload(), both.payload());
            assert_eq!(loaded.sink_len(), 0);
        }
    }
}

#[cfg(feature = "green-power-translation-table")]
mod translations;
