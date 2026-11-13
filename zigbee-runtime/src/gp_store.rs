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
use crate::role::DeviceRole;
use crate::security_journal::crc32;

/// Largest encoded snapshot payload: an entry count, then every entry as a
/// length byte followed by its Proxy Table attribute encoding.
pub const MAX_ENCODED_GP_PROXY_TABLE_LEN: usize =
    1 + GP_PROXY_TABLE_ENTRIES * (1 + MAX_ENTRY_ENCODED_LEN);

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
}

impl core::fmt::Debug for PersistentGpProxyTable {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // The payload carries GPD keys: never print it.
        f.debug_struct("PersistentGpProxyTable")
            .field("extended_pan_id", &self.extended_pan_id)
            .field("entries", &self.payload[0])
            .finish_non_exhaustive()
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
        for entry in entries {
            let out = &mut snapshot.payload[at + 1..at + 1 + MAX_ENTRY_ENCODED_LEN];
            let n = entry.encode(out).map_err(|_| GpStoreError::Corrupt)?;
            snapshot.payload[at] = n as u8;
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
    }

    pub fn extended_pan_id(&self) -> IeeeAddress {
        self.extended_pan_id
    }

    /// Number of stored entries.
    pub fn len(&self) -> usize {
        self.payload[0] as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn payload(&self) -> &[u8] {
        &self.payload[..self.len]
    }

    /// Decode the stored entries. Trailing bytes or a malformed entry yield
    /// [`GpStoreError::Corrupt`].
    pub fn entries(&self) -> impl Iterator<Item = Result<ProxyEntry, GpStoreError>> + '_ {
        Self::entries_in(self.payload())
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
const COUNTER_VERSION: u8 = 1;
const COUNTER_CRC_OFFSET: usize = 20;
const COUNTER_COMMIT_OFFSET: usize = 24;

const _: () = assert!(RECORD_PAYLOAD_OFFSET + MAX_ENCODED_GP_PROXY_TABLE_LEN <= RECORD_CRC_OFFSET);
const _: () = assert!(GP_PROXY_JOURNAL_SLOT_SIZE.is_multiple_of(GP_COUNTER_RECORD_SIZE));
const _: () = assert!(GP_PROXY_TABLE_ENTRIES <= u8::MAX as usize);

/// Where each entry's GPD ID tag and frame counter sit in a snapshot
/// payload. A zero counter offset means the entry carries no counter.
struct PayloadLayout {
    entries: usize,
    tags: [u32; GP_PROXY_TABLE_ENTRIES],
    counter_at: [u16; GP_PROXY_TABLE_ENTRIES],
}

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

    /// Entry index, GPD ID tag and frame counter of a committed, CRC-valid
    /// counter record bound to the full record `generation`.
    fn counter_record(unit: &[u8], generation: u32) -> Option<(usize, u32, u32)> {
        let word =
            |at: usize| u32::from_le_bytes([unit[at], unit[at + 1], unit[at + 2], unit[at + 3]]);
        (unit[0..4] == COUNTER_MAGIC
            && unit[4] == COUNTER_VERSION
            && unit[COUNTER_COMMIT_OFFSET..COUNTER_COMMIT_OFFSET + 4] == RECORD_COMMIT
            && crc32(&unit[..COUNTER_CRC_OFFSET]) == word(COUNTER_CRC_OFFSET)
            && word(8) == generation)
            .then(|| (unit[5] as usize, word(12), word(16)))
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
                    if let Some((index, tag, counter)) = Self::counter_record(unit, generation)
                        && index < layout.entries
                        && layout.counter_at[index] != 0
                        && layout.tags[index] == tag
                    {
                        base.counters[index] = counter;
                    }
                }
            }
            if let Some(snapshot) = snapshot {
                for index in 0..layout.entries {
                    let at = layout.counter_at[index] as usize;
                    if at != 0 {
                        snapshot.payload[at..at + 4]
                            .copy_from_slice(&base.counters[index].to_le_bytes());
                    }
                }
            }
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
        counter: u32,
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
        unit[6..8].fill(0);
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

    /// Persist a counter-only change as counter records. `Ok(false)` when a
    /// full record is needed (structural change or no room).
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
        let changed = (0..layout.entries)
            .filter(|&index| {
                layout
                    .counter(payload, index)
                    .is_some_and(|counter| counter != base.counters[index])
            })
            .count();
        if base.append as usize + changed * GP_COUNTER_RECORD_SIZE > SECTOR_SIZE {
            return Ok(false);
        }
        for index in 0..layout.entries {
            let Some(counter) = layout.counter(payload, index) else {
                continue;
            };
            if counter == base.counters[index] {
                continue;
            }
            if !self.write_counter_record(&base, index, layout.tags[index], counter)? {
                return Ok(false);
            }
            base.counters[index] = counter;
            base.append += GP_COUNTER_RECORD_SIZE as u16;
            self.current = Some(base);
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
        let (generation, sector, slot) = match self.current {
            Some(base) => {
                let generation = base
                    .generation
                    .checked_add(1)
                    .ok_or(GpStoreError::GenerationExhausted)?;
                let sector = base.sector as usize;
                match self.first_erased_slot(sector, base.slot as usize + 1)? {
                    Some(slot) => (generation, sector, slot),
                    None => {
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
        };
        for index in 0..layout.entries {
            base.counters[index] = layout.counter(payload, index).unwrap_or(0);
        }
        self.current = Some(base);
        Ok(())
    }
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
    /// Restore the Proxy Table for the joined network and re-establish the
    /// GPEP group membership it requires.
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
        let result = PersistentGpProxyTable::capture(extended_pan_id, self.gp_proxy().entries())
            .and_then(|snapshot| store.store(&snapshot));
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
mod tests {
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
    struct FlashError;

    impl embedded_storage::nor_flash::NorFlashError for FlashError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    struct MockFlash {
        data: [u8; 2 * SECTOR],
        /// Successful program operations left before every write fails.
        programs_left: Option<usize>,
        erases: usize,
    }

    impl MockFlash {
        fn new() -> Self {
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
                .write_counter_record(&base, 0, tag ^ 1, 999)
                .unwrap()
        );
        base.append += GP_COUNTER_RECORD_SIZE as u16;
        // Entry index past the table.
        assert!(journal.write_counter_record(&base, 2, tag, 999).unwrap());
        base.append += GP_COUNTER_RECORD_SIZE as u16;
        // Older generation.
        base.generation = base.generation.wrapping_sub(1);
        assert!(journal.write_counter_record(&base, 0, tag, 999).unwrap());
        base.append += GP_COUNTER_RECORD_SIZE as u16;
        // The matching record still applies.
        base.generation = base.generation.wrapping_add(1);
        assert!(journal.write_counter_record(&base, 0, tag, 333).unwrap());
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

        let mut p = P::new(ProxyConfig::default());
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
        let mut p = P::new(ProxyConfig::default());
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
}
