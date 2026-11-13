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
//!   ordering; an interrupted write or erase leaves the previous snapshot);
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

const RECORD_MAGIC: [u8; 4] = *b"ZBGP";
const RECORD_VERSION: u8 = 1;
const RECORD_EPID_OFFSET: usize = 12;
const RECORD_PAYLOAD_OFFSET: usize = 20;
const RECORD_CRC_OFFSET: usize = GP_PROXY_JOURNAL_SLOT_SIZE - 12;
const RECORD_COMMIT_OFFSET: usize = GP_PROXY_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

const _: () = assert!(RECORD_PAYLOAD_OFFSET + MAX_ENCODED_GP_PROXY_TABLE_LEN <= RECORD_CRC_OFFSET);

/// Atomic two-sector journal of GP Proxy Table snapshots.
///
/// Each commit appends a CRC-protected record to the next erased slot of the
/// current sector, writing its commit marker last and verifying the whole
/// slot by read-back. A full sector rolls over by erasing the other sector,
/// so an interrupted erase or write always leaves the previous snapshot.
pub struct GpProxyJournal<S, const SECTOR_SIZE: usize = GP_PROXY_JOURNAL_SECTOR_SIZE> {
    storage: S,
    sectors: [u32; 2],
    /// `(generation, sector)` of the newest committed record, once scanned.
    current: Option<(u32, usize)>,
    scanned: bool,
}

impl<S: NorFlash, const SECTOR_SIZE: usize> GpProxyJournal<S, SECTOR_SIZE> {
    const SLOTS_PER_SECTOR: usize = SECTOR_SIZE / GP_PROXY_JOURNAL_SLOT_SIZE;

    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        assert!(SECTOR_SIZE >= GP_PROXY_JOURNAL_SLOT_SIZE);
        assert!(SECTOR_SIZE.is_multiple_of(GP_PROXY_JOURNAL_SLOT_SIZE));
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

    /// Scan both sectors, copying the newest valid record into `snapshot`
    /// (when given). Equal generations prefer scan order.
    fn scan(
        &mut self,
        mut snapshot: Option<&mut PersistentGpProxyTable>,
    ) -> Result<bool, GpStoreError> {
        if !self.geometry_ok() {
            return Err(GpStoreError::Hardware);
        }
        let mut record = [0u8; GP_PROXY_JOURNAL_SLOT_SIZE];
        let mut best: Option<(u32, usize)> = None;
        for sector in 0..2 {
            for slot in 0..Self::SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some((generation, len)) = Self::header(&record) else {
                    continue;
                };
                if best.is_some_and(|(best_generation, _)| generation <= best_generation) {
                    continue;
                }
                let payload = &record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + len];
                if PersistentGpProxyTable::validate_payload(payload).is_err() {
                    continue;
                }
                if let Some(snapshot) = snapshot.as_deref_mut() {
                    let mut extended_pan_id = [0u8; 8];
                    extended_pan_id
                        .copy_from_slice(&record[RECORD_EPID_OFFSET..RECORD_EPID_OFFSET + 8]);
                    snapshot.set_validated(extended_pan_id, payload);
                }
                best = Some((generation, sector));
            }
        }
        self.current = best;
        self.scanned = true;
        Ok(best.is_some())
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, GpStoreError> {
        let mut record = [0u8; GP_PROXY_JOURNAL_SLOT_SIZE];
        for slot in 0..Self::SLOTS_PER_SECTOR {
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
        let (generation, sector, slot) = match self.current {
            Some((generation, sector)) => {
                let generation = generation
                    .checked_add(1)
                    .ok_or(GpStoreError::GenerationExhausted)?;
                match self.first_erased_slot(sector)? {
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
        self.current = Some((generation, sector));
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
            let snapshot = PersistentGpProxyTable::capture(EPID, &[entry(1, counter)]).unwrap();
            journal.store(&snapshot).unwrap();
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
        let second = PersistentGpProxyTable::capture(EPID, &[entry(1, 6)]).unwrap();
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
        let second = PersistentGpProxyTable::capture(EPID, &[entry(1, 6)]).unwrap();
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
            journal.store(&snapshot),
            Err(GpStoreError::GenerationExhausted)
        );
        // The stored record is still readable.
        assert_eq!(load(&mut journal).unwrap(), snapshot);
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
