//! Crash-safe persistence for APS binding and group tables.
//!
//! APS tables are network-scoped application state. They are stored separately
//! from the high-frequency security counter journal and are bound to the
//! extended PAN ID, so stale bindings from a previous network are never
//! restored into a newly commissioned one.

use embedded_storage::nor_flash::NorFlash;
use zigbee_aps::binding::{
    BindingDst, BindingDstMode, BindingEntry, BindingTable, MAX_BINDING_ENTRIES,
};
use zigbee_aps::group::{GroupTable, MAX_ENDPOINTS_PER_GROUP, MAX_GROUPS};
use zigbee_types::IeeeAddress;

const BINDING_ENTRY_LEN: usize = 21;
const GROUP_ENTRY_MAX_LEN: usize = 3 + MAX_ENDPOINTS_PER_GROUP;

/// Largest encoded APS table snapshot for the selected role feature set.
pub const MAX_ENCODED_APS_TABLES_LEN: usize =
    8 + 1 + MAX_BINDING_ENTRIES * BINDING_ENTRY_LEN + 1 + MAX_GROUPS * GROUP_ENTRY_MAX_LEN;

/// Stable fingerprint of the live APS tables and their Zigbee network.
///
/// Comparing this value with the last successful checkpoint makes every
/// binding/group mutation path persistence-aware without maintaining dirty
/// flags at each ZDO, ZCL, or Finding & Binding call site.
pub fn aps_table_fingerprint(
    extended_pan_id: IeeeAddress,
    bindings: &BindingTable,
    groups: &GroupTable,
) -> u32 {
    fn update(mut hash: u32, bytes: &[u8]) -> u32 {
        for byte in bytes {
            hash ^= u32::from(*byte);
            hash = hash.wrapping_mul(0x0100_0193);
        }
        hash
    }

    let mut hash = update(0x811C_9DC5, &[bindings.len() as u8, groups.len() as u8]);
    if bindings.is_empty() && groups.is_empty() {
        return hash;
    }
    hash = update(hash, &extended_pan_id);
    for entry in bindings.entries() {
        hash = update(hash, &entry.src_addr);
        hash = update(hash, &[entry.src_endpoint]);
        hash = update(hash, &entry.cluster_id.to_le_bytes());
        match entry.dst {
            BindingDst::Group(group) => {
                hash = update(hash, &[BindingDstMode::Group as u8]);
                hash = update(hash, &group.to_le_bytes());
            }
            BindingDst::Unicast {
                dst_addr,
                dst_endpoint,
            } => {
                hash = update(hash, &[BindingDstMode::Extended as u8]);
                hash = update(hash, &dst_addr);
                hash = update(hash, &[dst_endpoint]);
            }
        }
    }
    for group in groups.groups() {
        hash = update(hash, &group.group_address.to_le_bytes());
        hash = update(hash, &[group.endpoint_list.len() as u8]);
        hash = update(hash, group.endpoint_list.as_slice());
    }
    hash
}

/// APS table persistence failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApsTableStoreError {
    Corrupt,
    Full,
    Hardware,
    GenerationExhausted,
    ForeignNetwork,
}

/// Network-bound APS binding and group table snapshot.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PersistentApsTables {
    extended_pan_id: IeeeAddress,
    bindings: BindingTable,
    groups: GroupTable,
}

impl PersistentApsTables {
    /// Empty snapshot bound to `extended_pan_id`.
    pub fn new(extended_pan_id: IeeeAddress) -> Self {
        Self {
            extended_pan_id,
            bindings: BindingTable::new(),
            groups: GroupTable::new(),
        }
    }

    /// Capture the live APS tables.
    pub fn capture(
        extended_pan_id: IeeeAddress,
        bindings: &BindingTable,
        groups: &GroupTable,
    ) -> Result<Self, ApsTableStoreError> {
        let snapshot = Self {
            extended_pan_id,
            bindings: bindings.clone(),
            groups: groups.clone(),
        };
        snapshot.validate()?;
        Ok(snapshot)
    }

    pub const fn extended_pan_id(&self) -> IeeeAddress {
        self.extended_pan_id
    }

    pub fn matches_network(&self, extended_pan_id: &IeeeAddress) -> bool {
        self.extended_pan_id == *extended_pan_id
    }

    pub const fn bindings(&self) -> &BindingTable {
        &self.bindings
    }

    pub const fn groups(&self) -> &GroupTable {
        &self.groups
    }

    pub fn is_empty(&self) -> bool {
        self.bindings.is_empty() && self.groups.is_empty()
    }

    pub fn fingerprint(&self) -> u32 {
        aps_table_fingerprint(self.extended_pan_id, &self.bindings, &self.groups)
    }

    pub fn validate(&self) -> Result<(), ApsTableStoreError> {
        if !self.is_empty()
            && (self.extended_pan_id == [0u8; 8] || self.extended_pan_id == [0xFFu8; 8])
        {
            return Err(ApsTableStoreError::Corrupt);
        }

        for entry in self.bindings.entries() {
            if entry.src_addr == [0u8; 8] || !(1..=240).contains(&entry.src_endpoint) {
                return Err(ApsTableStoreError::Corrupt);
            }
            match (&entry.dst_addr_mode, &entry.dst) {
                (BindingDstMode::Group, BindingDst::Group(_)) => {}
                (
                    BindingDstMode::Extended,
                    BindingDst::Unicast {
                        dst_addr,
                        dst_endpoint,
                    },
                ) if *dst_addr != [0u8; 8]
                    && *dst_addr != [0xFFu8; 8]
                    && (1..=240).contains(dst_endpoint) => {}
                _ => return Err(ApsTableStoreError::Corrupt),
            }
        }

        for group in self.groups.groups() {
            if group.endpoint_list.is_empty() {
                return Err(ApsTableStoreError::Corrupt);
            }
            for (index, endpoint) in group.endpoint_list.iter().enumerate() {
                if !(1..=240).contains(endpoint)
                    || group.endpoint_list[index + 1..].contains(endpoint)
                {
                    return Err(ApsTableStoreError::Corrupt);
                }
            }
        }
        Ok(())
    }

    /// Encode the version-1 payload and return its used length.
    pub fn encode(&self, out: &mut [u8; MAX_ENCODED_APS_TABLES_LEN]) -> usize {
        out.fill(0);
        out[0..8].copy_from_slice(&self.extended_pan_id);
        out[8] = self.bindings.len() as u8;
        let mut offset = 9;

        for entry in self.bindings.entries() {
            out[offset..offset + 8].copy_from_slice(&entry.src_addr);
            out[offset + 8] = entry.src_endpoint;
            out[offset + 9..offset + 11].copy_from_slice(&entry.cluster_id.to_le_bytes());
            out[offset + 11] = entry.dst_addr_mode as u8;
            match entry.dst {
                BindingDst::Group(group) => {
                    out[offset + 12..offset + 14].copy_from_slice(&group.to_le_bytes());
                }
                BindingDst::Unicast {
                    dst_addr,
                    dst_endpoint,
                } => {
                    out[offset + 12..offset + 20].copy_from_slice(&dst_addr);
                    out[offset + 20] = dst_endpoint;
                }
            }
            offset += BINDING_ENTRY_LEN;
        }

        out[offset] = self.groups.len() as u8;
        offset += 1;
        for group in self.groups.groups() {
            out[offset..offset + 2].copy_from_slice(&group.group_address.to_le_bytes());
            out[offset + 2] = group.endpoint_list.len() as u8;
            offset += 3;
            out[offset..offset + group.endpoint_list.len()].copy_from_slice(&group.endpoint_list);
            offset += group.endpoint_list.len();
        }
        offset
    }

    /// Decode and validate a version-1 payload.
    pub fn decode(bytes: &[u8]) -> Result<Self, ApsTableStoreError> {
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(bytes.get(0..8).ok_or(ApsTableStoreError::Corrupt)?);
        let binding_count = usize::from(*bytes.get(8).ok_or(ApsTableStoreError::Corrupt)?);
        if binding_count > MAX_BINDING_ENTRIES {
            return Err(ApsTableStoreError::Corrupt);
        }

        let mut snapshot = Self::new(extended_pan_id);
        let mut offset = 9;
        for _ in 0..binding_count {
            let entry = bytes
                .get(offset..offset + BINDING_ENTRY_LEN)
                .ok_or(ApsTableStoreError::Corrupt)?;
            let mut src_addr = [0u8; 8];
            src_addr.copy_from_slice(&entry[0..8]);
            let src_endpoint = entry[8];
            let cluster_id = u16::from_le_bytes([entry[9], entry[10]]);
            let binding = match entry[11] {
                mode if mode == BindingDstMode::Group as u8 => {
                    if entry[14..21].iter().any(|byte| *byte != 0) {
                        return Err(ApsTableStoreError::Corrupt);
                    }
                    BindingEntry::group(
                        src_addr,
                        src_endpoint,
                        cluster_id,
                        u16::from_le_bytes([entry[12], entry[13]]),
                    )
                }
                mode if mode == BindingDstMode::Extended as u8 => {
                    let mut dst_addr = [0u8; 8];
                    dst_addr.copy_from_slice(&entry[12..20]);
                    BindingEntry::unicast(src_addr, src_endpoint, cluster_id, dst_addr, entry[20])
                }
                _ => return Err(ApsTableStoreError::Corrupt),
            };
            snapshot
                .bindings
                .add(binding)
                .map_err(|_| ApsTableStoreError::Corrupt)?;
            offset += BINDING_ENTRY_LEN;
        }

        let group_count = usize::from(*bytes.get(offset).ok_or(ApsTableStoreError::Corrupt)?);
        offset += 1;
        if group_count > MAX_GROUPS {
            return Err(ApsTableStoreError::Corrupt);
        }
        for _ in 0..group_count {
            let header = bytes
                .get(offset..offset + 3)
                .ok_or(ApsTableStoreError::Corrupt)?;
            let group_address = u16::from_le_bytes([header[0], header[1]]);
            let endpoint_count = usize::from(header[2]);
            offset += 3;
            if endpoint_count == 0
                || endpoint_count > MAX_ENDPOINTS_PER_GROUP
                || snapshot.groups.find(group_address).is_some()
            {
                return Err(ApsTableStoreError::Corrupt);
            }
            let endpoints = bytes
                .get(offset..offset + endpoint_count)
                .ok_or(ApsTableStoreError::Corrupt)?;
            for (index, endpoint) in endpoints.iter().enumerate() {
                if endpoints[index + 1..].contains(endpoint)
                    || !snapshot.groups.add_group(group_address, *endpoint)
                {
                    return Err(ApsTableStoreError::Corrupt);
                }
            }
            offset += endpoint_count;
        }
        if offset != bytes.len() {
            return Err(ApsTableStoreError::Corrupt);
        }
        snapshot.validate()?;
        Ok(snapshot)
    }
}

impl Default for PersistentApsTables {
    fn default() -> Self {
        Self::new([0u8; 8])
    }
}

/// Durable APS-table storage.
pub trait ApsTableStore {
    fn load(&mut self) -> Result<Option<PersistentApsTables>, ApsTableStoreError>;
    fn store(&mut self, tables: &PersistentApsTables) -> Result<(), ApsTableStoreError>;
}

/// Volatile APS table backend for tests and products without a partition.
#[derive(Debug, Default)]
pub struct RamApsTableStore {
    tables: Option<PersistentApsTables>,
}

impl RamApsTableStore {
    pub const fn new() -> Self {
        Self { tables: None }
    }
}

impl ApsTableStore for RamApsTableStore {
    fn load(&mut self) -> Result<Option<PersistentApsTables>, ApsTableStoreError> {
        Ok(self.tables.clone())
    }

    fn store(&mut self, tables: &PersistentApsTables) -> Result<(), ApsTableStoreError> {
        tables.validate()?;
        self.tables = Some(tables.clone());
        Ok(())
    }
}

/// Erase-unit size assumed for each journal sector.
pub const APS_TABLE_JOURNAL_SECTOR_SIZE: usize = 4096;
/// Size of one APS-table journal slot.
pub const APS_TABLE_JOURNAL_SLOT_SIZE: usize = 1024;
pub const APS_TABLE_JOURNAL_SLOTS_PER_SECTOR: usize =
    APS_TABLE_JOURNAL_SECTOR_SIZE / APS_TABLE_JOURNAL_SLOT_SIZE;

const RECORD_MAGIC: [u8; 4] = *b"ZBAT";
const RECORD_VERSION: u8 = 1;
const RECORD_ENCODED_OFFSET: usize = 12;
const RECORD_CRC_OFFSET: usize = APS_TABLE_JOURNAL_SLOT_SIZE - 12;
const RECORD_PREFIX_LEN: usize = APS_TABLE_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT_OFFSET: usize = APS_TABLE_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

const _: () = assert!(RECORD_ENCODED_OFFSET + MAX_ENCODED_APS_TABLES_LEN <= RECORD_CRC_OFFSET);

/// Atomic two-sector APS table journal.
pub struct ApsTableJournal<S> {
    storage: S,
    sectors: [u32; 2],
    cached: Option<LocatedTables>,
    scanned: bool,
}

#[derive(Clone)]
struct LocatedTables {
    generation: u32,
    sector: usize,
    tables: PersistentApsTables,
}

impl<S: NorFlash> ApsTableJournal<S> {
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        Self {
            storage,
            sectors: [first_sector, second_sector],
            cached: None,
            scanned: false,
        }
    }

    pub fn storage(&self) -> &S {
        &self.storage
    }

    pub fn into_storage(self) -> S {
        self.storage
    }

    fn geometry_ok(&self) -> bool {
        self.sectors[0] != self.sectors[1]
            && self.sectors[0].abs_diff(self.sectors[1]) >= APS_TABLE_JOURNAL_SECTOR_SIZE as u32
            && S::READ_SIZE != 0
            && S::WRITE_SIZE != 0
            && S::ERASE_SIZE != 0
            && APS_TABLE_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
            && APS_TABLE_JOURNAL_SLOT_SIZE.is_multiple_of(S::WRITE_SIZE)
            && APS_TABLE_JOURNAL_SECTOR_SIZE.is_multiple_of(S::ERASE_SIZE)
            && RECORD_PREFIX_LEN.is_multiple_of(S::WRITE_SIZE)
            && RECORD_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
            && RECORD_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            && self
                .sectors
                .iter()
                .all(|sector| (*sector as usize).is_multiple_of(S::ERASE_SIZE))
            && self.sectors.iter().all(|sector| {
                (*sector as usize)
                    .checked_add(APS_TABLE_JOURNAL_SECTOR_SIZE)
                    .is_some_and(|end| end <= self.storage.capacity())
            })
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        output: &mut [u8; APS_TABLE_JOURNAL_SLOT_SIZE],
    ) -> Result<(), ApsTableStoreError> {
        self.storage
            .read(
                self.sectors[sector] + (slot * APS_TABLE_JOURNAL_SLOT_SIZE) as u32,
                output,
            )
            .map_err(|_| ApsTableStoreError::Hardware)
    }

    fn decode_record(
        record: &[u8; APS_TABLE_JOURNAL_SLOT_SIZE],
    ) -> Option<(u32, PersistentApsTables)> {
        if record[0..4] != RECORD_MAGIC
            || record[4] != RECORD_VERSION
            || record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let encoded_len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if encoded_len > MAX_ENCODED_APS_TABLES_LEN
            || RECORD_ENCODED_OFFSET + encoded_len > RECORD_CRC_OFFSET
        {
            return None;
        }
        let expected_crc = u32::from_le_bytes([
            record[RECORD_CRC_OFFSET],
            record[RECORD_CRC_OFFSET + 1],
            record[RECORD_CRC_OFFSET + 2],
            record[RECORD_CRC_OFFSET + 3],
        ]);
        if crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]) != expected_crc {
            return None;
        }
        let generation = u32::from_le_bytes([record[8], record[9], record[10], record[11]]);
        let tables = PersistentApsTables::decode(
            &record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len],
        )
        .ok()?;
        Some((generation, tables))
    }

    fn newest(&mut self) -> Result<Option<LocatedTables>, ApsTableStoreError> {
        let mut newest: Option<LocatedTables> = None;
        let mut record = [0u8; APS_TABLE_JOURNAL_SLOT_SIZE];
        for sector in 0..2 {
            for slot in 0..APS_TABLE_JOURNAL_SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some((generation, tables)) = Self::decode_record(&record) else {
                    continue;
                };
                if newest
                    .as_ref()
                    .is_none_or(|current| generation > current.generation)
                {
                    newest = Some(LocatedTables {
                        generation,
                        sector,
                        tables,
                    });
                }
            }
        }
        Ok(newest)
    }

    fn current(&mut self) -> Result<Option<LocatedTables>, ApsTableStoreError> {
        if !self.geometry_ok() {
            return Err(ApsTableStoreError::Hardware);
        }
        if !self.scanned {
            self.cached = self.newest()?;
            self.scanned = true;
        }
        Ok(self.cached.clone())
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, ApsTableStoreError> {
        let mut record = [0u8; APS_TABLE_JOURNAL_SLOT_SIZE];
        for slot in 0..APS_TABLE_JOURNAL_SLOTS_PER_SECTOR {
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
        tables: &PersistentApsTables,
    ) -> Result<(), ApsTableStoreError> {
        tables.validate()?;
        let mut record = [0xFFu8; APS_TABLE_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = RECORD_VERSION;
        let mut encoded = [0u8; MAX_ENCODED_APS_TABLES_LEN];
        let encoded_len = tables.encode(&mut encoded);
        record[5..7].copy_from_slice(&(encoded_len as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len]
            .copy_from_slice(&encoded[..encoded_len]);
        let crc = crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.sectors[sector] + (slot * APS_TABLE_JOURNAL_SLOT_SIZE) as u32;
        self.storage
            .write(address, &record[..RECORD_PREFIX_LEN])
            .map_err(|_| ApsTableStoreError::Hardware)?;
        self.storage
            .write(address + RECORD_COMMIT_OFFSET as u32, &RECORD_COMMIT)
            .map_err(|_| ApsTableStoreError::Hardware)?;

        let mut verify = [0u8; APS_TABLE_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut verify)?;
        match Self::decode_record(&verify) {
            Some((stored_generation, stored_tables))
                if stored_generation == generation && stored_tables == *tables =>
            {
                Ok(())
            }
            _ => Err(ApsTableStoreError::Hardware),
        }
    }

    fn cache_result(
        &mut self,
        result: &Result<(), ApsTableStoreError>,
        generation: u32,
        sector: usize,
        tables: &PersistentApsTables,
    ) {
        if result.is_ok() {
            self.cached = Some(LocatedTables {
                generation,
                sector,
                tables: tables.clone(),
            });
        } else {
            self.cached = None;
            self.scanned = false;
        }
    }
}

impl<S: NorFlash> ApsTableStore for ApsTableJournal<S> {
    fn load(&mut self) -> Result<Option<PersistentApsTables>, ApsTableStoreError> {
        Ok(self.current()?.map(|located| located.tables))
    }

    fn store(&mut self, tables: &PersistentApsTables) -> Result<(), ApsTableStoreError> {
        let current = self.current()?;
        let generation = match &current {
            Some(located) => located
                .generation
                .checked_add(1)
                .ok_or(ApsTableStoreError::GenerationExhausted)?,
            None => 0,
        };

        if let Some(located) = current {
            if let Some(slot) = self.first_erased_slot(located.sector)? {
                let result = self.write_record(located.sector, slot, generation, tables);
                self.cache_result(&result, generation, located.sector, tables);
                return result;
            }
            let target = 1 - located.sector;
            let sector = self.sectors[target];
            let result = self
                .storage
                .erase(sector, sector + APS_TABLE_JOURNAL_SECTOR_SIZE as u32)
                .map_err(|_| ApsTableStoreError::Hardware)
                .and_then(|()| self.write_record(target, 0, generation, tables));
            self.cache_result(&result, generation, target, tables);
            return result;
        }

        let sector = self.sectors[0];
        let result = self
            .storage
            .erase(sector, sector + APS_TABLE_JOURNAL_SECTOR_SIZE as u32)
            .map_err(|_| ApsTableStoreError::Hardware)
            .and_then(|()| self.write_record(0, 0, generation, tables));
        self.cache_result(&result, generation, 0, tables);
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    const LOCAL: IeeeAddress = [0x11; 8];
    const REMOTE: IeeeAddress = [0x22; 8];
    const EPID: IeeeAddress = [0x33; 8];

    fn snapshot() -> PersistentApsTables {
        let mut tables = PersistentApsTables::new(EPID);
        tables
            .bindings
            .add(BindingEntry::unicast(LOCAL, 1, 0x0402, REMOTE, 2))
            .unwrap();
        tables
            .bindings
            .add(BindingEntry::group(LOCAL, 1, 0x0006, 0x1234))
            .unwrap();
        assert!(tables.groups.add_group(0x1234, 1));
        assert!(tables.groups.add_group(0x1234, 2));
        tables
    }

    #[test]
    fn snapshot_round_trip() {
        let expected = snapshot();
        let mut encoded = [0u8; MAX_ENCODED_APS_TABLES_LEN];
        let len = expected.encode(&mut encoded);
        assert_eq!(PersistentApsTables::decode(&encoded[..len]), Ok(expected));
    }

    #[test]
    fn snapshot_rejects_foreign_or_malformed_state() {
        let mut tables = snapshot();
        tables.extended_pan_id = [0; 8];
        assert_eq!(tables.validate(), Err(ApsTableStoreError::Corrupt));

        let mut encoded = [0u8; MAX_ENCODED_APS_TABLES_LEN];
        let len = snapshot().encode(&mut encoded);
        encoded[9 + 11] = 0xFF;
        assert_eq!(
            PersistentApsTables::decode(&encoded[..len]),
            Err(ApsTableStoreError::Corrupt)
        );
    }

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct MockError;

    impl embedded_storage::nor_flash::NorFlashError for MockError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    #[derive(Clone)]
    struct MockFlash {
        bytes: [u8; APS_TABLE_JOURNAL_SECTOR_SIZE * 2],
        programs_before_failure: Option<usize>,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                bytes: [0xFF; APS_TABLE_JOURNAL_SECTOR_SIZE * 2],
                programs_before_failure: None,
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = MockError;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            let end = start.checked_add(bytes.len()).ok_or(MockError)?;
            let source = self.bytes.get(start..end).ok_or(MockError)?;
            bytes.copy_from_slice(source);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = APS_TABLE_JOURNAL_SECTOR_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            let range = self
                .bytes
                .get_mut(from as usize..to as usize)
                .ok_or(MockError)?;
            range.fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            if let Some(remaining) = self.programs_before_failure.as_mut() {
                if *remaining == 0 {
                    return Err(MockError);
                }
                *remaining -= 1;
            }
            let start = offset as usize;
            let end = start.checked_add(bytes.len()).ok_or(MockError)?;
            let destination = self.bytes.get_mut(start..end).ok_or(MockError)?;
            for (dst, src) in destination.iter_mut().zip(bytes) {
                if (*dst & *src) != *src {
                    return Err(MockError);
                }
                *dst &= *src;
            }
            Ok(())
        }
    }

    #[test]
    fn journal_round_trip_and_generation_rollover() {
        let mut journal =
            ApsTableJournal::new(MockFlash::new(), 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        let mut expected = snapshot();
        for endpoint in 3..=7 {
            assert!(expected.groups.add_group(0x1234, endpoint));
            journal.store(&expected).unwrap();
        }

        let flash = journal.into_storage();
        let mut reopened = ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(reopened.load(), Ok(Some(expected)));
    }

    #[test]
    fn interrupted_commit_preserves_the_previous_generation() {
        let mut journal =
            ApsTableJournal::new(MockFlash::new(), 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        let expected = snapshot();
        journal.store(&expected).unwrap();

        let mut flash = journal.into_storage();
        flash.programs_before_failure = Some(1);
        let mut interrupted = ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        let mut replacement = expected.clone();
        assert!(replacement.groups.add_group(0x4567, 1));
        assert_eq!(
            interrupted.store(&replacement),
            Err(ApsTableStoreError::Hardware)
        );

        let flash = interrupted.into_storage();
        let mut reopened = ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(reopened.load(), Ok(Some(expected)));
    }

    #[test]
    fn interrupted_sector_rollover_preserves_the_full_previous_sector() {
        let mut journal =
            ApsTableJournal::new(MockFlash::new(), 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        let mut expected = snapshot();
        for endpoint in 3..=6 {
            assert!(expected.groups.add_group(0x1234, endpoint));
            journal.store(&expected).unwrap();
        }

        let mut flash = journal.into_storage();
        flash.programs_before_failure = Some(1);
        let mut interrupted = ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        let mut replacement = expected.clone();
        assert!(replacement.groups.add_group(0x1234, 7));
        assert_eq!(
            interrupted.store(&replacement),
            Err(ApsTableStoreError::Hardware)
        );

        let flash = interrupted.into_storage();
        let mut reopened = ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(reopened.load(), Ok(Some(expected)));
    }

    #[test]
    fn invalid_journal_geometry_is_rejected() {
        let mut journal = ApsTableJournal::new(MockFlash::new(), 0, 0);
        assert_eq!(journal.load(), Err(ApsTableStoreError::Hardware));
    }
}
