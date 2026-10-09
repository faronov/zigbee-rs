//! Crash-safe persistence for APS binding and group tables.
//!
//! APS tables are network-scoped application state. They are stored separately
//! from the high-frequency security counter journal and are bound to the
//! extended PAN ID, so stale bindings from a previous network are never
//! restored into a newly commissioned one.

use embedded_storage::nor_flash::NorFlash;
use heapless::Vec;
use zigbee_aps::binding::{
    BindingDst, BindingDstMode, BindingEntry, BindingTable, MAX_BINDING_ENTRIES,
};
use zigbee_aps::group::{GroupTable, MAX_ENDPOINTS_PER_GROUP, MAX_GROUPS};
use zigbee_aps::security::{ApsKeyType, ApsLinkKeyEntry, ApsSecurity, MAX_KEY_TABLE_ENTRIES};
use zigbee_types::IeeeAddress;

const BINDING_ENTRY_LEN: usize = 21;
const GROUP_ENTRY_MAX_LEN: usize = 3 + MAX_ENDPOINTS_PER_GROUP;
const APPLICATION_KEY_ENTRY_LEN: usize = 33;
pub(crate) const APPLICATION_KEY_LOW_WATER: u32 = 32;
pub(crate) const APPLICATION_KEY_RESERVATION: u32 = 0x400;

/// Largest encoded APS table snapshot for the selected role feature set.
pub const MAX_ENCODED_APS_TABLES_LEN: usize = 8
    + 1
    + MAX_BINDING_ENTRIES * BINDING_ENTRY_LEN
    + 1
    + MAX_GROUPS * GROUP_ENTRY_MAX_LEN
    + 1
    + MAX_KEY_TABLE_ENTRIES * APPLICATION_KEY_ENTRY_LEN;

/// Stable fingerprint of the live APS tables and their Zigbee network.
///
/// Comparing this value with the last successful checkpoint makes every
/// binding/group mutation path persistence-aware without maintaining dirty
/// flags at each ZDO, ZCL, or Finding & Binding call site.
pub fn aps_table_fingerprint(
    extended_pan_id: IeeeAddress,
    bindings: &BindingTable,
    groups: &GroupTable,
    security: &ApsSecurity,
) -> u32 {
    fn update(mut hash: u32, bytes: &[u8]) -> u32 {
        for byte in bytes {
            hash ^= u32::from(*byte);
            hash = hash.wrapping_mul(0x0100_0193);
        }
        hash
    }

    let application_key_count = security
        .key_table()
        .iter()
        .filter(|entry| entry.key_type == ApsKeyType::ApplicationLinkKey)
        .count();
    let mut hash = update(
        0x811C_9DC5,
        &[
            bindings.len() as u8,
            groups.len() as u8,
            application_key_count as u8,
        ],
    );
    if bindings.is_empty() && groups.is_empty() && application_key_count == 0 {
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
    for entry in security
        .key_table()
        .iter()
        .filter(|entry| entry.key_type == ApsKeyType::ApplicationLinkKey)
    {
        hash = update(hash, &entry.partner_address);
        hash = update(hash, &entry.key);
        hash = update(hash, &entry.outgoing_frame_counter_limit.to_le_bytes());
    }
    hash
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentApplicationLinkKey {
    pub partner_address: IeeeAddress,
    pub key: [u8; 16],
    pub outgoing_frame_counter_limit: u32,
    pub incoming_frame_counter: u32,
    pub incoming_frame_counter_valid: bool,
}

/// APS table persistence failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApsTableStoreError {
    PersistenceRequired,
    Corrupt,
    Full,
    Hardware,
    GenerationExhausted,
    ForeignNetwork,
    CounterExhausted,
}

/// Network-bound APS binding and group table snapshot.
#[derive(Debug, PartialEq, Eq)]
pub struct PersistentApsTables {
    extended_pan_id: IeeeAddress,
    bindings: BindingTable,
    groups: GroupTable,
    application_keys: Vec<PersistentApplicationLinkKey, MAX_KEY_TABLE_ENTRIES>,
}

impl Clone for PersistentApsTables {
    fn clone(&self) -> Self {
        Self {
            extended_pan_id: self.extended_pan_id,
            bindings: self.bindings.clone(),
            groups: self.groups.clone(),
            application_keys: self.application_keys.clone(),
        }
    }

    /// Field-wise, so a reload into caller-owned storage never materializes
    /// a whole second snapshot on the stack (HW-04).
    fn clone_from(&mut self, source: &Self) {
        self.extended_pan_id = source.extended_pan_id;
        self.bindings.clone_from(&source.bindings);
        self.groups.clone_from(&source.groups);
        self.application_keys.clone_from(&source.application_keys);
    }
}

impl PersistentApsTables {
    /// Empty snapshot bound to `extended_pan_id`.
    pub fn new(extended_pan_id: IeeeAddress) -> Self {
        Self {
            extended_pan_id,
            bindings: BindingTable::new(),
            groups: GroupTable::new(),
            application_keys: Vec::new(),
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
            application_keys: Vec::new(),
        };
        snapshot.validate()?;
        Ok(snapshot)
    }

    /// Capture APS tables and reserve finite counter ranges for every
    /// application link key.
    pub fn capture_with_security(
        extended_pan_id: IeeeAddress,
        bindings: &BindingTable,
        groups: &GroupTable,
        security: &ApsSecurity,
    ) -> Result<Self, ApsTableStoreError> {
        let mut snapshot = Self::new(extended_pan_id);
        snapshot.capture_with_security_into(extended_pan_id, bindings, groups, security)?;
        Ok(snapshot)
    }

    /// [`capture_with_security`](Self::capture_with_security) into existing
    /// storage, so callers keep exactly one snapshot on the stack.
    pub(crate) fn capture_with_security_into(
        &mut self,
        extended_pan_id: IeeeAddress,
        bindings: &BindingTable,
        groups: &GroupTable,
        security: &ApsSecurity,
    ) -> Result<(), ApsTableStoreError> {
        self.extended_pan_id = extended_pan_id;
        self.bindings.clone_from(bindings);
        self.groups.clone_from(groups);
        self.application_keys.clear();
        self.validate()?;
        let snapshot = self;
        for entry in security
            .key_table()
            .iter()
            .filter(|entry| entry.key_type == ApsKeyType::ApplicationLinkKey)
        {
            let low = entry
                .outgoing_frame_counter_limit
                .saturating_sub(entry.outgoing_frame_counter)
                <= APPLICATION_KEY_LOW_WATER;
            let limit = if entry.outgoing_frame_counter >= entry.outgoing_frame_counter_limit || low
            {
                let current = entry
                    .outgoing_frame_counter
                    .max(entry.outgoing_frame_counter_limit);
                current
                    .checked_add(APPLICATION_KEY_RESERVATION)
                    .ok_or(ApsTableStoreError::CounterExhausted)?
            } else {
                entry.outgoing_frame_counter_limit
            };
            snapshot
                .application_keys
                .push(PersistentApplicationLinkKey {
                    partner_address: entry.partner_address,
                    key: entry.key,
                    outgoing_frame_counter_limit: limit,
                    incoming_frame_counter: entry.incoming_frame_counter,
                    incoming_frame_counter_valid: entry.incoming_frame_counter_valid,
                })
                .map_err(|_| ApsTableStoreError::Full)?;
        }
        snapshot.validate()
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

    pub fn application_keys(&self) -> &[PersistentApplicationLinkKey] {
        self.application_keys.as_slice()
    }

    /// Mutable table access for moving a consumed snapshot into the live APS
    /// layer without materializing a clone (HW-04).
    pub(crate) fn tables_mut(&mut self) -> (&mut BindingTable, &mut GroupTable) {
        (&mut self.bindings, &mut self.groups)
    }

    pub(crate) fn application_keys_mut(&mut self) -> &mut [PersistentApplicationLinkKey] {
        self.application_keys.as_mut_slice()
    }

    pub fn is_empty(&self) -> bool {
        self.bindings.is_empty() && self.groups.is_empty() && self.application_keys.is_empty()
    }

    pub fn fingerprint(&self) -> u32 {
        let mut security = ApsSecurity::new();
        for stored in &self.application_keys {
            let _ = security.add_key(ApsLinkKeyEntry {
                partner_address: stored.partner_address,
                key: stored.key,
                key_type: ApsKeyType::ApplicationLinkKey,
                outgoing_frame_counter: stored.outgoing_frame_counter_limit.saturating_sub(1),
                outgoing_frame_counter_limit: stored.outgoing_frame_counter_limit,
                incoming_frame_counter: stored.incoming_frame_counter,
                incoming_frame_counter_valid: stored.incoming_frame_counter_valid,
            });
        }
        aps_table_fingerprint(
            self.extended_pan_id,
            &self.bindings,
            &self.groups,
            &security,
        )
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
        for (index, key) in self.application_keys.iter().enumerate() {
            if key.partner_address == [0u8; 8]
                || key.partner_address == [0xFFu8; 8]
                || key.outgoing_frame_counter_limit == 0
                || self.application_keys[index + 1..]
                    .iter()
                    .any(|other| other.partner_address == key.partner_address)
            {
                return Err(ApsTableStoreError::Corrupt);
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
        out[offset] = self.application_keys.len() as u8;
        offset += 1;
        for key in &self.application_keys {
            out[offset..offset + 8].copy_from_slice(&key.partner_address);
            out[offset + 8..offset + 24].copy_from_slice(&key.key);
            out[offset + 24..offset + 28]
                .copy_from_slice(&key.outgoing_frame_counter_limit.to_le_bytes());
            out[offset + 28..offset + 32]
                .copy_from_slice(&key.incoming_frame_counter.to_le_bytes());
            out[offset + 32] = u8::from(key.incoming_frame_counter_valid);
            offset += APPLICATION_KEY_ENTRY_LEN;
        }
        offset
    }

    /// Decode and validate a version-1 payload.
    pub fn decode(bytes: &[u8]) -> Result<Self, ApsTableStoreError> {
        let mut snapshot = Self::default();
        Self::decode_into(bytes, &mut snapshot)?;
        Ok(snapshot)
    }

    /// [`decode`](Self::decode) into existing storage without a by-value
    /// temporary. On error `snapshot` holds an unspecified partial value and
    /// must be discarded by the caller.
    pub(crate) fn decode_into(bytes: &[u8], snapshot: &mut Self) -> Result<(), ApsTableStoreError> {
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(bytes.get(0..8).ok_or(ApsTableStoreError::Corrupt)?);
        let binding_count = usize::from(*bytes.get(8).ok_or(ApsTableStoreError::Corrupt)?);
        if binding_count > MAX_BINDING_ENTRIES {
            return Err(ApsTableStoreError::Corrupt);
        }

        snapshot.extended_pan_id = extended_pan_id;
        snapshot.bindings.clear();
        snapshot.groups.clear();
        snapshot.application_keys.clear();
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
        // Version-1 snapshots ended after the group table. Treat them as
        // carrying no application keys.
        if offset == bytes.len() {
            return snapshot.validate();
        }
        let application_key_count =
            usize::from(*bytes.get(offset).ok_or(ApsTableStoreError::Corrupt)?);
        offset += 1;
        if application_key_count > MAX_KEY_TABLE_ENTRIES {
            return Err(ApsTableStoreError::Corrupt);
        }
        for _ in 0..application_key_count {
            let encoded = bytes
                .get(offset..offset + APPLICATION_KEY_ENTRY_LEN)
                .ok_or(ApsTableStoreError::Corrupt)?;
            if encoded[32] > 1 {
                return Err(ApsTableStoreError::Corrupt);
            }
            let mut partner_address = [0u8; 8];
            partner_address.copy_from_slice(&encoded[0..8]);
            let mut key = [0u8; 16];
            key.copy_from_slice(&encoded[8..24]);
            snapshot
                .application_keys
                .push(PersistentApplicationLinkKey {
                    partner_address,
                    key,
                    outgoing_frame_counter_limit: u32::from_le_bytes([
                        encoded[24],
                        encoded[25],
                        encoded[26],
                        encoded[27],
                    ]),
                    incoming_frame_counter: u32::from_le_bytes([
                        encoded[28],
                        encoded[29],
                        encoded[30],
                        encoded[31],
                    ]),
                    incoming_frame_counter_valid: encoded[32] != 0,
                })
                .map_err(|_| ApsTableStoreError::Full)?;
            offset += APPLICATION_KEY_ENTRY_LEN;
        }
        if offset != bytes.len() {
            return Err(ApsTableStoreError::Corrupt);
        }
        snapshot.validate()
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

    /// [`load`](Self::load) into caller-owned storage. Returns `Ok(false)`
    /// and leaves `tables` untouched when no snapshot is stored.
    fn load_into(&mut self, tables: &mut PersistentApsTables) -> Result<bool, ApsTableStoreError> {
        match self.load()? {
            Some(loaded) => {
                *tables = loaded;
                Ok(true)
            }
            None => Ok(false),
        }
    }

    /// Whether [`load`](Self::load) would return a non-empty snapshot,
    /// without materializing it.
    fn stored_is_nonempty(&mut self) -> Result<bool, ApsTableStoreError> {
        Ok(self.load()?.is_some_and(|tables| !tables.is_empty()))
    }
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
pub const APS_TABLE_JOURNAL_SLOT_SIZE: usize = 2048;
pub const APS_TABLE_JOURNAL_SLOTS_PER_SECTOR: usize =
    APS_TABLE_JOURNAL_SECTOR_SIZE / APS_TABLE_JOURNAL_SLOT_SIZE;

const RECORD_MAGIC: [u8; 4] = *b"ZBAT";
const RECORD_VERSION: u8 = 2;
const RECORD_ENCODED_OFFSET: usize = 12;
const RECORD_CRC_OFFSET: usize = APS_TABLE_JOURNAL_SLOT_SIZE - 12;
const RECORD_PREFIX_LEN: usize = APS_TABLE_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT_OFFSET: usize = APS_TABLE_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";
const LEGACY_SLOT_SIZE: usize = 1024;
const LEGACY_SLOTS_PER_SECTOR: usize = APS_TABLE_JOURNAL_SECTOR_SIZE / LEGACY_SLOT_SIZE;
const LEGACY_CRC_OFFSET: usize = LEGACY_SLOT_SIZE - 12;
const LEGACY_COMMIT_OFFSET: usize = LEGACY_SLOT_SIZE - 8;

/// Bounded read-back window for erase checks and committed-record
/// verification, so neither needs a second full slot buffer on the stack.
const READBACK_CHUNK: usize = 128;
/// v2 slots plus v1 legacy slots across both sectors.
const MAX_SCAN_CANDIDATES: usize =
    2 * (APS_TABLE_JOURNAL_SLOTS_PER_SECTOR + LEGACY_SLOTS_PER_SECTOR);

const _: () = assert!(RECORD_ENCODED_OFFSET + MAX_ENCODED_APS_TABLES_LEN <= RECORD_CRC_OFFSET);
const _: () = assert!(APS_TABLE_JOURNAL_SLOT_SIZE.is_multiple_of(READBACK_CHUNK));

/// Atomic two-sector APS table journal.
pub struct ApsTableJournal<S> {
    storage: S,
    sectors: [u32; 2],
    cached: Option<LocatedTables>,
    scanned: bool,
}

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
            && READBACK_CHUNK.is_multiple_of(S::READ_SIZE)
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

    /// Validate a committed v2 record's header, length and CRC and return its
    /// generation and payload range. The payload itself is decoded only for
    /// the selected candidate.
    fn record_header(record: &[u8; APS_TABLE_JOURNAL_SLOT_SIZE]) -> Option<(u32, usize)> {
        if record[0..4] != RECORD_MAGIC
            || !matches!(record[4], 1 | RECORD_VERSION)
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
        Some((generation, encoded_len))
    }

    /// v1 counterpart of [`record_header`](Self::record_header), over the
    /// first [`LEGACY_SLOT_SIZE`] bytes of `record`.
    fn legacy_record_header(record: &[u8; APS_TABLE_JOURNAL_SLOT_SIZE]) -> Option<(u32, usize)> {
        let record = &record[..LEGACY_SLOT_SIZE];
        if record[0..4] != RECORD_MAGIC
            || record[4] != 1
            || record[LEGACY_COMMIT_OFFSET..LEGACY_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let encoded_len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if RECORD_ENCODED_OFFSET + encoded_len > LEGACY_CRC_OFFSET {
            return None;
        }
        let expected_crc = u32::from_le_bytes([
            record[LEGACY_CRC_OFFSET],
            record[LEGACY_CRC_OFFSET + 1],
            record[LEGACY_CRC_OFFSET + 2],
            record[LEGACY_CRC_OFFSET + 3],
        ]);
        if crate::security_journal::crc32(&record[..LEGACY_CRC_OFFSET]) != expected_crc {
            return None;
        }
        let generation = u32::from_le_bytes([record[8], record[9], record[10], record[11]]);
        Some((generation, encoded_len))
    }

    /// Read candidate `index` (v2 slots first, then v1 legacy slots, each in
    /// sector/slot order) into `record` and validate its header.
    fn read_candidate(
        &mut self,
        index: usize,
        record: &mut [u8; APS_TABLE_JOURNAL_SLOT_SIZE],
    ) -> Result<Option<(u32, usize)>, ApsTableStoreError> {
        let v2_slots = 2 * APS_TABLE_JOURNAL_SLOTS_PER_SECTOR;
        if index < v2_slots {
            let sector = index / APS_TABLE_JOURNAL_SLOTS_PER_SECTOR;
            self.read_slot(sector, index % APS_TABLE_JOURNAL_SLOTS_PER_SECTOR, record)?;
            return Ok(Self::record_header(record));
        }
        if !LEGACY_SLOT_SIZE.is_multiple_of(S::READ_SIZE) {
            return Ok(None);
        }
        let index = index - v2_slots;
        let sector = index / LEGACY_SLOTS_PER_SECTOR;
        let slot = index % LEGACY_SLOTS_PER_SECTOR;
        self.storage
            .read(
                self.sectors[sector] + (slot * LEGACY_SLOT_SIZE) as u32,
                &mut record[..LEGACY_SLOT_SIZE],
            )
            .map_err(|_| ApsTableStoreError::Hardware)?;
        Ok(Self::legacy_record_header(record))
    }

    fn candidate_sector(index: usize) -> usize {
        let v2_slots = 2 * APS_TABLE_JOURNAL_SLOTS_PER_SECTOR;
        if index < v2_slots {
            index / APS_TABLE_JOURNAL_SLOTS_PER_SECTOR
        } else {
            (index - v2_slots) / LEGACY_SLOTS_PER_SECTOR
        }
    }

    /// Select the newest decodable record and decode it straight into the
    /// cache with one reusable slot buffer.
    ///
    /// Equivalent to decoding every committed record and keeping the first
    /// one with the highest generation: candidates are tried in descending
    /// generation and, for equal generations, in scan order, and the first one
    /// whose payload decodes wins. A malformed payload falls through to the
    /// next candidate exactly as before.
    fn scan_into_cache(&mut self) -> Result<(), ApsTableStoreError> {
        self.cached = None;
        let mut record = [0u8; APS_TABLE_JOURNAL_SLOT_SIZE];
        let mut generations = [None::<u32>; MAX_SCAN_CANDIDATES];
        for (index, generation) in generations.iter_mut().enumerate() {
            *generation = self.read_candidate(index, &mut record)?.map(|(g, _)| g);
        }
        while let Some(index) = (0..MAX_SCAN_CANDIDATES)
            .filter(|index| generations[*index].is_some())
            .min_by_key(|index| (core::cmp::Reverse(generations[*index]), *index))
        {
            let generation = generations[index].take();
            let Some((read_generation, encoded_len)) = self.read_candidate(index, &mut record)?
            else {
                continue;
            };
            if Some(read_generation) != generation {
                continue;
            }
            let located = self.cached.get_or_insert_with(|| LocatedTables {
                generation: 0,
                sector: 0,
                tables: PersistentApsTables::default(),
            });
            located.generation = read_generation;
            located.sector = Self::candidate_sector(index);
            if PersistentApsTables::decode_into(
                &record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len],
                &mut located.tables,
            )
            .is_ok()
            {
                return Ok(());
            }
        }
        self.cached = None;
        Ok(())
    }

    /// Scan once; any scan error leaves no cached state and forces a rescan.
    fn ensure_scanned(&mut self) -> Result<(), ApsTableStoreError> {
        if !self.geometry_ok() {
            return Err(ApsTableStoreError::Hardware);
        }
        if !self.scanned {
            if let Err(error) = self.scan_into_cache() {
                self.cached = None;
                return Err(error);
            }
            self.scanned = true;
        }
        Ok(())
    }

    /// Generation and sector of the newest committed record, without
    /// copying the tables.
    fn current_location(&mut self) -> Result<Option<(u32, usize)>, ApsTableStoreError> {
        self.ensure_scanned()?;
        Ok(self
            .cached
            .as_ref()
            .map(|located| (located.generation, located.sector)))
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, ApsTableStoreError> {
        let mut chunk = [0u8; READBACK_CHUNK];
        'slots: for slot in 0..APS_TABLE_JOURNAL_SLOTS_PER_SECTOR {
            let base = self.sectors[sector] + (slot * APS_TABLE_JOURNAL_SLOT_SIZE) as u32;
            for offset in (0..APS_TABLE_JOURNAL_SLOT_SIZE).step_by(READBACK_CHUNK) {
                self.storage
                    .read(base + offset as u32, &mut chunk)
                    .map_err(|_| ApsTableStoreError::Hardware)?;
                if chunk.iter().any(|byte| *byte != 0xFF) {
                    continue 'slots;
                }
            }
            return Ok(Some(slot));
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
        let payload =
            &mut record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + MAX_ENCODED_APS_TABLES_LEN];
        let encoded_len = match <&mut [u8; MAX_ENCODED_APS_TABLES_LEN]>::try_from(payload) {
            Ok(payload) => tables.encode(payload),
            Err(_) => return Err(ApsTableStoreError::Corrupt),
        };
        // `encode` zero-fills its whole buffer; unused payload bytes stay
        // erased exactly as in the original separately-encoded record.
        record[RECORD_ENCODED_OFFSET + encoded_len
            ..RECORD_ENCODED_OFFSET + MAX_ENCODED_APS_TABLES_LEN]
            .fill(0xFF);
        record[5..7].copy_from_slice(&(encoded_len as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        let crc = crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.sectors[sector] + (slot * APS_TABLE_JOURNAL_SLOT_SIZE) as u32;
        self.storage
            .write(address, &record[..RECORD_PREFIX_LEN])
            .map_err(|_| ApsTableStoreError::Hardware)?;
        // TLSR8258 can only program from SRAM; stage the marker on the stack.
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + RECORD_COMMIT_OFFSET as u32,
                &record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4],
            )
            .map_err(|_| ApsTableStoreError::Hardware)?;

        // Byte-exact read-back of the whole committed slot against the staged
        // record (header, payload, erased tail, CRC and commit marker).
        let mut chunk = [0u8; READBACK_CHUNK];
        for (offset, expected) in record.chunks_exact(READBACK_CHUNK).enumerate() {
            self.storage
                .read(address + (offset * READBACK_CHUNK) as u32, &mut chunk)
                .map_err(|_| ApsTableStoreError::Hardware)?;
            if chunk != *expected {
                return Err(ApsTableStoreError::Hardware);
            }
        }
        Ok(())
    }

    fn cache_result(
        &mut self,
        result: &Result<(), ApsTableStoreError>,
        generation: u32,
        sector: usize,
        tables: &PersistentApsTables,
    ) {
        if result.is_ok() {
            let located = self.cached.get_or_insert_with(|| LocatedTables {
                generation: 0,
                sector: 0,
                tables: PersistentApsTables::default(),
            });
            located.generation = generation;
            located.sector = sector;
            located.tables.clone_from(tables);
        } else {
            self.cached = None;
            self.scanned = false;
        }
    }
}

impl<S: NorFlash> ApsTableStore for ApsTableJournal<S> {
    fn load(&mut self) -> Result<Option<PersistentApsTables>, ApsTableStoreError> {
        self.ensure_scanned()?;
        Ok(self.cached.as_ref().map(|located| located.tables.clone()))
    }

    fn load_into(&mut self, tables: &mut PersistentApsTables) -> Result<bool, ApsTableStoreError> {
        self.ensure_scanned()?;
        match self.cached.as_ref() {
            Some(located) => {
                tables.clone_from(&located.tables);
                Ok(true)
            }
            None => Ok(false),
        }
    }

    fn stored_is_nonempty(&mut self) -> Result<bool, ApsTableStoreError> {
        self.ensure_scanned()?;
        Ok(self
            .cached
            .as_ref()
            .is_some_and(|located| !located.tables.is_empty()))
    }

    fn store(&mut self, tables: &PersistentApsTables) -> Result<(), ApsTableStoreError> {
        let current = self.current_location()?;
        let generation = match current {
            Some((generation, _)) => generation
                .checked_add(1)
                .ok_or(ApsTableStoreError::GenerationExhausted)?,
            None => 0,
        };

        if let Some((_, current_sector)) = current {
            if let Some(slot) = self.first_erased_slot(current_sector)? {
                let result = self.write_record(current_sector, slot, generation, tables);
                self.cache_result(&result, generation, current_sector, tables);
                return result;
            }
            let target = 1 - current_sector;
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
        fail_reads: bool,
        /// Absolute offset whose programming is silently skipped (a write
        /// that reports success but does not stick).
        drop_program_at: Option<usize>,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                bytes: [0xFF; APS_TABLE_JOURNAL_SECTOR_SIZE * 2],
                programs_before_failure: None,
                fail_reads: false,
                drop_program_at: None,
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = MockError;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            if self.fail_reads {
                return Err(MockError);
            }
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
            let drop_at = self.drop_program_at;
            let destination = self.bytes.get_mut(start..end).ok_or(MockError)?;
            for (index, (dst, src)) in destination.iter_mut().zip(bytes).enumerate() {
                if (*dst & *src) != *src {
                    return Err(MockError);
                }
                if drop_at != Some(start + index) {
                    *dst &= *src;
                }
            }
            Ok(())
        }
    }

    /// HW-04 HIL fixture: a valid APS journal owned by a foreign network, so a
    /// router restore takes `ForeignNetwork` -> clear -> `ApsTableStore::store`.
    /// Run with `HW04_FOREIGN_APS_IMAGE=<path> cargo test -- --ignored`.
    #[test]
    #[ignore = "writes the HW-04 HIL foreign-APS NV image"]
    fn hw04_dump_foreign_aps_image() {
        let path = std::env::var("HW04_FOREIGN_APS_IMAGE").expect("HW04_FOREIGN_APS_IMAGE");
        // HW04_APS_EPID (16 hex digits, on-air byte order) binds the fixture
        // to a real network so restore keeps it; default is foreign.
        let parse = |hex: std::string::String| {
            let mut bytes = [0u8; 8];
            for (i, byte) in bytes.iter_mut().enumerate() {
                *byte = u8::from_str_radix(&hex[2 * i..2 * i + 2], 16).unwrap();
            }
            bytes
        };
        let epid = std::env::var("HW04_APS_EPID").map_or([0x5A; 8], parse);
        let mut journal =
            ApsTableJournal::new(MockFlash::new(), 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        let mut tables = PersistentApsTables::new(epid);
        assert!(tables.groups.add_group(0x4242, 1));
        // HW04_APS_SRC_IEEE / HW04_APS_DST_IEEE (on-air order) add a group
        // and a unicast Identify binding owned by the device under test.
        if let Ok(src) = std::env::var("HW04_APS_SRC_IEEE").map(parse) {
            tables
                .bindings
                .add(BindingEntry::group(src, 1, 0x0003, 0x4242))
                .unwrap();
            if let Ok(dst) = std::env::var("HW04_APS_DST_IEEE").map(parse) {
                tables
                    .bindings
                    .add(BindingEntry::unicast(src, 1, 0x0003, dst, 1))
                    .unwrap();
            }
        }
        journal.store(&tables).unwrap();
        let flash = journal.into_storage();
        std::fs::write(path, &flash.bytes[..]).unwrap();
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
    fn journal_migrates_version_one_1024_byte_slots() {
        let expected = snapshot();
        let mut encoded = [0u8; MAX_ENCODED_APS_TABLES_LEN];
        let current_len = expected.encode(&mut encoded);
        let legacy_len = current_len - 1;
        let mut record = [0xFFu8; LEGACY_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = 1;
        record[5..7].copy_from_slice(&(legacy_len as u16).to_le_bytes());
        record[8..12].copy_from_slice(&7u32.to_le_bytes());
        record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + legacy_len]
            .copy_from_slice(&encoded[..legacy_len]);
        let crc = crate::security_journal::crc32(&record[..LEGACY_CRC_OFFSET]);
        record[LEGACY_CRC_OFFSET..LEGACY_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        record[LEGACY_COMMIT_OFFSET..LEGACY_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);

        let mut flash = MockFlash::new();
        flash.bytes[..LEGACY_SLOT_SIZE].copy_from_slice(&record);
        let mut journal = ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(journal.load(), Ok(Some(expected.clone())));

        let mut replacement = expected;
        assert!(replacement.groups.add_group(0x4567, 1));
        journal.store(&replacement).unwrap();
        let flash = journal.into_storage();
        let mut reopened = ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(reopened.load(), Ok(Some(replacement)));
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

    #[test]
    fn hw03_commit_marker_is_not_programmed_from_static_constant() {
        use crate::flash_source_guard::StaticSourceGuard;
        static FORBIDDEN: [&[u8]; 1] = [&RECORD_COMMIT];
        let mut journal = ApsTableJournal::new(
            StaticSourceGuard::new(MockFlash::new(), &FORBIDDEN),
            0,
            APS_TABLE_JOURNAL_SECTOR_SIZE as u32,
        );
        let tables = snapshot();
        journal.store(&tables).unwrap();
        assert_eq!(journal.storage().rejected, 0);
        assert_eq!(journal.load(), Ok(Some(tables)));
    }

    fn open_journal(flash: MockFlash) -> ApsTableJournal<MockFlash> {
        ApsTableJournal::new(flash, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32)
    }

    /// Tables at every capacity limit, so the in-place encoding covers the
    /// largest payload the record can carry.
    fn maximal_snapshot() -> PersistentApsTables {
        let mut tables = PersistentApsTables::new(EPID);
        for index in 0..MAX_BINDING_ENTRIES {
            let entry = if index % 2 == 0 {
                BindingEntry::unicast(LOCAL, 1, index as u16, REMOTE, (index % 200 + 1) as u8)
            } else {
                BindingEntry::group(LOCAL, 2, index as u16, 0x1000 + index as u16)
            };
            tables.bindings.add(entry).unwrap();
        }
        for group in 0..MAX_GROUPS {
            for endpoint in 1..=MAX_ENDPOINTS_PER_GROUP {
                assert!(
                    tables
                        .groups
                        .add_group(0x2000 + group as u16, endpoint as u8)
                );
            }
        }
        for index in 0..MAX_KEY_TABLE_ENTRIES {
            let mut partner_address = [0x40; 8];
            partner_address[7] = index as u8 + 1;
            tables
                .application_keys
                .push(PersistentApplicationLinkKey {
                    partner_address,
                    key: [index as u8; 16],
                    outgoing_frame_counter_limit: 0x1000 + index as u32,
                    incoming_frame_counter: index as u32,
                    incoming_frame_counter_valid: index % 2 == 0,
                })
                .unwrap();
        }
        tables.validate().unwrap();
        tables
    }

    /// Independent reference for the committed v2 slot layout, built the way
    /// the journal did before HW-04 (separate encode buffer, then copy).
    fn reference_record(generation: u32, tables: &PersistentApsTables) -> [u8; 2048] {
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
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        record
    }

    /// Committed v2 record with a valid CRC around an arbitrary payload.
    fn raw_record(generation: u32, payload: &[u8]) -> [u8; 2048] {
        let mut record = [0xFFu8; APS_TABLE_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = RECORD_VERSION;
        record[5..7].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + payload.len()]
            .copy_from_slice(payload);
        let crc = crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        record
    }

    fn encoded(tables: &PersistentApsTables) -> ([u8; MAX_ENCODED_APS_TABLES_LEN], usize) {
        let mut encoded = [0u8; MAX_ENCODED_APS_TABLES_LEN];
        let len = tables.encode(&mut encoded);
        (encoded, len)
    }

    #[test]
    fn hw04_in_place_encoding_keeps_exact_on_flash_bytes() {
        for tables in [
            PersistentApsTables::new(EPID),
            snapshot(),
            maximal_snapshot(),
        ] {
            let mut journal = open_journal(MockFlash::new());
            journal.store(&tables).unwrap();
            journal.store(&tables).unwrap();
            let flash = journal.into_storage();
            assert_eq!(flash.bytes[..2048], reference_record(0, &tables));
            assert_eq!(flash.bytes[2048..4096], reference_record(1, &tables));
            let mut reopened = open_journal(flash);
            assert_eq!(reopened.load(), Ok(Some(tables)));
        }
    }

    #[test]
    fn hw04_malformed_newest_payload_falls_back_to_previous_generation() {
        let older = snapshot();
        let mut journal = open_journal(MockFlash::new());
        journal.store(&older).unwrap();
        let mut flash = journal.into_storage();
        // CRC-valid, committed, higher generation, but undecodable payload.
        let (mut payload, len) = encoded(&older);
        payload[9 + 11] = 0xFF;
        flash.bytes[APS_TABLE_JOURNAL_SECTOR_SIZE..APS_TABLE_JOURNAL_SECTOR_SIZE + 2048]
            .copy_from_slice(&raw_record(9, &payload[..len]));

        let mut reopened = open_journal(flash);
        assert_eq!(reopened.load(), Ok(Some(older.clone())));
        // The next generation follows the selected decodable record.
        let mut newer = older;
        assert!(newer.groups.add_group(0x7777, 3));
        reopened.store(&newer).unwrap();
        let flash = reopened.into_storage();
        assert_eq!(flash.bytes[2048..4096], reference_record(1, &newer));
        assert_eq!(open_journal(flash).load(), Ok(Some(newer)));
    }

    #[test]
    fn hw04_equal_generations_keep_the_first_record_in_scan_order() {
        let first = snapshot();
        let mut second = snapshot();
        assert!(second.groups.add_group(0x5555, 4));
        let mut flash = MockFlash::new();
        let (payload, len) = encoded(&first);
        flash.bytes[..2048].copy_from_slice(&raw_record(4, &payload[..len]));
        let (payload, len) = encoded(&second);
        flash.bytes[APS_TABLE_JOURNAL_SECTOR_SIZE..APS_TABLE_JOURNAL_SECTOR_SIZE + 2048]
            .copy_from_slice(&raw_record(4, &payload[..len]));
        assert_eq!(open_journal(flash).load(), Ok(Some(first)));
    }

    #[test]
    fn hw04_verify_mismatch_is_a_hardware_error_and_invalidates_the_cache() {
        let expected = snapshot();
        let mut journal = open_journal(MockFlash::new());
        journal.store(&expected).unwrap();

        let mut replacement = expected.clone();
        assert!(replacement.groups.add_group(0x4567, 1));
        // A payload byte in slot 1 silently fails to program.
        journal.storage.drop_program_at = Some(2048 + RECORD_ENCODED_OFFSET);
        assert_eq!(
            journal.store(&replacement),
            Err(ApsTableStoreError::Hardware)
        );
        assert!(journal.cached.is_none() && !journal.scanned);
        journal.storage.drop_program_at = None;
        // The CRC no longer matches, so the rescan returns the old generation.
        assert_eq!(journal.load(), Ok(Some(expected)));
    }

    #[test]
    fn hw04_scan_read_error_leaves_no_cached_state() {
        let expected = snapshot();
        let mut journal = open_journal(MockFlash::new());
        journal.store(&expected).unwrap();
        let mut flash = journal.into_storage();
        flash.fail_reads = true;
        let mut reopened = open_journal(flash);
        assert_eq!(reopened.load(), Err(ApsTableStoreError::Hardware));
        assert!(reopened.cached.is_none() && !reopened.scanned);
        assert_eq!(reopened.store(&expected), Err(ApsTableStoreError::Hardware));
        reopened.storage.fail_reads = false;
        assert_eq!(reopened.load(), Ok(Some(expected)));
    }

    #[test]
    fn hw04_store_after_restore_uses_cached_location_without_clone() {
        // Restore, then store twice more: generations 1 and 2 roll into the
        // second sector exactly as before, from location metadata alone.
        let mut tables = snapshot();
        let mut journal = open_journal(MockFlash::new());
        journal.store(&tables).unwrap();
        journal.store(&tables).unwrap();
        let mut reopened = open_journal(journal.into_storage());
        assert_eq!(reopened.load(), Ok(Some(tables.clone())));
        assert!(tables.groups.add_group(0x6666, 5));
        reopened.store(&tables).unwrap();
        let flash = reopened.into_storage();
        assert_eq!(
            flash.bytes[APS_TABLE_JOURNAL_SECTOR_SIZE..APS_TABLE_JOURNAL_SECTOR_SIZE + 2048],
            reference_record(2, &tables)
        );
        assert_eq!(open_journal(flash).load(), Ok(Some(tables)));
    }

    /// HW-04: the journal's `load_into` / `stored_is_nonempty` overrides
    /// agree with `load` and with the trait defaults (via `RamApsTableStore`)
    /// for absent, empty and populated snapshots, including after reopen.
    #[test]
    fn hw04_load_into_and_stored_is_nonempty_match_load() {
        let sentinel = maximal_snapshot();
        let mut ram = RamApsTableStore::new();
        let mut journal = open_journal(MockFlash::new());

        let mut target = sentinel.clone();
        assert_eq!(journal.load_into(&mut target), Ok(false));
        assert_eq!(target, sentinel);
        assert_eq!(ram.load_into(&mut target), Ok(false));
        assert_eq!(target, sentinel);
        assert_eq!(journal.stored_is_nonempty(), Ok(false));
        assert_eq!(ram.stored_is_nonempty(), Ok(false));

        for tables in [PersistentApsTables::new(EPID), snapshot()] {
            journal.store(&tables).unwrap();
            ram.store(&tables).unwrap();
            let expected_nonempty = !tables.is_empty();
            for store in [&mut journal as &mut dyn ApsTableStore, &mut ram] {
                let mut target = sentinel.clone();
                assert_eq!(store.load_into(&mut target), Ok(true));
                assert_eq!(target, tables);
                assert_eq!(store.stored_is_nonempty(), Ok(expected_nonempty));
            }
        }

        let mut reopened = open_journal(journal.into_storage());
        assert_eq!(reopened.stored_is_nonempty(), Ok(true));
        let mut target = sentinel;
        assert_eq!(reopened.load_into(&mut target), Ok(true));
        assert_eq!(target, snapshot());

        reopened.storage.fail_reads = true;
        let mut failing = open_journal(reopened.into_storage());
        assert_eq!(
            failing.stored_is_nonempty(),
            Err(ApsTableStoreError::Hardware)
        );
        let mut untouched = snapshot();
        assert_eq!(
            failing.load_into(&mut untouched),
            Err(ApsTableStoreError::Hardware)
        );
        assert_eq!(untouched, snapshot());
    }
}
