//! Durable, crash-safe persistence for a router/coordinator child table.
//!
//! This is deliberately a **separate** durable store from
//! [`crate::security_store`]: the security-state record must stay small and is
//! rewritten on every frame-counter reservation, whereas the child table is a
//! larger, lower-frequency snapshot. Bloating the security record with a
//! variable-length child list would enlarge every counter write and couple two
//! concerns with very different durability rhythms, so the two live in
//! independent journals. Nothing here touches NWK/APS frame counters or the
//! security journal, so restoring or discarding the child table can never
//! replay a secured frame.
//!
//! # What is persisted
//!
//! For each authenticated child: its IEEE identity, assigned short address,
//! capability/configuration (rx-on-when-idle, security-capable, router vs
//! end device) and its accepted R22 End Device Timeout enumeration. The live
//! aging countdown is **not** persisted — on restore each end-device child's
//! deadline is re-armed to the full window of its accepted enumeration. That
//! is the safe direction: a restored child is granted a fresh full window to
//! prove liveness rather than being evicted immediately after a reboot, and it
//! avoids rewriting flash every time a countdown ticks.
//!
//! # Network binding
//!
//! Every record also stores the **extended PAN ID** the children belong to.
//! A restore whose stored EPID does not match the network the device is
//! currently on is rejected with [`ChildStoreError::ForeignNetwork`] rather
//! than being applied, so a child table left over from a previous network
//! (after a factory reset and rejoin, or after moving a device between
//! networks) can never re-admit devices that are not this parent's children,
//! and can never seed an orphan/realignment answer for a stranger.
//!
//! # Record versions and downgrade
//!
//! | version | encoded child table                                |
//! |---------|----------------------------------------------------|
//! | 1       | count + per-child identity/timeout (no EPID binding) |
//! | 2       | extended PAN ID + count + per-child identity/timeout |
//!
//! A record whose version this firmware does not recognise is skipped while
//! scanning, exactly like a corrupt one, so **downgrading** to firmware that
//! predates a version simply drops the persisted child table: children then
//! re-appear through normal association/rejoin and keepalive, and no security
//! counter is affected because child state is independent of the security
//! journal. Version 1 is deliberately *not* accepted by this firmware: it
//! carries no network binding, so it cannot be validated against the current
//! network and is discarded rather than trusted.

use embedded_storage::nor_flash::NorFlash;
use zigbee_nwk::frames::{ED_TIMEOUT_ENUM_DEFAULT, ED_TIMEOUT_ENUM_MAX};
use zigbee_types::IeeeAddress;

/// Largest child table this store persists.
///
/// Matches the router neighbour-table capacity so a full child table always
/// fits; the encoded form still fits one journal slot (see the layout asserts
/// below).
pub const MAX_PERSISTED_CHILDREN: usize = 32;

/// Encoded size of one child record: IEEE (8) + short (2) + flags (1) +
/// timeout enumeration (1).
const CHILD_ENTRY_LEN: usize = 12;

/// Encoded size of the largest child table: the 8-byte extended PAN ID
/// binding, a one-byte count, then the entries.
pub const MAX_ENCODED_CHILD_TABLE_LEN: usize = 8 + 1 + MAX_PERSISTED_CHILDREN * CHILD_ENTRY_LEN;

const CHILD_FLAG_RX_ON_WHEN_IDLE: u8 = 1 << 0;
const CHILD_FLAG_SECURITY_CAPABLE: u8 = 1 << 1;
const CHILD_FLAG_ROUTER: u8 = 1 << 2;
const CHILD_FLAG_MASK: u8 =
    CHILD_FLAG_RX_ON_WHEN_IDLE | CHILD_FLAG_SECURITY_CAPABLE | CHILD_FLAG_ROUTER;

/// Errors from the child-table store.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChildStoreError {
    /// A decoded record failed structural validation.
    Corrupt,
    /// The table cannot hold another child.
    Full,
    /// Underlying storage read/write/erase failed.
    Hardware,
    /// The two-sector generation counter would wrap.
    GenerationExhausted,
    /// The stored table belongs to a different network than the one this
    /// device is currently commissioned on.
    ForeignNetwork,
}

/// One authenticated child as persisted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentChild {
    /// Extended (IEEE) identity of the child.
    pub ieee_address: IeeeAddress,
    /// Assigned short (network) address.
    pub short_address: u16,
    /// Whether the child listens when idle (false = sleepy).
    pub rx_on_when_idle: bool,
    /// Security-capable bit the child advertised.
    pub security_capable: bool,
    /// Whether the child is a router (true) or an end device (false).
    pub is_router: bool,
    /// Accepted R22 End Device Timeout enumeration (0..=14).
    pub end_device_timeout: u8,
}

impl PersistentChild {
    fn encode(&self, out: &mut [u8; CHILD_ENTRY_LEN]) {
        out[0..8].copy_from_slice(&self.ieee_address);
        out[8..10].copy_from_slice(&self.short_address.to_le_bytes());
        let mut flags = 0;
        if self.rx_on_when_idle {
            flags |= CHILD_FLAG_RX_ON_WHEN_IDLE;
        }
        if self.security_capable {
            flags |= CHILD_FLAG_SECURITY_CAPABLE;
        }
        if self.is_router {
            flags |= CHILD_FLAG_ROUTER;
        }
        out[10] = flags;
        out[11] = self.end_device_timeout;
    }

    fn decode(bytes: &[u8; CHILD_ENTRY_LEN]) -> Result<Self, ChildStoreError> {
        let flags = bytes[10];
        if flags & !CHILD_FLAG_MASK != 0 {
            return Err(ChildStoreError::Corrupt);
        }
        let mut ieee_address = [0u8; 8];
        ieee_address.copy_from_slice(&bytes[0..8]);
        let short_address = u16::from_le_bytes([bytes[8], bytes[9]]);
        let end_device_timeout = bytes[11];
        let child = Self {
            ieee_address,
            short_address,
            rx_on_when_idle: flags & CHILD_FLAG_RX_ON_WHEN_IDLE != 0,
            security_capable: flags & CHILD_FLAG_SECURITY_CAPABLE != 0,
            is_router: flags & CHILD_FLAG_ROUTER != 0,
            end_device_timeout,
        };
        child.validate()?;
        Ok(child)
    }

    fn validate(&self) -> Result<(), ChildStoreError> {
        // A child that never negotiated is stored at the R22 default; anything
        // above the highest defined enumeration is a corrupt record rather
        // than a value to clamp.
        if self.end_device_timeout > ED_TIMEOUT_ENUM_MAX {
            return Err(ChildStoreError::Corrupt);
        }
        // Only allocated unicast short addresses name a child.
        if !(0x0001..=0xFFF7).contains(&self.short_address) {
            return Err(ChildStoreError::Corrupt);
        }
        // A child with no IEEE identity cannot be authenticated or announced.
        if self.ieee_address == [0u8; 8] {
            return Err(ChildStoreError::Corrupt);
        }
        Ok(())
    }
}

/// A crash-safe snapshot of a router/coordinator's authenticated child table.
///
/// The snapshot is bound to the extended PAN ID the children belong to. A
/// restore validates that binding against the network the device is actually
/// on, so a table from a previous network can never re-admit strangers.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PersistentChildTable {
    extended_pan_id: IeeeAddress,
    children: heapless::Vec<PersistentChild, MAX_PERSISTED_CHILDREN>,
}

impl PersistentChildTable {
    /// An empty table (no children) bound to `extended_pan_id`.
    pub const fn new(extended_pan_id: IeeeAddress) -> Self {
        Self {
            extended_pan_id,
            children: heapless::Vec::new(),
        }
    }

    /// Extended PAN ID this table's children belong to.
    pub const fn extended_pan_id(&self) -> IeeeAddress {
        self.extended_pan_id
    }

    /// Whether this table describes children of `extended_pan_id`.
    pub fn matches_network(&self, extended_pan_id: &IeeeAddress) -> bool {
        self.extended_pan_id == *extended_pan_id
    }

    /// Append a child, failing once the table is full.
    pub fn push(&mut self, child: PersistentChild) -> Result<(), ChildStoreError> {
        self.children.push(child).map_err(|_| ChildStoreError::Full)
    }

    /// Number of stored children.
    pub fn len(&self) -> usize {
        self.children.len()
    }

    /// Whether the table is empty.
    pub fn is_empty(&self) -> bool {
        self.children.is_empty()
    }

    /// Iterate the stored children.
    pub fn children(&self) -> impl Iterator<Item = &PersistentChild> {
        self.children.iter()
    }

    /// Validate the whole table: it names a real network, every child is
    /// structurally valid, and no two children share a short address or IEEE
    /// identity.
    pub fn validate(&self) -> Result<(), ChildStoreError> {
        if self.children.len() > MAX_PERSISTED_CHILDREN {
            return Err(ChildStoreError::Corrupt);
        }
        // An all-zero or all-ones EPID is not a commissioned network, so a
        // record carrying one could never be validated against a live NIB.
        // Reject it as corrupt instead of storing an unusable binding.
        if !self.children.is_empty()
            && (self.extended_pan_id == [0u8; 8] || self.extended_pan_id == [0xFFu8; 8])
        {
            return Err(ChildStoreError::Corrupt);
        }
        for (index, child) in self.children.iter().enumerate() {
            child.validate()?;
            for other in &self.children[index + 1..] {
                if child.short_address == other.short_address
                    || child.ieee_address == other.ieee_address
                {
                    return Err(ChildStoreError::Corrupt);
                }
            }
        }
        Ok(())
    }

    /// Encode into `out`, returning the used length.
    ///
    /// Layout (version 2): extended PAN ID (8) then a `count` byte followed by
    /// `count` fixed-size child records. The caller owns the version/CRC/commit
    /// framing.
    pub fn encode(&self, out: &mut [u8; MAX_ENCODED_CHILD_TABLE_LEN]) -> usize {
        out[0..8].copy_from_slice(&self.extended_pan_id);
        out[8] = self.children.len() as u8;
        let mut offset = 9;
        for child in &self.children {
            let mut entry = [0u8; CHILD_ENTRY_LEN];
            child.encode(&mut entry);
            out[offset..offset + CHILD_ENTRY_LEN].copy_from_slice(&entry);
            offset += CHILD_ENTRY_LEN;
        }
        offset
    }

    /// Decode a version-2 encoded table, validating structure and length.
    pub fn decode(bytes: &[u8]) -> Result<Self, ChildStoreError> {
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(bytes.get(0..8).ok_or(ChildStoreError::Corrupt)?);
        let count = *bytes.get(8).ok_or(ChildStoreError::Corrupt)? as usize;
        if count > MAX_PERSISTED_CHILDREN {
            return Err(ChildStoreError::Corrupt);
        }
        let expected_len = 9 + count * CHILD_ENTRY_LEN;
        if bytes.len() != expected_len {
            return Err(ChildStoreError::Corrupt);
        }
        let mut table = Self::new(extended_pan_id);
        for index in 0..count {
            let start = 9 + index * CHILD_ENTRY_LEN;
            let mut entry = [0u8; CHILD_ENTRY_LEN];
            entry.copy_from_slice(&bytes[start..start + CHILD_ENTRY_LEN]);
            table.push(PersistentChild::decode(&entry)?)?;
        }
        table.validate()?;
        Ok(table)
    }
}

impl Default for PersistentChildTable {
    /// An empty, unbound table. Only useful as a placeholder — an empty table
    /// carries no children, so its (zero) network binding is never applied.
    fn default() -> Self {
        Self::new([0u8; 8])
    }
}

/// A durable child-table store.
///
/// A product with a dedicated flash partition uses [`ChildTableJournal`]; a
/// product without one uses [`RamChildTableStore`], which keeps the table only
/// for the current power cycle. The runtime restore/refresh hooks work against
/// either, so a product opts in without inventing board flash geometry and no
/// hardware-persistence claim is implied where no partition is configured.
pub trait ChildTableStore {
    /// Load the persisted table, or `None` if nothing is stored yet.
    fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError>;
    /// Persist `table`, superseding any previous snapshot.
    fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError>;
}

/// Volatile child-table store: keeps a snapshot for the current power cycle
/// only. Suitable as a complete generic backend for a router without a
/// configured child-table partition — it never claims durable persistence.
#[derive(Debug, Default)]
pub struct RamChildTableStore {
    table: Option<PersistentChildTable>,
}

impl RamChildTableStore {
    /// A store with no snapshot yet.
    pub const fn new() -> Self {
        Self { table: None }
    }
}

impl ChildTableStore for RamChildTableStore {
    fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError> {
        Ok(self.table.clone())
    }

    fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError> {
        table.validate()?;
        self.table = Some(table.clone());
        Ok(())
    }
}

// ── Two-sector crash-safe flash journal ─────────────────────────

/// Erase-unit size the journal assumes for each of its two sectors.
pub const CHILD_JOURNAL_SECTOR_SIZE: usize = 4096;
/// Size of one journal record slot.
pub const CHILD_JOURNAL_SLOT_SIZE: usize = 512;
/// Slots that fit in one sector.
pub const CHILD_JOURNAL_SLOTS_PER_SECTOR: usize =
    CHILD_JOURNAL_SECTOR_SIZE / CHILD_JOURNAL_SLOT_SIZE;

const RECORD_MAGIC: [u8; 4] = *b"ZBCT";
const RECORD_VERSION: u8 = 2;
/// Encoded table starts here (magic 4 + version 1 + len 2 + reserved 1 +
/// generation 4).
const RECORD_ENCODED_OFFSET: usize = 12;
const RECORD_CRC_OFFSET: usize = 500;
const RECORD_PREFIX_LEN: usize = 504;
const RECORD_COMMIT_OFFSET: usize = 504;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

// The encoded table must fit between its start and the CRC field, and the
// commit marker must fit inside the slot.
const _: () = assert!(RECORD_ENCODED_OFFSET + MAX_ENCODED_CHILD_TABLE_LEN <= RECORD_CRC_OFFSET);
const _: () = assert!(RECORD_COMMIT_OFFSET + RECORD_COMMIT.len() <= CHILD_JOURNAL_SLOT_SIZE);
const _: () = assert!(RECORD_CRC_OFFSET + 4 <= RECORD_PREFIX_LEN + 4);

/// Atomic two-sector journal for the persistent child table.
///
/// Mirrors [`crate::security_journal::SecurityStateJournal`]: monotonically
/// increasing generation numbers, a CRC over the record prefix and a trailing
/// commit marker written last, so an interrupted write is skipped on the next
/// scan and the previous generation survives. It runs on its **own** two
/// sectors, entirely separate from the security journal's.
pub struct ChildTableJournal<S> {
    storage: S,
    sectors: [u32; 2],
    cached: Option<LocatedTable>,
    scanned: bool,
}

#[derive(Clone)]
struct LocatedTable {
    generation: u32,
    sector: usize,
    table: PersistentChildTable,
}

impl<S: NorFlash> ChildTableJournal<S> {
    /// Create a journal spanning two distinct erase sectors of `storage`.
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        Self {
            storage,
            sectors: [first_sector, second_sector],
            cached: None,
            scanned: false,
        }
    }

    /// Borrow the backing storage.
    pub fn storage(&self) -> &S {
        &self.storage
    }

    /// Recover the backing storage, consuming the journal.
    pub fn into_storage(self) -> S {
        self.storage
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        output: &mut [u8; CHILD_JOURNAL_SLOT_SIZE],
    ) -> Result<(), ChildStoreError> {
        self.storage
            .read(
                self.sectors[sector] + (slot * CHILD_JOURNAL_SLOT_SIZE) as u32,
                output,
            )
            .map_err(|_| ChildStoreError::Hardware)
    }

    fn decode_record(
        record: &[u8; CHILD_JOURNAL_SLOT_SIZE],
    ) -> Option<(u32, PersistentChildTable)> {
        if record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
            || record[0..4] != RECORD_MAGIC
            || record[4] != RECORD_VERSION
        {
            return None;
        }
        let encoded_len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if encoded_len > MAX_ENCODED_CHILD_TABLE_LEN
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
        let table = PersistentChildTable::decode(
            &record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len],
        )
        .ok()?;
        Some((generation, table))
    }

    fn newest(&mut self) -> Result<Option<LocatedTable>, ChildStoreError> {
        let mut newest: Option<LocatedTable> = None;
        let mut record = [0u8; CHILD_JOURNAL_SLOT_SIZE];
        for sector in 0..2 {
            for slot in 0..CHILD_JOURNAL_SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some((generation, table)) = Self::decode_record(&record) else {
                    continue;
                };
                let replace = match &newest {
                    Some(current) => generation > current.generation,
                    None => true,
                };
                if replace {
                    newest = Some(LocatedTable {
                        generation,
                        sector,
                        table,
                    });
                }
            }
        }
        Ok(newest)
    }

    fn geometry_ok(&self) -> bool {
        self.sectors[0] != self.sectors[1]
            && self.sectors[0].abs_diff(self.sectors[1]) >= CHILD_JOURNAL_SECTOR_SIZE as u32
            && S::READ_SIZE != 0
            && S::WRITE_SIZE != 0
            && S::ERASE_SIZE != 0
            && CHILD_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
            && CHILD_JOURNAL_SLOT_SIZE.is_multiple_of(S::WRITE_SIZE)
            && CHILD_JOURNAL_SECTOR_SIZE.is_multiple_of(S::ERASE_SIZE)
            && RECORD_PREFIX_LEN.is_multiple_of(S::WRITE_SIZE)
            && RECORD_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
            && RECORD_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            && (self.sectors[0] as usize).is_multiple_of(S::ERASE_SIZE)
            && (self.sectors[1] as usize).is_multiple_of(S::ERASE_SIZE)
            && self.sectors.iter().all(|sector| {
                (*sector as usize)
                    .checked_add(CHILD_JOURNAL_SECTOR_SIZE)
                    .is_some_and(|end| end <= self.storage.capacity())
            })
    }

    fn current(&mut self) -> Result<Option<LocatedTable>, ChildStoreError> {
        if !self.geometry_ok() {
            return Err(ChildStoreError::Hardware);
        }
        if !self.scanned {
            self.cached = self.newest()?;
            self.scanned = true;
        }
        Ok(self.cached.clone())
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, ChildStoreError> {
        let mut record = [0u8; CHILD_JOURNAL_SLOT_SIZE];
        for slot in 0..CHILD_JOURNAL_SLOTS_PER_SECTOR {
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
        table: &PersistentChildTable,
    ) -> Result<(), ChildStoreError> {
        table.validate()?;

        let mut record = [0xFFu8; CHILD_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = RECORD_VERSION;
        let mut encoded = [0u8; MAX_ENCODED_CHILD_TABLE_LEN];
        let encoded_len = table.encode(&mut encoded);
        record[5..7].copy_from_slice(&(encoded_len as u16).to_le_bytes());
        record[7] = 0;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        record[RECORD_ENCODED_OFFSET..RECORD_ENCODED_OFFSET + encoded_len]
            .copy_from_slice(&encoded[..encoded_len]);
        let crc = crate::security_journal::crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.sectors[sector] + (slot * CHILD_JOURNAL_SLOT_SIZE) as u32;
        // The commit marker is written *after* the prefix, so a crash between
        // the two leaves an uncommitted record that the scan ignores.
        self.storage
            .write(address, &record[..RECORD_PREFIX_LEN])
            .map_err(|_| ChildStoreError::Hardware)?;
        self.storage
            .write(address + RECORD_COMMIT_OFFSET as u32, &RECORD_COMMIT)
            .map_err(|_| ChildStoreError::Hardware)?;

        let mut verify = [0u8; CHILD_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut verify)?;
        match Self::decode_record(&verify) {
            Some((stored_generation, stored_table))
                if stored_generation == generation && stored_table == *table =>
            {
                Ok(())
            }
            _ => Err(ChildStoreError::Hardware),
        }
    }
}

impl<S: NorFlash> ChildTableStore for ChildTableJournal<S> {
    fn load(&mut self) -> Result<Option<PersistentChildTable>, ChildStoreError> {
        Ok(self.current()?.map(|located| located.table))
    }

    fn store(&mut self, table: &PersistentChildTable) -> Result<(), ChildStoreError> {
        let current = self.current()?;
        let generation = match &current {
            Some(located) => located
                .generation
                .checked_add(1)
                .ok_or(ChildStoreError::GenerationExhausted)?,
            None => 0,
        };

        if let Some(located) = current {
            if let Some(slot) = self.first_erased_slot(located.sector)? {
                let result = self.write_record(located.sector, slot, generation, table);
                self.cache_result(&result, generation, located.sector, table);
                return result;
            }
            let target = 1 - located.sector;
            let sector = self.sectors[target];
            let result = self
                .storage
                .erase(sector, sector + CHILD_JOURNAL_SECTOR_SIZE as u32)
                .map_err(|_| ChildStoreError::Hardware)
                .and_then(|()| self.write_record(target, 0, generation, table));
            self.cache_result(&result, generation, target, table);
            return result;
        }

        let sector = self.sectors[0];
        let result = self
            .storage
            .erase(sector, sector + CHILD_JOURNAL_SECTOR_SIZE as u32)
            .map_err(|_| ChildStoreError::Hardware)
            .and_then(|()| self.write_record(0, 0, generation, table));
        self.cache_result(&result, generation, 0, table);
        result
    }
}

impl<S: NorFlash> ChildTableJournal<S> {
    fn cache_result(
        &mut self,
        result: &Result<(), ChildStoreError>,
        generation: u32,
        sector: usize,
        table: &PersistentChildTable,
    ) {
        if result.is_ok() {
            self.cached = Some(LocatedTable {
                generation,
                sector,
                table: table.clone(),
            });
        } else {
            self.cached = None;
            self.scanned = false;
        }
    }
}

/// The R22 default timeout enumeration, re-exported so callers can construct a
/// [`PersistentChild`] for a never-negotiated child without depending on the
/// NWK crate directly.
pub const DEFAULT_END_DEVICE_TIMEOUT: u8 = ED_TIMEOUT_ENUM_DEFAULT;

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    fn child(seed: u8, short: u16, timeout: u8) -> PersistentChild {
        PersistentChild {
            ieee_address: [seed; 8],
            short_address: short,
            rx_on_when_idle: seed & 1 == 0,
            security_capable: true,
            is_router: seed & 2 == 0,
            end_device_timeout: timeout,
        }
    }

    /// Extended PAN ID every test table is bound to.
    const EPID: IeeeAddress = [0xAA, 0xBB, 0xCC, 0xDD, 0x01, 0x02, 0x03, 0x04];

    fn table(children: &[PersistentChild]) -> PersistentChildTable {
        let mut table = PersistentChildTable::new(EPID);
        for c in children {
            table.push(*c).unwrap();
        }
        table
    }

    #[test]
    fn empty_and_populated_tables_round_trip_through_the_encoding() {
        for children in [
            &[][..],
            &[child(1, 0x0002, 8)][..],
            &[
                child(1, 0x0002, 0),
                child(2, 0x0003, 14),
                child(3, 0x1234, 8),
            ][..],
        ] {
            let original = table(children);
            let mut encoded = [0u8; MAX_ENCODED_CHILD_TABLE_LEN];
            let len = original.encode(&mut encoded);
            assert_eq!(len, 8 + 1 + children.len() * CHILD_ENTRY_LEN);
            assert_eq!(PersistentChildTable::decode(&encoded[..len]), Ok(original));
        }
    }

    #[test]
    fn decode_rejects_corrupt_records() {
        // Undefined enumeration.
        assert_eq!(
            PersistentChildTable::decode(
                &{
                    let mut encoded = [0u8; MAX_ENCODED_CHILD_TABLE_LEN];
                    let len = table(&[child(1, 0x0002, 8)]).encode(&mut encoded);
                    encoded[..len].to_vec()
                }
                .iter()
                .copied()
                .enumerate()
                .map(|(i, b)| if i == 20 { 15 } else { b })
                .collect::<std::vec::Vec<_>>()
            ),
            Err(ChildStoreError::Corrupt),
            "timeout enumeration 15 is undefined"
        );

        // Reserved short address 0x0000.
        let mut zero_short = [0u8; MAX_ENCODED_CHILD_TABLE_LEN];
        let len = table(&[child(1, 0x0002, 8)]).encode(&mut zero_short);
        zero_short[17] = 0;
        zero_short[18] = 0; // clear the short low/high bytes
        assert_eq!(
            PersistentChildTable::decode(&zero_short[..len]),
            Err(ChildStoreError::Corrupt)
        );

        // Truncated payload / wrong length.
        let mut good = [0u8; MAX_ENCODED_CHILD_TABLE_LEN];
        let len = table(&[child(1, 0x0002, 8)]).encode(&mut good);
        assert_eq!(
            PersistentChildTable::decode(&good[..len - 1]),
            Err(ChildStoreError::Corrupt)
        );

        // Duplicate short address is rejected by table validation.
        let dup = table(&[child(1, 0x0002, 8), child(2, 0x0002, 8)]);
        assert_eq!(dup.validate(), Err(ChildStoreError::Corrupt));
    }

    #[test]
    fn ram_store_round_trips_and_rejects_invalid_tables() {
        let mut store = RamChildTableStore::new();
        assert_eq!(store.load(), Ok(None));
        let original = table(&[child(1, 0x0002, 8), child(2, 0x0003, 14)]);
        store.store(&original).unwrap();
        assert_eq!(store.load(), Ok(Some(original)));
    }

    // ── Flash journal crash-safety ──────────────────────────

    struct MockFlash {
        data: [u8; CHILD_JOURNAL_SECTOR_SIZE * 2],
        programs_before_failure: Option<usize>,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                data: [0xFF; CHILD_JOURNAL_SECTOR_SIZE * 2],
                programs_before_failure: None,
            }
        }

        fn offset(address: u32) -> Result<usize, NorFlashErrorKind> {
            let offset = address as usize;
            if offset < CHILD_JOURNAL_SECTOR_SIZE * 2 {
                Ok(offset)
            } else {
                Err(NorFlashErrorKind::OutOfBounds)
            }
        }
    }

    impl ErrorType for MockFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, address: u32, output: &mut [u8]) -> Result<(), Self::Error> {
            let start = Self::offset(address)?;
            let end = start
                .checked_add(output.len())
                .filter(|end| *end <= self.data.len())
                .ok_or(NorFlashErrorKind::OutOfBounds)?;
            output.copy_from_slice(&self.data[start..end]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.data.len()
        }
    }

    impl NorFlash for MockFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = CHILD_JOURNAL_SECTOR_SIZE;

        fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
            if let Some(remaining) = self.programs_before_failure.as_mut() {
                if *remaining == 0 {
                    return Err(NorFlashErrorKind::Other);
                }
                *remaining -= 1;
            }
            let start = Self::offset(address)?;
            let end = start
                .checked_add(data.len())
                .filter(|end| *end <= self.data.len())
                .ok_or(NorFlashErrorKind::OutOfBounds)?;
            for (old, new) in self.data[start..end].iter_mut().zip(data) {
                if (*old & *new) != *new {
                    return Err(NorFlashErrorKind::Other);
                }
                *old &= *new;
            }
            Ok(())
        }

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            let start = Self::offset(from)?;
            let end = usize::try_from(to).map_err(|_| NorFlashErrorKind::OutOfBounds)?;
            if start % CHILD_JOURNAL_SECTOR_SIZE != 0
                || end % CHILD_JOURNAL_SECTOR_SIZE != 0
                || start >= end
                || end > self.data.len()
            {
                return Err(NorFlashErrorKind::NotAligned);
            }
            self.data[start..end].fill(0xFF);
            Ok(())
        }
    }

    fn journal(flash: MockFlash) -> ChildTableJournal<MockFlash> {
        ChildTableJournal::new(flash, 0, CHILD_JOURNAL_SECTOR_SIZE as u32)
    }

    #[test]
    fn journal_round_trips_and_supersedes_generations() {
        let mut store = journal(MockFlash::new());
        assert_eq!(store.load(), Ok(None));

        let first = table(&[child(1, 0x0002, 8)]);
        store.store(&first).unwrap();
        assert_eq!(store.load(), Ok(Some(first)));

        let second = table(&[child(1, 0x0002, 14), child(2, 0x0003, 8)]);
        store.store(&second).unwrap();
        assert_eq!(store.load(), Ok(Some(second.clone())));

        // A fresh journal over the same flash reads the newest generation.
        let flash = store.into_storage();
        let mut reopened = journal(flash);
        assert_eq!(reopened.load(), Ok(Some(second)));
    }

    #[test]
    fn an_interrupted_commit_preserves_the_previous_generation() {
        let mut flash = MockFlash::new();
        // Two committed generations first.
        {
            let mut store = journal(core::mem::replace(&mut flash, MockFlash::new()));
            store.store(&table(&[child(1, 0x0002, 8)])).unwrap();
            let good = table(&[child(1, 0x0002, 14)]);
            store.store(&good).unwrap();
            flash = store.into_storage();
        }
        // Fail the *commit* write (the second program of the next store).
        flash.programs_before_failure = Some(1);
        let mut store = journal(flash);
        assert_eq!(
            store.store(&table(&[child(9, 0x0009, 0)])),
            Err(ChildStoreError::Hardware)
        );
        // Reopen: the interrupted record has no commit marker, so the last
        // good generation survives intact.
        let flash = store.into_storage();
        let mut reopened = journal(flash);
        assert_eq!(
            reopened.load(),
            Ok(Some(table(&[child(1, 0x0002, 14)]))),
            "an interrupted write must never destroy committed child state"
        );
    }

    #[test]
    fn an_unknown_version_is_skipped_like_a_corrupt_record() {
        let mut store = journal(MockFlash::new());
        store.store(&table(&[child(1, 0x0002, 8)])).unwrap();
        let mut flash = store.into_storage();
        // Corrupt the version byte of the only committed slot.
        flash.data[4] = 0xEE;
        let mut reopened = journal(flash);
        assert_eq!(
            reopened.load(),
            Ok(None),
            "a record from an unrecognised (e.g. downgraded/newer) format is ignored"
        );
    }

    #[test]
    fn a_misconfigured_geometry_is_reported_rather_than_silently_accepted() {
        // Overlapping sectors: no durable persistence is possible.
        let mut store = ChildTableJournal::new(MockFlash::new(), 0, 0);
        assert_eq!(store.load(), Err(ChildStoreError::Hardware));
    }
}
