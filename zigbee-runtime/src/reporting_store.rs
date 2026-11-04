//! Durable ZCL attribute-reporting configuration.
//!
//! [`ReportingEngine`] state is RAM-only: without this module a sleepy sensor
//! forgets every coordinator `Configure Reporting` (0x06) on reset and falls
//! back to its product defaults until the coordinator happens to re-run its
//! interview. This module is the opt-in persistence for that state:
//!
//! - [`PersistentReporting`] is a bounded snapshot of every engine
//!   configuration, encoded as `endpoint`, `cluster` and the standard ZCL
//!   Configure Reporting attribute record, bound to the network's extended
//!   PAN ID so a snapshot is never applied to a different network;
//! - [`ReportingJournal`] is an atomic two-sector flash journal of those
//!   snapshots (CRC, commit marker, read-back verification, generation
//!   ordering; an interrupted write or erase leaves the previous snapshot);
//! - [`ReportingPersistence`] restores a matching snapshot once after the
//!   product applied its defaults, and afterwards writes only when the
//!   engine's configuration actually changed while joined.
//!
//! A snapshot may also carry the client-changeable Poll Control (0x0020)
//! values ([`PollControlState`]) a coordinator wrote, bound to the same
//! extended PAN ID; see [`ReportingPersistence::restore_with`].
//!
//! Reporting timers and last reported values are not persisted: a restored
//! configuration starts a fresh minimum/maximum interval.

use embedded_storage::nor_flash::NorFlash;
use zigbee_mac::MacDriver;
use zigbee_types::IeeeAddress;
use zigbee_zcl::clusters::poll_control::{PollControlCluster, PollControlState};
use zigbee_zcl::foundation::reporting::{MAX_REPORT_CONFIGS, ReportingConfig, ReportingEngine};

use crate::ZigbeeDevice;
use crate::role::DeviceRole;
use crate::security_journal::crc32;

/// Largest encoded record: endpoint (1), cluster (2), direction (1),
/// attribute (2), type (1), min (2), max (2) and an at most 8-byte analog
/// reportable change.
pub const MAX_ENCODED_REPORTING_RECORD_LEN: usize = 3 + 8 + 8;
/// Tag of the optional trailing Poll Control section.
const POLL_CONTROL_TAG: u8 = 0x20;
/// Encoded Poll Control section: tag and [`PollControlState`].
const POLL_CONTROL_SECTION_LEN: usize = 1 + PollControlState::ENCODED_LEN;
/// Largest encoded snapshot payload: a record count, every record and the
/// optional Poll Control section.
pub const MAX_ENCODED_REPORTING_LEN: usize =
    1 + MAX_REPORT_CONFIGS * MAX_ENCODED_REPORTING_RECORD_LEN + POLL_CONTROL_SECTION_LEN;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReportingStoreError {
    /// Flash access failed or the configured geometry is unusable.
    Hardware,
    /// A configuration cannot be represented or a stored payload is invalid.
    Corrupt,
    /// The 32-bit journal generation is exhausted.
    GenerationExhausted,
}

/// Bounded, network-bound snapshot of a [`ReportingEngine`].
#[derive(Clone, PartialEq, Eq)]
pub struct PersistentReporting {
    extended_pan_id: IeeeAddress,
    len: usize,
    bytes: [u8; MAX_ENCODED_REPORTING_LEN],
}

impl core::fmt::Debug for PersistentReporting {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PersistentReporting")
            .field("extended_pan_id", &self.extended_pan_id)
            .field("payload", &self.payload())
            .finish()
    }
}

impl PersistentReporting {
    /// Snapshot every engine configuration for `extended_pan_id`.
    pub fn capture(
        engine: &ReportingEngine,
        extended_pan_id: IeeeAddress,
    ) -> Result<Self, ReportingStoreError> {
        let mut bytes = [0u8; MAX_ENCODED_REPORTING_LEN];
        let mut count = 0u8;
        let mut pos = 1;
        for (endpoint, cluster_id, config) in engine.configs() {
            let record = bytes
                .get_mut(pos..pos + MAX_ENCODED_REPORTING_RECORD_LEN)
                .ok_or(ReportingStoreError::Corrupt)?;
            record[0] = endpoint;
            record[1..3].copy_from_slice(&cluster_id.to_le_bytes());
            let n = config
                .encode_record(&mut record[3..])
                .ok_or(ReportingStoreError::Corrupt)?;
            pos += 3 + n;
            count += 1;
        }
        bytes[0] = count;
        Ok(Self {
            extended_pan_id,
            len: pos,
            bytes,
        })
    }

    /// Append the client-changeable Poll Control values.
    pub fn with_poll_control(
        mut self,
        state: PollControlState,
    ) -> Result<Self, ReportingStoreError> {
        if self.poll_control_offset().is_some() {
            return Err(ReportingStoreError::Corrupt);
        }
        let section = self
            .bytes
            .get_mut(self.len..self.len + POLL_CONTROL_SECTION_LEN)
            .ok_or(ReportingStoreError::Corrupt)?;
        section[0] = POLL_CONTROL_TAG;
        section[1..].copy_from_slice(&state.encode());
        self.len += POLL_CONTROL_SECTION_LEN;
        Ok(self)
    }

    /// Stored Poll Control values, if the snapshot carries them.
    pub fn poll_control(&self) -> Option<PollControlState> {
        let offset = self.poll_control_offset()?;
        PollControlState::decode(&self.payload()[offset + 1..])
    }

    /// Offset of a well-formed trailing Poll Control section.
    fn poll_control_offset(&self) -> Option<usize> {
        let records_end = self.for_each_record(|_, _, _| {}).ok()?;
        (records_end != self.len).then_some(records_end)
    }

    /// Extended PAN ID of the network this snapshot belongs to.
    pub const fn extended_pan_id(&self) -> IeeeAddress {
        self.extended_pan_id
    }

    /// Encoded payload: record count followed by every record.
    pub fn payload(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    /// Number of stored configurations.
    pub fn len(&self) -> usize {
        self.bytes[0] as usize
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Change-detection fingerprint over network and payload.
    pub fn fingerprint(&self) -> u32 {
        let mut prefixed = [0u8; 8 + MAX_ENCODED_REPORTING_LEN];
        prefixed[..8].copy_from_slice(&self.extended_pan_id);
        prefixed[8..8 + self.len].copy_from_slice(self.payload());
        crc32(&prefixed[..8 + self.len])
    }

    fn from_parts(
        extended_pan_id: IeeeAddress,
        payload: &[u8],
    ) -> Result<Self, ReportingStoreError> {
        if payload.is_empty() || payload.len() > MAX_ENCODED_REPORTING_LEN {
            return Err(ReportingStoreError::Corrupt);
        }
        let mut bytes = [0u8; MAX_ENCODED_REPORTING_LEN];
        bytes[..payload.len()].copy_from_slice(payload);
        let snapshot = Self {
            extended_pan_id,
            len: payload.len(),
            bytes,
        };
        snapshot.for_each_record(|_, _, _| {})?;
        Ok(snapshot)
    }

    /// Decode every record, failing on any malformed or trailing byte other
    /// than a complete Poll Control section. Returns where the records end.
    fn for_each_record(
        &self,
        mut visit: impl FnMut(u8, u16, ReportingConfig),
    ) -> Result<usize, ReportingStoreError> {
        let count = self.len();
        if count > MAX_REPORT_CONFIGS || self.extended_pan_id == [0; 8] {
            return Err(ReportingStoreError::Corrupt);
        }
        let mut pos = 1;
        for _ in 0..count {
            let head = self
                .payload()
                .get(pos..pos + 3)
                .ok_or(ReportingStoreError::Corrupt)?;
            let endpoint = head[0];
            let cluster_id = u16::from_le_bytes([head[1], head[2]]);
            let (config, consumed) = ReportingConfig::parse_record(&self.payload()[pos + 3..])
                .ok_or(ReportingStoreError::Corrupt)?;
            pos += 3 + consumed;
            visit(endpoint, cluster_id, config);
        }
        // Nothing but an optional, complete Poll Control section may follow.
        if pos != self.len
            && (pos + POLL_CONTROL_SECTION_LEN != self.len
                || self.payload()[pos] != POLL_CONTROL_TAG)
        {
            return Err(ReportingStoreError::Corrupt);
        }
        Ok(pos)
    }

    /// Apply every stored configuration to `engine`, replacing configurations
    /// for the same attribute. The whole snapshot is validated before the
    /// engine is touched.
    pub fn apply(&self, engine: &mut ReportingEngine) -> Result<usize, ReportingStoreError> {
        self.for_each_record(|_, _, _| {})?;
        let mut applied = 0;
        let mut result = Ok(());
        self.for_each_record(|endpoint, cluster_id, config| {
            if result.is_ok() {
                result = engine
                    .configure_for_cluster(endpoint, cluster_id, config)
                    .map(|()| applied += 1)
                    .map_err(|_| ReportingStoreError::Corrupt);
            }
        })?;
        result.map(|()| applied)
    }
}

/// Durable storage for one [`PersistentReporting`] snapshot.
pub trait ReportingStore {
    fn load(&mut self) -> Result<Option<PersistentReporting>, ReportingStoreError>;
    fn store(&mut self, snapshot: &PersistentReporting) -> Result<(), ReportingStoreError>;
}

/// Volatile store for tests and products without reporting persistence.
#[derive(Default)]
pub struct RamReportingStore {
    snapshot: Option<PersistentReporting>,
    pub writes: usize,
}

impl RamReportingStore {
    pub const fn new() -> Self {
        Self {
            snapshot: None,
            writes: 0,
        }
    }
}

impl ReportingStore for RamReportingStore {
    fn load(&mut self) -> Result<Option<PersistentReporting>, ReportingStoreError> {
        Ok(self.snapshot.clone())
    }

    fn store(&mut self, snapshot: &PersistentReporting) -> Result<(), ReportingStoreError> {
        self.snapshot = Some(snapshot.clone());
        self.writes += 1;
        Ok(())
    }
}

/// Erase-unit size assumed for each journal sector.
pub const REPORTING_JOURNAL_SECTOR_SIZE: usize = 4096;
/// Size of one reporting journal slot.
pub const REPORTING_JOURNAL_SLOT_SIZE: usize = 512;

const RECORD_MAGIC: [u8; 4] = *b"ZBRP";
const RECORD_VERSION: u8 = 1;
const RECORD_EPID_OFFSET: usize = 12;
const RECORD_PAYLOAD_OFFSET: usize = 20;
const RECORD_CRC_OFFSET: usize = REPORTING_JOURNAL_SLOT_SIZE - 12;
const RECORD_COMMIT_OFFSET: usize = REPORTING_JOURNAL_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

const _: () = assert!(RECORD_PAYLOAD_OFFSET + MAX_ENCODED_REPORTING_LEN <= RECORD_CRC_OFFSET);

/// Atomic two-sector journal of reporting snapshots.
///
/// Each commit appends a CRC-protected record to the next erased slot of the
/// current sector, writing its commit marker last and verifying the whole
/// slot by read-back. A full sector rolls over by erasing the other sector,
/// so an interrupted erase or write always leaves the previous snapshot.
pub struct ReportingJournal<S, const SECTOR_SIZE: usize = REPORTING_JOURNAL_SECTOR_SIZE> {
    storage: S,
    sectors: [u32; 2],
    /// `(generation, sector)` of the newest committed record, once scanned.
    current: Option<(u32, usize)>,
    scanned: bool,
}

impl<S: NorFlash, const SECTOR_SIZE: usize> ReportingJournal<S, SECTOR_SIZE> {
    const SLOTS_PER_SECTOR: usize = SECTOR_SIZE / REPORTING_JOURNAL_SLOT_SIZE;

    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        assert!(SECTOR_SIZE >= REPORTING_JOURNAL_SLOT_SIZE);
        assert!(SECTOR_SIZE.is_multiple_of(REPORTING_JOURNAL_SLOT_SIZE));
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
            && REPORTING_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
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
        self.sectors[sector] + (slot * REPORTING_JOURNAL_SLOT_SIZE) as u32
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        record: &mut [u8; REPORTING_JOURNAL_SLOT_SIZE],
    ) -> Result<(), ReportingStoreError> {
        let address = self.slot_address(sector, slot);
        self.storage
            .read(address, record)
            .map_err(|_| ReportingStoreError::Hardware)
    }

    /// Generation and payload length of a committed, CRC-valid record.
    fn header(record: &[u8; REPORTING_JOURNAL_SLOT_SIZE]) -> Option<(u32, usize)> {
        if record[0..4] != RECORD_MAGIC
            || record[4] != RECORD_VERSION
            || record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let len = u16::from_le_bytes([record[5], record[6]]) as usize;
        if len == 0 || len > MAX_ENCODED_REPORTING_LEN {
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

    fn decode(
        record: &[u8; REPORTING_JOURNAL_SLOT_SIZE],
        len: usize,
    ) -> Result<PersistentReporting, ReportingStoreError> {
        let mut extended_pan_id = [0u8; 8];
        extended_pan_id.copy_from_slice(&record[RECORD_EPID_OFFSET..RECORD_EPID_OFFSET + 8]);
        PersistentReporting::from_parts(
            extended_pan_id,
            &record[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + len],
        )
    }

    /// Newest decodable record; equal generations prefer scan order.
    fn scan(&mut self) -> Result<Option<PersistentReporting>, ReportingStoreError> {
        if !self.geometry_ok() {
            return Err(ReportingStoreError::Hardware);
        }
        let mut record = [0u8; REPORTING_JOURNAL_SLOT_SIZE];
        let mut best: Option<(u32, usize, PersistentReporting)> = None;
        for sector in 0..2 {
            for slot in 0..Self::SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some((generation, len)) = Self::header(&record) else {
                    continue;
                };
                if best
                    .as_ref()
                    .is_some_and(|(best_generation, _, _)| generation <= *best_generation)
                {
                    continue;
                }
                if let Ok(snapshot) = Self::decode(&record, len) {
                    best = Some((generation, sector, snapshot));
                }
            }
        }
        self.current = best
            .as_ref()
            .map(|(generation, sector, _)| (*generation, *sector));
        self.scanned = true;
        Ok(best.map(|(_, _, snapshot)| snapshot))
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, ReportingStoreError> {
        let mut record = [0u8; REPORTING_JOURNAL_SLOT_SIZE];
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
        snapshot: &PersistentReporting,
    ) -> Result<(), ReportingStoreError> {
        let payload = snapshot.payload();
        let mut record = [0xFFu8; REPORTING_JOURNAL_SLOT_SIZE];
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
            .map_err(|_| ReportingStoreError::Hardware)?;
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + RECORD_COMMIT_OFFSET as u32,
                &record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4],
            )
            .map_err(|_| ReportingStoreError::Hardware)?;

        let mut readback = [0u8; REPORTING_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut readback)?;
        if readback != record {
            return Err(ReportingStoreError::Hardware);
        }
        Ok(())
    }

    fn erase_sector(&mut self, sector: usize) -> Result<(), ReportingStoreError> {
        let start = self.sectors[sector];
        self.storage
            .erase(start, start + SECTOR_SIZE as u32)
            .map_err(|_| ReportingStoreError::Hardware)
    }

    fn commit(&mut self, snapshot: &PersistentReporting) -> Result<(), ReportingStoreError> {
        if !self.scanned {
            self.scan()?;
        }
        let (generation, sector, slot) = match self.current {
            Some((generation, sector)) => {
                let generation = generation
                    .checked_add(1)
                    .ok_or(ReportingStoreError::GenerationExhausted)?;
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

impl<S: NorFlash, const SECTOR_SIZE: usize> ReportingStore for ReportingJournal<S, SECTOR_SIZE> {
    fn load(&mut self) -> Result<Option<PersistentReporting>, ReportingStoreError> {
        self.scan()
    }

    fn store(&mut self, snapshot: &PersistentReporting) -> Result<(), ReportingStoreError> {
        let result = self.commit(snapshot);
        if result.is_err() {
            // Rescan before the next attempt instead of trusting the cache.
            self.scanned = false;
            self.current = None;
        }
        result
    }
}

/// Restore-once / write-on-change policy over a [`ReportingStore`].
pub struct ReportingPersistence<S> {
    store: S,
    saved: Option<u32>,
}

impl<S: ReportingStore> ReportingPersistence<S> {
    pub const fn new(store: S) -> Self {
        Self { store, saved: None }
    }

    pub fn store_mut(&mut self) -> &mut S {
        &mut self.store
    }

    /// Apply the stored snapshot when it belongs to the joined network.
    ///
    /// Call once after the product installed its default reporting, so a
    /// stored coordinator configuration replaces the defaults it covers.
    /// Returns the number of restored configurations.
    pub fn restore<M: MacDriver, R: DeviceRole>(
        &mut self,
        device: &mut ZigbeeDevice<M, R>,
    ) -> Result<usize, ReportingStoreError> {
        self.restore_with(device, None)
    }

    /// [`restore`](Self::restore), also restoring the stored Poll Control
    /// values into `poll_control` (refused if outside its product limits).
    pub fn restore_with<M: MacDriver, R: DeviceRole>(
        &mut self,
        device: &mut ZigbeeDevice<M, R>,
        mut poll_control: Option<&mut PollControlCluster>,
    ) -> Result<usize, ReportingStoreError> {
        let restored = match (device.joined_extended_pan_id(), self.store.load()?) {
            (Some(epid), Some(snapshot)) if snapshot.extended_pan_id() == epid => {
                let applied = snapshot.apply(device.reporting_mut())?;
                if let (Some(state), Some(cluster)) =
                    (snapshot.poll_control(), poll_control.as_deref_mut())
                {
                    cluster.restore_state(state);
                }
                applied
            }
            _ => 0,
        };
        if let Some(epid) = device.joined_extended_pan_id() {
            let state = poll_control.map(|cluster| cluster.state());
            self.saved = Some(Self::capture(device, epid, state)?.fingerprint());
        }
        Ok(restored)
    }

    fn capture<M: MacDriver, R: DeviceRole>(
        device: &ZigbeeDevice<M, R>,
        epid: IeeeAddress,
        poll_control: Option<PollControlState>,
    ) -> Result<PersistentReporting, ReportingStoreError> {
        let snapshot = PersistentReporting::capture(device.reporting(), epid)?;
        match poll_control {
            Some(state) => snapshot.with_poll_control(state),
            None => Ok(snapshot),
        }
    }

    /// Persist the engine configuration if it changed while joined.
    ///
    /// Returns `true` when a new snapshot was committed. A failed commit is
    /// retried by the next call.
    pub fn sync<M: MacDriver, R: DeviceRole>(
        &mut self,
        device: &ZigbeeDevice<M, R>,
    ) -> Result<bool, ReportingStoreError> {
        self.sync_with(device, None)
    }

    /// [`sync`](Self::sync), also persisting `poll_control`'s
    /// client-changeable values when they changed.
    pub fn sync_with<M: MacDriver, R: DeviceRole>(
        &mut self,
        device: &ZigbeeDevice<M, R>,
        poll_control: Option<&PollControlCluster>,
    ) -> Result<bool, ReportingStoreError> {
        let Some(epid) = device.joined_extended_pan_id() else {
            return Ok(false);
        };
        let snapshot = Self::capture(device, epid, poll_control.map(PollControlCluster::state))?;
        let fingerprint = snapshot.fingerprint();
        if self.saved == Some(fingerprint) {
            return Ok(false);
        }
        self.store.store(&snapshot)?;
        self.saved = Some(fingerprint);
        Ok(true)
    }
}

impl<M: MacDriver, R: DeviceRole> ZigbeeDevice<M, R> {
    /// Extended PAN ID of the joined network, `None` while not joined.
    pub fn joined_extended_pan_id(&self) -> Option<IeeeAddress> {
        self.is_joined()
            .then(|| self.bdb.zdo().nwk().nib().extended_pan_id)
            .filter(|epid| *epid != [0; 8])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};
    use zigbee_zcl::AttributeId;
    use zigbee_zcl::data_types::{ZclDataType, ZclValue};
    use zigbee_zcl::foundation::reporting::ReportDirection;

    const EPID: IeeeAddress = [0x33; 8];
    const SECTOR: usize = REPORTING_JOURNAL_SECTOR_SIZE;

    fn temperature(min: u16, max: u16, change: i16) -> ReportingConfig {
        ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0),
            data_type: ZclDataType::I16,
            min_interval: min,
            max_interval: max,
            reportable_change: Some(ZclValue::I16(change)),
        }
    }

    fn engine(max: u16) -> ReportingEngine {
        let mut engine = ReportingEngine::new();
        engine
            .configure_for_cluster(1, 0x0402, temperature(30, max, 50))
            .unwrap();
        engine
            .configure_for_cluster(
                1,
                0x0001,
                ReportingConfig {
                    direction: ReportDirection::Receive,
                    attribute_id: AttributeId(0x21),
                    data_type: ZclDataType::NoData,
                    min_interval: 0,
                    max_interval: 600,
                    reportable_change: None,
                },
            )
            .unwrap();
        engine
    }

    fn max_interval(engine: &ReportingEngine) -> u16 {
        engine
            .get_config(1, 0x0402, ReportDirection::Send, AttributeId(0))
            .unwrap()
            .max_interval
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

    fn journal(flash: MockFlash) -> ReportingJournal<MockFlash> {
        ReportingJournal::new(flash, 0, SECTOR as u32)
    }

    #[test]
    fn snapshot_round_trips_through_the_engine() {
        let snapshot = PersistentReporting::capture(&engine(900), EPID).unwrap();
        assert_eq!(snapshot.len(), 2);
        let mut restored = ReportingEngine::new();
        restored
            .configure_for_cluster(1, 0x0402, temperature(10, 300, 50))
            .unwrap();
        assert_eq!(snapshot.apply(&mut restored), Ok(2));
        assert_eq!(max_interval(&restored), 900);
        assert_eq!(
            PersistentReporting::capture(&restored, EPID).unwrap(),
            snapshot
        );
    }

    #[test]
    fn malformed_payloads_are_rejected_before_touching_the_engine() {
        let snapshot = PersistentReporting::capture(&engine(900), EPID).unwrap();
        let payload = snapshot.payload();
        for len in 1..payload.len() {
            assert_eq!(
                PersistentReporting::from_parts(EPID, &payload[..len]),
                Err(ReportingStoreError::Corrupt)
            );
        }
        let mut trailing = [0u8; MAX_ENCODED_REPORTING_LEN];
        trailing[..payload.len()].copy_from_slice(payload);
        assert!(PersistentReporting::from_parts(EPID, &trailing[..payload.len() + 1]).is_err());
        assert!(PersistentReporting::from_parts([0; 8], payload).is_err());
        assert!(PersistentReporting::from_parts(EPID, &[]).is_err());
    }

    const POLL: PollControlState = PollControlState {
        check_in_interval: 13200,
        long_poll_interval: 120,
        short_poll_interval: 2,
        fast_poll_timeout: 80,
    };

    #[test]
    fn poll_control_section_round_trips_and_old_snapshots_stay_readable() {
        let plain = PersistentReporting::capture(&engine(900), EPID).unwrap();
        assert_eq!(plain.poll_control(), None);
        let with_poll = plain.clone().with_poll_control(POLL).unwrap();
        assert_eq!(with_poll.payload().len(), plain.payload().len() + 13);
        assert_eq!(with_poll.poll_control(), Some(POLL));
        assert_ne!(with_poll.fingerprint(), plain.fingerprint());
        // Only one section.
        assert_eq!(
            with_poll.clone().with_poll_control(POLL),
            Err(ReportingStoreError::Corrupt)
        );
        // Records still apply with the section present.
        let mut restored = ReportingEngine::new();
        assert_eq!(with_poll.apply(&mut restored), Ok(2));
        assert_eq!(max_interval(&restored), 900);

        // Through the journal, and an older record without the section.
        let mut journal = journal(MockFlash::new());
        journal.store(&plain).unwrap();
        assert_eq!(journal.load().unwrap().unwrap().poll_control(), None);
        journal.store(&with_poll).unwrap();
        let loaded = journal.load().unwrap().unwrap();
        assert_eq!(loaded, with_poll);
        assert_eq!(loaded.poll_control(), Some(POLL));

        // An empty engine still carries Poll Control.
        let empty = PersistentReporting::capture(&ReportingEngine::new(), EPID)
            .unwrap()
            .with_poll_control(POLL)
            .unwrap();
        assert_eq!(empty.poll_control(), Some(POLL));
        assert_eq!(
            PersistentReporting::from_parts(EPID, empty.payload())
                .unwrap()
                .poll_control(),
            Some(POLL)
        );
    }

    #[test]
    fn malformed_poll_control_sections_are_rejected() {
        let with_poll = PersistentReporting::capture(&engine(900), EPID)
            .unwrap()
            .with_poll_control(POLL)
            .unwrap();
        let payload = with_poll.payload();
        let records_end = payload.len() - 13;
        // Truncated section.
        for len in records_end + 1..payload.len() {
            assert!(PersistentReporting::from_parts(EPID, &payload[..len]).is_err());
        }
        // Wrong tag.
        let mut bytes = [0u8; MAX_ENCODED_REPORTING_LEN];
        bytes[..payload.len()].copy_from_slice(payload);
        bytes[records_end] = 0x21;
        assert!(PersistentReporting::from_parts(EPID, &bytes[..payload.len()]).is_err());
        // Extra byte after the section.
        bytes[records_end] = POLL_CONTROL_TAG;
        assert!(PersistentReporting::from_parts(EPID, &bytes[..payload.len() + 1]).is_err());
    }

    #[test]
    fn analog_configuration_without_change_cannot_be_captured() {
        let mut engine = ReportingEngine::new();
        let mut config = temperature(1, 2, 3);
        config.reportable_change = None;
        engine.configure_for_cluster(1, 0x0402, config).unwrap();
        assert_eq!(
            PersistentReporting::capture(&engine, EPID),
            Err(ReportingStoreError::Corrupt)
        );
    }

    #[test]
    fn journal_on_erased_flash_is_empty_and_keeps_the_newest_record() {
        let mut journal = journal(MockFlash::new());
        assert_eq!(journal.load(), Ok(None));
        for max in [300, 600, 900] {
            let snapshot = PersistentReporting::capture(&engine(max), EPID).unwrap();
            journal.store(&snapshot).unwrap();
        }
        let mut reopened =
            ReportingJournal::<_, SECTOR>::new(journal.into_storage(), 0, SECTOR as u32);
        let loaded = reopened.load().unwrap().unwrap();
        let mut restored = ReportingEngine::new();
        loaded.apply(&mut restored).unwrap();
        assert_eq!(max_interval(&restored), 900);
    }

    #[test]
    fn journal_rolls_over_and_survives_every_reopen() {
        let mut journal = journal(MockFlash::new());
        let slots = SECTOR / REPORTING_JOURNAL_SLOT_SIZE;
        for round in 0..(3 * slots as u16 + 1) {
            let snapshot = PersistentReporting::capture(&engine(100 + round), EPID).unwrap();
            journal.store(&snapshot).unwrap();
            let flash = journal.into_storage();
            let mut reopened = ReportingJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
            let mut restored = ReportingEngine::new();
            reopened
                .load()
                .unwrap()
                .unwrap()
                .apply(&mut restored)
                .unwrap();
            assert_eq!(max_interval(&restored), 100 + round);
            journal = reopened;
        }
        assert!(journal.into_storage().erases >= 3);
    }

    #[test]
    fn interrupted_writes_keep_the_previous_snapshot() {
        let slots = SECTOR / REPORTING_JOURNAL_SLOT_SIZE;
        // Interrupt before the prefix, before the commit marker, and on the
        // first record after a rollover erase.
        for prefill in [1usize, 2, slots] {
            for programs_left in [0usize, 1] {
                let mut journal = journal(MockFlash::new());
                for max in 0..prefill as u16 {
                    journal
                        .store(&PersistentReporting::capture(&engine(200 + max), EPID).unwrap())
                        .unwrap();
                }
                let mut flash = journal.into_storage();
                flash.programs_left = Some(programs_left);
                let mut journal = ReportingJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
                assert_eq!(
                    journal.store(&PersistentReporting::capture(&engine(999), EPID).unwrap()),
                    Err(ReportingStoreError::Hardware)
                );
                let mut flash = journal.into_storage();
                flash.programs_left = None;
                let mut reopened = ReportingJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
                let mut restored = ReportingEngine::new();
                reopened
                    .load()
                    .unwrap()
                    .unwrap()
                    .apply(&mut restored)
                    .unwrap();
                assert_eq!(max_interval(&restored), 200 + prefill as u16 - 1);
                // The torn slot is skipped and the next commit still lands.
                reopened
                    .store(&PersistentReporting::capture(&engine(777), EPID).unwrap())
                    .unwrap();
                let mut restored = ReportingEngine::new();
                reopened
                    .load()
                    .unwrap()
                    .unwrap()
                    .apply(&mut restored)
                    .unwrap();
                assert_eq!(max_interval(&restored), 777);
            }
        }
    }

    #[test]
    fn corrupted_record_falls_back_to_the_previous_generation() {
        let mut journal = journal(MockFlash::new());
        for max in [300, 600] {
            journal
                .store(&PersistentReporting::capture(&engine(max), EPID).unwrap())
                .unwrap();
        }
        let mut flash = journal.into_storage();
        flash.data[REPORTING_JOURNAL_SLOT_SIZE + RECORD_PAYLOAD_OFFSET] ^= 0x01;
        let mut reopened = ReportingJournal::<_, SECTOR>::new(flash, 0, SECTOR as u32);
        let mut restored = ReportingEngine::new();
        reopened
            .load()
            .unwrap()
            .unwrap()
            .apply(&mut restored)
            .unwrap();
        assert_eq!(max_interval(&restored), 300);
    }

    #[test]
    fn exhausted_generation_is_an_explicit_error() {
        let mut journal = journal(MockFlash::new());
        let snapshot = PersistentReporting::capture(&engine(300), EPID).unwrap();
        journal.store(&snapshot).unwrap();
        journal.current = Some((u32::MAX, 0));
        assert_eq!(
            journal.store(&snapshot),
            Err(ReportingStoreError::GenerationExhausted)
        );
    }

    #[test]
    fn invalid_geometry_is_a_hardware_error() {
        let mut overlapping = ReportingJournal::<_, SECTOR>::new(MockFlash::new(), 0, 0);
        assert_eq!(overlapping.load(), Err(ReportingStoreError::Hardware));
        let mut outside =
            ReportingJournal::<_, SECTOR>::new(MockFlash::new(), 0, 2 * SECTOR as u32);
        assert_eq!(outside.load(), Err(ReportingStoreError::Hardware));
    }
}
