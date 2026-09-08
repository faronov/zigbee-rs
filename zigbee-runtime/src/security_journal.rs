//! Atomic two-sector journal for persistent Zigbee security state.
//!
//! # Record versions
//!
//! | version | encoded state | CRC offset | added                        |
//! |---------|---------------|------------|------------------------------|
//! | 1       | 80 bytes      | 92         | initial layout               |
//! | 2       | 97 bytes      | 112        | staged network key           |
//! | 3       | 98 bytes      | 112        | R22 End Device Timeout state |
//! | 4       | 98 bytes      | 112        | `nwkUpdateId` validity bit   |
//! | 5       | 99 bytes      | 112        | BDB join-link-key type       |
//!
//! Slot size, record prefix length and commit offset never changed, so a
//! newer firmware reads every older record in place and the two-sector
//! crash-safety scheme is unaffected.
//!
//! Versions 1..=3 predate the validity bit. Their `update_id` byte was
//! authoritative in the firmware that wrote it, so those records decode with
//! `update_id_valid = true`; only version 4 or newer may say "unknown". A
//! version 3 record carrying bit 7 is corrupt, not an early version 4.
//!
//! # Replay-log geometry and endurance
//!
//! Authenticated incoming NWK and APS frames append an exact replay floor
//! before protocol or application side effects. One 128-byte slot contains
//! four committed replay entries. A rollover writes the active replay-domain
//! snapshot first and the security-state record last, so a power cut cannot
//! activate a partial generation.
//!
//! The default 4 KiB logical sector has 32 slots and can represent all 112
//! replay domains of a router build, but a completely populated snapshot uses
//! 28 replay slots plus one state slot. Only three slots (12 frame commits)
//! then remain before the next erase/rollover. Products with many peers,
//! especially parent routers and coordinators, must select substantially
//! larger logical sectors and verify flash endurance for their traffic rate.
//! A 16 KiB logical sector means a 32 KiB two-sector partition.
//!
//! # Downgrade is not supported
//!
//! Once this firmware has written a version 5 record, **downgrading to
//! firmware that predates version 5 is unsupported**. Older firmware does not
//! recognise version 5 and skips those records while scanning, so it would
//! select the newest record it *can* decode — an older generation with stale
//! counters, a stale parent and possibly a stale network key. Reusing those
//! reservations would replay NWK/APS frame counters. Recommission the device
//! instead of downgrading.

use embedded_storage::nor_flash::NorFlash;

use crate::security_store::{
    ENCODED_SECURITY_STATE_LEN, LEGACY_ENCODED_SECURITY_STATE_LEN, MAX_PERSISTENT_REPLAY_COUNTERS,
    PersistentReplayCounter, PersistentSecurityState, ReplayCounterTombstone, SecurityStateStore,
    SecurityStoreError, StateFormat, V2_ENCODED_SECURITY_STATE_LEN, V4_ENCODED_SECURITY_STATE_LEN,
    V5_ENCODED_SECURITY_STATE_LEN, V6_ENCODED_SECURITY_STATE_LEN,
};

pub const SECURITY_JOURNAL_SECTOR_SIZE: usize = 4096;
pub const SECURITY_JOURNAL_SLOT_SIZE: usize = 128;
/// Security-record format epoch that production boot policy must not roll
/// behind after this firmware has committed a record.
pub const SECURITY_JOURNAL_FORMAT_VERSION: u8 = 8;
pub const SECURITY_JOURNAL_SLOTS_PER_SECTOR: usize =
    SECURITY_JOURNAL_SECTOR_SIZE / SECURITY_JOURNAL_SLOT_SIZE;

const RECORD_MAGIC: [u8; 4] = *b"ZBSS";
const LEGACY_RECORD_VERSION: u8 = 1;
const V2_RECORD_VERSION: u8 = 2;
const V3_RECORD_VERSION: u8 = 3;
const V4_RECORD_VERSION: u8 = 4;
const V5_RECORD_VERSION: u8 = 5;
const V6_RECORD_VERSION: u8 = 6;
const V7_RECORD_VERSION: u8 = 7;
const RECORD_VERSION: u8 = SECURITY_JOURNAL_FORMAT_VERSION;
const LEGACY_RECORD_CRC_OFFSET: usize = 92;
const V6_RECORD_CRC_OFFSET: usize = 112;
const RECORD_CRC_OFFSET: usize = 116;
const RECORD_PREFIX_LEN: usize = 120;
const RECORD_COMMIT_OFFSET: usize = 124;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";
const REPLAY_RECORD_MAGIC: [u8; 4] = *b"ZBSR";
const REPLAY_RECORD_VERSION: u8 = 1;
const REPLAY_HEADER_LEN: usize = 16;
const REPLAY_ENTRY_LEN: usize = 28;
const REPLAY_ENTRY_PREFIX_LEN: usize = 24;
const REPLAY_ENTRY_COMMIT_OFFSET: usize = 24;
const REPLAY_ENTRY_COMMIT: [u8; 4] = *b"RCMT";
pub const SECURITY_JOURNAL_REPLAY_ENTRIES_PER_SLOT: usize =
    (SECURITY_JOURNAL_SLOT_SIZE - REPLAY_HEADER_LEN) / REPLAY_ENTRY_LEN;
const REPLAY_ENTRIES_PER_SLOT: usize = SECURITY_JOURNAL_REPLAY_ENTRIES_PER_SLOT;
pub const SECURITY_JOURNAL_MAX_REPLAY_SNAPSHOT_SLOTS: usize =
    MAX_PERSISTENT_REPLAY_COUNTERS.div_ceil(SECURITY_JOURNAL_REPLAY_ENTRIES_PER_SLOT);
pub const SECURITY_JOURNAL_MIN_SECTOR_SIZE: usize =
    (SECURITY_JOURNAL_MAX_REPLAY_SNAPSHOT_SLOTS + 1) * SECURITY_JOURNAL_SLOT_SIZE;
const _: () = assert!(SECURITY_JOURNAL_REPLAY_ENTRIES_PER_SLOT == 4);
const _: () = assert!(
    MAX_PERSISTENT_REPLAY_COUNTERS
        <= (SECURITY_JOURNAL_SLOTS_PER_SECTOR - 1) * SECURITY_JOURNAL_REPLAY_ENTRIES_PER_SLOT
);

// The encoded state starts at byte 12 and must stay clear of the CRC field.
const _: () = assert!(12 + ENCODED_SECURITY_STATE_LEN <= RECORD_CRC_OFFSET);
const _: () = assert!(12 + V6_ENCODED_SECURITY_STATE_LEN <= V6_RECORD_CRC_OFFSET);
const _: () = assert!(12 + V5_ENCODED_SECURITY_STATE_LEN <= V6_RECORD_CRC_OFFSET);
const _: () = assert!(12 + V4_ENCODED_SECURITY_STATE_LEN <= V6_RECORD_CRC_OFFSET);
const _: () = assert!(12 + V2_ENCODED_SECURITY_STATE_LEN <= V6_RECORD_CRC_OFFSET);
const _: () = assert!(12 + LEGACY_ENCODED_SECURITY_STATE_LEN <= LEGACY_RECORD_CRC_OFFSET);

/// Crash-safe security-state journal with two independently erasable sectors.
///
/// `SECTOR_SIZE` is the size of each logical journal sector, not a product
/// partition address. It defaults to 4 KiB for existing products. A product
/// whose flash erase unit is larger selects its actual sector size in the
/// type, for example
/// `SecurityStateJournal<PartitionFlash, { 8 * 1024 }>`. The product-owned
/// `PartitionFlash` remains responsible for translating the relative journal
/// offsets to its protected physical partition.
pub struct SecurityStateJournal<S, const SECTOR_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE> {
    storage: S,
    sectors: [u32; 2],
    cached: Option<LocatedState>,
    cached_replay: heapless::Vec<PersistentReplayCounter, MAX_PERSISTENT_REPLAY_COUNTERS>,
    replay_generation: Option<u32>,
    scanned: bool,
}

#[derive(Clone, Copy)]
struct LocatedState {
    generation: u32,
    sector: usize,
    state: PersistentSecurityState,
}

impl<S, const SECTOR_SIZE: usize> SecurityStateJournal<S, SECTOR_SIZE> {
    /// Bytes in one configured logical journal sector.
    pub const SECTOR_SIZE: usize = SECTOR_SIZE;
    /// Fixed record size independent of the selected physical erase size.
    pub const SLOT_SIZE: usize = SECURITY_JOURNAL_SLOT_SIZE;
    /// Number of committed-record slots in one configured journal sector.
    pub const SLOTS_PER_SECTOR: usize = SECTOR_SIZE / SECURITY_JOURNAL_SLOT_SIZE;
    /// Replay entries available after a rollover containing every supported
    /// active replay domain and one security-state record.
    pub const FULL_TABLE_APPEND_CAPACITY: usize = Self::SLOTS_PER_SECTOR
        .saturating_sub(SECURITY_JOURNAL_MAX_REPLAY_SNAPSHOT_SLOTS + 1)
        * SECURITY_JOURNAL_REPLAY_ENTRIES_PER_SLOT;
}

impl<S: NorFlash> SecurityStateJournal<S> {
    /// Construct the default 4 KiB-sector journal.
    ///
    /// Existing products should continue to use this constructor. Products
    /// with a different physical erase sector use
    /// [`Self::new_with_sector_size`] through a type alias that fixes
    /// `SECTOR_SIZE`.
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        Self::new_with_sector_size(storage, first_sector, second_sector)
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> SecurityStateJournal<S, SECTOR_SIZE> {
    /// Construct a journal whose two logical sectors are `SECTOR_SIZE` bytes.
    ///
    /// Sector offsets are relative to `storage`; the product owns any
    /// translation from those offsets to physical flash partitions.
    pub const fn new_with_sector_size(storage: S, first_sector: u32, second_sector: u32) -> Self {
        assert!(SECTOR_SIZE >= SECURITY_JOURNAL_SLOT_SIZE);
        assert!(SECTOR_SIZE.is_multiple_of(SECURITY_JOURNAL_SLOT_SIZE));
        Self {
            storage,
            sectors: [first_sector, second_sector],
            cached: None,
            cached_replay: heapless::Vec::new(),
            replay_generation: None,
            scanned: false,
        }
    }

    pub fn storage(&self) -> &S {
        &self.storage
    }

    pub fn storage_mut(&mut self) -> &mut S {
        self.cached = None;
        self.cached_replay.clear();
        self.replay_generation = None;
        self.scanned = false;
        &mut self.storage
    }

    pub fn into_storage(self) -> S {
        self.storage
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        output: &mut [u8; SECURITY_JOURNAL_SLOT_SIZE],
    ) -> Result<(), SecurityStoreError> {
        self.storage
            .read(
                self.sectors[sector] + (slot * SECURITY_JOURNAL_SLOT_SIZE) as u32,
                output,
            )
            .map_err(|_| SecurityStoreError::Hardware)
    }

    fn decode_record(
        record: &[u8; SECURITY_JOURNAL_SLOT_SIZE],
    ) -> Option<(u32, PersistentSecurityState)> {
        if record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
            || record[0..4] != RECORD_MAGIC
        {
            return None;
        }

        let (crc_offset, format) = match (record[4], record[5] as usize) {
            (RECORD_VERSION, ENCODED_SECURITY_STATE_LEN) => (RECORD_CRC_OFFSET, StateFormat::V8),
            (V7_RECORD_VERSION, ENCODED_SECURITY_STATE_LEN) => (RECORD_CRC_OFFSET, StateFormat::V7),
            (V6_RECORD_VERSION, V6_ENCODED_SECURITY_STATE_LEN) => {
                (V6_RECORD_CRC_OFFSET, StateFormat::V6)
            }
            (V5_RECORD_VERSION, V5_ENCODED_SECURITY_STATE_LEN) => {
                (V6_RECORD_CRC_OFFSET, StateFormat::V5)
            }
            (V4_RECORD_VERSION, V4_ENCODED_SECURITY_STATE_LEN) => {
                (V6_RECORD_CRC_OFFSET, StateFormat::V4)
            }
            // Same length as the current format, so the version byte is the
            // only thing that distinguishes them: a v3 record must not be
            // allowed to carry the v4 validity bit.
            (V3_RECORD_VERSION, V4_ENCODED_SECURITY_STATE_LEN) => {
                (V6_RECORD_CRC_OFFSET, StateFormat::V3)
            }
            (V2_RECORD_VERSION, V2_ENCODED_SECURITY_STATE_LEN) => {
                (V6_RECORD_CRC_OFFSET, StateFormat::V2)
            }
            (LEGACY_RECORD_VERSION, LEGACY_ENCODED_SECURITY_STATE_LEN) => {
                (LEGACY_RECORD_CRC_OFFSET, StateFormat::V1)
            }
            _ => return None,
        };
        let expected_crc = u32::from_le_bytes([
            record[crc_offset],
            record[crc_offset + 1],
            record[crc_offset + 2],
            record[crc_offset + 3],
        ]);
        if crc32(&record[..crc_offset]) != expected_crc {
            return None;
        }

        let generation = u32::from_le_bytes([record[8], record[9], record[10], record[11]]);
        // Each version decodes through its own fixed-size buffer and its own
        // entry point, so an older record can never be read with the newer
        // field offsets or accept a flag bit its layout predates.
        let state = match format {
            StateFormat::V8 => {
                let mut encoded_state = [0u8; ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode(&encoded_state).ok()?
            }
            StateFormat::V7 => {
                let mut encoded_state = [0u8; ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_v7(&encoded_state).ok()?
            }
            StateFormat::V6 => {
                let mut encoded_state = [0u8; V6_ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + V6_ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_v6(&encoded_state).ok()?
            }
            StateFormat::V5 => {
                let mut encoded_state = [0u8; V5_ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + V5_ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_v5(&encoded_state).ok()?
            }
            StateFormat::V4 => {
                let mut encoded_state = [0u8; V4_ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + V4_ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_v4(&encoded_state).ok()?
            }
            StateFormat::V3 => {
                let mut encoded_state = [0u8; V4_ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + V4_ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_v3(&encoded_state).ok()?
            }
            StateFormat::V2 => {
                let mut encoded_state = [0u8; V2_ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + V2_ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_v2(&encoded_state).ok()?
            }
            StateFormat::V1 => {
                let mut encoded_state = [0u8; LEGACY_ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + LEGACY_ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_legacy(&encoded_state).ok()?
            }
        };
        Some((generation, state))
    }

    fn newest(&mut self) -> Result<Option<LocatedState>, SecurityStoreError> {
        let mut newest: Option<LocatedState> = None;
        let mut record = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        for sector in 0..2 {
            for slot in 0..Self::SLOTS_PER_SECTOR {
                self.read_slot(sector, slot, &mut record)?;
                let Some((generation, state)) = Self::decode_record(&record) else {
                    continue;
                };
                let replace = match newest {
                    Some(current) => generation > current.generation,
                    None => true,
                };
                if replace {
                    newest = Some(LocatedState {
                        generation,
                        sector,
                        state,
                    });
                }
            }
        }
        Ok(newest)
    }

    fn current(&mut self) -> Result<Option<LocatedState>, SecurityStoreError> {
        if self.sectors[0] == self.sectors[1]
            || SECTOR_SIZE < SECURITY_JOURNAL_SLOT_SIZE
            || !SECTOR_SIZE.is_multiple_of(SECURITY_JOURNAL_SLOT_SIZE)
            || SECTOR_SIZE > u32::MAX as usize
            || self.sectors[0].abs_diff(self.sectors[1]) < SECTOR_SIZE as u32
            || S::READ_SIZE == 0
            || S::WRITE_SIZE == 0
            || S::ERASE_SIZE == 0
            || !SECURITY_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
            || !SECURITY_JOURNAL_SLOT_SIZE.is_multiple_of(S::WRITE_SIZE)
            || !SECTOR_SIZE.is_multiple_of(S::ERASE_SIZE)
            || !RECORD_PREFIX_LEN.is_multiple_of(S::WRITE_SIZE)
            || !RECORD_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
            || !RECORD_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            || !REPLAY_HEADER_LEN.is_multiple_of(S::WRITE_SIZE)
            || !REPLAY_ENTRY_PREFIX_LEN.is_multiple_of(S::WRITE_SIZE)
            || !REPLAY_ENTRY_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
            || !REPLAY_ENTRY_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            || !(self.sectors[0] as usize).is_multiple_of(S::ERASE_SIZE)
            || !(self.sectors[1] as usize).is_multiple_of(S::ERASE_SIZE)
            || self.sectors.iter().any(|sector| {
                (*sector as usize)
                    .checked_add(SECTOR_SIZE)
                    .is_none_or(|end| end > self.storage.capacity())
            })
        {
            return Err(SecurityStoreError::Hardware);
        }
        if !self.scanned {
            self.cached = self.newest()?;
            self.scanned = true;
        }
        Ok(self.cached)
    }

    fn replay_header_generation(record: &[u8; SECURITY_JOURNAL_SLOT_SIZE]) -> Option<u32> {
        if record[0..4] != REPLAY_RECORD_MAGIC
            || record[4] != REPLAY_RECORD_VERSION
            || record[5] as usize != REPLAY_ENTRY_LEN
            || record[6] as usize != REPLAY_ENTRIES_PER_SLOT
            || record[7] != 0
        {
            return None;
        }
        let expected_crc = u32::from_le_bytes([record[12], record[13], record[14], record[15]]);
        if crc32(&record[..12]) != expected_crc {
            return None;
        }
        Some(u32::from_le_bytes([
            record[8], record[9], record[10], record[11],
        ]))
    }

    fn replay_entry_range(index: usize) -> core::ops::Range<usize> {
        let start = REPLAY_HEADER_LEN + index * REPLAY_ENTRY_LEN;
        start..start + REPLAY_ENTRY_LEN
    }

    fn replay_entry_is_erased(entry: &[u8]) -> bool {
        entry.iter().all(|byte| *byte == 0xFF)
    }

    fn decode_replay_entry(entry: &[u8]) -> Option<PersistentReplayCounter> {
        if entry.len() != REPLAY_ENTRY_LEN
            || entry[REPLAY_ENTRY_COMMIT_OFFSET..] != REPLAY_ENTRY_COMMIT
        {
            return None;
        }
        let expected_crc = u32::from_le_bytes([entry[20], entry[21], entry[22], entry[23]]);
        if crc32(&entry[..20]) != expected_crc || entry[18..20] != [0, 0] {
            return None;
        }
        let mut source = [0u8; 8];
        source.copy_from_slice(&entry[..8]);
        if source == [0; 8] || source == [0xFF; 8] {
            return None;
        }
        let counter = u32::from_le_bytes([entry[8], entry[9], entry[10], entry[11]]);
        let key_fingerprint = u32::from_le_bytes([entry[12], entry[13], entry[14], entry[15]]);
        let replay = match entry[16] {
            0 => PersistentReplayCounter::Nwk(zigbee_nwk::security::NwkReplayCounter {
                source,
                key_sequence: entry[17],
                key_fingerprint,
                counter,
            }),
            1 => {
                let key_type = zigbee_aps::security::ApsKeyType::from_u8(entry[17])?;
                if !matches!(
                    key_type,
                    zigbee_aps::security::ApsKeyType::TrustCenterLinkKey
                        | zigbee_aps::security::ApsKeyType::ApplicationLinkKey
                ) {
                    return None;
                }
                PersistentReplayCounter::Aps(zigbee_aps::security::ApsReplayCounter {
                    origin: zigbee_aps::security::ApsReplayOrigin::KeyPair {
                        partner: source,
                        key_type,
                    },
                    key_fingerprint,
                    counter,
                })
            }
            2 if entry[17] == 0 => {
                PersistentReplayCounter::Aps(zigbee_aps::security::ApsReplayCounter {
                    origin: zigbee_aps::security::ApsReplayOrigin::PreconfiguredGlobal { source },
                    key_fingerprint,
                    counter,
                })
            }
            3 if entry[17] == 0 => {
                PersistentReplayCounter::Aps(zigbee_aps::security::ApsReplayCounter {
                    origin: zigbee_aps::security::ApsReplayOrigin::DistributedGlobal { source },
                    key_fingerprint,
                    counter,
                })
            }
            _ => return None,
        };
        Some(replay)
    }

    fn encode_replay_entry(replay: PersistentReplayCounter) -> [u8; REPLAY_ENTRY_LEN] {
        let mut entry = [0xFFu8; REPLAY_ENTRY_LEN];
        let (source, counter, key_fingerprint, kind, selector) = match replay {
            PersistentReplayCounter::Nwk(replay) => (
                replay.source,
                replay.counter,
                replay.key_fingerprint,
                0,
                replay.key_sequence,
            ),
            PersistentReplayCounter::Aps(replay) => match replay.origin {
                zigbee_aps::security::ApsReplayOrigin::KeyPair { partner, key_type } => (
                    partner,
                    replay.counter,
                    replay.key_fingerprint,
                    1,
                    key_type as u8,
                ),
                zigbee_aps::security::ApsReplayOrigin::PreconfiguredGlobal { source } => {
                    (source, replay.counter, replay.key_fingerprint, 2, 0)
                }
                zigbee_aps::security::ApsReplayOrigin::DistributedGlobal { source } => {
                    (source, replay.counter, replay.key_fingerprint, 3, 0)
                }
            },
        };
        entry[..8].copy_from_slice(&source);
        entry[8..12].copy_from_slice(&counter.to_le_bytes());
        entry[12..16].copy_from_slice(&key_fingerprint.to_le_bytes());
        entry[16] = kind;
        entry[17] = selector;
        entry[18..20].fill(0);
        let crc = crc32(&entry[..20]);
        entry[20..24].copy_from_slice(&crc.to_le_bytes());
        entry
    }

    fn merge_replay(
        replay: &mut heapless::Vec<PersistentReplayCounter, MAX_PERSISTENT_REPLAY_COUNTERS>,
        update: PersistentReplayCounter,
    ) -> Result<(), SecurityStoreError> {
        if let Some(stored) = replay.iter_mut().find(|stored| stored.same_domain(&update)) {
            if update.counter() > stored.counter() {
                *stored = update;
            }
            return Ok(());
        }
        replay.push(update).map_err(|_| SecurityStoreError::Full)
    }

    fn scan_replay(
        &mut self,
        located: LocatedState,
    ) -> Result<
        heapless::Vec<PersistentReplayCounter, MAX_PERSISTENT_REPLAY_COUNTERS>,
        SecurityStoreError,
    > {
        let mut replay = heapless::Vec::new();
        let mut record = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        for slot in 0..Self::SLOTS_PER_SECTOR {
            self.read_slot(located.sector, slot, &mut record)?;
            if Self::replay_header_generation(&record) != Some(located.generation) {
                continue;
            }
            for index in 0..REPLAY_ENTRIES_PER_SLOT {
                let entry = &record[Self::replay_entry_range(index)];
                if Self::replay_entry_is_erased(entry) {
                    break;
                }
                let Some(update) = Self::decode_replay_entry(entry) else {
                    break;
                };
                Self::merge_replay(&mut replay, update)?;
            }
        }
        Ok(replay)
    }

    fn ensure_replay_cache(&mut self, located: LocatedState) -> Result<(), SecurityStoreError> {
        if self.replay_generation != Some(located.generation) {
            self.cached_replay = self.scan_replay(located)?;
            self.replay_generation = Some(located.generation);
        }
        Ok(())
    }

    fn slot_is_erased(&mut self, sector: usize, slot: usize) -> Result<bool, SecurityStoreError> {
        let mut record = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut record)?;
        Ok(record.iter().all(|byte| *byte == 0xFF))
    }

    fn find_erased_run(
        &mut self,
        sector: usize,
        needed: usize,
    ) -> Result<Option<usize>, SecurityStoreError> {
        if needed == 0 || needed > Self::SLOTS_PER_SECTOR {
            return Ok(None);
        }
        for start in 0..=Self::SLOTS_PER_SECTOR - needed {
            let mut erased = true;
            for slot in start..start + needed {
                if !self.slot_is_erased(sector, slot)? {
                    erased = false;
                    break;
                }
            }
            if erased {
                return Ok(Some(start));
            }
        }
        Ok(None)
    }

    fn write_replay_header(
        &mut self,
        sector: usize,
        slot: usize,
        generation: u32,
    ) -> Result<(), SecurityStoreError> {
        let mut header = [0xFFu8; REPLAY_HEADER_LEN];
        header[..4].copy_from_slice(&REPLAY_RECORD_MAGIC);
        header[4] = REPLAY_RECORD_VERSION;
        header[5] = REPLAY_ENTRY_LEN as u8;
        header[6] = REPLAY_ENTRIES_PER_SLOT as u8;
        header[7] = 0;
        header[8..12].copy_from_slice(&generation.to_le_bytes());
        let crc = crc32(&header[..12]);
        header[12..16].copy_from_slice(&crc.to_le_bytes());
        let address = self.sectors[sector] + (slot * SECURITY_JOURNAL_SLOT_SIZE) as u32;
        self.storage
            .write(address, &header)
            .map_err(|_| SecurityStoreError::Hardware)?;
        let mut verify = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut verify)?;
        if Self::replay_header_generation(&verify) == Some(generation) {
            Ok(())
        } else {
            Err(SecurityStoreError::Hardware)
        }
    }

    fn write_replay_entry(
        &mut self,
        sector: usize,
        slot: usize,
        index: usize,
        replay: PersistentReplayCounter,
    ) -> Result<(), SecurityStoreError> {
        let entry = Self::encode_replay_entry(replay);
        let offset = REPLAY_HEADER_LEN + index * REPLAY_ENTRY_LEN;
        let address = self.sectors[sector] + (slot * SECURITY_JOURNAL_SLOT_SIZE + offset) as u32;
        self.storage
            .write(address, &entry[..REPLAY_ENTRY_PREFIX_LEN])
            .map_err(|_| SecurityStoreError::Hardware)?;
        self.storage
            .write(
                address + REPLAY_ENTRY_COMMIT_OFFSET as u32,
                &REPLAY_ENTRY_COMMIT,
            )
            .map_err(|_| SecurityStoreError::Hardware)?;
        let mut verify = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut verify)?;
        if Self::decode_replay_entry(&verify[Self::replay_entry_range(index)]) == Some(replay) {
            Ok(())
        } else {
            Err(SecurityStoreError::Hardware)
        }
    }

    fn replay_snapshot_slots(replay_count: usize) -> usize {
        replay_count.div_ceil(REPLAY_ENTRIES_PER_SLOT)
    }

    fn write_replay_snapshot(
        &mut self,
        sector: usize,
        first_slot: usize,
        generation: u32,
        replay: &[PersistentReplayCounter],
    ) -> Result<(), SecurityStoreError> {
        for (slot_offset, chunk) in replay.chunks(REPLAY_ENTRIES_PER_SLOT).enumerate() {
            let slot = first_slot + slot_offset;
            self.write_replay_header(sector, slot, generation)?;
            for (index, update) in chunk.iter().copied().enumerate() {
                self.write_replay_entry(sector, slot, index, update)?;
            }
        }
        Ok(())
    }

    fn activate_generation(
        &mut self,
        sector: usize,
        state_slot: usize,
        generation: u32,
        state: &PersistentSecurityState,
        replay: &heapless::Vec<PersistentReplayCounter, MAX_PERSISTENT_REPLAY_COUNTERS>,
    ) -> Result<(), SecurityStoreError> {
        self.write_replay_snapshot(sector, state_slot + 1, generation, replay.as_slice())?;
        self.write_record(sector, state_slot, generation, state)
    }

    fn find_replay_append_position(
        &mut self,
        located: LocatedState,
    ) -> Result<Option<(usize, usize, bool)>, SecurityStoreError> {
        let mut record = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        for slot in 0..Self::SLOTS_PER_SECTOR {
            self.read_slot(located.sector, slot, &mut record)?;
            if record.iter().all(|byte| *byte == 0xFF) {
                return Ok(Some((slot, 0, true)));
            }
            if Self::replay_header_generation(&record) != Some(located.generation) {
                continue;
            }
            for index in 0..REPLAY_ENTRIES_PER_SLOT {
                let entry = &record[Self::replay_entry_range(index)];
                if Self::replay_entry_is_erased(entry) {
                    return Ok(Some((slot, index, false)));
                }
                if Self::decode_replay_entry(entry).is_none() {
                    break;
                }
            }
        }
        Ok(None)
    }

    fn write_record(
        &mut self,
        sector: usize,
        slot: usize,
        generation: u32,
        state: &PersistentSecurityState,
    ) -> Result<(), SecurityStoreError> {
        state.validate()?;

        let mut record = [0xFFu8; SECURITY_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = RECORD_VERSION;
        record[5] = ENCODED_SECURITY_STATE_LEN as u8;
        record[8..12].copy_from_slice(&generation.to_le_bytes());
        let mut encoded_state = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut encoded_state);
        record[12..12 + ENCODED_SECURITY_STATE_LEN].copy_from_slice(&encoded_state);
        let crc = crc32(&record[..RECORD_CRC_OFFSET]);
        record[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.sectors[sector] + (slot * SECURITY_JOURNAL_SLOT_SIZE) as u32;
        self.storage
            .write(address, &record[..RECORD_PREFIX_LEN])
            .map_err(|_| SecurityStoreError::Hardware)?;
        let commit = RECORD_COMMIT;
        self.storage
            .write(address + RECORD_COMMIT_OFFSET as u32, &commit)
            .map_err(|_| SecurityStoreError::Hardware)?;

        let mut verify = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        self.read_slot(sector, slot, &mut verify)?;
        match Self::decode_record(&verify) {
            Some((stored_generation, stored_state))
                if stored_generation == generation && stored_state == *state =>
            {
                Ok(())
            }
            _ => Err(SecurityStoreError::Hardware),
        }
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> SecurityStateStore
    for SecurityStateJournal<S, SECTOR_SIZE>
{
    fn load(&mut self) -> Result<Option<PersistentSecurityState>, SecurityStoreError> {
        Ok(self.current()?.map(|located| located.state))
    }

    fn store(&mut self, state: &PersistentSecurityState) -> Result<(), SecurityStoreError> {
        let current = self.current()?;
        let generation = match current {
            Some(located) => located
                .generation
                .checked_add(1)
                .ok_or(SecurityStoreError::GenerationExhausted)?,
            None => 0,
        };

        let mut replay = heapless::Vec::new();
        if let Some(located) = current
            && located.state.commissioned
            && state.commissioned
            && located.state.extended_pan_id == state.extended_pan_id
            && located.state.ieee_address == state.ieee_address
        {
            self.ensure_replay_cache(located)?;
            replay = self.cached_replay.clone();
        }
        let needed = 1 + Self::replay_snapshot_slots(replay.len());
        if needed > Self::SLOTS_PER_SECTOR {
            return Err(SecurityStoreError::Full);
        }

        let (target, state_slot, erase) = match current {
            Some(located) => match self.find_erased_run(located.sector, needed)? {
                Some(slot) => (located.sector, slot, false),
                None => (1 - located.sector, 0, true),
            },
            None => (0, 0, true),
        };
        let sector = self.sectors[target];
        let result = (|| {
            if erase {
                self.storage
                    .erase(sector, sector + SECTOR_SIZE as u32)
                    .map_err(|_| SecurityStoreError::Hardware)?;
            }
            self.activate_generation(target, state_slot, generation, state, &replay)
        })();
        if result.is_ok() {
            self.cached = Some(LocatedState {
                generation,
                sector: target,
                state: *state,
            });
            self.cached_replay = replay;
            self.replay_generation = Some(generation);
        } else {
            self.cached = None;
            self.cached_replay.clear();
            self.replay_generation = None;
            self.scanned = false;
        }
        result
    }

    fn visit_replay_counters(
        &mut self,
        visitor: &mut dyn FnMut(PersistentReplayCounter),
    ) -> Result<(), SecurityStoreError> {
        let Some(located) = self.current()? else {
            return Ok(());
        };
        self.ensure_replay_cache(located)?;
        for replay in self.cached_replay.iter().copied() {
            visitor(replay);
        }
        Ok(())
    }

    fn commit_replay_counter(
        &mut self,
        replay: PersistentReplayCounter,
    ) -> Result<(), SecurityStoreError> {
        let Some(located) = self.current()? else {
            return Err(SecurityStoreError::NotFound);
        };
        if !located.state.commissioned {
            return Err(SecurityStoreError::Corrupt);
        }
        self.ensure_replay_cache(located)?;
        if self
            .cached_replay
            .iter()
            .find(|stored| stored.same_domain(&replay))
            .is_some_and(|stored| stored.counter() >= replay.counter())
        {
            return Ok(());
        }

        if let Some((slot, index, initialize)) = self.find_replay_append_position(located)? {
            let result = (|| {
                if initialize {
                    self.write_replay_header(located.sector, slot, located.generation)?;
                }
                self.write_replay_entry(located.sector, slot, index, replay)
            })();
            if result.is_ok() {
                Self::merge_replay(&mut self.cached_replay, replay)?;
            } else {
                self.cached_replay.clear();
                self.replay_generation = None;
            }
            return result;
        }

        let generation = located
            .generation
            .checked_add(1)
            .ok_or(SecurityStoreError::GenerationExhausted)?;
        let mut compacted = self.cached_replay.clone();
        Self::merge_replay(&mut compacted, replay)?;
        let needed = 1 + Self::replay_snapshot_slots(compacted.len());
        if needed > Self::SLOTS_PER_SECTOR {
            return Err(SecurityStoreError::Full);
        }
        let target = 1 - located.sector;
        let sector = self.sectors[target];
        let result = self
            .storage
            .erase(sector, sector + SECTOR_SIZE as u32)
            .map_err(|_| SecurityStoreError::Hardware)
            .and_then(|()| {
                self.activate_generation(target, 0, generation, &located.state, &compacted)
            });
        if result.is_ok() {
            self.cached = Some(LocatedState {
                generation,
                sector: target,
                state: located.state,
            });
            self.cached_replay = compacted;
            self.replay_generation = Some(generation);
        } else {
            self.cached = None;
            self.cached_replay.clear();
            self.replay_generation = None;
            self.scanned = false;
        }
        result
    }

    fn tombstone_replay_counters(
        &mut self,
        tombstone: ReplayCounterTombstone,
    ) -> Result<(), SecurityStoreError> {
        let Some(located) = self.current()? else {
            return Ok(());
        };
        self.ensure_replay_cache(located)?;
        let mut compacted = self.cached_replay.clone();
        let previous_len = compacted.len();
        compacted.retain(|replay| !replay.matches_tombstone(tombstone));
        if compacted.len() == previous_len {
            return Ok(());
        }

        let generation = located
            .generation
            .checked_add(1)
            .ok_or(SecurityStoreError::GenerationExhausted)?;
        let needed = 1 + Self::replay_snapshot_slots(compacted.len());
        if needed > Self::SLOTS_PER_SECTOR {
            return Err(SecurityStoreError::Full);
        }
        let (target, state_slot, erase) = match self.find_erased_run(located.sector, needed)? {
            Some(slot) => (located.sector, slot, false),
            None => (1 - located.sector, 0, true),
        };
        let sector = self.sectors[target];
        let result = (|| {
            if erase {
                self.storage
                    .erase(sector, sector + SECTOR_SIZE as u32)
                    .map_err(|_| SecurityStoreError::Hardware)?;
            }
            self.activate_generation(target, state_slot, generation, &located.state, &compacted)
        })();
        if result.is_ok() {
            self.cached = Some(LocatedState {
                generation,
                sector: target,
                state: located.state,
            });
            self.cached_replay = compacted;
            self.replay_generation = Some(generation);
        } else {
            self.cached = None;
            self.cached_replay.clear();
            self.replay_generation = None;
            self.scanned = false;
        }
        result
    }

    fn retain_replay_counters(
        &mut self,
        retain: &dyn Fn(PersistentReplayCounter) -> bool,
    ) -> Result<usize, SecurityStoreError> {
        let Some(located) = self.current()? else {
            return Ok(0);
        };
        self.ensure_replay_cache(located)?;
        let mut compacted = self.cached_replay.clone();
        let previous_len = compacted.len();
        compacted.retain(|replay| retain(*replay));
        let removed = previous_len - compacted.len();
        if removed == 0 {
            return Ok(0);
        }

        let generation = located
            .generation
            .checked_add(1)
            .ok_or(SecurityStoreError::GenerationExhausted)?;
        let needed = 1 + Self::replay_snapshot_slots(compacted.len());
        if needed > Self::SLOTS_PER_SECTOR {
            return Err(SecurityStoreError::Full);
        }
        let (target, state_slot, erase) = match self.find_erased_run(located.sector, needed)? {
            Some(slot) => (located.sector, slot, false),
            None => (1 - located.sector, 0, true),
        };
        let sector = self.sectors[target];
        let result = (|| {
            if erase {
                self.storage
                    .erase(sector, sector + SECTOR_SIZE as u32)
                    .map_err(|_| SecurityStoreError::Hardware)?;
            }
            self.activate_generation(target, state_slot, generation, &located.state, &compacted)
        })();
        if result.is_ok() {
            self.cached = Some(LocatedState {
                generation,
                sector: target,
                state: located.state,
            });
            self.cached_replay = compacted;
            self.replay_generation = Some(generation);
        } else {
            self.cached = None;
            self.cached_replay.clear();
            self.replay_generation = None;
            self.scanned = false;
        }
        result.map(|()| removed)
    }
}

pub(crate) fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for byte in data {
        crc ^= *byte as u32;
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    struct MockFlash {
        data: [u8; SECURITY_JOURNAL_SECTOR_SIZE * 2],
        programs_before_failure: Option<usize>,
    }

    impl MockFlash {
        fn new() -> Self {
            Self {
                data: [0xFF; SECURITY_JOURNAL_SECTOR_SIZE * 2],
                programs_before_failure: None,
            }
        }

        fn offset(address: u32) -> Result<usize, NorFlashErrorKind> {
            let offset = address as usize;
            if offset < SECURITY_JOURNAL_SECTOR_SIZE * 2 {
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
        const ERASE_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE;

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
            if start % SECURITY_JOURNAL_SECTOR_SIZE != 0
                || end % SECURITY_JOURNAL_SECTOR_SIZE != 0
                || start >= end
                || end > self.data.len()
            {
                return Err(NorFlashErrorKind::NotAligned);
            }
            self.data[start..end].fill(0xFF);
            Ok(())
        }
    }

    fn state(counter: u32) -> PersistentSecurityState {
        let mut state = PersistentSecurityState::empty();
        state.global_counter_limit = counter;
        state
    }

    fn commissioned_state() -> PersistentSecurityState {
        let mut state = PersistentSecurityState::empty();
        state.commissioned = true;
        state.extended_pan_id = [1; 8];
        state.pan_id = 0x1234;
        state.short_address = 0x5678;
        state.ieee_address = [2; 8];
        state.channel = 15;
        state.depth = 1;
        state.parent_address = 0x0000;
        state.network_key = [3; 16];
        state.global_counter_limit = 0x400;
        state.tclk_present = true;
        state.trust_center_address = [4; 8];
        state.trust_center_link_key = [5; 16];
        state.tclk_counter_limit = 0x400;
        state
    }

    fn key_fingerprint(key: &[u8; 16]) -> u32 {
        let mut hash = 0x811C_9DC5u32;
        for byte in key {
            hash ^= u32::from(*byte);
            hash = hash.wrapping_mul(0x0100_0193);
        }
        hash
    }

    fn nwk_replay(counter: u32) -> PersistentReplayCounter {
        nwk_replay_for([0x33; 8], key_fingerprint(&[3; 16]), counter)
    }

    fn nwk_replay_for(
        source: [u8; 8],
        key_fingerprint: u32,
        counter: u32,
    ) -> PersistentReplayCounter {
        PersistentReplayCounter::Nwk(zigbee_nwk::security::NwkReplayCounter {
            source,
            key_sequence: 0,
            key_fingerprint,
            counter,
        })
    }

    fn aps_key_pair_replay(
        partner: [u8; 8],
        key_fingerprint: u32,
        counter: u32,
    ) -> PersistentReplayCounter {
        PersistentReplayCounter::Aps(zigbee_aps::security::ApsReplayCounter {
            origin: zigbee_aps::security::ApsReplayOrigin::KeyPair {
                partner,
                key_type: zigbee_aps::security::ApsKeyType::ApplicationLinkKey,
            },
            key_fingerprint,
            counter,
        })
    }

    fn aps_global_replay(
        source: [u8; 8],
        key_fingerprint: u32,
        counter: u32,
    ) -> PersistentReplayCounter {
        PersistentReplayCounter::Aps(zigbee_aps::security::ApsReplayCounter {
            origin: zigbee_aps::security::ApsReplayOrigin::PreconfiguredGlobal { source },
            key_fingerprint,
            counter,
        })
    }

    /// Write a pre-v4 record by hand, exactly as the older firmware did.
    ///
    /// Content the target version predates is stripped: flags bit 7
    /// (`update_id_valid`, v4) for every older version, and flags bit 6 plus
    /// encoded byte 11 (the R22 End Device Timeout fields, v3) for the shorter
    /// v1/v2 layouts. The bytes on flash are therefore byte-for-byte what the
    /// older firmware would have written. Everything past `encoded_len` stays
    /// erased (0xFF), which is what makes this a real migration test: a
    /// decoder that wrongly indexed encoded byte 97 would read 0xFF and reject
    /// the record.
    fn write_migrated_record(
        flash: &mut MockFlash,
        version: u8,
        encoded_len: usize,
        crc_offset: usize,
        state: &PersistentSecurityState,
    ) {
        let mut current = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut current);
        if version < V4_RECORD_VERSION {
            current[0] &= !(1 << 7);
        }
        if encoded_len < V4_ENCODED_SECURITY_STATE_LEN {
            current[0] &= !(1 << 6);
            current[11] = 0;
        }
        let mut record = [0xFFu8; SECURITY_JOURNAL_SLOT_SIZE];
        record[0..4].copy_from_slice(&RECORD_MAGIC);
        record[4] = version;
        record[5] = encoded_len as u8;
        record[8..12].copy_from_slice(&1u32.to_le_bytes());
        record[12..12 + encoded_len].copy_from_slice(&current[..encoded_len]);
        let crc = crc32(&record[..crc_offset]);
        record[crc_offset..crc_offset + 4].copy_from_slice(&crc.to_le_bytes());
        record[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        flash.data[..SECURITY_JOURNAL_SLOT_SIZE].copy_from_slice(&record);
    }

    #[test]
    fn committed_records_round_trip() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(journal.load(), Ok(None));
        journal.store(&state(0x400)).unwrap();
        assert_eq!(journal.load().unwrap().unwrap().global_counter_limit, 0x400);
    }

    #[test]
    fn fanout_intent_is_versioned_without_changing_record_geometry() {
        let mut expected = commissioned_state();
        expected.staged_network_key_present = true;
        expected.staged_network_key = [0xC7; 16];
        expected.staged_key_sequence = expected.key_sequence.wrapping_add(1);
        expected.network_key_forwarding_pending = true;
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&expected).unwrap();
        journal.storage_mut();
        assert_eq!(journal.load(), Ok(Some(expected)));
        assert_eq!(journal.storage().data[4], 8);
        assert_eq!(journal.storage().data[5], 103);
        assert_eq!(SECURITY_JOURNAL_SLOT_SIZE, 128);

        // Old staged-key records lack descriptor provenance. They must not
        // acquire fanout intent merely by being decoded by newer firmware.
        expected.network_key_forwarding_pending = false;
        let mut flash = MockFlash::new();
        write_migrated_record(
            &mut flash,
            V7_RECORD_VERSION,
            ENCODED_SECURITY_STATE_LEN,
            RECORD_CRC_OFFSET,
            &expected,
        );
        let mut migrated = SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(migrated.load(), Ok(Some(expected)));
        expected.network_key_forwarding_pending = true;
        migrated.store(&expected).unwrap();
        migrated.storage_mut();
        assert_eq!(migrated.load(), Ok(Some(expected)));

        let mut encoded = [0; ENCODED_SECURITY_STATE_LEN];
        expected.encode(&mut encoded);
        assert_eq!(
            PersistentSecurityState::decode_v7(&encoded),
            Err(SecurityStoreError::Corrupt)
        );
        encoded[100] |= 0x80;
        assert_eq!(
            PersistentSecurityState::decode(&encoded),
            Err(SecurityStoreError::Corrupt)
        );
        expected.staged_network_key_present = false;
        expected.staged_network_key = [0; 16];
        expected.staged_key_sequence = 0;
        assert_eq!(expected.validate(), Err(SecurityStoreError::Corrupt));
    }

    #[test]
    fn fanout_key_and_intent_commit_together_across_interrupted_rollover() {
        for programs in 0..=2 {
            let mut journal =
                SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
            let mut old = commissioned_state();
            for _ in 0..SECURITY_JOURNAL_SLOTS_PER_SECTOR {
                old.global_counter_limit += 0x400;
                journal.store(&old).unwrap();
            }
            let mut next = old;
            next.staged_network_key_present = true;
            next.staged_network_key = [0xC7; 16];
            next.staged_key_sequence = old.key_sequence.wrapping_add(1);
            next.network_key_forwarding_pending = true;
            journal.storage_mut().programs_before_failure = Some(programs);
            let result = journal.store(&next);
            assert_eq!(result.is_ok(), programs == 2);
            journal.storage_mut().programs_before_failure = None;
            assert_eq!(
                journal.load(),
                Ok(Some(if programs == 2 { next } else { old }))
            );
        }
    }

    #[test]
    fn current_records_round_trip_the_end_device_timeout() {
        let mut expected = state(0x400);
        expected.parent_information = 0x02;
        expected.parent_information_valid = true;
        expected.end_device_timeout = 14;
        expected.node_join_link_key_type =
            zigbee_bdb::NodeJoinLinkKeyType::InstallCodeDerivedPreconfiguredLinkKey;

        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&expected).unwrap();
        // Force a rescan so the value comes back off the flash, not the cache.
        journal.storage_mut();
        assert_eq!(journal.load(), Ok(Some(expected)));
        assert_eq!(journal.storage().data[4], RECORD_VERSION);
        assert_eq!(
            journal.storage().data[5] as usize,
            ENCODED_SECURITY_STATE_LEN
        );
        assert_eq!(journal.storage().data[12 + 98], 0x02);
    }

    /// Version 4 must be able to persist "this device holds no authoritative
    /// `nwkUpdateId`" and read it back as exactly that — the whole reason the
    /// revision exists.
    #[test]
    fn current_records_round_trip_update_id_validity() {
        for (update_id, valid) in [(0x2A, true), (0x00, true), (0x00, false)] {
            let mut expected = state(0x400);
            expected.update_id = update_id;
            expected.update_id_valid = valid;

            let mut journal =
                SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
            journal.store(&expected).unwrap();
            // Force a rescan so the value comes back off the flash.
            journal.storage_mut();
            assert_eq!(journal.load(), Ok(Some(expected)), "{update_id}/{valid}");
            assert_eq!(journal.storage().data[4], RECORD_VERSION);
            // No byte was added: the validity lives in flags bit 7 of the
            // unchanged 98-byte state.
            assert_eq!(
                journal.storage().data[5] as usize,
                ENCODED_SECURITY_STATE_LEN
            );
            assert_eq!(journal.storage().data[12] & (1 << 7) != 0, valid);
            assert_eq!(journal.storage().data[12 + 3], update_id);
        }
    }

    /// A version 3 record is the same length as a version 4 one, so only the
    /// version byte separates them. Its `update_id` was authoritative when it
    /// was written and must stay so.
    #[test]
    fn version_three_records_load_with_an_authoritative_update_id() {
        for update_id in [0x00u8, 0x2A] {
            let mut stored = state(0x400);
            stored.update_id = update_id;
            stored.update_id_valid = true;
            stored.parent_information = 0x02;
            stored.parent_information_valid = true;
            stored.end_device_timeout = 14;

            let mut flash = MockFlash::new();
            write_migrated_record(
                &mut flash,
                V3_RECORD_VERSION,
                V4_ENCODED_SECURITY_STATE_LEN,
                V6_RECORD_CRC_OFFSET,
                &stored,
            );
            assert_eq!(flash.data[12] & (1 << 7), 0, "v3 never wrote bit 7");

            let mut journal =
                SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
            let migrated = journal.load().unwrap().unwrap();
            assert_eq!(migrated, stored, "update_id {update_id}");
            assert!(migrated.update_id_valid);
            // The version 3 fields it owns are still decoded in place.
            assert_eq!(migrated.parent_information, 0x02);
            assert_eq!(migrated.end_device_timeout, 14);
        }
    }

    #[test]
    fn version_three_records_reject_the_version_four_flag_bit() {
        // Byte 0 bit 7 only exists from v4 onwards; a v3 record carrying it is
        // corrupt rather than a valid "update ID is valid" record. The same
        // bytes under the v4 version byte are accepted, so the version byte —
        // not the length — is what makes the difference.
        for (version, expected) in [(V3_RECORD_VERSION, None), (V4_RECORD_VERSION, Some(0x2Au8))] {
            let mut stored = state(0x400);
            stored.update_id = 0x2A;
            stored.update_id_valid = true;

            let mut flash = MockFlash::new();
            write_migrated_record(
                &mut flash,
                version,
                V4_ENCODED_SECURITY_STATE_LEN,
                V6_RECORD_CRC_OFFSET,
                &stored,
            );
            // Set bit 7 regardless of the version the record claims.
            flash.data[12] |= 1 << 7;
            let crc = crc32(&flash.data[..V6_RECORD_CRC_OFFSET]);
            flash.data[V6_RECORD_CRC_OFFSET..V6_RECORD_CRC_OFFSET + 4]
                .copy_from_slice(&crc.to_le_bytes());

            let mut journal =
                SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
            assert_eq!(
                journal.load().unwrap().map(|state| state.update_id),
                expected,
                "version {version}"
            );
        }
    }

    /// The migrating decoders must not invent an update state that the older
    /// record could not have held.
    #[test]
    fn version_one_and_two_records_keep_their_stored_update_id() {
        let mut stored = state(0x400);
        stored.update_id = 0x2A;
        stored.update_id_valid = true;

        for (version, encoded_len, crc_offset) in [
            (
                LEGACY_RECORD_VERSION,
                LEGACY_ENCODED_SECURITY_STATE_LEN,
                LEGACY_RECORD_CRC_OFFSET,
            ),
            (
                V2_RECORD_VERSION,
                V2_ENCODED_SECURITY_STATE_LEN,
                V6_RECORD_CRC_OFFSET,
            ),
        ] {
            let mut flash = MockFlash::new();
            write_migrated_record(&mut flash, version, encoded_len, crc_offset, &stored);
            assert_eq!(flash.data[12] & (1 << 7), 0, "v{version} never wrote bit 7");

            let mut journal =
                SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
            let migrated = journal.load().unwrap().unwrap();
            assert_eq!(migrated.update_id, 0x2A, "v{version}");
            assert!(migrated.update_id_valid, "v{version}");
        }
    }

    #[test]
    fn legacy_version_one_record_is_still_loaded() {
        // A v1 record predates the validity bit; its stored update ID was
        // authoritative in the firmware that wrote it, so it migrates as
        // known-good.
        let mut expected = state(0x400);
        expected.update_id_valid = true;
        let mut flash = MockFlash::new();
        write_migrated_record(
            &mut flash,
            LEGACY_RECORD_VERSION,
            LEGACY_ENCODED_SECURITY_STATE_LEN,
            LEGACY_RECORD_CRC_OFFSET,
            &expected,
        );
        let mut journal = SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(journal.load(), Ok(Some(expected)));
    }

    #[test]
    fn version_one_and_two_records_migrate_to_the_default_timeout() {
        // A v1/v2 record has no End Device Timeout fields; the migrated state
        // must fall back to "not negotiated, default enumeration 8".
        let mut stored = state(0x400);
        stored.parent_information = 0x03;
        stored.parent_information_valid = true;
        stored.end_device_timeout = 14;

        for (version, encoded_len, crc_offset) in [
            (
                LEGACY_RECORD_VERSION,
                LEGACY_ENCODED_SECURITY_STATE_LEN,
                LEGACY_RECORD_CRC_OFFSET,
            ),
            (
                V2_RECORD_VERSION,
                V2_ENCODED_SECURITY_STATE_LEN,
                V6_RECORD_CRC_OFFSET,
            ),
        ] {
            let mut flash = MockFlash::new();
            write_migrated_record(&mut flash, version, encoded_len, crc_offset, &stored);
            let mut journal =
                SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
            let migrated = journal.load().unwrap().unwrap();
            assert_eq!(migrated.global_counter_limit, 0x400, "v{version}");
            assert_eq!(migrated.parent_information, 0, "v{version}");
            assert!(!migrated.parent_information_valid, "v{version}");
            assert_eq!(migrated.end_device_timeout, 8, "v{version}");
        }
    }

    #[test]
    fn version_two_records_keep_the_staged_network_key() {
        let mut stored = state(0x400);
        stored.commissioned = true;
        stored.channel = 15;
        stored.pan_id = 0x1234;
        stored.short_address = 0x5678;
        stored.ieee_address = [2; 8];
        stored.network_key = [3; 16];
        stored.key_sequence = 4;
        stored.staged_network_key_present = true;
        stored.staged_network_key = [8; 16];
        stored.staged_key_sequence = 5;
        stored.tclk_present = true;
        stored.trust_center_address = [6; 8];
        stored.tclk_counter_limit = 0x800;

        let mut flash = MockFlash::new();
        write_migrated_record(
            &mut flash,
            V2_RECORD_VERSION,
            V2_ENCODED_SECURITY_STATE_LEN,
            V6_RECORD_CRC_OFFSET,
            &stored,
        );
        let mut journal = SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        let migrated = journal.load().unwrap().unwrap();
        assert!(migrated.staged_network_key_present);
        assert_eq!(migrated.staged_network_key, [8; 16]);
        assert_eq!(migrated.staged_key_sequence, 5);
        assert_eq!(migrated.end_device_timeout, 8);
    }

    #[test]
    fn version_five_records_migrate_a_secondary_key_as_staged() {
        let mut stored = commissioned_state();
        stored.key_sequence = 4;
        stored.staged_network_key_present = true;
        stored.staged_network_key = [8; 16];
        stored.staged_key_sequence = 5;

        let mut flash = MockFlash::new();
        write_migrated_record(
            &mut flash,
            V5_RECORD_VERSION,
            V5_ENCODED_SECURITY_STATE_LEN,
            V6_RECORD_CRC_OFFSET,
            &stored,
        );
        let mut journal = SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        let migrated = journal.load().unwrap().unwrap();
        assert!(migrated.staged_network_key_present);
        assert_eq!(migrated.staged_key_sequence, 5);
        assert!(!migrated.secondary_network_key_is_previous);
    }

    #[test]
    fn version_two_records_reject_the_version_three_flag_bit() {
        // Byte 0 bit 6 only exists from v3 onwards; a v2 record carrying it is
        // corrupt rather than a valid "parent information is valid" record.
        let mut flash = MockFlash::new();
        write_migrated_record(
            &mut flash,
            V2_RECORD_VERSION,
            V2_ENCODED_SECURITY_STATE_LEN,
            V6_RECORD_CRC_OFFSET,
            &state(0x400),
        );
        flash.data[12] |= 1 << 6;
        let crc = crc32(&flash.data[..V6_RECORD_CRC_OFFSET]);
        flash.data[V6_RECORD_CRC_OFFSET..V6_RECORD_CRC_OFFSET + 4]
            .copy_from_slice(&crc.to_le_bytes());

        let mut journal = SecurityStateJournal::new(flash, 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(journal.load(), Ok(None));
    }

    #[test]
    fn corrupt_end_device_timeout_fields_are_rejected() {
        for (offset, value) in [
            // byte 97 of the encoded state: undefined timeout enumeration.
            (12 + 97, 15u8),
            // byte 11 of the encoded state: reserved parent-information bit.
            (12 + 11, 0x04),
        ] {
            let mut journal =
                SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
            let mut stored = state(0x400);
            stored.parent_information = 0x01;
            stored.parent_information_valid = true;
            journal.store(&stored).unwrap();

            let flash = journal.storage_mut();
            flash.data[offset] = value;
            let crc = crc32(&flash.data[..RECORD_CRC_OFFSET]);
            flash.data[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4]
                .copy_from_slice(&crc.to_le_bytes());

            assert_eq!(journal.load(), Ok(None), "offset {offset} value {value}");
        }
    }

    #[test]
    fn parent_information_without_validity_is_rejected() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        let mut stored = state(0x400);
        stored.parent_information = 0x01;
        stored.parent_information_valid = true;
        journal.store(&stored).unwrap();

        let flash = journal.storage_mut();
        // Clear the validity flag while leaving the advertised bits behind.
        flash.data[12] &= !(1 << 6);
        let crc = crc32(&flash.data[..RECORD_CRC_OFFSET]);
        flash.data[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        assert_eq!(journal.load(), Ok(None));
    }

    #[test]
    fn newest_committed_record_wins() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&state(0x400)).unwrap();
        journal.store(&state(0x800)).unwrap();
        assert_eq!(journal.load().unwrap().unwrap().global_counter_limit, 0x800);
    }

    #[test]
    fn compact_replay_entries_round_trip_and_merge_by_domain() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        journal.commit_replay_counter(nwk_replay(7)).unwrap();
        journal.commit_replay_counter(nwk_replay(9)).unwrap();
        let aps = PersistentReplayCounter::Aps(zigbee_aps::security::ApsReplayCounter {
            origin: zigbee_aps::security::ApsReplayOrigin::PreconfiguredGlobal {
                source: [0x44; 8],
            },
            key_fingerprint: key_fingerprint(&zigbee_aps::security::DEFAULT_TC_LINK_KEY),
            counter: 3,
        });
        journal.commit_replay_counter(aps).unwrap();

        journal.storage_mut();
        let mut restored = heapless::Vec::<PersistentReplayCounter, 4>::new();
        journal
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.len(), 2);
        assert!(restored.contains(&nwk_replay(9)));
        assert!(restored.contains(&aps));
    }

    #[test]
    fn device_replay_tombstone_survives_reboot() {
        let removed = [0x31; 8];
        let retained = [0x32; 8];
        let network_fingerprint = key_fingerprint(&[3; 16]);
        let application_fingerprint = key_fingerprint(&[6; 16]);
        let global_fingerprint = key_fingerprint(&zigbee_aps::security::DEFAULT_TC_LINK_KEY);
        let removed_nwk = nwk_replay_for(removed, network_fingerprint, 7);
        let retained_nwk = nwk_replay_for(retained, network_fingerprint, 8);
        let removed_aps = aps_key_pair_replay(removed, application_fingerprint, 3);
        let retained_global = aps_global_replay(removed, global_fingerprint, 4);

        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        for replay in [removed_nwk, retained_nwk, removed_aps, retained_global] {
            journal.commit_replay_counter(replay).unwrap();
        }
        journal
            .tombstone_replay_counters(ReplayCounterTombstone::Device(removed))
            .unwrap();

        let mut rebooted = SecurityStateJournal::new(
            journal.into_storage(),
            0,
            SECURITY_JOURNAL_SECTOR_SIZE as u32,
        );
        let mut restored = heapless::Vec::<PersistentReplayCounter, 4>::new();
        rebooted
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.len(), 2);
        assert!(restored.contains(&retained_nwk));
        assert!(restored.contains(&retained_global));
        assert!(!restored.contains(&removed_nwk));
        assert!(!restored.contains(&removed_aps));
    }

    #[test]
    fn key_replay_tombstone_removes_only_matching_domains() {
        let retired_fingerprint = key_fingerprint(&[6; 16]);
        let retained_fingerprint = key_fingerprint(&[7; 16]);
        let removed_nwk = nwk_replay_for([0x41; 8], retired_fingerprint, 7);
        let removed_aps = aps_key_pair_replay([0x42; 8], retired_fingerprint, 3);
        let retained_nwk = nwk_replay_for([0x43; 8], retained_fingerprint, 8);
        let retained_aps = aps_key_pair_replay([0x44; 8], retained_fingerprint, 4);

        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        for replay in [removed_nwk, removed_aps, retained_nwk, retained_aps] {
            journal.commit_replay_counter(replay).unwrap();
        }
        journal
            .tombstone_replay_counters(ReplayCounterTombstone::KeyFingerprint(retired_fingerprint))
            .unwrap();

        let mut rebooted = SecurityStateJournal::new(
            journal.into_storage(),
            0,
            SECURITY_JOURNAL_SECTOR_SIZE as u32,
        );
        let mut restored = heapless::Vec::<PersistentReplayCounter, 4>::new();
        rebooted
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.len(), 2);
        assert!(restored.contains(&retained_nwk));
        assert!(restored.contains(&retained_aps));
        assert!(!restored.contains(&removed_nwk));
        assert!(!restored.contains(&removed_aps));
    }

    #[test]
    fn exact_replay_tombstone_preserves_live_domains_with_the_same_key() {
        let shared_fingerprint = key_fingerprint(&[6; 16]);
        let removed = aps_key_pair_replay([0x45; 8], shared_fingerprint, 3);
        let retained_pair = aps_key_pair_replay([0x46; 8], shared_fingerprint, 4);
        let retained_global = aps_global_replay([0x45; 8], shared_fingerprint, 5);

        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        for replay in [removed, retained_pair, retained_global] {
            journal.commit_replay_counter(replay).unwrap();
        }
        journal
            .tombstone_replay_counters(ReplayCounterTombstone::ReplayDomain(removed))
            .unwrap();

        let mut rebooted = SecurityStateJournal::new(
            journal.into_storage(),
            0,
            SECURITY_JOURNAL_SECTOR_SIZE as u32,
        );
        let mut restored = heapless::Vec::<PersistentReplayCounter, 4>::new();
        rebooted
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.len(), 2);
        assert!(!restored.contains(&removed));
        assert!(restored.contains(&retained_pair));
        assert!(restored.contains(&retained_global));
    }

    #[test]
    fn replay_retention_compacts_all_rejected_domains_atomically() {
        let shared_fingerprint = key_fingerprint(&[6; 16]);
        let removed_pair = aps_key_pair_replay([0x45; 8], shared_fingerprint, 3);
        let retained_pair = aps_key_pair_replay([0x46; 8], shared_fingerprint, 4);
        let removed_nwk = nwk_replay_for([0x47; 8], key_fingerprint(&[3; 16]), 7);

        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        for replay in [removed_pair, retained_pair, removed_nwk] {
            journal.commit_replay_counter(replay).unwrap();
        }
        assert_eq!(
            journal
                .retain_replay_counters(&|replay| replay == retained_pair)
                .unwrap(),
            2
        );

        let mut rebooted = SecurityStateJournal::new(
            journal.into_storage(),
            0,
            SECURITY_JOURNAL_SECTOR_SIZE as u32,
        );
        let mut restored = heapless::Vec::<PersistentReplayCounter, 3>::new();
        rebooted
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.as_slice(), &[retained_pair]);
    }

    #[test]
    fn torn_replay_retention_keeps_the_previous_generation() {
        let removed = nwk_replay_for([0x51; 8], key_fingerprint(&[3; 16]), 7);
        let retained = aps_key_pair_replay([0x52; 8], key_fingerprint(&[6; 16]), 3);
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        journal.commit_replay_counter(removed).unwrap();
        journal.commit_replay_counter(retained).unwrap();

        journal.storage_mut().programs_before_failure = Some(4);
        assert_eq!(
            journal.retain_replay_counters(&|replay| replay == retained),
            Err(SecurityStoreError::Hardware)
        );
        journal.storage_mut().programs_before_failure = None;

        let mut rebooted = SecurityStateJournal::new(
            journal.into_storage(),
            0,
            SECURITY_JOURNAL_SECTOR_SIZE as u32,
        );
        let mut restored = heapless::Vec::<PersistentReplayCounter, 2>::new();
        rebooted
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.len(), 2);
        assert!(restored.contains(&removed));
        assert!(restored.contains(&retained));
    }

    #[test]
    fn torn_replay_tombstone_keeps_the_previous_generation() {
        let removed = [0x51; 8];
        let removed_replay = nwk_replay_for(removed, key_fingerprint(&[3; 16]), 7);
        let retained_replay = aps_key_pair_replay([0x52; 8], key_fingerprint(&[6; 16]), 3);
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        journal.commit_replay_counter(removed_replay).unwrap();
        journal.commit_replay_counter(retained_replay).unwrap();

        journal.storage_mut().programs_before_failure = Some(4);
        assert_eq!(
            journal.tombstone_replay_counters(ReplayCounterTombstone::Device(removed)),
            Err(SecurityStoreError::Hardware)
        );
        journal.storage_mut().programs_before_failure = None;

        let mut rebooted = SecurityStateJournal::new(
            journal.into_storage(),
            0,
            SECURITY_JOURNAL_SECTOR_SIZE as u32,
        );
        let mut restored = heapless::Vec::<PersistentReplayCounter, 4>::new();
        rebooted
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.len(), 2);
        assert!(restored.contains(&removed_replay));
        assert!(restored.contains(&retained_replay));
    }

    #[test]
    fn torn_replay_entry_is_never_activated() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        journal.commit_replay_counter(nwk_replay(7)).unwrap();

        journal.storage_mut().programs_before_failure = Some(1);
        assert_eq!(
            journal.commit_replay_counter(nwk_replay(8)),
            Err(SecurityStoreError::Hardware)
        );
        journal.storage_mut().programs_before_failure = None;
        let mut restored = heapless::Vec::<PersistentReplayCounter, 2>::new();
        journal
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.as_slice(), &[nwk_replay(7)]);
    }

    #[test]
    fn replay_rollover_keeps_the_old_generation_until_state_commit() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        let capacity = (SECURITY_JOURNAL_SLOTS_PER_SECTOR - 1) * REPLAY_ENTRIES_PER_SLOT;
        for counter in 1..=capacity as u32 {
            journal.commit_replay_counter(nwk_replay(counter)).unwrap();
        }

        journal.storage_mut().programs_before_failure = Some(3);
        assert_eq!(
            journal.commit_replay_counter(nwk_replay(capacity as u32 + 1)),
            Err(SecurityStoreError::Hardware)
        );
        journal.storage_mut().programs_before_failure = None;
        let mut restored = heapless::Vec::<PersistentReplayCounter, 2>::new();
        journal
            .visit_replay_counters(&mut |entry| restored.push(entry).unwrap())
            .unwrap();
        assert_eq!(restored.as_slice(), &[nwk_replay(capacity as u32)]);
    }

    #[test]
    fn factory_new_state_discards_replay_entries() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&commissioned_state()).unwrap();
        journal.commit_replay_counter(nwk_replay(7)).unwrap();
        journal.store(&PersistentSecurityState::empty()).unwrap();

        journal.storage_mut();
        let mut count = 0usize;
        journal
            .visit_replay_counters(&mut |_| count = count.saturating_add(1))
            .unwrap();
        assert_eq!(count, 0);
    }

    #[test]
    fn rollover_keeps_previous_sector_until_new_commit() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        for counter in 1..=SECURITY_JOURNAL_SLOTS_PER_SECTOR {
            journal.store(&state(counter as u32 * 0x400)).unwrap();
        }
        let previous = journal.load().unwrap().unwrap();

        journal.storage_mut().programs_before_failure = Some(1);
        assert_eq!(
            journal.store(&state(previous.global_counter_limit + 0x400)),
            Err(SecurityStoreError::Hardware)
        );
        assert_eq!(journal.load(), Ok(Some(previous)));
    }

    #[test]
    fn rollover_selects_new_sector_after_commit() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        for counter in 1..=SECURITY_JOURNAL_SLOTS_PER_SECTOR + 1 {
            journal.store(&state(counter as u32 * 0x400)).unwrap();
        }
        assert_eq!(
            journal.load().unwrap().unwrap().global_counter_limit,
            (SECURITY_JOURNAL_SLOTS_PER_SECTOR as u32 + 1) * 0x400
        );
    }

    #[test]
    fn corrupt_newest_record_falls_back_to_previous_commit() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        journal.store(&state(0x400)).unwrap();
        journal.store(&state(0x800)).unwrap();
        journal.storage_mut().data[SECURITY_JOURNAL_SLOT_SIZE + 12] ^= 1;
        assert_eq!(journal.load().unwrap().unwrap().global_counter_limit, 0x400);
    }
}
