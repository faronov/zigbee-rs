//! Atomic two-sector journal for persistent Zigbee security state.
//!
//! # Record versions
//!
//! | version | encoded state | CRC offset | added                        |
//! |---------|---------------|------------|------------------------------|
//! | 1       | 80 bytes      | 92         | initial layout               |
//! | 2       | 97 bytes      | 112        | staged network key           |
//! | 3       | 98 bytes      | 112        | R22 End Device Timeout state |
//!
//! Slot size, record prefix length and commit offset never changed, so a
//! newer firmware reads every older record in place and the two-sector
//! crash-safety scheme is unaffected.
//!
//! # Downgrade is not supported
//!
//! Once this firmware has written a version 3 record, **downgrading to
//! firmware that predates version 3 is unsupported**. Older firmware does not
//! recognise version 3 and skips those records while scanning, so it would
//! select the newest record it *can* decode — an older generation with stale
//! counters, a stale parent and possibly a stale network key. Reusing those
//! reservations would replay NWK/APS frame counters. Recommission the device
//! instead of downgrading.

use embedded_storage::nor_flash::NorFlash;

use crate::security_store::{
    ENCODED_SECURITY_STATE_LEN, LEGACY_ENCODED_SECURITY_STATE_LEN, PersistentSecurityState,
    SecurityStateStore, SecurityStoreError, V2_ENCODED_SECURITY_STATE_LEN,
};

pub const SECURITY_JOURNAL_SECTOR_SIZE: usize = 4096;
pub const SECURITY_JOURNAL_SLOT_SIZE: usize = 128;
pub const SECURITY_JOURNAL_SLOTS_PER_SECTOR: usize =
    SECURITY_JOURNAL_SECTOR_SIZE / SECURITY_JOURNAL_SLOT_SIZE;

const RECORD_MAGIC: [u8; 4] = *b"ZBSS";
const LEGACY_RECORD_VERSION: u8 = 1;
const V2_RECORD_VERSION: u8 = 2;
const RECORD_VERSION: u8 = 3;
const LEGACY_RECORD_CRC_OFFSET: usize = 92;
const RECORD_CRC_OFFSET: usize = 112;
const RECORD_PREFIX_LEN: usize = 116;
const RECORD_COMMIT_OFFSET: usize = 124;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

// The encoded state starts at byte 12 and must stay clear of the CRC field.
const _: () = assert!(12 + ENCODED_SECURITY_STATE_LEN <= RECORD_CRC_OFFSET);
const _: () = assert!(12 + V2_ENCODED_SECURITY_STATE_LEN <= RECORD_CRC_OFFSET);
const _: () = assert!(12 + LEGACY_ENCODED_SECURITY_STATE_LEN <= LEGACY_RECORD_CRC_OFFSET);

pub struct SecurityStateJournal<S> {
    storage: S,
    sectors: [u32; 2],
    cached: Option<LocatedState>,
    scanned: bool,
}

#[derive(Clone, Copy)]
struct LocatedState {
    generation: u32,
    sector: usize,
    state: PersistentSecurityState,
}

impl<S: NorFlash> SecurityStateJournal<S> {
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

    pub fn storage_mut(&mut self) -> &mut S {
        self.cached = None;
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

        let (crc_offset, encoded_len) = match (record[4], record[5] as usize) {
            (RECORD_VERSION, ENCODED_SECURITY_STATE_LEN) => {
                (RECORD_CRC_OFFSET, ENCODED_SECURITY_STATE_LEN)
            }
            (V2_RECORD_VERSION, V2_ENCODED_SECURITY_STATE_LEN) => {
                (RECORD_CRC_OFFSET, V2_ENCODED_SECURITY_STATE_LEN)
            }
            (LEGACY_RECORD_VERSION, LEGACY_ENCODED_SECURITY_STATE_LEN) => {
                (LEGACY_RECORD_CRC_OFFSET, LEGACY_ENCODED_SECURITY_STATE_LEN)
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
        // Each version decodes through its own fixed-size buffer, so an older
        // record can never be read with the newer field offsets or accept a
        // flag bit its layout predates.
        let state = match encoded_len {
            ENCODED_SECURITY_STATE_LEN => {
                let mut encoded_state = [0u8; ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode(&encoded_state).ok()?
            }
            V2_ENCODED_SECURITY_STATE_LEN => {
                let mut encoded_state = [0u8; V2_ENCODED_SECURITY_STATE_LEN];
                encoded_state.copy_from_slice(&record[12..12 + V2_ENCODED_SECURITY_STATE_LEN]);
                PersistentSecurityState::decode_v2(&encoded_state).ok()?
            }
            _ => {
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
            for slot in 0..SECURITY_JOURNAL_SLOTS_PER_SECTOR {
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
            || self.sectors[0].abs_diff(self.sectors[1]) < SECURITY_JOURNAL_SECTOR_SIZE as u32
            || S::READ_SIZE == 0
            || S::WRITE_SIZE == 0
            || S::ERASE_SIZE == 0
            || !SECURITY_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
            || !SECURITY_JOURNAL_SLOT_SIZE.is_multiple_of(S::WRITE_SIZE)
            || !SECURITY_JOURNAL_SECTOR_SIZE.is_multiple_of(S::ERASE_SIZE)
            || !RECORD_PREFIX_LEN.is_multiple_of(S::WRITE_SIZE)
            || !RECORD_COMMIT_OFFSET.is_multiple_of(S::WRITE_SIZE)
            || !RECORD_COMMIT.len().is_multiple_of(S::WRITE_SIZE)
            || !(self.sectors[0] as usize).is_multiple_of(S::ERASE_SIZE)
            || !(self.sectors[1] as usize).is_multiple_of(S::ERASE_SIZE)
            || self.sectors.iter().any(|sector| {
                (*sector as usize)
                    .checked_add(SECURITY_JOURNAL_SECTOR_SIZE)
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

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, SecurityStoreError> {
        let mut record = [0u8; SECURITY_JOURNAL_SLOT_SIZE];
        for slot in 0..SECURITY_JOURNAL_SLOTS_PER_SECTOR {
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

impl<S: NorFlash> SecurityStateStore for SecurityStateJournal<S> {
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

        if let Some(located) = current {
            if let Some(slot) = self.first_erased_slot(located.sector)? {
                let result = self.write_record(located.sector, slot, generation, state);
                if result.is_ok() {
                    self.cached = Some(LocatedState {
                        generation,
                        sector: located.sector,
                        state: *state,
                    });
                } else {
                    self.cached = None;
                    self.scanned = false;
                }
                return result;
            }

            let target = 1 - located.sector;
            let sector = self.sectors[target];
            let result = self
                .storage
                .erase(sector, sector + SECURITY_JOURNAL_SECTOR_SIZE as u32)
                .map_err(|_| SecurityStoreError::Hardware)
                .and_then(|()| self.write_record(target, 0, generation, state));
            if result.is_ok() {
                self.cached = Some(LocatedState {
                    generation,
                    sector: target,
                    state: *state,
                });
            } else {
                self.cached = None;
                self.scanned = false;
            }
            return result;
        }

        let sector = self.sectors[0];
        let result = self
            .storage
            .erase(sector, sector + SECURITY_JOURNAL_SECTOR_SIZE as u32)
            .map_err(|_| SecurityStoreError::Hardware)
            .and_then(|()| self.write_record(0, 0, generation, state));
        if result.is_ok() {
            self.cached = Some(LocatedState {
                generation,
                sector: 0,
                state: *state,
            });
        } else {
            self.cached = None;
            self.scanned = false;
        }
        result
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

    /// Write a pre-v3 record by hand, exactly as the older firmware did.
    ///
    /// The v3-only content (flags bit 6 and encoded byte 11) is stripped, so
    /// the bytes on flash are byte-for-byte what firmware predating the R22
    /// End Device Timeout fields would have written. Everything past
    /// `encoded_len` stays erased (0xFF), which is what makes this a real
    /// migration test: a decoder that wrongly indexed encoded byte 97 would
    /// read 0xFF and reject the record.
    fn write_migrated_record(
        flash: &mut MockFlash,
        version: u8,
        encoded_len: usize,
        crc_offset: usize,
        state: &PersistentSecurityState,
    ) {
        let mut current = [0u8; ENCODED_SECURITY_STATE_LEN];
        state.encode(&mut current);
        if encoded_len < ENCODED_SECURITY_STATE_LEN {
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
    fn version_three_records_round_trip_the_end_device_timeout() {
        let mut expected = state(0x400);
        expected.parent_information = 0x02;
        expected.parent_information_valid = true;
        expected.end_device_timeout = 14;

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
    }

    #[test]
    fn legacy_version_one_record_is_still_loaded() {
        let expected = state(0x400);
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
                RECORD_CRC_OFFSET,
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
            RECORD_CRC_OFFSET,
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
    fn version_two_records_reject_the_version_three_flag_bit() {
        // Byte 0 bit 6 only exists from v3 onwards; a v2 record carrying it is
        // corrupt rather than a valid "parent information is valid" record.
        let mut flash = MockFlash::new();
        write_migrated_record(
            &mut flash,
            V2_RECORD_VERSION,
            V2_ENCODED_SECURITY_STATE_LEN,
            RECORD_CRC_OFFSET,
            &state(0x400),
        );
        flash.data[12] |= 1 << 6;
        let crc = crc32(&flash.data[..RECORD_CRC_OFFSET]);
        flash.data[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

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
