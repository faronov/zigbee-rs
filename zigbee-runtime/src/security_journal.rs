//! Atomic two-sector journal for persistent Zigbee security state.

use embedded_storage::nor_flash::NorFlash;

use crate::security_store::{
    ENCODED_SECURITY_STATE_LEN, PersistentSecurityState, SecurityStateStore, SecurityStoreError,
};

pub const SECURITY_JOURNAL_SECTOR_SIZE: usize = 4096;
pub const SECURITY_JOURNAL_SLOT_SIZE: usize = 128;
pub const SECURITY_JOURNAL_SLOTS_PER_SECTOR: usize =
    SECURITY_JOURNAL_SECTOR_SIZE / SECURITY_JOURNAL_SLOT_SIZE;

const RECORD_MAGIC: [u8; 4] = *b"ZBSS";
const RECORD_VERSION: u8 = 1;
const RECORD_CRC_OFFSET: usize = 92;
const RECORD_PREFIX_LEN: usize = 96;
const RECORD_COMMIT_OFFSET: usize = 124;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

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
            || record[4] != RECORD_VERSION
            || record[5] as usize != ENCODED_SECURITY_STATE_LEN
        {
            return None;
        }

        let expected_crc = u32::from_le_bytes([
            record[RECORD_CRC_OFFSET],
            record[RECORD_CRC_OFFSET + 1],
            record[RECORD_CRC_OFFSET + 2],
            record[RECORD_CRC_OFFSET + 3],
        ]);
        if crc32(&record[..RECORD_CRC_OFFSET]) != expected_crc {
            return None;
        }

        let generation = u32::from_le_bytes([record[8], record[9], record[10], record[11]]);
        let mut encoded_state = [0u8; ENCODED_SECURITY_STATE_LEN];
        encoded_state.copy_from_slice(&record[12..12 + ENCODED_SECURITY_STATE_LEN]);
        let state = PersistentSecurityState::decode(&encoded_state).ok()?;
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
            || SECURITY_JOURNAL_SLOT_SIZE % S::READ_SIZE != 0
            || SECURITY_JOURNAL_SLOT_SIZE % S::WRITE_SIZE != 0
            || SECURITY_JOURNAL_SECTOR_SIZE % S::ERASE_SIZE != 0
            || RECORD_PREFIX_LEN % S::WRITE_SIZE != 0
            || RECORD_COMMIT_OFFSET % S::WRITE_SIZE != 0
            || RECORD_COMMIT.len() % S::WRITE_SIZE != 0
            || self.sectors[0] as usize % S::ERASE_SIZE != 0
            || self.sectors[1] as usize % S::ERASE_SIZE != 0
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

fn crc32(data: &[u8]) -> u32 {
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

    #[test]
    fn committed_records_round_trip() {
        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        assert_eq!(journal.load(), Ok(None));
        journal.store(&state(0x400)).unwrap();
        assert_eq!(journal.load().unwrap().unwrap().global_counter_limit, 0x400);
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
