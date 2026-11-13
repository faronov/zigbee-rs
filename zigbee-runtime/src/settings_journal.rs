//! Small durable application settings.
//!
//! Products keep a few client-writable values, for example a ZCL delay
//! attribute, across resets. [`SettingsJournal`] stores one opaque record of
//! at most [`MAX_SETTINGS_PAYLOAD`] bytes in an atomic two-sector flash
//! journal. The product owns the payload encoding and the restore policy;
//! this module only guarantees that a load returns the newest completely
//! written record or nothing:
//!
//! - every record carries a magic, a version, its length, a generation and a
//!   CRC-32, and its commit marker is written last;
//! - the whole slot is verified by read-back after the commit;
//! - records are appended to the current sector; when it is full the other
//!   sector is erased and written, so an interrupted erase or write always
//!   leaves the previous record;
//! - the newest valid generation wins; generation exhaustion is an error,
//!   never a silent wraparound.
//!
//! [`SettingsStore`] abstracts the journal for host tests
//! ([`RamSettingsStore`]).

use embedded_storage::nor_flash::NorFlash;

use crate::security_journal::crc32;

/// Size of one journal slot.
pub const SETTINGS_SLOT_SIZE: usize = 64;

const RECORD_MAGIC: [u8; 4] = *b"ZBST";
const RECORD_VERSION: u8 = 1;
const RECORD_GENERATION_OFFSET: usize = 8;
const RECORD_PAYLOAD_OFFSET: usize = 12;
const RECORD_CRC_OFFSET: usize = SETTINGS_SLOT_SIZE - 12;
const RECORD_COMMIT_OFFSET: usize = SETTINGS_SLOT_SIZE - 8;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

/// Largest payload of one record.
pub const MAX_SETTINGS_PAYLOAD: usize = RECORD_CRC_OFFSET - RECORD_PAYLOAD_OFFSET;

/// Settings persistence failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SettingsError {
    /// Flash failed, read-back differed, or the journal geometry is invalid.
    Hardware,
    /// The payload is empty or longer than [`MAX_SETTINGS_PAYLOAD`].
    InvalidLength,
    /// The generation counter cannot advance; the journal must be erased.
    GenerationExhausted,
}

/// One stored settings payload.
#[derive(Clone, PartialEq, Eq)]
pub struct SettingsRecord {
    len: u8,
    bytes: [u8; MAX_SETTINGS_PAYLOAD],
}

impl core::fmt::Debug for SettingsRecord {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("SettingsRecord")
            .field(&self.as_slice())
            .finish()
    }
}

impl SettingsRecord {
    /// A record holding `payload` (1..=[`MAX_SETTINGS_PAYLOAD`] bytes).
    pub fn new(payload: &[u8]) -> Result<Self, SettingsError> {
        if payload.is_empty() || payload.len() > MAX_SETTINGS_PAYLOAD {
            return Err(SettingsError::InvalidLength);
        }
        let mut bytes = [0u8; MAX_SETTINGS_PAYLOAD];
        bytes[..payload.len()].copy_from_slice(payload);
        Ok(Self {
            len: payload.len() as u8,
            bytes,
        })
    }

    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }
}

/// Durable storage of one settings record.
pub trait SettingsStore {
    /// The newest stored record, if any.
    fn load(&mut self) -> Result<Option<SettingsRecord>, SettingsError>;
    /// Replace the stored record.
    fn store(&mut self, record: &SettingsRecord) -> Result<(), SettingsError>;
}

/// RAM-only [`SettingsStore`] for tests and products without flash.
#[derive(Debug, Default)]
pub struct RamSettingsStore {
    record: Option<SettingsRecord>,
    writes: usize,
}

impl RamSettingsStore {
    pub const fn new() -> Self {
        Self {
            record: None,
            writes: 0,
        }
    }

    /// Successful [`SettingsStore::store`] calls so far.
    pub fn writes(&self) -> usize {
        self.writes
    }
}

impl SettingsStore for RamSettingsStore {
    fn load(&mut self) -> Result<Option<SettingsRecord>, SettingsError> {
        Ok(self.record.clone())
    }

    fn store(&mut self, record: &SettingsRecord) -> Result<(), SettingsError> {
        self.record = Some(record.clone());
        self.writes += 1;
        Ok(())
    }
}

/// Atomic two-sector journal of [`SettingsRecord`]s; see the module
/// documentation for the guarantees.
pub struct SettingsJournal<S, const SECTOR_SIZE: usize> {
    storage: S,
    sectors: [u32; 2],
    /// `(generation, sector)` of the newest committed record, once scanned.
    current: Option<(u32, usize)>,
    scanned: bool,
}

impl<S: NorFlash, const SECTOR_SIZE: usize> SettingsJournal<S, SECTOR_SIZE> {
    const SLOTS_PER_SECTOR: usize = SECTOR_SIZE / SETTINGS_SLOT_SIZE;

    /// Journal over the two erase sectors at `first_sector` and
    /// `second_sector` (offsets into `storage`).
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        assert!(SECTOR_SIZE >= SETTINGS_SLOT_SIZE);
        assert!(SECTOR_SIZE.is_multiple_of(SETTINGS_SLOT_SIZE));
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
            && SETTINGS_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
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
        self.sectors[sector] + (slot * SETTINGS_SLOT_SIZE) as u32
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
        record: &mut [u8; SETTINGS_SLOT_SIZE],
    ) -> Result<(), SettingsError> {
        let address = self.slot_address(sector, slot);
        self.storage
            .read(address, record)
            .map_err(|_| SettingsError::Hardware)
    }

    /// Generation and payload of a committed, CRC-valid slot.
    fn decode(slot: &[u8; SETTINGS_SLOT_SIZE]) -> Option<(u32, SettingsRecord)> {
        if slot[0..4] != RECORD_MAGIC
            || slot[4] != RECORD_VERSION
            || slot[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let mut crc = [0u8; 4];
        crc.copy_from_slice(&slot[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4]);
        if crc32(&slot[..RECORD_CRC_OFFSET]) != u32::from_le_bytes(crc) {
            return None;
        }
        let len = usize::from(slot[5]);
        if len > MAX_SETTINGS_PAYLOAD {
            return None;
        }
        let record =
            SettingsRecord::new(&slot[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + len]).ok()?;
        let mut generation = [0u8; 4];
        generation.copy_from_slice(&slot[RECORD_GENERATION_OFFSET..RECORD_GENERATION_OFFSET + 4]);
        Some((u32::from_le_bytes(generation), record))
    }

    /// Newest valid record; equal generations prefer scan order.
    fn scan(&mut self) -> Result<Option<SettingsRecord>, SettingsError> {
        if !self.geometry_ok() {
            return Err(SettingsError::Hardware);
        }
        let mut slot = [0u8; SETTINGS_SLOT_SIZE];
        let mut best: Option<(u32, usize, SettingsRecord)> = None;
        for sector in 0..2 {
            for index in 0..Self::SLOTS_PER_SECTOR {
                self.read_slot(sector, index, &mut slot)?;
                let Some((generation, record)) = Self::decode(&slot) else {
                    continue;
                };
                if best
                    .as_ref()
                    .is_none_or(|(newest, _, _)| generation > *newest)
                {
                    best = Some((generation, sector, record));
                }
            }
        }
        self.current = best
            .as_ref()
            .map(|(generation, sector, _)| (*generation, *sector));
        self.scanned = true;
        Ok(best.map(|(_, _, record)| record))
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, SettingsError> {
        let mut slot = [0u8; SETTINGS_SLOT_SIZE];
        for index in 0..Self::SLOTS_PER_SECTOR {
            self.read_slot(sector, index, &mut slot)?;
            if slot.iter().all(|byte| *byte == 0xFF) {
                return Ok(Some(index));
            }
        }
        Ok(None)
    }

    fn write_slot(
        &mut self,
        sector: usize,
        index: usize,
        generation: u32,
        record: &SettingsRecord,
    ) -> Result<(), SettingsError> {
        let payload = record.as_slice();
        let mut slot = [0xFFu8; SETTINGS_SLOT_SIZE];
        slot[0..4].copy_from_slice(&RECORD_MAGIC);
        slot[4] = RECORD_VERSION;
        slot[5] = payload.len() as u8;
        slot[6..8].fill(0);
        slot[RECORD_GENERATION_OFFSET..RECORD_GENERATION_OFFSET + 4]
            .copy_from_slice(&generation.to_le_bytes());
        slot[RECORD_PAYLOAD_OFFSET..RECORD_CRC_OFFSET].fill(0);
        slot[RECORD_PAYLOAD_OFFSET..RECORD_PAYLOAD_OFFSET + payload.len()].copy_from_slice(payload);
        let crc = crc32(&slot[..RECORD_CRC_OFFSET]);
        slot[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());

        let address = self.slot_address(sector, index);
        self.storage
            .write(address, &slot[..RECORD_COMMIT_OFFSET])
            .map_err(|_| SettingsError::Hardware)?;
        slot[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + RECORD_COMMIT_OFFSET as u32,
                &slot[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4],
            )
            .map_err(|_| SettingsError::Hardware)?;

        let mut readback = [0u8; SETTINGS_SLOT_SIZE];
        self.read_slot(sector, index, &mut readback)?;
        if readback != slot {
            return Err(SettingsError::Hardware);
        }
        Ok(())
    }

    fn erase_sector(&mut self, sector: usize) -> Result<(), SettingsError> {
        let start = self.sectors[sector];
        self.storage
            .erase(start, start + SECTOR_SIZE as u32)
            .map_err(|_| SettingsError::Hardware)
    }

    fn commit(&mut self, record: &SettingsRecord) -> Result<(), SettingsError> {
        if !self.scanned {
            self.scan()?;
        }
        let (generation, sector, index) = match self.current {
            Some((generation, sector)) => {
                let generation = generation
                    .checked_add(1)
                    .ok_or(SettingsError::GenerationExhausted)?;
                match self.first_erased_slot(sector)? {
                    Some(index) => (generation, sector, index),
                    None => {
                        let target = 1 - sector;
                        self.erase_sector(target)?;
                        (generation, target, 0)
                    }
                }
            }
            None => match self.first_erased_slot(0)? {
                Some(index) => (0, 0, index),
                None => {
                    self.erase_sector(0)?;
                    (0, 0, 0)
                }
            },
        };
        self.write_slot(sector, index, generation, record)?;
        self.current = Some((generation, sector));
        Ok(())
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> SettingsStore for SettingsJournal<S, SECTOR_SIZE> {
    fn load(&mut self) -> Result<Option<SettingsRecord>, SettingsError> {
        self.scan()
    }

    fn store(&mut self, record: &SettingsRecord) -> Result<(), SettingsError> {
        let result = self.commit(record);
        if result.is_err() {
            // Rescan before the next attempt instead of trusting the cache.
            self.scanned = false;
            self.current = None;
        }
        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashErrorKind, ReadNorFlash};

    const SECTOR: usize = 512;
    const SLOTS: usize = SECTOR / SETTINGS_SLOT_SIZE;

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

    type Journal = SettingsJournal<MockFlash, SECTOR>;

    fn journal(flash: MockFlash) -> Journal {
        SettingsJournal::new(flash, 0, SECTOR as u32)
    }

    fn record(byte: u8) -> SettingsRecord {
        SettingsRecord::new(&[byte, byte ^ 0x5A, 3]).unwrap()
    }

    #[test]
    fn erased_flash_loads_nothing_and_a_record_round_trips() {
        let mut j = journal(MockFlash::new());
        assert_eq!(j.load(), Ok(None));
        j.store(&record(1)).unwrap();
        assert_eq!(j.load(), Ok(Some(record(1))));
        // A fresh journal over the same flash finds it too.
        let mut j = journal(j.into_storage());
        assert_eq!(j.load(), Ok(Some(record(1))));
    }

    #[test]
    fn payload_length_is_bounded() {
        assert_eq!(SettingsRecord::new(&[]), Err(SettingsError::InvalidLength));
        assert_eq!(
            SettingsRecord::new(&[0; MAX_SETTINGS_PAYLOAD + 1]),
            Err(SettingsError::InvalidLength)
        );
        let full = SettingsRecord::new(&[0xA5; MAX_SETTINGS_PAYLOAD]).unwrap();
        let mut j = journal(MockFlash::new());
        j.store(&full).unwrap();
        assert_eq!(j.load(), Ok(Some(full)));
    }

    #[test]
    fn newest_record_survives_sector_rollover() {
        let mut j = journal(MockFlash::new());
        for n in 0..(3 * SLOTS) as u8 {
            j.store(&record(n)).unwrap();
            assert_eq!(j.load(), Ok(Some(record(n))));
        }
        let flash = j.into_storage();
        // Erased flash needs no erase; then one erase per rollover.
        assert_eq!(flash.erases, 2);
        let mut j = journal(flash);
        assert_eq!(j.load(), Ok(Some(record((3 * SLOTS - 1) as u8))));
    }

    #[test]
    fn torn_writes_keep_the_previous_record() {
        // Every program step of a commit: payload half, then the marker.
        for programs in 0..2 {
            let mut j = journal(MockFlash::new());
            j.store(&record(1)).unwrap();
            let mut flash = j.into_storage();
            flash.programs_left = Some(programs);
            let mut j = journal(flash);
            assert_eq!(j.store(&record(2)), Err(SettingsError::Hardware));
            let mut flash = j.into_storage();
            flash.programs_left = None;
            let mut j = journal(flash);
            assert_eq!(j.load(), Ok(Some(record(1))), "programs {programs}");
            // The half-written slot is skipped, not reused.
            j.store(&record(3)).unwrap();
            assert_eq!(j.load(), Ok(Some(record(3))));
        }
    }

    #[test]
    fn interrupted_rollover_keeps_the_previous_record() {
        let mut j = journal(MockFlash::new());
        for n in 0..SLOTS as u8 {
            j.store(&record(n)).unwrap();
        }
        let mut flash = j.into_storage();
        // The rollover erases sector 1, then the write fails.
        flash.programs_left = Some(0);
        let mut j = journal(flash);
        assert_eq!(j.store(&record(0xEE)), Err(SettingsError::Hardware));
        let mut flash = j.into_storage();
        flash.programs_left = None;
        let mut j = journal(flash);
        assert_eq!(j.load(), Ok(Some(record(SLOTS as u8 - 1))));
    }

    #[test]
    fn corrupt_and_garbage_slots_are_ignored() {
        let mut j = journal(MockFlash::new());
        j.store(&record(1)).unwrap();
        j.store(&record(2)).unwrap();
        let mut flash = j.into_storage();
        // Flip a payload bit of the newest record: its CRC fails.
        flash.data[SETTINGS_SLOT_SIZE + RECORD_PAYLOAD_OFFSET] ^= 0x01;
        // Random non-erased bytes in sector 1.
        for (i, byte) in flash.data[SECTOR..SECTOR + 100].iter_mut().enumerate() {
            *byte = (i as u8).wrapping_mul(37);
        }
        let mut j = journal(flash);
        assert_eq!(j.load(), Ok(Some(record(1))));
        // A length beyond the payload area is rejected even with a valid CRC.
        let mut flash = j.into_storage();
        let mut slot = [0u8; SETTINGS_SLOT_SIZE];
        slot.copy_from_slice(&flash.data[..SETTINGS_SLOT_SIZE]);
        slot[5] = MAX_SETTINGS_PAYLOAD as u8 + 1;
        slot[RECORD_GENERATION_OFFSET] = 9;
        let crc = crc32(&slot[..RECORD_CRC_OFFSET]);
        slot[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        flash.data[..SETTINGS_SLOT_SIZE].copy_from_slice(&slot);
        let mut j = journal(flash);
        assert_eq!(j.load(), Ok(None));
    }

    #[test]
    fn generation_exhaustion_is_an_error() {
        let mut j = journal(MockFlash::new());
        j.store(&record(1)).unwrap();
        let mut flash = j.into_storage();
        // Rewrite the record with the last generation and a valid CRC.
        let mut slot = [0u8; SETTINGS_SLOT_SIZE];
        slot.copy_from_slice(&flash.data[..SETTINGS_SLOT_SIZE]);
        slot[RECORD_GENERATION_OFFSET..RECORD_GENERATION_OFFSET + 4]
            .copy_from_slice(&u32::MAX.to_le_bytes());
        let crc = crc32(&slot[..RECORD_CRC_OFFSET]);
        slot[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        flash.data[..SETTINGS_SLOT_SIZE].copy_from_slice(&slot);
        let mut j = journal(flash);
        assert_eq!(j.load(), Ok(Some(record(1))));
        assert_eq!(j.store(&record(2)), Err(SettingsError::GenerationExhausted));
        assert_eq!(j.load(), Ok(Some(record(1))));
    }

    #[test]
    fn invalid_geometry_is_refused() {
        let mut j: SettingsJournal<MockFlash, SECTOR> =
            SettingsJournal::new(MockFlash::new(), 0, (SECTOR / 2) as u32);
        assert_eq!(j.load(), Err(SettingsError::Hardware));
        assert_eq!(j.store(&record(1)), Err(SettingsError::Hardware));
    }
}
