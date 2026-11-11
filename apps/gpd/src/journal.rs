//! Durable GPD state: SrcID, GPD key, security frame-counter reservation and
//! commissioned channel.
//!
//! [`GpdJournal`] is an atomic two-sector flash journal of fixed 64-byte
//! records (CRC, commit marker written last, read-back verification,
//! generation ordering). An interrupted write or erase always leaves the
//! previous record, so the persisted counter bound never moves backwards
//! and a counter value is never reused after a power loss.
//!
//! The record holds the GPD key in clear: like every GPD, the key's
//! confidentiality on the device relies on the debug interface being locked
//! in production.

use embedded_storage::nor_flash::NorFlash;
use zigbee_crypto::AesKey;

use crate::GpdKind;

/// Size of one journal record.
pub const GPD_JOURNAL_SLOT_SIZE: usize = 64;

const RECORD_MAGIC: [u8; 4] = *b"ZGPD";
const RECORD_VERSION: u8 = 1;
const RECORD_CRC_OFFSET: usize = 52;
const RECORD_COMMIT_OFFSET: usize = 56;
const RECORD_COMMIT: [u8; 4] = *b"CMIT";

const FLAG_COMMISSIONED: u8 = 0x01;

/// Persistent GPD state.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct GpdRecord {
    /// Device kind the record was provisioned for.
    pub kind: GpdKind,
    /// GPD SrcID.
    pub src_id: u32,
    /// GPD (individual) key.
    pub key: AesKey,
    /// Exclusive upper bound of security frame counters that may have been
    /// used (see `zigbee_gp::gpd::CounterReservation`).
    pub counter_limit: u32,
    /// Commissioned operational channel (11..=26), if commissioned.
    pub channel: Option<u8>,
}

impl core::fmt::Debug for GpdRecord {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Never print the key.
        f.debug_struct("GpdRecord")
            .field("kind", &self.kind)
            .field("src_id", &format_args!("0x{:08X}", self.src_id))
            .field("counter_limit", &self.counter_limit)
            .field("channel", &self.channel)
            .finish_non_exhaustive()
    }
}

impl GpdRecord {
    fn encode(&self, generation: u32) -> [u8; GPD_JOURNAL_SLOT_SIZE] {
        let mut r = [0xFFu8; GPD_JOURNAL_SLOT_SIZE];
        r[0..4].copy_from_slice(&RECORD_MAGIC);
        r[4] = RECORD_VERSION;
        r[5] = self.kind as u8;
        r[6] = if self.channel.is_some() {
            FLAG_COMMISSIONED
        } else {
            0
        };
        r[7] = self.channel.unwrap_or(0);
        r[8..12].copy_from_slice(&generation.to_le_bytes());
        r[12..16].copy_from_slice(&self.src_id.to_le_bytes());
        r[16..32].copy_from_slice(&self.key);
        r[32..36].copy_from_slice(&self.counter_limit.to_le_bytes());
        let crc = crc32(&r[..RECORD_CRC_OFFSET]);
        r[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        r
    }

    /// Generation and record of a committed, CRC-valid slot.
    fn decode(r: &[u8; GPD_JOURNAL_SLOT_SIZE]) -> Option<(u32, Self)> {
        if r[0..4] != RECORD_MAGIC
            || r[4] != RECORD_VERSION
            || r[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4] != RECORD_COMMIT
        {
            return None;
        }
        let crc = u32::from_le_bytes(
            r[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4]
                .try_into()
                .ok()?,
        );
        if crc32(&r[..RECORD_CRC_OFFSET]) != crc {
            return None;
        }
        let kind = GpdKind::from_u8(r[5])?;
        let channel = match r[6] & FLAG_COMMISSIONED {
            0 => None,
            _ if (11..=26).contains(&r[7]) => Some(r[7]),
            _ => return None,
        };
        let mut key = [0u8; 16];
        key.copy_from_slice(&r[16..32]);
        Some((
            u32::from_le_bytes(r[8..12].try_into().ok()?),
            Self {
                kind,
                src_id: u32::from_le_bytes(r[12..16].try_into().ok()?),
                key,
                counter_limit: u32::from_le_bytes(r[32..36].try_into().ok()?),
                channel,
            },
        ))
    }
}

/// Journal failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JournalError {
    /// Flash access failed, read-back mismatched, or the geometry is unusable.
    Hardware,
    /// The 32-bit generation is exhausted.
    GenerationExhausted,
}

/// Persistent store of the [`GpdRecord`].
pub trait GpdStore {
    /// Load the newest committed record.
    fn load(&mut self) -> Result<Option<GpdRecord>, JournalError>;
    /// Durably store `record`; on success a later [`load`](Self::load)
    /// returns it even after a power loss.
    fn store(&mut self, record: &GpdRecord) -> Result<(), JournalError>;
}

/// Atomic two-sector journal of [`GpdRecord`]s.
pub struct GpdJournal<S, const SECTOR_SIZE: usize> {
    storage: S,
    sectors: [u32; 2],
    /// `(generation, sector)` of the newest committed record, once scanned.
    current: Option<(u32, usize)>,
    scanned: bool,
}

impl<S: NorFlash, const SECTOR_SIZE: usize> GpdJournal<S, SECTOR_SIZE> {
    const SLOTS_PER_SECTOR: usize = SECTOR_SIZE / GPD_JOURNAL_SLOT_SIZE;

    /// Journal over two erase-aligned sectors at `first_sector` and
    /// `second_sector` (offsets inside `storage`).
    pub const fn new(storage: S, first_sector: u32, second_sector: u32) -> Self {
        assert!(SECTOR_SIZE >= GPD_JOURNAL_SLOT_SIZE);
        assert!(SECTOR_SIZE.is_multiple_of(GPD_JOURNAL_SLOT_SIZE));
        Self {
            storage,
            sectors: [first_sector, second_sector],
            current: None,
            scanned: false,
        }
    }

    /// Release the storage.
    pub fn into_storage(self) -> S {
        self.storage
    }

    fn geometry_ok(&self) -> bool {
        self.sectors[0].abs_diff(self.sectors[1]) >= SECTOR_SIZE as u32
            && S::READ_SIZE != 0
            && S::WRITE_SIZE != 0
            && S::ERASE_SIZE != 0
            && GPD_JOURNAL_SLOT_SIZE.is_multiple_of(S::READ_SIZE)
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
        self.sectors[sector] + (slot * GPD_JOURNAL_SLOT_SIZE) as u32
    }

    fn read_slot(
        &mut self,
        sector: usize,
        slot: usize,
    ) -> Result<[u8; GPD_JOURNAL_SLOT_SIZE], JournalError> {
        let mut r = [0u8; GPD_JOURNAL_SLOT_SIZE];
        let address = self.slot_address(sector, slot);
        self.storage
            .read(address, &mut r)
            .map_err(|_| JournalError::Hardware)?;
        Ok(r)
    }

    fn scan(&mut self) -> Result<Option<GpdRecord>, JournalError> {
        if !self.geometry_ok() {
            return Err(JournalError::Hardware);
        }
        let mut best: Option<(u32, usize, GpdRecord)> = None;
        for sector in 0..2 {
            for slot in 0..Self::SLOTS_PER_SECTOR {
                let raw = self.read_slot(sector, slot)?;
                let Some((generation, record)) = GpdRecord::decode(&raw) else {
                    continue;
                };
                if best.is_some_and(|(g, _, _)| generation <= g) {
                    continue;
                }
                best = Some((generation, sector, record));
            }
        }
        self.current = best.map(|(g, s, _)| (g, s));
        self.scanned = true;
        Ok(best.map(|(_, _, r)| r))
    }

    fn first_erased_slot(&mut self, sector: usize) -> Result<Option<usize>, JournalError> {
        for slot in 0..Self::SLOTS_PER_SECTOR {
            if self.read_slot(sector, slot)?.iter().all(|b| *b == 0xFF) {
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
        record: &GpdRecord,
    ) -> Result<(), JournalError> {
        let mut raw = record.encode(generation);
        let address = self.slot_address(sector, slot);
        self.storage
            .write(address, &raw[..RECORD_COMMIT_OFFSET])
            .map_err(|_| JournalError::Hardware)?;
        raw[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        self.storage
            .write(
                address + RECORD_COMMIT_OFFSET as u32,
                &raw[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4],
            )
            .map_err(|_| JournalError::Hardware)?;
        if self.read_slot(sector, slot)? != raw {
            return Err(JournalError::Hardware);
        }
        Ok(())
    }

    fn erase_sector(&mut self, sector: usize) -> Result<(), JournalError> {
        let start = self.sectors[sector];
        self.storage
            .erase(start, start + SECTOR_SIZE as u32)
            .map_err(|_| JournalError::Hardware)
    }

    fn commit(&mut self, record: &GpdRecord) -> Result<(), JournalError> {
        if !self.scanned {
            self.scan()?;
        }
        let (generation, sector, slot) = match self.current {
            Some((generation, sector)) => {
                let generation = generation
                    .checked_add(1)
                    .ok_or(JournalError::GenerationExhausted)?;
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
        self.write_record(sector, slot, generation, record)?;
        self.current = Some((generation, sector));
        Ok(())
    }
}

impl<S: NorFlash, const SECTOR_SIZE: usize> GpdStore for GpdJournal<S, SECTOR_SIZE> {
    fn load(&mut self) -> Result<Option<GpdRecord>, JournalError> {
        self.scan()
    }

    fn store(&mut self, record: &GpdRecord) -> Result<(), JournalError> {
        let result = self.commit(record);
        if result.is_err() {
            // Re-scan before the next commit: a torn slot must be skipped.
            self.scanned = false;
        }
        result
    }
}

/// IEEE 802.3 CRC-32 (reflected, polynomial 0xEDB88320).
fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xFFFF_FFFFu32;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = (crc & 1).wrapping_neg();
            crc = (crc >> 1) ^ (0xEDB8_8320 & mask);
        }
    }
    !crc
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash};

    pub(crate) const SECTOR: usize = 1024;

    #[derive(Debug)]
    pub(crate) struct FlashError;

    impl NorFlashError for FlashError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    /// NOR model: programming only clears bits; optional write budget.
    pub(crate) struct MockFlash {
        pub data: [u8; 2 * SECTOR],
        pub programs_left: Option<usize>,
        pub erases: usize,
    }

    impl MockFlash {
        pub(crate) fn new() -> Self {
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
                *cell &= *byte;
            }
            Ok(())
        }
    }

    pub(crate) type Journal = GpdJournal<MockFlash, SECTOR>;

    pub(crate) fn journal(flash: MockFlash) -> Journal {
        GpdJournal::new(flash, 0, SECTOR as u32)
    }

    fn record(limit: u32, channel: Option<u8>) -> GpdRecord {
        GpdRecord {
            kind: GpdKind::Sensor,
            src_id: 0x1234_5678,
            key: [0x5A; 16],
            counter_limit: limit,
            channel,
        }
    }

    #[test]
    fn crc32_matches_ieee_check_value() {
        assert_eq!(crc32(b"123456789"), 0xCBF4_3926);
    }

    #[test]
    fn erased_flash_loads_nothing() {
        assert_eq!(journal(MockFlash::new()).load(), Ok(None));
    }

    #[test]
    fn record_round_trips() {
        let mut j = journal(MockFlash::new());
        let r = record(16, Some(15));
        j.store(&r).unwrap();
        let mut reopened = journal(j.into_storage());
        assert_eq!(reopened.load(), Ok(Some(r)));
    }

    #[test]
    fn newest_generation_wins_across_rollover() {
        let mut j = journal(MockFlash::new());
        let slots = 2 * (SECTOR / GPD_JOURNAL_SLOT_SIZE) + 5;
        for i in 0..slots as u32 {
            j.store(&record(16 * (i + 1), None)).unwrap();
        }
        let flash = j.into_storage();
        assert!(flash.erases >= 3);
        let mut reopened = journal(flash);
        assert_eq!(
            reopened.load().unwrap().unwrap().counter_limit,
            16 * slots as u32
        );
    }

    #[test]
    fn interrupted_write_keeps_previous_record() {
        let mut j = journal(MockFlash::new());
        let first = record(16, Some(11));
        j.store(&first).unwrap();
        let mut flash = j.into_storage();
        // Body programmed, commit marker not.
        flash.programs_left = Some(1);
        let mut j = journal(flash);
        assert_eq!(j.store(&record(32, Some(11))), Err(JournalError::Hardware));
        let mut flash = j.into_storage();
        flash.programs_left = None;
        let mut j = journal(flash);
        assert_eq!(j.load(), Ok(Some(first)));
        // The torn slot is skipped by the next commit.
        let second = record(32, Some(11));
        j.store(&second).unwrap();
        assert_eq!(journal(j.into_storage()).load(), Ok(Some(second)));
    }

    #[test]
    fn interrupted_rollover_erase_keeps_previous_record() {
        let mut j = journal(MockFlash::new());
        let per_sector = SECTOR / GPD_JOURNAL_SLOT_SIZE;
        for i in 0..per_sector as u32 {
            j.store(&record(i + 1, None)).unwrap();
        }
        // Sector 0 is full; the next commit erases sector 1 then programs.
        // Power is lost after the erase, before any program.
        let mut flash = j.into_storage();
        flash.programs_left = Some(0);
        let mut j = journal(flash);
        assert_eq!(j.store(&record(999, None)), Err(JournalError::Hardware));
        let mut flash = j.into_storage();
        flash.programs_left = None;
        assert_eq!(
            journal(flash).load().unwrap().unwrap().counter_limit,
            per_sector as u32
        );
    }

    #[test]
    fn corrupted_crc_falls_back_to_older_record() {
        let mut j = journal(MockFlash::new());
        let first = record(16, None);
        j.store(&first).unwrap();
        j.store(&record(32, None)).unwrap();
        let mut flash = j.into_storage();
        flash.data[GPD_JOURNAL_SLOT_SIZE + 33] ^= 0x01;
        assert_eq!(journal(flash).load(), Ok(Some(first)));
    }

    #[test]
    fn invalid_channel_or_kind_is_rejected() {
        let mut raw = record(16, Some(15)).encode(0);
        raw[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        assert!(GpdRecord::decode(&raw).is_some());
        let mut bad = raw;
        bad[7] = 27;
        let crc = crc32(&bad[..RECORD_CRC_OFFSET]);
        bad[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        assert!(GpdRecord::decode(&bad).is_none());
        let mut bad = raw;
        bad[5] = 0x7E;
        let crc = crc32(&bad[..RECORD_CRC_OFFSET]);
        bad[RECORD_CRC_OFFSET..RECORD_CRC_OFFSET + 4].copy_from_slice(&crc.to_le_bytes());
        assert!(GpdRecord::decode(&bad).is_none());
    }

    #[test]
    fn exhausted_generation_is_reported() {
        let mut j = journal(MockFlash::new());
        let r = record(16, None);
        j.store(&r).unwrap();
        let mut flash = j.into_storage();
        let mut raw = r.encode(u32::MAX);
        raw[RECORD_COMMIT_OFFSET..RECORD_COMMIT_OFFSET + 4].copy_from_slice(&RECORD_COMMIT);
        flash.data[..GPD_JOURNAL_SLOT_SIZE].copy_from_slice(&raw);
        let mut j = journal(flash);
        assert_eq!(j.store(&r), Err(JournalError::GenerationExhausted));
        assert_eq!(j.load(), Ok(Some(r)));
    }

    #[test]
    fn unusable_geometry_is_a_hardware_error() {
        let mut j = GpdJournal::<_, SECTOR>::new(MockFlash::new(), 0, 0);
        assert_eq!(j.load(), Err(JournalError::Hardware));
        let mut j = GpdJournal::<_, SECTOR>::new(MockFlash::new(), 0, 2 * SECTOR as u32);
        assert_eq!(j.load(), Err(JournalError::Hardware));
    }
}
