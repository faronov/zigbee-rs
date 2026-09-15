//! Product-owned CC2340R52 security-state flash partition.
//!
//! The final 8 KiB of main flash contain two logical 4-KiB journal sectors.
//! Each logical sector spans two physical 2-KiB CC2340 erase sectors.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash, check_erase, check_read,
    check_write,
};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

pub const FLASH_CAPACITY: usize = 512 * 1024;
pub const PHYSICAL_ERASE_SIZE: usize = 2 * 1024;
pub const SECURITY_PARTITION_START: u32 = 0x0007_E000;
pub const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
pub const SECURITY_PARTITION_END: u32 = SECURITY_PARTITION_START + SECURITY_PARTITION_SIZE as u32;
pub const SECURITY_SECTOR_A: u32 = 0;
pub const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

const _: () = assert!(SECURITY_JOURNAL_SECTOR_SIZE == 4096);
const _: () = assert!(SECURITY_JOURNAL_SECTOR_SIZE.is_multiple_of(PHYSICAL_ERASE_SIZE));
const _: () = assert!((SECURITY_PARTITION_START as usize).is_multiple_of(PHYSICAL_ERASE_SIZE));
const _: () = assert!(SECURITY_PARTITION_END as usize == FLASH_CAPACITY);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionError<E> {
    OutOfBounds,
    NotAligned,
    Backend(E),
}

impl<E: NorFlashError> NorFlashError for PartitionError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::Backend(error) => error.kind(),
        }
    }
}

/// A bounded logical view over the product's protected final 8 KiB.
pub struct SecurityPartition<F> {
    flash: F,
}

impl<F> SecurityPartition<F> {
    pub const fn new(flash: F) -> Self {
        Self { flash }
    }

    pub const fn inner(&self) -> &F {
        &self.flash
    }

    pub fn inner_mut(&mut self) -> &mut F {
        &mut self.flash
    }

    pub fn into_inner(self) -> F {
        self.flash
    }
}

impl<F: ErrorType> ErrorType for SecurityPartition<F>
where
    F::Error: NorFlashError,
{
    type Error = PartitionError<F::Error>;
}

impl<F: ReadNorFlash> ReadNorFlash for SecurityPartition<F>
where
    F::Error: NorFlashError,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(map_layout_error)?;
        let physical = physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .read(physical, bytes)
            .map_err(PartitionError::Backend)
    }

    fn capacity(&self) -> usize {
        SECURITY_PARTITION_SIZE
    }
}

impl<F: NorFlash> NorFlash for SecurityPartition<F>
where
    F::Error: NorFlashError,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(map_layout_error)?;
        let length = usize::try_from(to - from).map_err(|_| PartitionError::OutOfBounds)?;
        let physical_from = physical_offset(from, length).ok_or(PartitionError::OutOfBounds)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .erase(physical_from, physical_to)
            .map_err(PartitionError::Backend)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(map_layout_error)?;
        let physical = physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .write(physical, bytes)
            .map_err(PartitionError::Backend)
    }
}

const fn map_layout_error<E>(error: NorFlashErrorKind) -> PartitionError<E> {
    match error {
        NorFlashErrorKind::OutOfBounds => PartitionError::OutOfBounds,
        NorFlashErrorKind::NotAligned => PartitionError::NotAligned,
        _ => PartitionError::OutOfBounds,
    }
}

/// Translate a logical partition range to its physical start address.
pub const fn physical_offset(offset: u32, length: usize) -> Option<u32> {
    let offset = offset as usize;
    let Some(end) = offset.checked_add(length) else {
        return None;
    };
    if end > SECURITY_PARTITION_SIZE {
        return None;
    }
    SECURITY_PARTITION_START.checked_add(offset as u32)
}

pub type SecurityStore<F> = SecurityStateJournal<SecurityPartition<F>>;

pub fn security_journal<F: NorFlash>(flash: F) -> SecurityStore<F>
where
    F::Error: NorFlashError,
{
    SecurityStateJournal::new(
        SecurityPartition::new(flash),
        SECURITY_SECTOR_A,
        SECURITY_SECTOR_B,
    )
}

#[cfg(target_os = "none")]
pub type HardwareSecurityStore = SecurityStore<lp_em_cc2340r5::flash::InternalFlash>;

#[cfg(target_os = "none")]
pub fn security_store(flash: lp_em_cc2340r5::flash::InternalFlash) -> HardwareSecurityStore {
    security_journal(flash)
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::vec;
    use std::vec::Vec;
    use zigbee_runtime::security_store::{PersistentSecurityState, SecurityStateStore};

    struct MockMainFlash {
        bytes: Vec<u8>,
        erased_physical_sectors: usize,
    }

    impl MockMainFlash {
        fn new() -> Self {
            Self {
                bytes: vec![0xFF; FLASH_CAPACITY],
                erased_physical_sectors: 0,
            }
        }
    }

    impl ErrorType for MockMainFlash {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for MockMainFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            check_read(self, offset, bytes.len())?;
            let start = offset as usize;
            bytes.copy_from_slice(&self.bytes[start..start + bytes.len()]);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockMainFlash {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = PHYSICAL_ERASE_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            check_erase(self, from, to)?;
            self.bytes[from as usize..to as usize].fill(0xFF);
            self.erased_physical_sectors += (to - from) as usize / PHYSICAL_ERASE_SIZE;
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            check_write(self, offset, bytes.len())?;
            let start = offset as usize;
            for (stored, new) in self.bytes[start..start + bytes.len()].iter_mut().zip(bytes) {
                if *stored & *new != *new {
                    return Err(NorFlashErrorKind::Other);
                }
                *stored &= *new;
            }
            Ok(())
        }
    }

    #[test]
    fn partition_occupies_exactly_the_last_eight_kibibytes() {
        assert_eq!(SECURITY_PARTITION_START, 0x7E000);
        assert_eq!(SECURITY_PARTITION_SIZE, 0x2000);
        assert_eq!(SECURITY_PARTITION_END as usize, FLASH_CAPACITY);
        assert_eq!(
            physical_offset(SECURITY_PARTITION_SIZE as u32, 0),
            Some(SECURITY_PARTITION_END)
        );
        assert_eq!(physical_offset(SECURITY_PARTITION_SIZE as u32, 1), None);
    }

    #[test]
    fn each_logical_journal_sector_spans_two_physical_sectors() {
        assert_eq!(SECURITY_JOURNAL_SECTOR_SIZE / PHYSICAL_ERASE_SIZE, 2);
        let mut partition = SecurityPartition::new(MockMainFlash::new());
        partition
            .erase(0, SECURITY_JOURNAL_SECTOR_SIZE as u32)
            .unwrap();
        assert_eq!(partition.inner().erased_physical_sectors, 2);
    }

    #[test]
    fn logical_writes_are_confined_to_the_reserved_partition() {
        let mut partition = SecurityPartition::new(MockMainFlash::new());
        partition.write(0, &[0xA5, 0x5A, 0x00, 0xFF]).unwrap();
        assert_eq!(
            &partition.inner().bytes
                [SECURITY_PARTITION_START as usize..SECURITY_PARTITION_START as usize + 4],
            &[0xA5, 0x5A, 0x00, 0xFF]
        );
        assert!(
            partition.inner().bytes[..SECURITY_PARTITION_START as usize]
                .iter()
                .all(|byte| *byte == 0xFF)
        );
    }

    #[test]
    fn security_journal_round_trips_and_rolls_to_the_second_sector() {
        let mut journal = security_journal(MockMainFlash::new());
        assert_eq!(journal.load(), Ok(None));

        let mut expected = PersistentSecurityState::default();
        for generation in 0..=32 {
            expected.global_counter_limit = generation * 0x400;
            journal.store(&expected).unwrap();
        }

        journal.storage_mut();
        assert_eq!(journal.load(), Ok(Some(expected)));
        let partition = journal.into_storage();
        assert_eq!(partition.inner().erased_physical_sectors, 4);
    }

    #[test]
    fn linker_script_matches_rust_partition_constants() {
        let script = include_str!("../link/memory.x");
        assert!(script.contains("LENGTH = 504K"));
        assert!(script.contains("0x0007E000"));
        assert!(script.contains("0x00080000"));
        assert!(script.contains("36K"));
    }
}
