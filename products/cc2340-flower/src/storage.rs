//! Product-owned CC2340R52 flash layout.
//!
//! ```text
//! 0x7E000..0x80000  SecurityStateJournal: two logical 4-KiB sectors
//! 0x7D800..0x7E000  detected-sensor record (one 2-KiB erase sector)
//! 0x00000..0x7D800  application image
//! ```
//!
//! [`FlashWindow`] confines every access to its partition, so neither
//! store can reach the application image or the other partition.

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

pub const DETECTION_PARTITION_START: u32 = 0x0007_D800;
pub const DETECTION_PARTITION_SIZE: usize = PHYSICAL_ERASE_SIZE;

/// End of the application image (`FLASH` region in `link/memory.x`).
pub const APPLICATION_FLASH_END: u32 = DETECTION_PARTITION_START;

const _: () = assert!(SECURITY_JOURNAL_SECTOR_SIZE == 4096);
const _: () = assert!(SECURITY_JOURNAL_SECTOR_SIZE.is_multiple_of(PHYSICAL_ERASE_SIZE));
const _: () = assert!((SECURITY_PARTITION_START as usize).is_multiple_of(PHYSICAL_ERASE_SIZE));
const _: () = assert!((DETECTION_PARTITION_START as usize).is_multiple_of(PHYSICAL_ERASE_SIZE));
const _: () = assert!(SECURITY_PARTITION_END as usize == FLASH_CAPACITY);
const _: () = assert!(
    DETECTION_PARTITION_START as usize + DETECTION_PARTITION_SIZE
        == SECURITY_PARTITION_START as usize
);

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

/// Bounded view `[START, START + SIZE)` of the main flash.
pub struct FlashWindow<F, const START: u32, const SIZE: usize> {
    flash: F,
}

impl<F, const START: u32, const SIZE: usize> FlashWindow<F, START, SIZE> {
    pub const fn new(flash: F) -> Self {
        Self { flash }
    }

    pub const fn inner(&self) -> &F {
        &self.flash
    }

    pub fn into_inner(self) -> F {
        self.flash
    }

    /// Translate a window range to its physical start address.
    pub const fn physical_offset(offset: u32, length: usize) -> Option<u32> {
        let Some(end) = (offset as usize).checked_add(length) else {
            return None;
        };
        if end > SIZE {
            return None;
        }
        START.checked_add(offset)
    }
}

impl<F: ErrorType, const START: u32, const SIZE: usize> ErrorType for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError,
{
    type Error = PartitionError<F::Error>;
}

impl<F: ReadNorFlash, const START: u32, const SIZE: usize> ReadNorFlash
    for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(map_layout_error)?;
        let physical =
            Self::physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .read(physical, bytes)
            .map_err(PartitionError::Backend)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<F: NorFlash, const START: u32, const SIZE: usize> NorFlash for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(map_layout_error)?;
        let length = usize::try_from(to - from).map_err(|_| PartitionError::OutOfBounds)?;
        let physical_from =
            Self::physical_offset(from, length).ok_or(PartitionError::OutOfBounds)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .erase(physical_from, physical_to)
            .map_err(PartitionError::Backend)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(map_layout_error)?;
        let physical =
            Self::physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .write(physical, bytes)
            .map_err(PartitionError::Backend)
    }
}

const fn map_layout_error<E>(error: NorFlashErrorKind) -> PartitionError<E> {
    match error {
        NorFlashErrorKind::NotAligned => PartitionError::NotAligned,
        _ => PartitionError::OutOfBounds,
    }
}

pub type SecurityPartition<F> = FlashWindow<F, SECURITY_PARTITION_START, SECURITY_PARTITION_SIZE>;
pub type DetectionPartition<F> =
    FlashWindow<F, DETECTION_PARTITION_START, DETECTION_PARTITION_SIZE>;
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
pub type HardwareSecurityStore = SecurityStore<flower_cc2340r5::flash::InternalFlash>;

#[cfg(target_os = "none")]
pub fn security_store(flash: flower_cc2340r5::flash::InternalFlash) -> HardwareSecurityStore {
    security_journal(flash)
}

#[cfg(test)]
pub(crate) mod mock {
    extern crate std;

    use super::*;
    use std::vec;
    use std::vec::Vec;

    /// Whole 512-KiB main flash with NOR semantics (writes only clear bits).
    pub struct MockMainFlash {
        pub bytes: Vec<u8>,
        pub erased_physical_sectors: usize,
        pub writes: usize,
    }

    impl MockMainFlash {
        pub fn new() -> Self {
            Self {
                bytes: vec![0xFF; FLASH_CAPACITY],
                erased_physical_sectors: 0,
                writes: 0,
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
            self.writes += 1;
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::mock::MockMainFlash;
    use super::*;
    use zigbee_runtime::security_store::{PersistentSecurityState, SecurityStateStore};

    #[test]
    fn partitions_tile_the_top_of_flash_without_overlap() {
        assert_eq!(DETECTION_PARTITION_START, 0x7D800);
        assert_eq!(SECURITY_PARTITION_START, 0x7E000);
        assert_eq!(SECURITY_PARTITION_END as usize, FLASH_CAPACITY);
        assert_eq!(APPLICATION_FLASH_END, 0x7D800);
        assert_eq!(
            DetectionPartition::<MockMainFlash>::physical_offset(
                DETECTION_PARTITION_SIZE as u32,
                0
            ),
            Some(SECURITY_PARTITION_START)
        );
        assert_eq!(
            DetectionPartition::<MockMainFlash>::physical_offset(
                DETECTION_PARTITION_SIZE as u32,
                1
            ),
            None
        );
    }

    #[test]
    fn detection_window_cannot_touch_the_security_journal() {
        let mut window = DetectionPartition::new(MockMainFlash::new());
        assert_eq!(
            window.write(DETECTION_PARTITION_SIZE as u32, &[0; 4]),
            Err(PartitionError::OutOfBounds)
        );
        assert_eq!(
            window.erase(0, 2 * PHYSICAL_ERASE_SIZE as u32),
            Err(PartitionError::OutOfBounds)
        );
        window.erase(0, PHYSICAL_ERASE_SIZE as u32).unwrap();
        window.write(0, &[0xA5, 0x5A, 0x00, 0x11]).unwrap();
        let flash = window.into_inner();
        let start = DETECTION_PARTITION_START as usize;
        assert_eq!(&flash.bytes[start..start + 4], &[0xA5, 0x5A, 0x00, 0x11]);
        assert!(flash.bytes[..start].iter().all(|byte| *byte == 0xFF));
        assert!(flash.bytes[start + 4..].iter().all(|byte| *byte == 0xFF));
    }

    #[test]
    fn security_journal_round_trips_through_a_borrowed_flash() {
        let mut flash = MockMainFlash::new();
        let expected = PersistentSecurityState {
            global_counter_limit: 0x4000,
            ..PersistentSecurityState::default()
        };
        security_journal(&mut flash).store(&expected).unwrap();
        // A fresh journal over the same flash (as at the next boot) sees it.
        assert_eq!(security_journal(&mut flash).load(), Ok(Some(expected)));
        let start = SECURITY_PARTITION_START as usize;
        assert!(flash.bytes[..start].iter().all(|byte| *byte == 0xFF));
    }

    #[test]
    fn linker_script_matches_rust_partition_constants() {
        let script = include_str!("../link/memory.x");
        assert!(script.contains("LENGTH = 502K"));
        assert!(script.contains("0x0007D800"));
        assert!(script.contains("0x0007E000"));
        assert!(script.contains("0x00080000"));
        assert!(script.contains("36K"));
        assert_eq!(502 * 1024, APPLICATION_FLASH_END);
    }
}
