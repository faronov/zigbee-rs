//! Product-owned top-of-flash security-state journal.

use zigbee_runtime::security_journal::SECURITY_JOURNAL_SECTOR_SIZE;

pub const FLASH_CAPACITY: usize = 1024 * 1024;
/// Last 8 KiB of the nRF52840 main flash (pages 254-255).
pub const SECURITY_PARTITION_START: u32 = 0x000F_E000;
pub const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
pub const SECURITY_SECTOR_A: u32 = 0;
pub const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

const _: () =
    assert!(SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE == FLASH_CAPACITY);

#[cfg(target_os = "none")]
mod target {
    use super::*;
    use embassy_nrf::nvmc::{Error, Nvmc, PAGE_SIZE};
    use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
    use zigbee_runtime::security_journal::SecurityStateJournal;

    const _: () = assert!(PAGE_SIZE == SECURITY_JOURNAL_SECTOR_SIZE);

    pub struct SecurityFlash<'d> {
        nvmc: Nvmc<'d>,
    }

    impl<'d> SecurityFlash<'d> {
        pub fn new(nvmc: Nvmc<'d>) -> Self {
            Self { nvmc }
        }

        fn physical_offset(offset: u32, length: usize) -> Result<u32, Error> {
            (offset as usize)
                .checked_add(length)
                .filter(|end| *end <= SECURITY_PARTITION_SIZE)
                .ok_or(Error::OutOfBounds)?;
            SECURITY_PARTITION_START
                .checked_add(offset)
                .ok_or(Error::OutOfBounds)
        }
    }

    impl ErrorType for SecurityFlash<'_> {
        type Error = Error;
    }

    impl ReadNorFlash for SecurityFlash<'_> {
        const READ_SIZE: usize = Nvmc::READ_SIZE;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            self.nvmc
                .read(Self::physical_offset(offset, bytes.len())?, bytes)
        }

        fn capacity(&self) -> usize {
            SECURITY_PARTITION_SIZE
        }
    }

    impl NorFlash for SecurityFlash<'_> {
        const WRITE_SIZE: usize = Nvmc::WRITE_SIZE;
        const ERASE_SIZE: usize = Nvmc::ERASE_SIZE;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            if from >= to {
                return Err(Error::OutOfBounds);
            }
            let length = usize::try_from(to - from).map_err(|_| Error::OutOfBounds)?;
            let physical_from = Self::physical_offset(from, length)?;
            let physical_to = physical_from
                .checked_add(to - from)
                .ok_or(Error::OutOfBounds)?;
            self.nvmc.erase(physical_from, physical_to)
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            self.nvmc
                .write(Self::physical_offset(offset, bytes.len())?, bytes)
        }
    }

    pub type SecurityStore<'d> = SecurityStateJournal<SecurityFlash<'d>>;

    pub fn security_store(nvmc: Nvmc<'_>) -> SecurityStore<'_> {
        SecurityStateJournal::new(
            SecurityFlash::new(nvmc),
            SECURITY_SECTOR_A,
            SECURITY_SECTOR_B,
        )
    }
}

#[cfg(target_os = "none")]
pub use target::{SecurityFlash, SecurityStore, security_store};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn security_journal_owns_exactly_the_top_two_pages() {
        assert_eq!(SECURITY_JOURNAL_SECTOR_SIZE, 4096);
        assert_eq!(SECURITY_PARTITION_SIZE, 8192);
        assert_eq!(SECURITY_PARTITION_START, 0x000F_E000);
        assert_eq!(
            SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE,
            FLASH_CAPACITY
        );
        assert_eq!(SECURITY_SECTOR_A, 0);
        assert_eq!(SECURITY_SECTOR_B, 4096);
    }
}
