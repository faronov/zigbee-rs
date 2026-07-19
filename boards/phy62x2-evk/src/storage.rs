//! PHY62x2 flash partitions and Zigbee security storage wiring.

use crate::time;
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use phy6222_hal::flash::{FlashError, Phy62x2Flash};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

#[cfg(feature = "phy6222")]
const FLASH_CAPACITY: usize = 512 * 1024;
#[cfg(feature = "phy6252")]
const FLASH_CAPACITY: usize = 256 * 1024;

#[cfg(feature = "phy6222")]
const SECURITY_PARTITION_START: u32 = 0x0007_E000;
#[cfg(feature = "phy6252")]
const SECURITY_PARTITION_START: u32 = 0x0003_E000;

const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

const _: () =
    assert!(SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE == FLASH_CAPACITY);

pub struct SecurityFlash {
    flash: Phy62x2Flash,
}

impl SecurityFlash {
    const fn new() -> Self {
        Self {
            flash: Phy62x2Flash::new(FLASH_CAPACITY),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SECURITY_PARTITION_SIZE)
            .ok_or(FlashError::OutOfRange)?;
        SECURITY_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashError::OutOfRange)
    }
}

impl ErrorType for SecurityFlash {
    type Error = FlashError;
}

impl ReadNorFlash for SecurityFlash {
    const READ_SIZE: usize = Phy62x2Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SECURITY_PARTITION_SIZE
    }
}

impl NorFlash for SecurityFlash {
    const WRITE_SIZE: usize = Phy62x2Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Phy62x2Flash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::OutOfRange);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashError::OutOfRange)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashError::OutOfRange)?;
        time::run_flash_operation(|| self.flash.erase(physical_from, physical_to))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        time::run_flash_operation(|| self.flash.write(physical, bytes))
    }
}

pub type SecurityStore = SecurityStateJournal<SecurityFlash>;

pub const fn security_store() -> SecurityStore {
    SecurityStateJournal::new(SecurityFlash::new(), SECURITY_SECTOR_A, SECURITY_SECTOR_B)
}
