//! TB-04 flash partitions and Zigbee security storage wiring.

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use tlsr8258_hal::flash::{FlashError, Tlsr8258Flash};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

const FLASH_CAPACITY: usize = 512 * 1024;
const SECURITY_PARTITION_START: u32 = 0x0007_4000;
const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

const _: () =
    assert!(SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE <= FLASH_CAPACITY);

pub struct SecurityFlash {
    flash: Tlsr8258Flash,
}

impl SecurityFlash {
    const fn new() -> Self {
        Self {
            flash: Tlsr8258Flash::new(FLASH_CAPACITY),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SECURITY_PARTITION_SIZE)
            .ok_or(FlashError::AddressOverflow)?;
        SECURITY_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashError::AddressOverflow)
    }
}

impl ErrorType for SecurityFlash {
    type Error = FlashError;
}

impl ReadNorFlash for SecurityFlash {
    const READ_SIZE: usize = Tlsr8258Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SECURITY_PARTITION_SIZE
    }
}

impl NorFlash for SecurityFlash {
    const WRITE_SIZE: usize = Tlsr8258Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Tlsr8258Flash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::AddressOverflow);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashError::AddressOverflow)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashError::AddressOverflow)?;
        self.flash.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.write(physical, bytes)
    }
}

pub type SecurityStore = SecurityStateJournal<SecurityFlash>;

pub const fn security_store() -> SecurityStore {
    SecurityStateJournal::new(SecurityFlash::new(), SECURITY_SECTOR_A, SECURITY_SECTOR_B)
}
