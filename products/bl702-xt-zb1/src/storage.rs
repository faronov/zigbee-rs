//! Product-owned BL702 flash partition and Zigbee security journal.

use bl702_hal::flash::{FlashError, XipFlash};
use bl702_hal::peripherals::Flash;
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

pub const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
pub const SECURITY_PARTITION_START: u32 =
    (bl702_xt_zb1::ONBOARD_FLASH_CAPACITY - SECURITY_PARTITION_SIZE) as u32;
const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

const _: () = assert!(
    SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE
        == bl702_xt_zb1::ONBOARD_FLASH_CAPACITY
);
const _: () =
    assert!(SECURITY_JOURNAL_SECTOR_SIZE.is_multiple_of(<XipFlash as NorFlash>::ERASE_SIZE));

pub struct SecurityFlash {
    flash: XipFlash,
}

impl SecurityFlash {
    pub fn new(token: Flash) -> Result<Self, FlashError> {
        Ok(Self {
            flash: XipFlash::new(token, bl702_xt_zb1::ONBOARD_FLASH_CAPACITY)?,
        })
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SECURITY_PARTITION_SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        SECURITY_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashError::OutOfBounds)
    }
}

impl ErrorType for SecurityFlash {
    type Error = FlashError;
}

impl ReadNorFlash for SecurityFlash {
    const READ_SIZE: usize = XipFlash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SECURITY_PARTITION_SIZE
    }
}

impl NorFlash for SecurityFlash {
    const WRITE_SIZE: usize = XipFlash::WRITE_SIZE;
    const ERASE_SIZE: usize = XipFlash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::OutOfBounds);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashError::OutOfBounds)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashError::OutOfBounds)?;
        self.flash.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.write(physical, bytes)
    }
}

pub type SecurityStore = SecurityStateJournal<SecurityFlash>;

pub fn security_store(token: Flash) -> Result<SecurityStore, FlashError> {
    Ok(SecurityStateJournal::new(
        SecurityFlash::new(token)?,
        SECURITY_SECTOR_A,
        SECURITY_SECTOR_B,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn security_partition_occupies_the_last_two_flash_sectors() {
        assert_eq!(SECURITY_PARTITION_START, 0x000f_e000);
        assert_eq!(SECURITY_PARTITION_SIZE, 8192);
        assert_eq!(
            SecurityFlash::physical_offset(0, SECURITY_PARTITION_SIZE),
            Ok(SECURITY_PARTITION_START)
        );
        assert_eq!(
            SecurityFlash::physical_offset(SECURITY_PARTITION_SIZE as u32, 1),
            Err(FlashError::OutOfBounds)
        );
    }
}
