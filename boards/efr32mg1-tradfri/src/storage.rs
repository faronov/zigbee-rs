//! TRADFRI internal-flash partition wiring for application NV.

use efr32mg1_hal::flash::{Efr32mg1Flash, FlashError};
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use zigbee_runtime::log_nv::LogStructuredNv;
use zigbee_runtime::nv_storage::NvError;
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

pub const SECURITY_PARTITION_START: u32 = 0x0003_7000;
pub const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
pub const APP_NV_PARTITION_START: u32 = 0x0003_9000;
pub const APP_NV_PARTITION_SIZE: usize = 4096;
const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;
const NV_PAGE_A: u32 = 0;
const NV_PAGE_B: u32 = 2048;

const _: () = assert!(
    SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE
        == APP_NV_PARTITION_START as usize
);
const _: () =
    assert!(APP_NV_PARTITION_START as usize + APP_NV_PARTITION_SIZE == 0x0003_A000);
const _: () = assert!(
    SECURITY_JOURNAL_SECTOR_SIZE % <Efr32mg1Flash as NorFlash>::ERASE_SIZE == 0
);

pub struct PartitionFlash<const START: u32, const SIZE: usize> {
    flash: Efr32mg1Flash,
}

impl<const START: u32, const SIZE: usize> PartitionFlash<START, SIZE> {
    const fn new() -> Self {
        Self {
            flash: Efr32mg1Flash::new(),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        START.checked_add(offset).ok_or(FlashError::OutOfBounds)
    }
}

impl<const START: u32, const SIZE: usize> ErrorType for PartitionFlash<START, SIZE> {
    type Error = FlashError;
}

impl<const START: u32, const SIZE: usize> ReadNorFlash for PartitionFlash<START, SIZE> {
    const READ_SIZE: usize = Efr32mg1Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<const START: u32, const SIZE: usize> NorFlash for PartitionFlash<START, SIZE> {
    const WRITE_SIZE: usize = Efr32mg1Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Efr32mg1Flash::ERASE_SIZE;

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

pub type SecurityFlash =
    PartitionFlash<SECURITY_PARTITION_START, SECURITY_PARTITION_SIZE>;
pub type SecurityStore = SecurityStateJournal<SecurityFlash>;
pub type ApplicationFlash =
    PartitionFlash<APP_NV_PARTITION_START, APP_NV_PARTITION_SIZE>;
pub type ApplicationNv = LogStructuredNv<ApplicationFlash>;

pub fn security_store() -> SecurityStore {
    SecurityStateJournal::new(
        SecurityFlash::new(),
        SECURITY_SECTOR_A,
        SECURITY_SECTOR_B,
    )
}

pub fn application_nv() -> Result<ApplicationNv, NvError> {
    LogStructuredNv::new(ApplicationFlash::new(), NV_PAGE_A, NV_PAGE_B)
}
