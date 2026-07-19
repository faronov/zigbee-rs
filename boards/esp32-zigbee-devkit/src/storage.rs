//! ESP32 flash partition wiring for generic application NV.

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_storage::{FlashStorage, FlashStorageError};
use zigbee_runtime::log_nv::LogStructuredNv;
use zigbee_runtime::nv_storage::NvError;

const NV_PARTITION_START: u32 = 0x003F_E000;
const NV_PARTITION_SIZE: usize = 8192;
const NV_PAGE_A: u32 = 0;
const NV_PAGE_B: u32 = 4096;

pub struct ApplicationFlash {
    flash: FlashStorage,
}

impl ApplicationFlash {
    fn new() -> Self {
        Self {
            flash: FlashStorage::new(),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashStorageError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= NV_PARTITION_SIZE)
            .ok_or(FlashStorageError::OutOfBounds)?;
        NV_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashStorageError::OutOfBounds)
    }
}

impl ErrorType for ApplicationFlash {
    type Error = FlashStorageError;
}

impl ReadNorFlash for ApplicationFlash {
    const READ_SIZE: usize = FlashStorage::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        NV_PARTITION_SIZE
    }
}

impl NorFlash for ApplicationFlash {
    const WRITE_SIZE: usize = FlashStorage::WRITE_SIZE;
    const ERASE_SIZE: usize = FlashStorage::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashStorageError::OutOfBounds);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashStorageError::OutOfBounds)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashStorageError::OutOfBounds)?;
        self.flash.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.write(physical, bytes)
    }
}

pub type ApplicationNv = LogStructuredNv<ApplicationFlash>;

pub fn application_nv() -> Result<ApplicationNv, NvError> {
    LogStructuredNv::new(ApplicationFlash::new(), NV_PAGE_A, NV_PAGE_B)
}
