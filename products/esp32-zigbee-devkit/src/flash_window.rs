//! Bounded flash windows for product-owned journals.

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_storage::FlashStorageError;
use esp32_zigbee_devkit::flash::RawFlash;

/// A const-bounded window of the physical chip: relative offsets
/// `0..SIZE` map to `START..START + SIZE`; anything else is
/// `OutOfBounds`. Same contract as [`crate::storage::SecurityFlash`],
/// which stays a separate type so the sensor image is unchanged.
pub struct FlashWindow<const START: u32, const SIZE: usize> {
    flash: RawFlash,
}

impl<const START: u32, const SIZE: usize> FlashWindow<START, SIZE> {
    pub(crate) fn new() -> Self {
        Self {
            flash: RawFlash::new(),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashStorageError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SIZE)
            .ok_or(FlashStorageError::OutOfBounds)?;
        START
            .checked_add(offset)
            .ok_or(FlashStorageError::OutOfBounds)
    }
}

impl<const START: u32, const SIZE: usize> ErrorType for FlashWindow<START, SIZE> {
    type Error = FlashStorageError;
}

impl<const START: u32, const SIZE: usize> ReadNorFlash for FlashWindow<START, SIZE> {
    const READ_SIZE: usize = esp_storage::FlashStorage::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<const START: u32, const SIZE: usize> NorFlash for FlashWindow<START, SIZE> {
    const WRITE_SIZE: usize = esp_storage::FlashStorage::WRITE_SIZE;
    const ERASE_SIZE: usize = esp_storage::FlashStorage::ERASE_SIZE;

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
