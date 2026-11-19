//! Hardware [`OtaFlash`] of the NCP: whole-chip access through the ROM
//! flash driver, restricted to `otadata` and the two application slots of
//! [`crate::layout::ACTIVE`], and the running slot from live MMU evidence.

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use esp_storage::FlashStorage;
use esp32_zigbee_devkit_product::esp_image::{EXPECTED_CHIP_ID, ImageCompatibility};
use esp32_zigbee_devkit_product::ota::{OtaFlash, RunningImageError};
use zigbee_runtime::firmware_writer::FirmwareError;

use crate::layout::{SECTOR_SIZE, is_fw_writable};
use crate::running_image;

/// See the module documentation.
pub struct NcpOtaFlash {
    flash: FlashStorage,
}

impl NcpOtaFlash {
    pub fn new() -> Self {
        Self {
            flash: FlashStorage::new(),
        }
    }
}

impl Default for NcpOtaFlash {
    fn default() -> Self {
        Self::new()
    }
}

impl OtaFlash for NcpOtaFlash {
    fn running_slot(&mut self) -> Result<u8, RunningImageError> {
        use running_image::RunningImageError as Mmu;
        running_image::detect().map_err(|error| match error {
            Mmu::Unavailable => RunningImageError::Unavailable,
            Mmu::NotFlashMapped => RunningImageError::NotFlashMapped,
            Mmu::InvalidMapping => RunningImageError::InvalidMapping,
            Mmu::EncryptedFlash => RunningImageError::EncryptedFlash,
            Mmu::OutsideOtaSlots => RunningImageError::OutsideOtaSlots,
            Mmu::Changed => RunningImageError::Changed,
        })
    }

    fn image_compatibility(&mut self) -> Result<ImageCompatibility, FirmwareError> {
        use esp_hal::efuse::Efuse;
        let (major, minor) = Efuse::block_version();
        Ok(ImageCompatibility {
            chip_id: EXPECTED_CHIP_ID,
            chip_revision: Efuse::chip_revision(),
            efuse_block_revision: major as u16 * 100 + minor as u16,
        })
    }

    fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FirmwareError> {
        self.flash
            .read(address, buffer)
            .map_err(|_| FirmwareError::HardwareError)
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FirmwareError> {
        let length = u32::try_from(data.len()).map_err(|_| FirmwareError::OutOfRange)?;
        if !is_fw_writable(address, length) {
            return Err(FirmwareError::OutOfRange);
        }
        self.flash
            .write(address, data)
            .map_err(|_| FirmwareError::WriteFailed)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), FirmwareError> {
        if !address.is_multiple_of(SECTOR_SIZE) || !is_fw_writable(address, SECTOR_SIZE) {
            return Err(FirmwareError::OutOfRange);
        }
        self.flash
            .erase(address, address + SECTOR_SIZE)
            .map_err(|_| FirmwareError::EraseFailed)
    }
}
