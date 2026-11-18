//! The NCP's physical flash access: `zbnv` journal windows and the
//! fitted-flash JEDEC ID (spec 8.9, 11.5).
//!
//! The ESP32-H2 module may carry 2 or 4 MB, so this product does not use the
//! devkit board crate's fixed-size whole-chip `RawFlash`; every access goes
//! through a window of the active layout, and the ROM driver's own bounds
//! (from the bootloader image header) apply below that.

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_storage::{FlashStorage, FlashStorageError};

/// A const-bounded window of the physical chip: relative offsets `0..SIZE`
/// map to `START..START + SIZE`; anything else is `OutOfBounds`. Placement
/// (which window holds what) is the product's decision.
pub struct FlashWindow<const START: u32, const SIZE: usize> {
    flash: FlashStorage,
}

impl<const START: u32, const SIZE: usize> FlashWindow<START, SIZE> {
    pub fn new() -> Self {
        Self {
            flash: FlashStorage::new(),
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

impl<const START: u32, const SIZE: usize> Default for FlashWindow<START, SIZE> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const START: u32, const SIZE: usize> ErrorType for FlashWindow<START, SIZE> {
    type Error = FlashStorageError;
}

impl<const START: u32, const SIZE: usize> ReadNorFlash for FlashWindow<START, SIZE> {
    const READ_SIZE: usize = FlashStorage::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<const START: u32, const SIZE: usize> NorFlash for FlashWindow<START, SIZE> {
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

/// `esp_rom_spiflash_chip_t` of the ROM SPI flash driver.
#[repr(C)]
struct RomFlashChip {
    device_id: u32,
    chip_size: u32,
    block_size: u32,
    sector_size: u32,
    page_size: u32,
    status_mask: u32,
}

unsafe extern "C" {
    /// Issues RDID (0x9F) and stores the JEDEC ID in the ROM chip record.
    fn esp_rom_spi_flash_update_id();
    /// The ROM driver's `spiflash_legacy_data_t *`; its first member is the
    /// `esp_rom_spiflash_chip_t` record (`g_rom_flashchip`).
    static rom_spiflash_legacy_data: *const RomFlashChip;
}

/// Read the flash chip's JEDEC ID (`manufacturer << 16 | type << 8 |
/// capacity`) with the ROM driver, interrupts masked as for erase/write.
///
/// The fitted size is `1 << (id & 0xFF)` bytes for the usual capacity
/// codes; decoding and validating it is the caller's policy.
pub fn read_jedec_id() -> u32 {
    critical_section::with(|_| {
        // SAFETY: ROM routine and ROM data pointer of this chip family
        // (`esp32h2.rom.ld` / `esp32c6.rom.ld`); the ROM driver is
        // initialized by the second-stage bootloader before the app runs.
        unsafe {
            esp_rom_spi_flash_update_id();
            (*rom_spiflash_legacy_data).device_id
        }
    })
}
