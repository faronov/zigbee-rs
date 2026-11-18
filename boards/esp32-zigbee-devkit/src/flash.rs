//! Raw physical NOR flash access for the ESP32-C6/H2/C5 Zigbee devkits.
//!
//! Wraps `esp_storage::FlashStorage`, the ROM SPI flash driver for this
//! module's on-die 4 MiB flash. [`RawFlash`] is unbounded, partition-unaware
//! whole-chip access — the same role `efr32mg1-hal`'s `Efr32mg1Flash` plays
//! for the EFR32MG1 product. Partition layout, NV placement, and OTA slot
//! policy belong to the product crate that selects this board, not here.

#![cfg(target_os = "none")]

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use esp_storage::FlashStorage;

/// Total flash size fitted on the supported devkits.
pub const FLASH_SIZE: u32 = 0x0040_0000;
/// Flash sector (erase) size.
pub const SECTOR_SIZE: u32 = 4096;
/// Flash word (program) size.
pub const WORD_SIZE: u32 = 4;

/// Whole-chip raw flash access. No partition, NV, or OTA policy: callers pass
/// absolute chip addresses and get back exactly what the ROM SPI flash
/// routines return.
/// esp-storage 0.7 (C6/H2) or 0.9 (C5, borrows the FLASH token).
#[cfg(not(feature = "esp32c5"))]
type Storage = FlashStorage;
#[cfg(feature = "esp32c5")]
type Storage = FlashStorage<'static>;

pub struct RawFlash {
    flash: Storage,
}

impl RawFlash {
    #[cfg(not(feature = "esp32c5"))]
    pub fn new() -> Self {
        Self {
            flash: FlashStorage::new(),
        }
    }

    /// esp-storage 0.9 (the ESP32-C5 release train) takes the FLASH
    /// peripheral token; 0.7 took none. The product opens several
    /// `RawFlash` views (journal, OTA writer, otadata) exactly as it did with
    /// 0.7, so the token is conjured per view: every operation is a blocking
    /// ROM SPI-flash call inside esp-storage's critical section, and the
    /// views are never used concurrently (single-core, no flash access from
    /// interrupts).
    #[cfg(feature = "esp32c5")]
    pub fn new() -> Self {
        // SAFETY: see above; the token carries no state, and nothing else
        // in these images owns FLASH.
        let token = unsafe { esp_hal::peripherals::FLASH::steal() };
        Self {
            flash: FlashStorage::new(token),
        }
    }
}

impl Default for RawFlash {
    fn default() -> Self {
        Self::new()
    }
}

impl ErrorType for RawFlash {
    type Error = <Storage as ErrorType>::Error;
}

impl ReadNorFlash for RawFlash {
    const READ_SIZE: usize = Storage::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE as usize
    }
}

impl NorFlash for RawFlash {
    const WRITE_SIZE: usize = Storage::WRITE_SIZE;
    const ERASE_SIZE: usize = Storage::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.flash.erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.flash.write(offset, bytes)
    }
}
