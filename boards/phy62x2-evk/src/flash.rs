//! Whole-device internal flash resource.
//!
//! Product crates select protected partitions. This board wrapper only owns
//! the physical flash controller and preserves monotonic time while XIP is
//! unavailable during program/erase operations.

use crate::time;
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use phy6222_hal::flash::{FlashError, Phy62x2Flash};
use phy6222_hal::peripherals::FlashToken;

#[cfg(feature = "phy6222")]
pub const INTERNAL_FLASH_CAPACITY: usize = 512 * 1024;
#[cfg(feature = "phy6252")]
pub const INTERNAL_FLASH_CAPACITY: usize = 256 * 1024;

pub struct InternalFlash {
    flash: Phy62x2Flash,
}

impl InternalFlash {
    pub(crate) fn new(token: FlashToken) -> Result<Self, FlashError> {
        Ok(Self {
            flash: Phy62x2Flash::new(token, INTERNAL_FLASH_CAPACITY)?,
        })
    }
}

impl ErrorType for InternalFlash {
    type Error = FlashError;
}

impl ReadNorFlash for InternalFlash {
    const READ_SIZE: usize = Phy62x2Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.flash.capacity()
    }
}

impl NorFlash for InternalFlash {
    const WRITE_SIZE: usize = Phy62x2Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Phy62x2Flash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        time::run_flash_operation(|| self.flash.erase(from, to))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        time::run_flash_operation(|| self.flash.write(offset, bytes))
    }
}
