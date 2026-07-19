//! TRADFRI internal-flash partition wiring for application NV.

use efr32mg1_hal::flash::{Efr32mg1Flash, FlashError};
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use zigbee_runtime::log_nv::LogStructuredNv;
use zigbee_runtime::nv_storage::NvError;

const NV_PARTITION_START: u32 = 0x0003_E000;
const NV_PARTITION_SIZE: usize = 4096;
const NV_PAGE_A: u32 = 0;
const NV_PAGE_B: u32 = 2048;

pub struct ApplicationFlash {
    flash: Efr32mg1Flash,
}

impl ApplicationFlash {
    const fn new() -> Self {
        Self {
            flash: Efr32mg1Flash::new(),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= NV_PARTITION_SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        NV_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashError::OutOfBounds)
    }
}

impl ErrorType for ApplicationFlash {
    type Error = FlashError;
}

impl ReadNorFlash for ApplicationFlash {
    const READ_SIZE: usize = Efr32mg1Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        NV_PARTITION_SIZE
    }
}

impl NorFlash for ApplicationFlash {
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

pub type ApplicationNv = LogStructuredNv<ApplicationFlash>;

pub fn application_nv() -> Result<ApplicationNv, NvError> {
    LogStructuredNv::new(ApplicationFlash::new(), NV_PAGE_A, NV_PAGE_B)
}
