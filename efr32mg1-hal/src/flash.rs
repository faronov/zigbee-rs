//! EFR32MG1 Series 1 MSC flash controller.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

const FLASH_CAPACITY: usize = 256 * 1024;
const PAGE_SIZE: usize = 2048;
const PROGRAM_TIMEOUT: u32 = 100_000;

const MSC_BASE: u32 = 0x400E_0000;
const MSC_WRITECTRL: u32 = MSC_BASE + 0x008;
const MSC_WRITECMD: u32 = MSC_BASE + 0x00C;
const MSC_ADDRB: u32 = MSC_BASE + 0x010;
const MSC_WDATA: u32 = MSC_BASE + 0x018;
const MSC_STATUS: u32 = MSC_BASE + 0x01C;
const MSC_LOCK: u32 = MSC_BASE + 0x040;

const MSC_WRITECTRL_WREN: u32 = 1 << 0;
const MSC_WRITECMD_LADDRIM: u32 = 1 << 0;
const MSC_WRITECMD_ERASEPAGE: u32 = 1 << 1;
const MSC_WRITECMD_WRITEONCE: u32 = 1 << 3;
const MSC_STATUS_BUSY: u32 = 1 << 0;
const MSC_STATUS_LOCKED: u32 = 1 << 1;
const MSC_STATUS_INVADDR: u32 = 1 << 2;
const MSC_STATUS_WDATAREADY: u32 = 1 << 3;
const MSC_STATUS_WORDTIMEOUT: u32 = 1 << 4;
const MSC_LOCK_LOCKKEY_UNLOCK: u32 = 0x1B71;
const MSC_LOCK_LOCKKEY_LOCK: u32 = 0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    OutOfBounds,
    NotAligned,
    Timeout,
    Controller,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::Timeout | Self::Controller => NorFlashErrorKind::Other,
        }
    }
}

pub struct Efr32mg1Flash;

impl Efr32mg1Flash {
    pub const fn new() -> Self {
        Self
    }

    fn validate_range(offset: u32, length: usize) -> Result<(), FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= FLASH_CAPACITY)
            .map(|_| ())
            .ok_or(FlashError::OutOfBounds)
    }
}

impl Default for Efr32mg1Flash {
    fn default() -> Self {
        Self::new()
    }
}

impl ErrorType for Efr32mg1Flash {
    type Error = FlashError;
}

impl ReadNorFlash for Efr32mg1Flash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        Self::validate_range(offset, bytes.len())?;
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = unsafe { core::ptr::read_volatile((offset + index as u32) as *const u8) };
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        FLASH_CAPACITY
    }
}

impl NorFlash for Efr32mg1Flash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = PAGE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::OutOfBounds);
        }
        if from as usize % PAGE_SIZE != 0 || to as usize % PAGE_SIZE != 0 {
            return Err(FlashError::NotAligned);
        }
        Self::validate_range(from, (to - from) as usize)?;

        let mut page = from;
        while page < to {
            erase_page_ram(page)?;
            page += PAGE_SIZE as u32;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if offset as usize % Self::WRITE_SIZE != 0 || bytes.len() % Self::WRITE_SIZE != 0 {
            return Err(FlashError::NotAligned);
        }
        Self::validate_range(offset, bytes.len())?;

        for (index, chunk) in bytes.chunks_exact(4).enumerate() {
            let word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            program_word_ram(offset + (index * 4) as u32, word)?;
        }
        Ok(())
    }
}

#[inline(never)]
#[cfg_attr(target_arch = "arm", unsafe(link_section = ".data.ram_code"))]
fn erase_page_ram(offset: u32) -> Result<(), FlashError> {
    unsafe {
        let was_locked = core::ptr::read_volatile(MSC_LOCK as *const u32) != 0;
        core::ptr::write_volatile(MSC_LOCK as *mut u32, MSC_LOCK_LOCKKEY_UNLOCK);
        let writectrl = core::ptr::read_volatile(MSC_WRITECTRL as *const u32);
        core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, writectrl | MSC_WRITECTRL_WREN);
        core::ptr::write_volatile(MSC_ADDRB as *mut u32, offset);
        core::ptr::write_volatile(MSC_WRITECMD as *mut u32, MSC_WRITECMD_LADDRIM);

        let mut result =
            if core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_INVADDR != 0 {
                Err(FlashError::Controller)
            } else {
                core::ptr::write_volatile(MSC_WRITECMD as *mut u32, MSC_WRITECMD_ERASEPAGE);
                wait_series1_ready_ram()
            };

        let writectrl = core::ptr::read_volatile(MSC_WRITECTRL as *const u32);
        core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, writectrl & !MSC_WRITECTRL_WREN);
        if was_locked {
            core::ptr::write_volatile(MSC_LOCK as *mut u32, MSC_LOCK_LOCKKEY_LOCK);
        }
        if result.is_ok()
            && core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_LOCKED != 0
        {
            result = Err(FlashError::Controller);
        }
        result
    }
}

#[inline(never)]
#[cfg_attr(target_arch = "arm", unsafe(link_section = ".data.ram_code"))]
fn program_word_ram(offset: u32, word: u32) -> Result<(), FlashError> {
    unsafe {
        let was_locked = core::ptr::read_volatile(MSC_LOCK as *const u32) != 0;
        core::ptr::write_volatile(MSC_LOCK as *mut u32, MSC_LOCK_LOCKKEY_UNLOCK);
        let writectrl = core::ptr::read_volatile(MSC_WRITECTRL as *const u32);
        core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, writectrl | MSC_WRITECTRL_WREN);

        let mut result = wait_series1_ready_ram();
        if result.is_ok() {
            core::ptr::write_volatile(MSC_ADDRB as *mut u32, offset);
            core::ptr::write_volatile(MSC_WRITECMD as *mut u32, MSC_WRITECMD_LADDRIM);
            if core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_INVADDR != 0 {
                result = Err(FlashError::Controller);
            }
        }
        if result.is_ok() {
            let mut timeout = PROGRAM_TIMEOUT;
            while core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_WDATAREADY == 0
                && timeout != 0
            {
                timeout -= 1;
            }
            if timeout == 0 {
                result = Err(FlashError::Timeout);
            }
        }
        if result.is_ok() {
            core::ptr::write_volatile(MSC_WDATA as *mut u32, word);
            core::ptr::write_volatile(MSC_WRITECMD as *mut u32, MSC_WRITECMD_WRITEONCE);
            result = wait_series1_ready_ram();
        }

        let writectrl = core::ptr::read_volatile(MSC_WRITECTRL as *const u32);
        core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, writectrl & !MSC_WRITECTRL_WREN);
        if was_locked {
            core::ptr::write_volatile(MSC_LOCK as *mut u32, MSC_LOCK_LOCKKEY_LOCK);
        }
        result
    }
}

#[inline(never)]
#[cfg_attr(target_arch = "arm", unsafe(link_section = ".data.ram_code"))]
fn wait_series1_ready_ram() -> Result<(), FlashError> {
    for _ in 0..PROGRAM_TIMEOUT {
        let status = unsafe { core::ptr::read_volatile(MSC_STATUS as *const u32) };
        if status & MSC_STATUS_BUSY == 0 {
            return if status & (MSC_STATUS_LOCKED | MSC_STATUS_INVADDR | MSC_STATUS_WORDTIMEOUT)
                != 0
            {
                Err(FlashError::Controller)
            } else {
                Ok(())
            };
        }
    }
    Err(FlashError::Timeout)
}
