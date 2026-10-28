//! CC2340R52 main-flash access through TI's ROM HAPI table.
//!
//! Program data is copied to an aligned SRAM buffer. Interrupts and SysTick
//! are masked while the ROM operation runs, and VIMS cache state is restored
//! afterwards.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash, check_erase, check_read,
    check_write,
};

pub const FLASH_BASE: u32 = 0x0000_0000;
pub const FLASH_CAPACITY: usize = 512 * 1024;
pub const PHYSICAL_ERASE_SIZE: usize = 2 * 1024;
pub const FLASH_WRITE_SIZE: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    OutOfBounds,
    NotAligned,
    Hardware(u32),
    Unavailable,
}

impl From<NorFlashErrorKind> for FlashError {
    fn from(error: NorFlashErrorKind) -> Self {
        match error {
            NorFlashErrorKind::OutOfBounds => Self::OutOfBounds,
            NorFlashErrorKind::NotAligned => Self::NotAligned,
            _ => Self::Unavailable,
        }
    }
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::Hardware(_) | Self::Unavailable => NorFlashErrorKind::Other,
        }
    }
}

/// Singleton token for the CC2340R52 512-KiB main flash.
pub struct InternalFlash {
    _private: (),
}

impl InternalFlash {
    #[cfg(any(target_os = "none", test))]
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

impl ErrorType for InternalFlash {
    type Error = FlashError;
}

impl ReadNorFlash for InternalFlash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(FlashError::from)?;

        #[cfg(target_os = "none")]
        {
            for (index, output) in bytes.iter_mut().enumerate() {
                let address = offset as usize + index;
                *output = unsafe { core::ptr::read_volatile(address as *const u8) };
            }
            Ok(())
        }

        #[cfg(not(target_os = "none"))]
        {
            let _ = bytes;
            Err(FlashError::Unavailable)
        }
    }

    fn capacity(&self) -> usize {
        FLASH_CAPACITY
    }
}

impl NorFlash for InternalFlash {
    const WRITE_SIZE: usize = FLASH_WRITE_SIZE;
    const ERASE_SIZE: usize = PHYSICAL_ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(FlashError::from)?;

        #[cfg(target_os = "none")]
        {
            let mut sector = from;
            while sector < to {
                let status = erase_sector(sector);
                if status != 0 {
                    return Err(FlashError::Hardware(status));
                }
                sector += PHYSICAL_ERASE_SIZE as u32;
            }
            Ok(())
        }

        #[cfg(not(target_os = "none"))]
        {
            Err(FlashError::Unavailable)
        }
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(FlashError::from)?;

        #[cfg(target_os = "none")]
        {
            let mut written = 0usize;
            while written < bytes.len() {
                let length = core::cmp::min(ProgramBuffer::CAPACITY, bytes.len() - written);
                let mut buffer = ProgramBuffer([0; ProgramBuffer::CAPACITY]);
                buffer.0[..length].copy_from_slice(&bytes[written..written + length]);
                let address = offset
                    .checked_add(written as u32)
                    .ok_or(FlashError::OutOfBounds)?;
                let status = program(address, &buffer.0[..length]);
                if status != 0 {
                    return Err(FlashError::Hardware(status));
                }
                written += length;
            }
            Ok(())
        }

        #[cfg(not(target_os = "none"))]
        {
            let _ = bytes;
            Err(FlashError::Unavailable)
        }
    }
}

#[cfg(target_os = "none")]
#[repr(align(4))]
struct ProgramBuffer([u8; 64]);

#[cfg(target_os = "none")]
impl ProgramBuffer {
    const CAPACITY: usize = 64;
}

#[cfg(target_os = "none")]
const HAPI_TABLE_BASE: u32 = 0x0F00_004C;
#[cfg(target_os = "none")]
const HAPI_FLASH_SECTOR_ERASE_INDEX: u32 = 3;
#[cfg(target_os = "none")]
const HAPI_FLASH_PROGRAM_INDEX: u32 = 5;
#[cfg(target_os = "none")]
const FLASH_API_KEY: u32 = 0xB7E3_A08F;
#[cfg(target_os = "none")]
const VIMS_CACHE_CONTROL: *mut u32 = 0x4002_4424 as *mut u32;

#[cfg(target_os = "none")]
type FlashEraseFunction = unsafe extern "C" fn(u32, u32) -> u32;
#[cfg(target_os = "none")]
type FlashProgramFunction = unsafe extern "C" fn(u32, *const u8, u32, u32) -> u32;

#[cfg(target_os = "none")]
fn erase_sector(address: u32) -> u32 {
    crate::time::run_flash_operation(|| unsafe {
        let entry = hapi_entry(HAPI_FLASH_SECTOR_ERASE_INDEX);
        let erase: FlashEraseFunction = core::mem::transmute(entry);
        with_cache_disabled(|| erase(FLASH_API_KEY, address))
    })
}

#[cfg(target_os = "none")]
fn program(address: u32, bytes: &[u8]) -> u32 {
    crate::time::run_flash_operation(|| unsafe {
        let entry = hapi_entry(HAPI_FLASH_PROGRAM_INDEX);
        let program: FlashProgramFunction = core::mem::transmute(entry);
        with_cache_disabled(|| program(FLASH_API_KEY, bytes.as_ptr(), address, bytes.len() as u32))
    })
}

#[cfg(target_os = "none")]
unsafe fn hapi_entry(index: u32) -> usize {
    let pointer = (HAPI_TABLE_BASE + index * 4) as *const u32;
    unsafe { core::ptr::read_volatile(pointer) as usize }
}

#[cfg(target_os = "none")]
unsafe fn with_cache_disabled(operation: impl FnOnce() -> u32) -> u32 {
    let previous = unsafe { core::ptr::read_volatile(VIMS_CACHE_CONTROL) };
    unsafe { core::ptr::write_volatile(VIMS_CACHE_CONTROL, 0) };
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    let status = operation();
    core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
    unsafe { core::ptr::write_volatile(VIMS_CACHE_CONTROL, previous) };
    status
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cc2340r52_flash_geometry_is_explicit() {
        assert_eq!(FLASH_CAPACITY, 0x80000);
        assert_eq!(PHYSICAL_ERASE_SIZE, 0x800);
        assert_eq!(FLASH_CAPACITY % PHYSICAL_ERASE_SIZE, 0);
        assert_eq!(FLASH_WRITE_SIZE, 4);
    }

    #[test]
    fn host_backend_fails_closed() {
        let mut flash = InternalFlash::new();
        let mut byte = [0];
        assert_eq!(flash.read(0, &mut byte), Err(FlashError::Unavailable));
        assert_eq!(
            flash.read(FLASH_CAPACITY as u32, &mut [0]),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(flash.erase(1, 2048), Err(FlashError::NotAligned));
    }
}
