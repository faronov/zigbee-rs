//! EFR32 Series 2 (xG21/xG22) MSC flash controller.
//!
//! Sequencing follows GSDK 4.5 `em_msc.c` (`writeBurst`, `mscStatusWait`):
//! `STATUS.TIMEOUT` only reports that a burst ended because `WDATA` arrived
//! late (the next word starts a new write); it is not an error. Every
//! programmed burst is read back and compared, so a silently dropped word
//! is still reported. Hardware-proven on EFR32MG22 (BRD4183A) by the
//! security journal of `examples/efr32mg22-sensor`.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

/// Main flash of the F512 parts (e.g. EFR32MG22C224F512 on BRD4183A); the
/// MG21 devkit product also confines itself to this range. Also the
/// conservative bound when DEVINFO does not report a plausible size, and
/// the host-build capacity.
pub const FLASH_CAPACITY: usize = 512 * 1024;
/// Largest main flash in the supported families (EFR32MG21 F1024).
pub const MAX_FLASH_CAPACITY: usize = 1024 * 1024;
pub const FLASH_PAGE_SIZE: usize = 8192;
/// `DEVINFO.MSIZE`: `FLASH[15:0]` in KiB, `SRAM[26:16]` in KiB.
#[cfg(target_os = "none")]
const DEVINFO_MSIZE: u32 = 0x0FE0_800C;
const PROGRAM_TIMEOUT: u32 = 100_000;

const MSC_BASE: u32 = 0x4003_0000;
const MSC_WRITECTRL: u32 = MSC_BASE + 0x00C;
const MSC_WRITECMD: u32 = MSC_BASE + 0x010;
const MSC_ADDRB: u32 = MSC_BASE + 0x014;
const MSC_WDATA: u32 = MSC_BASE + 0x018;
const MSC_STATUS: u32 = MSC_BASE + 0x01C;
const MSC_LOCK: u32 = MSC_BASE + 0x03C;

const MSC_WRITECTRL_WREN: u32 = 1 << 0;
const MSC_WRITECMD_ERASEPAGE: u32 = 1 << 1;
const MSC_WRITECMD_WRITEEND: u32 = 1 << 2;
const MSC_STATUS_BUSY: u32 = 1 << 0;
const MSC_STATUS_LOCKED: u32 = 1 << 1;
const MSC_STATUS_INVADDR: u32 = 1 << 2;
const MSC_STATUS_WDATAREADY: u32 = 1 << 3;
const MSC_STATUS_PENDING: u32 = 1 << 5;
const MSC_STATUS_REGLOCK: u32 = 1 << 16;
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

/// Main-flash size of this die from `DEVINFO.MSIZE` (ZBT-1: 1 MiB,
/// BRD4183A: 512 KiB). Falls back to the smallest supported size,
/// [`FLASH_CAPACITY`], when the field is erased or implausible, so a bad
/// DEVINFO can only shrink the writable range. Products that depend on a
/// specific part size must compare this against their layout at boot.
pub fn device_flash_capacity() -> usize {
    #[cfg(target_os = "none")]
    {
        // SAFETY: DEVINFO is a read-only, always-mapped information page.
        let msize = unsafe { core::ptr::read_volatile(DEVINFO_MSIZE as *const u32) };
        let bytes = (msize & 0xFFFF) as usize * 1024;
        if bytes >= FLASH_CAPACITY
            && bytes <= MAX_FLASH_CAPACITY
            && bytes.is_multiple_of(FLASH_PAGE_SIZE)
        {
            return bytes;
        }
    }
    FLASH_CAPACITY
}

pub struct Efr32s2Flash {
    _private: (),
}

impl Efr32s2Flash {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    fn validate_range(offset: u32, length: usize) -> Result<(), FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= device_flash_capacity())
            .map(|_| ())
            .ok_or(FlashError::OutOfBounds)
    }
}

impl ErrorType for Efr32s2Flash {
    type Error = FlashError;
}

impl ReadNorFlash for Efr32s2Flash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        Self::validate_range(offset, bytes.len())?;
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = unsafe { core::ptr::read_volatile((offset + index as u32) as *const u8) };
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        device_flash_capacity()
    }
}

impl NorFlash for Efr32s2Flash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = FLASH_PAGE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::OutOfBounds);
        }
        if !(from as usize).is_multiple_of(FLASH_PAGE_SIZE)
            || !(to as usize).is_multiple_of(FLASH_PAGE_SIZE)
        {
            return Err(FlashError::NotAligned);
        }
        Self::validate_range(from, (to - from) as usize)?;

        let mut page = from;
        while page < to {
            erase_page_ram(page)?;
            page += FLASH_PAGE_SIZE as u32;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if !(offset as usize).is_multiple_of(Self::WRITE_SIZE)
            || !bytes.len().is_multiple_of(Self::WRITE_SIZE)
        {
            return Err(FlashError::NotAligned);
        }
        Self::validate_range(offset, bytes.len())?;

        let mut address = offset;
        let mut remaining = bytes;
        while !remaining.is_empty() {
            let page_remaining = FLASH_PAGE_SIZE - address as usize % FLASH_PAGE_SIZE;
            let burst_len = remaining.len().min(page_remaining);
            let burst = &remaining[..burst_len];
            program_burst_ram(address, burst)?;
            verify(address, burst)?;
            address += burst_len as u32;
            remaining = &remaining[burst_len..];
        }
        Ok(())
    }
}

/// Page-aligned window `[START, START + SIZE)` of the internal flash.
///
/// Offsets are relative to `START` and every access is bounds-checked, so
/// a journal bound to one partition cannot reach the bootloader, the
/// application or another product partition. Products mint partitions from
/// the unique [`Efr32s2Flash`] handle (see [`Self::new_unchecked`]).
pub struct FlashPartition<const START: u32, const SIZE: usize> {
    flash: Efr32s2Flash,
}

impl<const START: u32, const SIZE: usize> FlashPartition<START, SIZE> {
    const VALID: () = {
        assert!(SIZE > 0);
        assert!((START as usize).is_multiple_of(FLASH_PAGE_SIZE));
        assert!(SIZE.is_multiple_of(FLASH_PAGE_SIZE));
        assert!(START as usize + SIZE <= MAX_FLASH_CAPACITY);
    };

    /// # Safety
    ///
    /// The caller (the product's flash split) must guarantee that no other
    /// live owner issues MSC operations inside `[START, START + SIZE)`, i.e.
    /// partitions minted from one [`Efr32s2Flash`] are pairwise disjoint.
    pub const unsafe fn new_unchecked() -> Self {
        let () = Self::VALID;
        Self {
            flash: Efr32s2Flash::new(),
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

impl<const START: u32, const SIZE: usize> ErrorType for FlashPartition<START, SIZE> {
    type Error = FlashError;
}

impl<const START: u32, const SIZE: usize> ReadNorFlash for FlashPartition<START, SIZE> {
    const READ_SIZE: usize = Efr32s2Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash
            .read(Self::physical_offset(offset, bytes.len())?, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<const START: u32, const SIZE: usize> NorFlash for FlashPartition<START, SIZE> {
    const WRITE_SIZE: usize = Efr32s2Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Efr32s2Flash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::OutOfBounds);
        }
        let physical_from = Self::physical_offset(from, (to - from) as usize)?;
        self.flash.erase(physical_from, physical_from + (to - from))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.flash
            .write(Self::physical_offset(offset, bytes.len())?, bytes)
    }
}

/// Read back a programmed burst.
fn verify(offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    for (index, expected) in bytes.iter().enumerate() {
        let actual = unsafe { core::ptr::read_volatile((offset + index as u32) as *const u8) };
        if actual != *expected {
            return Err(FlashError::Controller);
        }
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (offset, bytes);
    Ok(())
}

#[inline(never)]
#[cfg_attr(target_arch = "arm", unsafe(link_section = ".data.ram_code"))]
fn erase_page_ram(offset: u32) -> Result<(), FlashError> {
    unsafe {
        let was_locked =
            core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_REGLOCK != 0;
        core::ptr::write_volatile(MSC_LOCK as *mut u32, MSC_LOCK_LOCKKEY_UNLOCK);
        let writectrl = core::ptr::read_volatile(MSC_WRITECTRL as *const u32);
        core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, writectrl | MSC_WRITECTRL_WREN);
        core::ptr::write_volatile(MSC_ADDRB as *mut u32, offset);

        let mut result =
            if core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_INVADDR != 0 {
                Err(FlashError::Controller)
            } else {
                core::ptr::write_volatile(MSC_WRITECMD as *mut u32, MSC_WRITECMD_ERASEPAGE);
                wait_series2_ready_ram()
            };
        if result.is_ok() {
            result = wait_series2_ready_ram();
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
fn program_burst_ram(offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
    unsafe {
        let was_locked =
            core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_REGLOCK != 0;
        core::ptr::write_volatile(MSC_LOCK as *mut u32, MSC_LOCK_LOCKKEY_UNLOCK);
        let writectrl = core::ptr::read_volatile(MSC_WRITECTRL as *const u32);
        core::ptr::write_volatile(MSC_WRITECTRL as *mut u32, writectrl | MSC_WRITECTRL_WREN);
        core::ptr::write_volatile(MSC_ADDRB as *mut u32, offset);

        let mut result =
            if core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_INVADDR != 0 {
                Err(FlashError::Controller)
            } else {
                Ok(())
            };

        for chunk in bytes.chunks_exact(4) {
            if result.is_err() {
                break;
            }
            let mut timeout = PROGRAM_TIMEOUT;
            while core::ptr::read_volatile(MSC_STATUS as *const u32) & MSC_STATUS_WDATAREADY == 0
                && timeout != 0
            {
                timeout -= 1;
            }
            if timeout == 0 {
                result = Err(FlashError::Timeout);
                break;
            }
            let word = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
            core::ptr::write_volatile(MSC_WDATA as *mut u32, word);
        }

        core::ptr::write_volatile(MSC_WRITECMD as *mut u32, MSC_WRITECMD_WRITEEND);
        if result.is_ok() {
            result = wait_series2_ready_ram();
        }
        if result.is_ok() {
            result = wait_series2_ready_ram();
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
fn wait_series2_ready_ram() -> Result<(), FlashError> {
    for _ in 0..PROGRAM_TIMEOUT {
        let status = unsafe { core::ptr::read_volatile(MSC_STATUS as *const u32) };
        if status & (MSC_STATUS_BUSY | MSC_STATUS_PENDING) == 0 {
            return if status & (MSC_STATUS_LOCKED | MSC_STATUS_INVADDR | MSC_STATUS_REGLOCK) != 0 {
                Err(FlashError::Controller)
            } else {
                Ok(())
            };
        }
    }
    Err(FlashError::Timeout)
}

#[cfg(test)]
mod tests {
    use super::*;

    type Window = FlashPartition<0x000F_4000, 0x4000>;

    #[test]
    fn partition_offsets_stay_inside_their_window() {
        assert_eq!(Window::physical_offset(0, 4), Ok(0x000F_4000));
        assert_eq!(Window::physical_offset(0x3FFC, 4), Ok(0x000F_7FFC));
        assert_eq!(
            Window::physical_offset(0x3FFD, 4),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            Window::physical_offset(u32::MAX, 1),
            Err(FlashError::OutOfBounds)
        );
        // SAFETY: host test; no MSC access is issued.
        let window = unsafe { Window::new_unchecked() };
        assert_eq!(window.capacity(), 0x4000);
    }
}
