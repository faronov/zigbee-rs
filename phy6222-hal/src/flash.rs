//! SPIF flash controller: XIP read, page program, sector erase.
//!
//! PHY6222 flash is memory-mapped at `0x11000000` for reads (XIP).
//! Writes and erases go through the SPIF controller at `0x4000C800`.
//! Cache must be bypassed during write/erase operations.

use crate::regs::*;
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

macro_rules! ram_nop {
    () => {
        unsafe {
            core::arch::asm!("nop", options(nomem, nostack, preserves_flags));
        }
    };
}

macro_rules! disable_interrupts {
    () => {{
        let primask: u32;
        unsafe {
            core::arch::asm!(
                "mrs {primask}, PRIMASK",
                "cpsid i",
                primask = out(reg) primask,
                options(nomem, nostack, preserves_flags)
            );
        }
        primask
    }};
}

macro_rules! restore_interrupts {
    ($primask:expr) => {
        if $primask & 1 == 0 {
            unsafe {
                core::arch::asm!("cpsie i", options(nomem, nostack, preserves_flags));
            }
        }
    };
}

const MAX_FLASH_CAPACITY: u32 = 512 * 1024;
pub const SECTOR_SIZE: u32 = 4096;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    OutOfRange,
    UnalignedSector,
    ControllerTimeout,
    DeviceBusyTimeout,
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfRange => NorFlashErrorKind::OutOfBounds,
            Self::UnalignedSector => NorFlashErrorKind::NotAligned,
            Self::ControllerTimeout | Self::DeviceBusyTimeout => NorFlashErrorKind::Other,
        }
    }
}

pub struct Phy62x2Flash {
    capacity: usize,
}

impl Phy62x2Flash {
    pub const fn new(capacity: usize) -> Self {
        Self { capacity }
    }

    fn validate_range(&self, offset: u32, length: usize) -> Result<(), FlashError> {
        let start = usize::try_from(offset).map_err(|_| FlashError::OutOfRange)?;
        start
            .checked_add(length)
            .filter(|end| *end <= self.capacity)
            .map(|_| ())
            .ok_or(FlashError::OutOfRange)
    }
}

impl ErrorType for Phy62x2Flash {
    type Error = FlashError;
}

impl ReadNorFlash for Phy62x2Flash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.validate_range(offset, bytes.len())?;
        read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.capacity
    }
}

impl NorFlash for Phy62x2Flash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = SECTOR_SIZE as usize;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::OutOfRange);
        }
        if from & (SECTOR_SIZE - 1) != 0 || to & (SECTOR_SIZE - 1) != 0 {
            return Err(FlashError::UnalignedSector);
        }
        self.validate_range(from, (to - from) as usize)?;
        let mut offset = from;
        while offset < to {
            erase_sector(offset)?;
            offset += SECTOR_SIZE;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if offset as usize % Self::WRITE_SIZE != 0 || bytes.len() % Self::WRITE_SIZE != 0 {
            return Err(FlashError::UnalignedSector);
        }
        self.validate_range(offset, bytes.len())?;
        write(offset, bytes)
    }
}

/// Read bytes from flash via XIP (memory-mapped, no SPIF commands needed).
pub fn read(offset: u32, buf: &mut [u8]) -> Result<(), FlashError> {
    validate_range(offset, buf.len())?;
    let addr = (FLASH_BASE + offset) as *const u8;
    for (i, b) in buf.iter_mut().enumerate() {
        *b = unsafe { core::ptr::read_volatile(addr.add(i)) };
    }
    Ok(())
}

/// Erase a 4 KB flash sector.
///
/// The complete operation executes from SRAM with interrupts disabled because
/// cache bypass makes XIP instruction fetches unsafe.
#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
pub fn erase_sector(offset: u32) -> Result<(), FlashError> {
    if offset & (SECTOR_SIZE - 1) != 0 {
        return Err(FlashError::UnalignedSector);
    }
    validate_range(offset, SECTOR_SIZE as usize)?;

    let primask = disable_interrupts!();
    enter_cache_bypass();
    let result = erase_sector_inner(offset);
    exit_cache_bypass();
    cache_flush();
    restore_interrupts!(primask);
    result
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn erase_sector_inner(offset: u32) -> Result<(), FlashError> {
    write_enable()?;
    reg_write(SPIF_FCMD_ADDR, offset);
    reg_write(SPIF_FCMD, (0x20u32 << 24) | 0x8_0001 | (2 << 16));
    spif_wait_idle()?;
    spif_wait_not_busy()
}

/// Write data to flash (page program, handles 256-byte page boundaries).
///
/// The complete operation executes from SRAM with interrupts disabled because
/// cache bypass makes XIP instruction fetches unsafe.
#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
pub fn write(offset: u32, data: &[u8]) -> Result<(), FlashError> {
    validate_range(offset, data.len())?;

    let primask = disable_interrupts!();
    enter_cache_bypass();
    let result = write_inner(offset, data);
    exit_cache_bypass();
    cache_flush();
    restore_interrupts!(primask);
    result
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn write_inner(offset: u32, data: &[u8]) -> Result<(), FlashError> {
    let mut pos = 0;
    while pos < data.len() {
        let page_boundary = ((offset as usize + pos) | 0xFF) + 1;
        let remaining_in_page = page_boundary - (offset as usize + pos);
        let chunk_len = (data.len() - pos).min(remaining_in_page).min(256);

        write_enable()?;

        // Set address
        reg_write(SPIF_FCMD_ADDR, offset + pos as u32);

        // Write data to FIFO
        let mut i = 0;
        while i < chunk_len {
            let mut word = 0xFFFF_FFFFu32;
            for b in 0..4 {
                if i + b < chunk_len {
                    word &= !(0xFF << (b * 8));
                    let byte = unsafe { *data.as_ptr().add(pos + i + b) };
                    word |= (byte as u32) << (b * 8);
                }
            }
            reg_write(SPIF_FCMD_WRDATA + (i as u32 / 4) * 4, word);
            i += 4;
        }

        // Page Program
        let wr_words = ((chunk_len + 3) / 4) as u32;
        let fcmd = (0x02u32 << 24)
            | 0x8_0001
            | (2 << 16)
            | 0x8000
            | (wr_words.saturating_sub(1) << 12);
        reg_write(SPIF_FCMD, fcmd);
        spif_wait_idle()?;
        spif_wait_not_busy()?;

        pos += chunk_len;
    }

    Ok(())
}

// ── Internal helpers ────────────────────────────────────────────

fn validate_range(offset: u32, len: usize) -> Result<(), FlashError> {
    let len = u32::try_from(len).map_err(|_| FlashError::OutOfRange)?;
    let end = offset.checked_add(len).ok_or(FlashError::OutOfRange)?;
    if end > MAX_FLASH_CAPACITY {
        return Err(FlashError::OutOfRange);
    }
    Ok(())
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn write_enable() -> Result<(), FlashError> {
    spif_wait_idle()?;
    reg_write(SPIF_FCMD, 0x0600_0001);
    spif_wait_idle()?;
    spif_wait_not_busy()
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn spif_wait_idle() -> Result<(), FlashError> {
    for _ in 0..100_000u32 {
        if reg_read(SPIF_FCMD) & 0x02 == 0 {
            if reg_read(SPIF_CONFIG) & 0x8000_0000 != 0 {
                return Ok(());
            }
        }
        ram_nop!();
    }
    Err(FlashError::ControllerTimeout)
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn spif_wait_not_busy() -> Result<(), FlashError> {
    for _ in 0..1_000_000u32 {
        reg_write(SPIF_FCMD, (0x05u32 << 24) | 0x80_0001 | (1 << 20));
        spif_wait_idle()?;
        let status = reg_read(SPIF_FCMD_RDDATA) & 0xFF;
        if status & 0x01 == 0 {
            return Ok(());
        }
        ram_nop!();
    }
    Err(FlashError::DeviceBusyTimeout)
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn enter_cache_bypass() {
    reg_write(CACHE_CTRL0, 0x02);
    reg_write(CACHE_BYPASS_REG, 1);
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn exit_cache_bypass() {
    reg_write(CACHE_CTRL0, 0x00);
    reg_write(CACHE_BYPASS_REG, 0);
}

#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
fn cache_flush() {
    reg_write(CACHE_CTRL0, 0x02);
    for _ in 0..8 {
        ram_nop!();
    }
    reg_write(CACHE_CTRL0, 0x03);
    for _ in 0..8 {
        ram_nop!();
    }
    reg_write(CACHE_CTRL0, 0x00);
}

/// Prepare the flash for a non-returning system-sleep transition.
///
/// The caller must remain in SRAM and trigger system sleep immediately after
/// this function returns. Returning to XIP code after the `0xB9` command is
/// invalid.
#[unsafe(link_section = ".data.ram_code")]
#[inline(never)]
pub(crate) fn prepare_deep_power_down() -> Result<(), FlashError> {
    enter_cache_bypass();
    if let Err(error) = spif_wait_idle() {
        exit_cache_bypass();
        cache_flush();
        return Err(error);
    }
    reg_write(SPIF_FCMD, 0xB900_0001);
    for _ in 0..32 {
        ram_nop!();
    }
    Ok(())
}
