//! Internal flash: memory-mapped reads, erase/program through the boot ROM
//! flash API (`ROM_API_FLASH_TABLE[5]` sector erase, `[6]` program, as
//! driverlib `flash.c` of SDK 8.33.00.16 calls them).
//!
//! Like TI's `NVSCC26XX`, every erase/program runs with interrupts masked
//! and the VIMS cache disabled (the flash is unreadable while its state
//! machine runs, and disabling the cache also drops stale lines). The RF
//! core keeps receiving and acknowledging from its own RAM meanwhile. A
//! sector erase masks interrupts for a few milliseconds: UART receive needs
//! hardware flow control or a host protocol that retransmits.
//!
//! The last sector holds the CCFG. [`Flash`] refuses to erase or program it:
//! nothing at run time needs it, and a torn CCFG write is the one flash
//! operation that can leave a dongle without its serial bootloader.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use crate::chip::{FLASH_SIZE, SECTOR_SIZE};
#[cfg(all(target_arch = "arm", target_os = "none"))]
use crate::mmio::FLASH_BASE;
use crate::mmio::{self, VIMS_BASE};

pub const FLASH_PAGE_SIZE: usize = SECTOR_SIZE;
/// Start of the CCFG sector (last sector of flash).
pub const CCFG_SECTOR: u32 = (FLASH_SIZE - SECTOR_SIZE) as u32;
/// Program granularity accepted by [`Flash`] (the ROM programs bytes; the
/// journals write whole words).
pub const WRITE_SIZE: usize = 4;

const ROM_FLASH_TABLE: u32 = 10;
const ROM_SECTOR_ERASE: u32 = 5;
const ROM_PROGRAM: u32 = 6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FlashError {
    OutOfBounds,
    NotAligned,
    /// The CCFG sector is never written at run time.
    Protected,
    /// The ROM reported an FSM error or the read-back differed.
    Controller(u32),
}

impl NorFlashError for FlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds | Self::Protected => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::Controller(_) => NorFlashErrorKind::Other,
        }
    }
}

/// The internal flash (application-wide singleton from [`crate::Peripherals`]).
pub struct Flash(());

impl Flash {
    pub(crate) const fn new() -> Self {
        Self(())
    }

    fn check(offset: u32, len: usize) -> Result<(), FlashError> {
        let end = (offset as usize)
            .checked_add(len)
            .filter(|end| *end <= FLASH_SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        if end > CCFG_SECTOR as usize {
            return Err(FlashError::Protected);
        }
        Ok(())
    }

    pub fn erase_sector(&mut self, addr: u32) -> Result<(), FlashError> {
        if !(addr as usize).is_multiple_of(SECTOR_SIZE) {
            return Err(FlashError::NotAligned);
        }
        Self::check(addr, SECTOR_SIZE)?;
        let status = with_flash_quiet(|| unsafe {
            let f: extern "C" fn(u32) -> u32 =
                core::mem::transmute(mmio::rom_fn(ROM_FLASH_TABLE, ROM_SECTOR_ERASE) as usize);
            f(addr)
        });
        if status != 0 {
            return Err(FlashError::Controller(status));
        }
        verify_erased(addr, SECTOR_SIZE)
    }

    pub fn program(&mut self, addr: u32, bytes: &[u8]) -> Result<(), FlashError> {
        if bytes.is_empty() {
            return Ok(());
        }
        Self::check(addr, bytes.len())?;
        // The ROM reads the source while the flash is busy: a source that
        // itself lies in flash is staged through RAM.
        if (bytes.as_ptr() as usize) < FLASH_SIZE {
            let mut staged = [0u8; 64];
            for (i, chunk) in bytes.chunks(staged.len()).enumerate() {
                staged[..chunk.len()].copy_from_slice(chunk);
                self.program(addr + (i * 64) as u32, &staged[..chunk.len()])?;
            }
            return Ok(());
        }
        let status = with_flash_quiet(|| unsafe {
            let f: extern "C" fn(*const u8, u32, u32) -> u32 =
                core::mem::transmute(mmio::rom_fn(ROM_FLASH_TABLE, ROM_PROGRAM) as usize);
            f(bytes.as_ptr(), addr, bytes.len() as u32)
        });
        if status != 0 {
            return Err(FlashError::Controller(status));
        }
        verify(addr, bytes)
    }
}

/// Run a ROM flash operation with interrupts masked and the cache disabled
/// (`NVSCC26XX` `disableFlashCache`/`restoreFlashCache`), then re-enable
/// flash standby (driverlib does so after every ROM call).
fn with_flash_quiet(op: impl FnOnce() -> u32) -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        cortex_m::interrupt::free(|_| {
            let mode = vims_mode();
            if mode != 0 {
                vims_set_mode(0);
            }
            let status = op();
            mmio::modify(FLASH_BASE + 0x24, 1 << 1, 0);
            if mode != 0 {
                vims_set_mode(mode);
            }
            status
        })
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = op;
        0
    }
}

fn vims_mode() -> u32 {
    loop {
        let stat = mmio::read(VIMS_BASE);
        if stat & (1 << 3) == 0 {
            return stat & 0x3;
        }
    }
}

fn vims_set_mode(mode: u32) {
    let _ = vims_mode();
    mmio::modify(VIMS_BASE + 0x4, 0x3, mode);
    while mmio::read(VIMS_BASE) & (1 << 3) != 0 {}
}

fn verify_erased(addr: u32, len: usize) -> Result<(), FlashError> {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    for word in (addr..addr + len as u32).step_by(4) {
        if mmio::read(word) != 0xFFFF_FFFF {
            return Err(FlashError::Controller(0x100));
        }
    }
    let _ = (addr, len);
    Ok(())
}

fn verify(addr: u32, bytes: &[u8]) -> Result<(), FlashError> {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    for (i, expected) in bytes.iter().enumerate() {
        if mmio::read8(addr + i as u32) != *expected {
            return Err(FlashError::Controller(0x200));
        }
    }
    let _ = (addr, bytes);
    Ok(())
}

impl ErrorType for Flash {
    type Error = FlashError;
}

impl ReadNorFlash for Flash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError> {
        (offset as usize)
            .checked_add(bytes.len())
            .filter(|end| *end <= FLASH_SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        for (i, b) in bytes.iter_mut().enumerate() {
            *b = mmio::read8(offset + i as u32);
        }
        Ok(())
    }

    fn capacity(&self) -> usize {
        FLASH_SIZE
    }
}

impl NorFlash for Flash {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
        if from > to
            || !(from as usize).is_multiple_of(SECTOR_SIZE)
            || !(to as usize).is_multiple_of(SECTOR_SIZE)
        {
            return Err(FlashError::NotAligned);
        }
        Self::check(from, (to - from) as usize)?;
        let mut addr = from;
        while addr < to {
            self.erase_sector(addr)?;
            addr += SECTOR_SIZE as u32;
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
        if !(offset as usize).is_multiple_of(WRITE_SIZE) || !bytes.len().is_multiple_of(WRITE_SIZE)
        {
            return Err(FlashError::NotAligned);
        }
        self.program(offset, bytes)
    }
}

/// A product-owned window `[START, START + SIZE)` of the internal flash
/// with partition-relative offsets.
pub struct FlashPartition<const START: u32, const SIZE: usize> {
    flash: Flash,
}

impl<const START: u32, const SIZE: usize> FlashPartition<START, SIZE> {
    const VALID: () = {
        assert!(SIZE > 0);
        assert!((START as usize).is_multiple_of(SECTOR_SIZE));
        assert!(SIZE.is_multiple_of(SECTOR_SIZE));
        assert!(START as usize + SIZE <= CCFG_SECTOR as usize);
    };

    /// # Safety
    ///
    /// The product's flash split must guarantee that partitions minted from
    /// the one [`Flash`] handle are pairwise disjoint and disjoint from the
    /// running image.
    pub const unsafe fn new_unchecked() -> Self {
        let () = Self::VALID;
        Self {
            flash: Flash::new(),
        }
    }

    fn physical(offset: u32, len: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(len)
            .filter(|end| *end <= SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        START.checked_add(offset).ok_or(FlashError::OutOfBounds)
    }
}

impl<const START: u32, const SIZE: usize> ErrorType for FlashPartition<START, SIZE> {
    type Error = FlashError;
}

impl<const START: u32, const SIZE: usize> ReadNorFlash for FlashPartition<START, SIZE> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), FlashError> {
        self.flash.read(Self::physical(offset, bytes.len())?, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<const START: u32, const SIZE: usize> NorFlash for FlashPartition<START, SIZE> {
    const WRITE_SIZE: usize = WRITE_SIZE;
    const ERASE_SIZE: usize = SECTOR_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
        if from > to {
            return Err(FlashError::OutOfBounds);
        }
        let start = Self::physical(from, (to - from) as usize)?;
        self.flash.erase(start, start + (to - from))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), FlashError> {
        self.flash
            .write(Self::physical(offset, bytes.len())?, bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ccfg_sector_is_refused() {
        let mut flash = Flash::new();
        assert_eq!(flash.erase_sector(CCFG_SECTOR), Err(FlashError::Protected));
        assert_eq!(
            flash.program(CCFG_SECTOR - 4, &[0; 8]),
            Err(FlashError::Protected)
        );
        assert_eq!(flash.erase_sector(1), Err(FlashError::NotAligned));
        assert_eq!(
            flash.program(FLASH_SIZE as u32, &[0]),
            Err(FlashError::OutOfBounds)
        );
    }

    #[test]
    fn partition_offsets_are_bounded() {
        type P = FlashPartition<0x4000, 0x4000>;
        assert_eq!(P::physical(0, 4), Ok(0x4000));
        assert_eq!(P::physical(0x3FFC, 4), Ok(0x7FFC));
        assert_eq!(P::physical(0x3FFD, 4), Err(FlashError::OutOfBounds));
    }
}
