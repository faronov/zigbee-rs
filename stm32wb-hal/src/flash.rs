//! Internal flash windows that coexist with a running CPU2.
//!
//! CPU2 executes from the top of the same flash and may itself program it.
//! Every CPU1 program or erase step therefore follows ST's flash driver
//! protocol (AN5289 "Flash memory management", RM0434 §3.3.12):
//!
//! 1. take HSEM [`hsem::FLASH`] for the flash controller;
//! 2. take HSEM [`hsem::BLOCK_FLASH_REQ_BY_CPU2`], which CPU2 holds during
//!    timing-critical radio windows, around the single step;
//! 3. let the controller wait out `PESD` (CPU2 program/erase suspend
//!    request), `CFGBSY` and `BSY` — embassy-stm32 does this.
//!
//! The coprocessor default (`FLASH_ACTIVITY_CONTROL_PES`) needs no SHCI
//! erase-activity notification; this crate does not change it.
//!
//! embassy-stm32 rejects anything at or above the CPU2 secure area (SFSA),
//! reports uncorrectable ECC errors as [`Error::Ecc`] instead of a crash,
//! and owns the NMI handler that makes that possible. [`FlashWindow`] adds
//! a hard window bound on top.

use core::marker::PhantomData;

use embassy_stm32::flash::{Error, Flash};
use embassy_stm32::mode::Blocking;
use embassy_stm32::pac;
use embassy_stm32::peripherals::FLASH;

use crate::hsem;
use crate::word_flash::{
    DOUBLE_WORD, DoubleWordFlash, DoubleWordRead, PHYSICAL_PAGE_SIZE, WordPerDoubleWord,
};

/// Main flash base address.
pub const FLASH_BASE: u32 = 0x0800_0000;

/// Semaphore spins before a flash step gives up. CPU2 holds the block
/// semaphore for radio windows of a few milliseconds at most.
const SEMAPHORE_ATTEMPTS: u32 = 2_000_000;

/// First byte of the CPU2 secure area (`FLASH_SFR.SFSA` pages).
pub fn secure_flash_start() -> u32 {
    let sfr = pac::FLASH.sfr().read();
    if sfr.fsd() {
        // Security disabled: CPU2 owns nothing; report the end of flash.
        FLASH_BASE + embassy_stm32::flash::FLASH_SIZE as u32
    } else {
        FLASH_BASE + u32::from(sfr.sfsa()) * PHYSICAL_PAGE_SIZE as u32
    }
}

/// Error of a [`FlashWindow`] step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashError {
    Flash(Error),
    /// CPU2 kept a flash semaphore for too long.
    SemaphoreTimeout,
    /// The window extends into the CPU2 secure area.
    SecureArea,
    OutOfBounds,
}

/// A page-aligned absolute window `[START, START + SIZE)` of main flash.
pub struct FlashWindow<const START: u32, const SIZE: usize> {
    flash: Flash<'static, Blocking>,
    _not_sync: PhantomData<*mut ()>,
}

impl<const START: u32, const SIZE: usize> FlashWindow<START, SIZE> {
    const VALID: () = assert!(
        START >= FLASH_BASE
            && (START - FLASH_BASE).is_multiple_of(PHYSICAL_PAGE_SIZE as u32)
            && SIZE > 0
            && SIZE.is_multiple_of(PHYSICAL_PAGE_SIZE)
    );

    /// Mint a window over a stolen flash peripheral.
    ///
    /// Fails if the window reaches into the CPU2 secure area as currently
    /// configured by the FUS.
    ///
    /// # Safety
    ///
    /// The window must be disjoint from the application image and from
    /// every other window, and all windows must be used from one execution
    /// context (their steps are blocking register sequences).
    pub unsafe fn steal() -> Result<Self, FlashError> {
        let () = Self::VALID;
        if START as u64 + SIZE as u64 > u64::from(secure_flash_start()) {
            return Err(FlashError::SecureArea);
        }
        hsem::enable();
        // SAFETY: the caller guarantees exclusive use of this window.
        let peripheral = unsafe { FLASH::steal() };
        Ok(Self {
            flash: Flash::new_blocking(peripheral, Default::default()),
            _not_sync: PhantomData,
        })
    }

    /// Wrap the window for the zigbee-runtime journals.
    pub fn into_word_flash(self) -> WordPerDoubleWord<Self> {
        WordPerDoubleWord::new(self)
    }

    fn offset(&self, offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        Ok(START - FLASH_BASE + offset)
    }

    fn guarded<R>(
        &mut self,
        step: impl FnOnce(&mut Flash<'static, Blocking>) -> Result<R, Error>,
    ) -> Result<R, FlashError> {
        let _flash = hsem::Guard::acquire(hsem::FLASH, SEMAPHORE_ATTEMPTS)
            .ok_or(FlashError::SemaphoreTimeout)?;
        let _block = hsem::Guard::acquire(hsem::BLOCK_FLASH_REQ_BY_CPU2, SEMAPHORE_ATTEMPTS)
            .ok_or(FlashError::SemaphoreTimeout)?;
        step(&mut self.flash).map_err(FlashError::Flash)
    }
}

impl<const START: u32, const SIZE: usize> DoubleWordFlash for FlashWindow<START, SIZE> {
    type Error = FlashError;

    fn physical_capacity(&self) -> usize {
        SIZE
    }

    fn read_double_word(&mut self, offset: u32) -> Result<DoubleWordRead, FlashError> {
        let physical = self.offset(offset, DOUBLE_WORD)?;
        let mut data = [0; DOUBLE_WORD];
        match self.flash.blocking_read(physical, &mut data) {
            Ok(()) => Ok(DoubleWordRead::Data(data)),
            Err(Error::Ecc) => Ok(DoubleWordRead::EccError),
            Err(error) => Err(FlashError::Flash(error)),
        }
    }

    fn program_double_word(
        &mut self,
        offset: u32,
        value: [u8; DOUBLE_WORD],
    ) -> Result<(), FlashError> {
        let physical = self.offset(offset, DOUBLE_WORD)?;
        self.guarded(|flash| flash.blocking_write(physical, &value))
    }

    fn erase_page(&mut self, offset: u32) -> Result<(), FlashError> {
        let physical = self.offset(offset, PHYSICAL_PAGE_SIZE)?;
        self.guarded(|flash| flash.blocking_erase(physical, physical + PHYSICAL_PAGE_SIZE as u32))
    }
}
