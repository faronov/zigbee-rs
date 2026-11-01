//! Bounded NVMC windows for the router-role journals.
//!
//! `embassy_nrf::nvmc::Nvmc` addresses the whole flash. A journal must only
//! ever see its own window, so [`NvmcPartition`] rebases every access to
//! `START` and rejects anything that would leave `SIZE`. The handles are
//! minted only by [`crate::router::router_stores`], which consumes the safe
//! NVMC handle first.

use core::marker::PhantomData;

use embassy_nrf::nvmc::{Error, Nvmc};
use embassy_nrf::peripherals::NVMC;
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

pub struct NvmcPartition<const START: u32, const SIZE: usize> {
    nvmc: Nvmc<'static>,
    _not_sync: PhantomData<*mut ()>,
}

impl<const START: u32, const SIZE: usize> NvmcPartition<START, SIZE> {
    /// Mint a window over a stolen NVMC handle.
    ///
    /// # Safety
    ///
    /// The window must be disjoint from every other flash user (application
    /// image, other journals), and the NVMC must only be driven from one
    /// execution context: its operations are synchronous register sequences
    /// that must not interleave.
    pub(crate) unsafe fn steal() -> Self {
        // SAFETY: the caller guarantees exclusive use of this window.
        let nvmc = Nvmc::new(unsafe { NVMC::steal() });
        Self {
            nvmc,
            _not_sync: PhantomData,
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, Error> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SIZE)
            .ok_or(Error::OutOfBounds)?;
        START.checked_add(offset).ok_or(Error::OutOfBounds)
    }
}

impl<const START: u32, const SIZE: usize> ErrorType for NvmcPartition<START, SIZE> {
    type Error = Error;
}

impl<const START: u32, const SIZE: usize> ReadNorFlash for NvmcPartition<START, SIZE> {
    const READ_SIZE: usize = Nvmc::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.nvmc.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<const START: u32, const SIZE: usize> NorFlash for NvmcPartition<START, SIZE> {
    const WRITE_SIZE: usize = Nvmc::WRITE_SIZE;
    const ERASE_SIZE: usize = Nvmc::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(Error::OutOfBounds);
        }
        let length = usize::try_from(to - from).map_err(|_| Error::OutOfBounds)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(Error::OutOfBounds)?;
        self.nvmc.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.nvmc.write(physical, bytes)
    }
}
