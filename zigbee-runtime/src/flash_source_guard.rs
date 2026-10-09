//! Host model of the TLSR8258 RAM-source programming contract (HW-03).
//!
//! The TLSR8258 flash programmer rejects source buffers that live in XIP
//! flash. Host flash mocks accept any pointer, so this wrapper rejects the
//! exact promoted `'static` marker constants that the journals must stage in
//! stack-backed record buffers before programming.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

#[derive(Debug)]
pub(crate) struct StaticSourceError;

impl NorFlashError for StaticSourceError {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::Other
    }
}

pub(crate) struct StaticSourceGuard<F> {
    pub(crate) inner: F,
    forbidden: &'static [&'static [u8]],
    pub(crate) rejected: usize,
}

impl<F> StaticSourceGuard<F> {
    pub(crate) fn new(inner: F, forbidden: &'static [&'static [u8]]) -> Self {
        Self {
            inner,
            forbidden,
            rejected: 0,
        }
    }
}

impl<F> ErrorType for StaticSourceGuard<F> {
    type Error = StaticSourceError;
}

impl<F: ReadNorFlash> ReadNorFlash for StaticSourceGuard<F> {
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.inner
            .read(offset, bytes)
            .map_err(|_| StaticSourceError)
    }

    fn capacity(&self) -> usize {
        self.inner.capacity()
    }
}

impl<F: NorFlash> NorFlash for StaticSourceGuard<F> {
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.inner.erase(from, to).map_err(|_| StaticSourceError)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if self
            .forbidden
            .iter()
            .any(|marker| core::ptr::eq(marker.as_ptr(), bytes.as_ptr()))
        {
            self.rejected += 1;
            return Err(StaticSourceError);
        }
        self.inner
            .write(offset, bytes)
            .map_err(|_| StaticSourceError)
    }
}
