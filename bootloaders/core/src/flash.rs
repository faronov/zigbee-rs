//! [`BootFlash`]: the loader's view of the flash device.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use crate::{BootConfig, Layout};

/// Flash view used by a loader: reads anywhere, but erases and programs
/// only inside `[min(primary, secondary), max(primary, secondary) + slot_size)`
/// and, in swap-move mode, the two status sectors.
///
/// The boot header, the loader itself, its `LoaderInfo` record and the
/// product's persistence partitions therefore cannot be touched by the
/// install copy, whatever the engine or a corrupt header asks for. A layout
/// whose slots are not adjacent must not place protected data between them.
pub struct BootFlash<F> {
    flash: F,
    start: u32,
    end: u32,
    /// Extra writable `[start, end)`: the swap-move status sectors.
    status: Option<(u32, u32)>,
}

impl<F> BootFlash<F> {
    /// The slots of `layout` only.
    pub fn new(flash: F, layout: &Layout) -> Self {
        let start = core::cmp::min(layout.primary, layout.secondary);
        let end = core::cmp::max(layout.primary, layout.secondary) + layout.slot_size;
        Self {
            flash,
            start,
            end,
            status: None,
        }
    }

    /// Everything `config` needs to write: the slots and, in swap-move
    /// mode, its status sectors.
    pub fn for_config(flash: F, config: &BootConfig<'_>) -> Self {
        #[cfg_attr(not(feature = "swap-move"), allow(unused_mut))]
        let mut view = Self::new(flash, &config.layout);
        #[cfg(feature = "swap-move")]
        if let crate::Mode::SwapMove { status } = config.mode {
            view.status = Some((status, status + 2 * config.layout.sector_size));
        }
        view
    }

    fn writable(&self, from: u32, len: usize) -> bool {
        let inside = |(start, end): (u32, u32)| {
            from >= start
                && u32::try_from(len)
                    .ok()
                    .and_then(|len| from.checked_add(len))
                    .is_some_and(|to| to <= end)
        };
        inside((self.start, self.end)) || self.status.is_some_and(inside)
    }

    pub fn into_inner(self) -> F {
        self.flash
    }
}

/// [`BootFlash`] error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootFlashError<E> {
    /// An erase or program outside the slots was refused.
    Protected,
    Flash(E),
}

impl<E: NorFlashError> NorFlashError for BootFlashError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Protected => NorFlashErrorKind::OutOfBounds,
            Self::Flash(error) => error.kind(),
        }
    }
}

impl<F: ErrorType> ErrorType for BootFlash<F> {
    type Error = BootFlashError<F::Error>;
}

impl<F: ReadNorFlash> ReadNorFlash for BootFlash<F> {
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash
            .read(offset, bytes)
            .map_err(BootFlashError::Flash)
    }

    fn capacity(&self) -> usize {
        self.flash.capacity()
    }
}

impl<F: NorFlash> NorFlash for BootFlash<F> {
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if to < from || !self.writable(from, (to - from) as usize) {
            return Err(BootFlashError::Protected);
        }
        self.flash.erase(from, to).map_err(BootFlashError::Flash)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if !self.writable(offset, bytes.len()) {
            return Err(BootFlashError::Protected);
        }
        self.flash
            .write(offset, bytes)
            .map_err(BootFlashError::Flash)
    }
}
