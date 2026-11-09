//! Application-side pieces shared by every product that downloads images
//! for a zigbee-rs loader: which loader is installed ([`LoaderChannel`]),
//! the keys both sides trust ([`shared_keys`]) and the bounded flash view
//! of the OTA writer ([`StagingFlash`]).
//!
//! A product combines them with [`SlotWriter`](crate::SlotWriter) (feature
//! `firmware-writer`): the writer only accepts images signed by a key the
//! installed loader also trusts, and its flash view cannot touch anything
//! but the secondary slot and the running image's trial log.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash, check_erase, check_read,
    check_write,
};

use crate::boot::{Layout, trial_log_area};
use crate::image::{KeySet, TrustedKey};
use crate::loader_info::TrustedIds;

/// Which loader starts the application, and so whether downloaded images
/// can be installed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoaderChannel {
    /// A zigbee-rs loader trusting at least one of the application's keys.
    Own,
    /// A zigbee-rs loader trusting none of them: staged images would be
    /// rejected at boot.
    Foreign,
    /// No loader record (or an unreadable one): nothing is known to install
    /// images.
    NoLoader,
}

impl LoaderChannel {
    /// Classify the loader records `ids` against the application's
    /// `trusted` keys.
    pub fn detect(ids: Option<&TrustedIds>, trusted: &'static [TrustedKey]) -> Self {
        match ids {
            None => Self::NoLoader,
            Some(_) if shared_keys(ids, trusted).is_empty() => Self::Foreign,
            Some(_) => Self::Own,
        }
    }

    /// Read the loader records at `info_addr` (the `LoaderInfo` record; the
    /// `LoaderKeys` record precedes it). A read error disables OTA rather
    /// than guessing.
    pub fn read<F: ReadNorFlash>(
        flash: &mut F,
        info_addr: u32,
        trusted: &'static [TrustedKey],
    ) -> Self {
        Self::detect(loader_ids(flash, info_addr).as_ref(), trusted)
    }

    pub const fn accepts_images(self) -> bool {
        matches!(self, Self::Own)
    }
}

/// The loader's key records at `info_addr`; `None` without a valid record
/// or on a read error.
pub fn loader_ids<F: ReadNorFlash>(flash: &mut F, info_addr: u32) -> Option<TrustedIds> {
    TrustedIds::read(flash, info_addr).ok().flatten()
}

/// The keys of `trusted` that the loader described by `ids` also trusts
/// (none without a loader record).
pub fn shared_keys(ids: Option<&TrustedIds>, trusted: &'static [TrustedKey]) -> KeySet {
    let mut keys = KeySet::new(trusted);
    keys.retain(|key| ids.is_some_and(|ids| ids.trusts(key)));
    keys
}

/// Error of a [`StagingFlash`] operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StagingError<E> {
    /// Outside the range the view admits for this operation.
    OutOfBounds,
    NotAligned,
    Backend(E),
}

impl<E: NorFlashError> NorFlashError for StagingError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::Backend(error) => error.kind(),
        }
    }
}

const fn layout_error<E>(error: NorFlashErrorKind) -> StagingError<E> {
    match error {
        NorFlashErrorKind::NotAligned => StagingError::NotAligned,
        _ => StagingError::OutOfBounds,
    }
}

/// The OTA writer's view of the device flash (absolute addresses) for an
/// overwrite-mode layout:
///
/// - reads below the end of the higher slot (the loader records, the
///   running image's header for downgrade prevention, the staged image);
/// - erases and writes inside the secondary slot;
/// - writes — never erases — inside the primary slot's trial log
///   ([`trial_log_area`]), where the running image confirms itself.
///
/// Everything else (the loader, the running image, persistence partitions
/// above the slots) is refused with [`StagingError::OutOfBounds`] whatever
/// the writer asks for.
pub struct StagingFlash<F> {
    flash: F,
    layout: Layout,
}

impl<F> StagingFlash<F> {
    pub const fn new(flash: F, layout: Layout) -> Self {
        Self { flash, layout }
    }

    pub fn into_inner(self) -> F {
        self.flash
    }

    const fn read_end(&self) -> u32 {
        let primary_end = self.layout.primary + self.layout.slot_size;
        let secondary_end = self.layout.secondary + self.layout.slot_size;
        if primary_end > secondary_end {
            primary_end
        } else {
            secondary_end
        }
    }

    const fn in_secondary(&self, from: u32, to: u32) -> bool {
        self.layout.secondary <= from
            && from <= to
            && to <= self.layout.secondary + self.layout.slot_size
    }

    const fn in_trial_log(&self, from: u32, to: u32) -> bool {
        let (start, end) = trial_log_area(&self.layout);
        start <= from && from <= to && to <= end
    }
}

impl<F: ErrorType> ErrorType for StagingFlash<F>
where
    F::Error: NorFlashError,
{
    type Error = StagingError<F::Error>;
}

impl<F: ReadNorFlash> ReadNorFlash for StagingFlash<F>
where
    F::Error: NorFlashError,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(layout_error)?;
        self.flash
            .read(offset, bytes)
            .map_err(StagingError::Backend)
    }

    fn capacity(&self) -> usize {
        self.read_end() as usize
    }
}

impl<F: NorFlash> NorFlash for StagingFlash<F>
where
    F::Error: NorFlashError,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(layout_error)?;
        if !self.in_secondary(from, to) {
            return Err(StagingError::OutOfBounds);
        }
        self.flash.erase(from, to).map_err(StagingError::Backend)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(layout_error)?;
        let end = offset
            .checked_add(bytes.len() as u32)
            .ok_or(StagingError::OutOfBounds)?;
        if !self.in_secondary(offset, end) && !self.in_trial_log(offset, end) {
            return Err(StagingError::OutOfBounds);
        }
        self.flash
            .write(offset, bytes)
            .map_err(StagingError::Backend)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::loader_info::{LoaderInfo, loader_block};

    const LAYOUT: Layout = Layout {
        primary: 0x4000,
        secondary: 0x40000,
        slot_size: 0x3C000,
        sector_size: 0x2000,
    };
    const FLASH: usize = 0x80000;
    const INFO: u32 = 0x3FE0;

    struct Mem(std::vec::Vec<u8>);

    impl Mem {
        fn new() -> Self {
            Self(std::vec![0xFF; FLASH])
        }
    }

    impl ErrorType for Mem {
        type Error = NorFlashErrorKind;
    }

    impl ReadNorFlash for Mem {
        const READ_SIZE: usize = 1;
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            bytes.copy_from_slice(&self.0[start..start + bytes.len()]);
            Ok(())
        }
        fn capacity(&self) -> usize {
            FLASH
        }
    }

    impl NorFlash for Mem {
        const WRITE_SIZE: usize = 4;
        const ERASE_SIZE: usize = 0x2000;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
            self.0[from as usize..to as usize].fill(0xFF);
            Ok(())
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            for (old, new) in self.0[start..start + bytes.len()].iter_mut().zip(bytes) {
                *old &= *new;
            }
            Ok(())
        }
    }

    static OURS: [TrustedKey; 2] = [TrustedKey::new([1; 64]), TrustedKey::new([2; 64])];
    static OTHER: [TrustedKey; 1] = [TrustedKey::new([3; 64])];

    #[test]
    fn channel_follows_the_loader_records() {
        let mut mem = Mem::new();
        assert_eq!(
            LoaderChannel::read(&mut mem, INFO, &OURS),
            LoaderChannel::NoLoader
        );
        let at = INFO as usize;
        mem.0[at..at + 32].copy_from_slice(&LoaderInfo::for_keys(1, &OURS[1..]).encode());
        assert_eq!(
            LoaderChannel::read(&mut mem, INFO, &OURS),
            LoaderChannel::Own
        );
        let ids = loader_ids(&mut mem, INFO);
        assert!(shared_keys(ids.as_ref(), &OURS).iter().eq(&OURS[1..]));
        mem.0[at..at + 32].copy_from_slice(&LoaderInfo::for_keys(1, &OTHER).encode());
        assert_eq!(
            LoaderChannel::read(&mut mem, INFO, &OURS),
            LoaderChannel::Foreign
        );
        mem.0[at + 20] ^= 1;
        assert_eq!(
            LoaderChannel::read(&mut mem, INFO, &OURS),
            LoaderChannel::NoLoader
        );
        assert!(shared_keys(None, &OURS).is_empty());
        assert!(LoaderChannel::Own.accepts_images());
        assert!(!LoaderChannel::Foreign.accepts_images());
        assert!(!LoaderChannel::NoLoader.accepts_images());
    }

    /// Key rotation: an extra key in the `LoaderKeys` record counts.
    #[test]
    fn extra_loader_keys_count() {
        let mut mem = Mem::new();
        let keys = [OTHER[0], OURS[0]];
        let block = INFO as usize + 32 - 64;
        mem.0[block..block + 64].copy_from_slice(&loader_block(2, &keys));
        assert_eq!(
            LoaderChannel::read(&mut mem, INFO, &OURS),
            LoaderChannel::Own
        );
        let ids = loader_ids(&mut mem, INFO);
        assert!(shared_keys(ids.as_ref(), &OURS).iter().eq(&OURS[..1]));
    }

    #[test]
    fn view_admits_only_the_secondary_and_the_trial_log() {
        let mut view = StagingFlash::new(Mem::new(), LAYOUT);
        let mut byte = [0u8; 4];
        // Reads: loader records and both slots, nothing above them.
        view.read(INFO, &mut byte).unwrap();
        view.read(0x7BFFC, &mut byte).unwrap();
        assert_eq!(
            view.read(0x7C000, &mut byte),
            Err(StagingError::OutOfBounds)
        );
        // Secondary: erase and write.
        view.erase(0x40000, 0x42000).unwrap();
        view.erase(0x7A000, 0x7C000).unwrap();
        view.write(0x40000, &[0; 4]).unwrap();
        view.write(0x7BFF0, &[0; 4]).unwrap();
        // Trial log: write only.
        let (log, log_end) = trial_log_area(&LAYOUT);
        view.write(log, &[0; 4]).unwrap();
        view.write(log_end - 4, &[0; 4]).unwrap();
        assert_eq!(view.erase(0x3E000, 0x40000), Err(StagingError::OutOfBounds));
        assert_eq!(view.write(log_end, &[0; 4]), Err(StagingError::OutOfBounds));
        // Loader, running image, journal.
        for address in [0, INFO, 0x4000, 0x20000, log - 4] {
            assert_eq!(view.write(address, &[0; 4]), Err(StagingError::OutOfBounds));
        }
        assert_eq!(view.erase(0x0, 0x2000), Err(StagingError::OutOfBounds));
        assert_eq!(view.erase(0x4000, 0x6000), Err(StagingError::OutOfBounds));
        assert_eq!(view.erase(0x3E000, 0x42000), Err(StagingError::OutOfBounds));
        assert_eq!(view.erase(0x7C000, 0x7E000), Err(StagingError::OutOfBounds));
        assert_eq!(view.write(0x7C000, &[0; 4]), Err(StagingError::OutOfBounds));
        assert_eq!(view.write(0x40001, &[0; 4]), Err(StagingError::NotAligned));
        let mem = view.into_inner();
        assert!(mem.0[..log as usize].iter().all(|byte| *byte == 0xFF));
        assert!(mem.0[0x7C000..].iter().all(|byte| *byte == 0xFF));
    }
}
