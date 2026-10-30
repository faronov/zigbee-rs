//! Product-owned BL702 flash partitions on the XT-ZB1 OTA layout
//! ([`bl702_xt_zb1_layout`]).
//!
//! ```text
//! 0xFE000..0x100000 SecurityStateJournal: two 4-KiB sectors (NvWindow)
//! 0x87000..0xFE000  secondary slot: OTA download (OtaFlash may write here)
//! 0x00000..0x87000  boot header, bootloader and running image: read-only
//! ```
//!
//! Every runtime flash user sees the single [`XipFlash`] through a bounded
//! view: the journal through [`NvWindow`], the OTA writer through
//! [`OtaFlash`]. Neither can erase or write the boot header, the bootloader,
//! the running image or the other's partition. [`SharedFlash`] lets both
//! views share the one flash token; each operation borrows it only for its
//! own duration (the executor is single-threaded and the ROM flash routines
//! run with interrupts masked).
//!
//! The journal keeps its previous physical location (the last two sectors),
//! so a board upgraded from the bootloader-less layout keeps its network
//! state when the full image is reprogrammed without erasing `0xFE000`.

use core::cell::RefCell;
use core::fmt::Debug;

use bl702_hal::flash::{FlashError, XipFlash};
use bl702_hal::peripherals::Flash;
use bl702_xt_zb1_layout::{FLASH_SIZE, NV_SIZE, NV_START, SECONDARY_SLOT};
use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash, check_erase, check_read,
    check_write,
};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

pub const SECURITY_PARTITION_SIZE: usize = NV_SIZE as usize;
pub const SECURITY_PARTITION_START: u32 = NV_START;
const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

/// OTA erases and writes are confined to the secondary slot.
pub const OTA_WRITE_START: u32 = SECONDARY_SLOT;
pub const OTA_WRITE_END: u32 = NV_START;

const _: () = assert!(FLASH_SIZE as usize == bl702_xt_zb1::ONBOARD_FLASH_CAPACITY);
const _: () = assert!(SECURITY_PARTITION_SIZE == 2 * SECURITY_JOURNAL_SECTOR_SIZE);
const _: () = assert!(
    SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE
        == bl702_xt_zb1::ONBOARD_FLASH_CAPACITY
);
const _: () =
    assert!(SECURITY_JOURNAL_SECTOR_SIZE.is_multiple_of(<XipFlash as NorFlash>::ERASE_SIZE));

/// The one physical flash of the product, shared by the journal and OTA.
pub fn main_flash(token: Flash) -> Result<XipFlash, FlashError> {
    XipFlash::new(token, bl702_xt_zb1::ONBOARD_FLASH_CAPACITY)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PartitionError<E> {
    OutOfBounds,
    NotAligned,
    Backend(E),
}

impl<E: NorFlashError> NorFlashError for PartitionError<E> {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
            Self::NotAligned => NorFlashErrorKind::NotAligned,
            Self::Backend(error) => error.kind(),
        }
    }
}

const fn map_layout_error<E>(error: NorFlashErrorKind) -> PartitionError<E> {
    match error {
        NorFlashErrorKind::NotAligned => PartitionError::NotAligned,
        _ => PartitionError::OutOfBounds,
    }
}

/// Bounded view `[START, START + SIZE)` of the main flash with offsets
/// relative to `START`. Backend failures are logged with their physical
/// address before the journal collapses them into a hardware error.
pub struct FlashWindow<F, const START: u32, const SIZE: usize> {
    flash: F,
}

impl<F, const START: u32, const SIZE: usize> FlashWindow<F, START, SIZE> {
    pub const fn new(flash: F) -> Self {
        Self { flash }
    }

    pub fn into_inner(self) -> F {
        self.flash
    }

    /// Translate a window range to its physical start address.
    pub const fn physical_offset(offset: u32, length: usize) -> Option<u32> {
        let Some(end) = (offset as usize).checked_add(length) else {
            return None;
        };
        if end > SIZE {
            return None;
        }
        START.checked_add(offset)
    }
}

impl<F: ErrorType, const START: u32, const SIZE: usize> ErrorType for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError,
{
    type Error = PartitionError<F::Error>;
}

impl<F: ReadNorFlash, const START: u32, const SIZE: usize> ReadNorFlash
    for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError + Debug,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(map_layout_error)?;
        let physical =
            Self::physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .read(physical, bytes)
            .inspect_err(|error| report("read", physical, bytes.len(), error))
            .map_err(PartitionError::Backend)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<F: NorFlash, const START: u32, const SIZE: usize> NorFlash for FlashWindow<F, START, SIZE>
where
    F::Error: NorFlashError + Debug,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(map_layout_error)?;
        let length = usize::try_from(to - from).map_err(|_| PartitionError::OutOfBounds)?;
        let physical_from =
            Self::physical_offset(from, length).ok_or(PartitionError::OutOfBounds)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .erase(physical_from, physical_to)
            .inspect_err(|error| report("erase", physical_from, length, error))
            .map_err(PartitionError::Backend)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(map_layout_error)?;
        let physical =
            Self::physical_offset(offset, bytes.len()).ok_or(PartitionError::OutOfBounds)?;
        self.flash
            .write(physical, bytes)
            .inspect_err(|error| report("write", physical, bytes.len(), error))
            .map_err(PartitionError::Backend)
    }
}

/// Name the failing physical flash operation. Compiled out when the final
/// image disables logging.
#[cold]
fn report(operation: &str, physical: u32, length: usize, error: &impl Debug) {
    log::warn!("security flash {operation} failed: addr=0x{physical:05x} len={length} {error:?}");
}

/// A [`NorFlash`] shared through a [`RefCell`]: every operation borrows
/// the flash only for its own duration, so several bounded views can use
/// one flash token in a single-threaded executor.
pub struct SharedFlash<'a, F> {
    flash: &'a RefCell<F>,
}

impl<'a, F> SharedFlash<'a, F> {
    pub const fn new(flash: &'a RefCell<F>) -> Self {
        Self { flash }
    }
}

impl<F: ErrorType> ErrorType for SharedFlash<'_, F> {
    type Error = F::Error;
}

impl<F: ReadNorFlash> ReadNorFlash for SharedFlash<'_, F> {
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash.borrow_mut().read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.flash.borrow().capacity()
    }
}

impl<F: NorFlash> NorFlash for SharedFlash<'_, F> {
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.flash.borrow_mut().erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.flash.borrow_mut().write(offset, bytes)
    }
}

/// The OTA writer's view, with absolute physical addresses: reads anywhere
/// below the journal (downgrade prevention reads the running image's
/// header, the channel check the LoaderInfo record), erases and writes only
/// inside the secondary slot.
pub struct OtaFlash<F> {
    flash: F,
}

impl<F> OtaFlash<F> {
    pub const fn new(flash: F) -> Self {
        Self { flash }
    }

    pub fn into_inner(self) -> F {
        self.flash
    }

    const fn writable(from: u32, to: u32) -> bool {
        OTA_WRITE_START <= from && from <= to && to <= OTA_WRITE_END
    }
}

impl<F: ErrorType> ErrorType for OtaFlash<F>
where
    F::Error: NorFlashError,
{
    type Error = PartitionError<F::Error>;
}

impl<F: ReadNorFlash> ReadNorFlash for OtaFlash<F>
where
    F::Error: NorFlashError,
{
    const READ_SIZE: usize = F::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        check_read(self, offset, bytes.len()).map_err(map_layout_error)?;
        self.flash
            .read(offset, bytes)
            .map_err(PartitionError::Backend)
    }

    fn capacity(&self) -> usize {
        OTA_WRITE_END as usize
    }
}

impl<F: NorFlash> NorFlash for OtaFlash<F>
where
    F::Error: NorFlashError,
{
    const WRITE_SIZE: usize = F::WRITE_SIZE;
    const ERASE_SIZE: usize = F::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        check_erase(self, from, to).map_err(map_layout_error)?;
        if !Self::writable(from, to) {
            return Err(PartitionError::OutOfBounds);
        }
        self.flash.erase(from, to).map_err(PartitionError::Backend)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        check_write(self, offset, bytes.len()).map_err(map_layout_error)?;
        let end = offset
            .checked_add(bytes.len() as u32)
            .ok_or(PartitionError::OutOfBounds)?;
        if !Self::writable(offset, end) {
            return Err(PartitionError::OutOfBounds);
        }
        self.flash
            .write(offset, bytes)
            .map_err(PartitionError::Backend)
    }
}

pub type NvWindow<F> = FlashWindow<F, SECURITY_PARTITION_START, SECURITY_PARTITION_SIZE>;
pub type SecurityStore<F> = SecurityStateJournal<NvWindow<F>>;

/// The Zigbee security-state journal on its partition of `flash`.
pub fn security_store<F: NorFlash>(flash: F) -> SecurityStore<F>
where
    F::Error: NorFlashError + Debug,
{
    SecurityStateJournal::new(NvWindow::new(flash), SECURITY_SECTOR_A, SECURITY_SECTOR_B)
}

#[cfg(test)]
pub(crate) mod mock {
    extern crate std;

    use std::vec::Vec;

    use super::*;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct MockError;

    impl NorFlashError for MockError {
        fn kind(&self) -> NorFlashErrorKind {
            NorFlashErrorKind::Other
        }
    }

    /// The whole 1 MiB flash, NOR semantics, 4 KiB erase.
    pub struct MockMainFlash {
        pub bytes: Vec<u8>,
    }

    impl MockMainFlash {
        pub fn new() -> Self {
            Self {
                bytes: std::vec![0xFF; FLASH_SIZE as usize],
            }
        }
    }

    impl ErrorType for MockMainFlash {
        type Error = MockError;
    }

    impl ReadNorFlash for MockMainFlash {
        const READ_SIZE: usize = 1;

        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), MockError> {
            let start = offset as usize;
            let source = self
                .bytes
                .get(start..start + bytes.len())
                .ok_or(MockError)?;
            bytes.copy_from_slice(source);
            Ok(())
        }

        fn capacity(&self) -> usize {
            self.bytes.len()
        }
    }

    impl NorFlash for MockMainFlash {
        const WRITE_SIZE: usize = 1;
        const ERASE_SIZE: usize = 4096;

        fn erase(&mut self, from: u32, to: u32) -> Result<(), MockError> {
            check_erase(self, from, to).map_err(|_| MockError)?;
            self.bytes[from as usize..to as usize].fill(0xFF);
            Ok(())
        }

        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), MockError> {
            check_write(self, offset, bytes.len()).map_err(|_| MockError)?;
            let start = offset as usize;
            for (cell, byte) in self.bytes[start..start + bytes.len()].iter_mut().zip(bytes) {
                *cell &= *byte;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::mock::MockMainFlash;
    use super::*;
    use bl702_xt_zb1_layout::{LOADER_INFO_ADDR, PRIMARY_SLOT, SLOT_SIZE};
    use zigbee_runtime::security_store::SecurityStateStore;

    #[test]
    fn security_partition_keeps_the_last_two_flash_sectors() {
        assert_eq!(SECURITY_PARTITION_START, 0x000f_e000);
        assert_eq!(SECURITY_PARTITION_SIZE, 8192);
        assert_eq!(
            NvWindow::<MockMainFlash>::physical_offset(0, SECURITY_PARTITION_SIZE),
            Some(SECURITY_PARTITION_START)
        );
        assert_eq!(
            NvWindow::<MockMainFlash>::physical_offset(SECURITY_PARTITION_SIZE as u32, 1),
            None
        );
    }

    #[test]
    fn ota_view_writes_only_the_secondary_slot() {
        let flash = RefCell::new(MockMainFlash::new());
        let mut ota = OtaFlash::new(SharedFlash::new(&flash));
        for (from, to) in [
            (0, 0x1000),
            (LOADER_INFO_ADDR & !0xFFF, PRIMARY_SLOT),
            (PRIMARY_SLOT, PRIMARY_SLOT + 0x1000),
            (SECONDARY_SLOT - 0x1000, SECONDARY_SLOT + 0x1000),
            (NV_START - 0x1000, NV_START + 0x1000),
            (NV_START, FLASH_SIZE),
        ] {
            assert!(ota.erase(from, to).is_err(), "{from:#x}..{to:#x}");
        }
        assert!(ota.write(NV_START - 1, &[0, 0]).is_err());
        assert!(ota.write(PRIMARY_SLOT, &[0]).is_err());
        assert!(
            ota.erase(SECONDARY_SLOT, SECONDARY_SLOT + SLOT_SIZE)
                .is_ok()
        );
        assert!(ota.write(NV_START - 1, &[0]).is_ok());
        let mut byte = [0];
        assert!(ota.read(LOADER_INFO_ADDR, &mut byte).is_ok());
        assert!(ota.read(NV_START, &mut byte).is_err());
        assert_eq!(flash.borrow().bytes[NV_START as usize - 1], 0);
    }

    #[test]
    fn journal_and_ota_share_one_flash_without_overlap() {
        let flash = RefCell::new(MockMainFlash::new());
        let mut store = security_store(SharedFlash::new(&flash));
        let mut ota = OtaFlash::new(SharedFlash::new(&flash));
        // Fill the whole secondary slot with zeros, then use the journal.
        ota.erase(SECONDARY_SLOT, NV_START).unwrap();
        let zeros = [0u8; 4096];
        let mut address = SECONDARY_SLOT;
        while address < NV_START {
            ota.write(address, &zeros).unwrap();
            address += 4096;
        }
        assert_eq!(store.load().unwrap(), None);
        let below = &flash.borrow().bytes[..SECONDARY_SLOT as usize];
        assert!(below.iter().all(|byte| *byte == 0xFF));
        let nv = &flash.borrow().bytes[NV_START as usize..];
        assert!(nv.iter().all(|byte| *byte == 0xFF));
    }
}
