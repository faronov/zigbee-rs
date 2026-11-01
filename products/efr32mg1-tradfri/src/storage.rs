//! Product-owned internal-flash partitions and Zigbee persistence wiring.

use efr32mg1_hal::flash::{Efr32mg1Flash, FlashError};
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use zigbee_runtime::log_nv::LogStructuredNv;
use zigbee_runtime::nv_storage::NvError;
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

pub const SECURITY_PARTITION_START: u32 = 0x0003_7000;
pub const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
pub const APP_NV_PARTITION_START: u32 = 0x0003_9000;
pub const APP_NV_PARTITION_SIZE: usize = 4096;
const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;
const NV_PAGE_A: u32 = 0;
const NV_PAGE_B: u32 = 2048;

const _: () = assert!(
    SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE == APP_NV_PARTITION_START as usize
);
const _: () = assert!(APP_NV_PARTITION_START as usize + APP_NV_PARTITION_SIZE == 0x0003_A000);
const _: () =
    assert!(SECURITY_JOURNAL_SECTOR_SIZE.is_multiple_of(<Efr32mg1Flash as NorFlash>::ERASE_SIZE));

// Both partitions sit after the application image limit and before the
// preserved native NVM3 region, and never touch the bootloader.
const _: () = assert!(SECURITY_PARTITION_START >= 0x0000_4000);
const _: () = assert!(APP_NV_PARTITION_START as usize + APP_NV_PARTITION_SIZE <= 0x0003_A000);

/// Exclusive ownership of the security-journal partition.
pub struct SecurityPartition(());
/// Exclusive ownership of the application-NV partition.
pub struct ApplicationNvPartition(());

/// Consume the chip's unique internal-flash handle and split it into the
/// product's disjoint persistence partitions.
///
/// After this, no safe code can reach the bootloader, application or
/// preserved NVM3 pages through the MSC.
pub fn split_flash(_flash: Efr32mg1Flash) -> (SecurityPartition, ApplicationNvPartition) {
    (SecurityPartition(()), ApplicationNvPartition(()))
}

/// Partition-bounded MSC view `[START, START + SIZE)`.
pub struct PartitionFlash<const START: u32, const SIZE: usize> {
    flash: Efr32mg1Flash,
}

impl<const START: u32, const SIZE: usize> PartitionFlash<START, SIZE> {
    /// # Safety
    ///
    /// The caller must hold the unique partition token for `[START,
    /// START + SIZE)` (minted only by [`split_flash`] or, for the
    /// router-only journals, by [`crate::router::router_stores`]).
    pub(crate) const unsafe fn new() -> Self {
        Self {
            // SAFETY: the internal-flash handle was consumed by
            // `split_flash`; this view only issues MSC operations inside its
            // own token-owned partition (checked by `physical_offset`).
            flash: unsafe { Efr32mg1Flash::new() },
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

impl<const START: u32, const SIZE: usize> ErrorType for PartitionFlash<START, SIZE> {
    type Error = FlashError;
}

impl<const START: u32, const SIZE: usize> ReadNorFlash for PartitionFlash<START, SIZE> {
    const READ_SIZE: usize = Efr32mg1Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SIZE
    }
}

impl<const START: u32, const SIZE: usize> NorFlash for PartitionFlash<START, SIZE> {
    const WRITE_SIZE: usize = Efr32mg1Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Efr32mg1Flash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::OutOfBounds);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashError::OutOfBounds)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashError::OutOfBounds)?;
        self.flash.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.write(physical, bytes)
    }
}

pub type SecurityFlash = PartitionFlash<SECURITY_PARTITION_START, SECURITY_PARTITION_SIZE>;
pub type SecurityStore = SecurityStateJournal<SecurityFlash>;
pub type ApplicationFlash = PartitionFlash<APP_NV_PARTITION_START, APP_NV_PARTITION_SIZE>;
pub type ApplicationNv = LogStructuredNv<ApplicationFlash>;

pub fn security_store(_partition: SecurityPartition) -> SecurityStore {
    // SAFETY: `SecurityPartition` is unique and owns this window.
    let flash = unsafe { SecurityFlash::new() };
    SecurityStateJournal::new(flash, SECURITY_SECTOR_A, SECURITY_SECTOR_B)
}

pub fn application_nv(_partition: ApplicationNvPartition) -> Result<ApplicationNv, NvError> {
    // SAFETY: `ApplicationNvPartition` is unique and owns this window.
    let flash = unsafe { ApplicationFlash::new() };
    LogStructuredNv::new(flash, NV_PAGE_A, NV_PAGE_B)
}
