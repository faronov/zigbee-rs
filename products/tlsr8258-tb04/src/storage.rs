//! Product-owned TLSR8258 flash partitions and Zigbee durable journals.
//!
//! Two independent two-sector journals live here, each behind its own
//! partition-bounded [`NorFlash`] view so neither can address the other's
//! sectors:
//!
//! - the **security journal** (network state, keys, frame counters), rewritten
//!   on every frame-counter reservation, and
//! - the **child-table journal** (router/coordinator child records), rewritten
//!   only on a child lifecycle transition.

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use tlsr8258_hal::flash::{FlashError, Tlsr8258Flash};
use tlsr8258_tb04::resources::OnboardFlash;
use zigbee_runtime::child_store::{CHILD_JOURNAL_SECTOR_SIZE, ChildTableJournal};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

use crate::{
    CHILD_TABLE_PARTITION_SIZE, CHILD_TABLE_PARTITION_START, FLASH_CAPACITY,
    SECURITY_PARTITION_SIZE, SECURITY_PARTITION_START,
};

const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;
const CHILD_SECTOR_A: u32 = 0;
const CHILD_SECTOR_B: u32 = CHILD_JOURNAL_SECTOR_SIZE as u32;

const _: () = assert!(SECURITY_PARTITION_SIZE == SECURITY_JOURNAL_SECTOR_SIZE * 2);
const _: () = assert!(CHILD_TABLE_PARTITION_SIZE == CHILD_JOURNAL_SECTOR_SIZE * 2);

/// Exclusive ownership of the security journal partition.
pub struct SecurityPartition(());
/// Exclusive ownership of the child-table journal partition.
pub struct ChildTablePartition(());

/// Split the board's single onboard-flash token into the product's disjoint
/// NV partitions.
///
/// The board crate rightly owns *one* physical flash device; how that device
/// is divided is product policy. Consuming the board token here and handing
/// back one zero-sized token per partition means the two journals cannot be
/// constructed twice or aliased, without the board knowing anything about
/// Zigbee persistence.
pub const fn split_flash(_token: OnboardFlash) -> (SecurityPartition, ChildTablePartition) {
    (SecurityPartition(()), ChildTablePartition(()))
}

pub struct SecurityFlash {
    flash: Tlsr8258Flash,
}

impl SecurityFlash {
    const fn new(_token: SecurityPartition) -> Self {
        Self {
            flash: Tlsr8258Flash::new(FLASH_CAPACITY),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SECURITY_PARTITION_SIZE)
            .ok_or(FlashError::AddressOverflow)?;
        SECURITY_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashError::AddressOverflow)
    }
}

impl ErrorType for SecurityFlash {
    type Error = FlashError;
}

impl ReadNorFlash for SecurityFlash {
    const READ_SIZE: usize = Tlsr8258Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SECURITY_PARTITION_SIZE
    }
}

impl NorFlash for SecurityFlash {
    const WRITE_SIZE: usize = Tlsr8258Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Tlsr8258Flash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::AddressOverflow);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashError::AddressOverflow)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashError::AddressOverflow)?;
        self.flash.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.write(physical, bytes)
    }
}

pub type SecurityStore = SecurityStateJournal<SecurityFlash>;

pub const fn security_store(token: SecurityPartition) -> SecurityStore {
    SecurityStateJournal::new(
        SecurityFlash::new(token),
        SECURITY_SECTOR_A,
        SECURITY_SECTOR_B,
    )
}

/// Partition-bounded view of the child-table journal region.
///
/// Structurally identical to [`SecurityFlash`] but clamped to the child-table
/// partition, so a bug in one journal cannot reach the other's sectors.
pub struct ChildTableFlash {
    flash: Tlsr8258Flash,
}

impl ChildTableFlash {
    const fn new(_token: ChildTablePartition) -> Self {
        Self {
            flash: Tlsr8258Flash::new(FLASH_CAPACITY),
        }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= CHILD_TABLE_PARTITION_SIZE)
            .ok_or(FlashError::AddressOverflow)?;
        CHILD_TABLE_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashError::AddressOverflow)
    }
}

impl ErrorType for ChildTableFlash {
    type Error = FlashError;
}

impl ReadNorFlash for ChildTableFlash {
    const READ_SIZE: usize = Tlsr8258Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        CHILD_TABLE_PARTITION_SIZE
    }
}

impl NorFlash for ChildTableFlash {
    const WRITE_SIZE: usize = Tlsr8258Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Tlsr8258Flash::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        if from >= to {
            return Err(FlashError::AddressOverflow);
        }
        let length = usize::try_from(to - from).map_err(|_| FlashError::AddressOverflow)?;
        let physical_from = Self::physical_offset(from, length)?;
        let physical_to = physical_from
            .checked_add(to - from)
            .ok_or(FlashError::AddressOverflow)?;
        self.flash.erase(physical_from, physical_to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.flash.write(physical, bytes)
    }
}

pub type ChildStore = ChildTableJournal<ChildTableFlash>;

/// Durable child-table store for a router/coordinator build.
///
/// A sensor product never constructs this, so the child-table journal code is
/// dead-code-eliminated from the sensor image.
pub const fn child_table_store(token: ChildTablePartition) -> ChildStore {
    ChildTableJournal::new(ChildTableFlash::new(token), CHILD_SECTOR_A, CHILD_SECTOR_B)
}
