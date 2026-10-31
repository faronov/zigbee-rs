//! Product-owned ZBT-1 partition and generic security-journal wiring.
//!
//! The board supplies raw internal flash only. This product owns the protected
//! final 16 KiB of the 1 MiB flash and passes relative offsets for its two physical
//! 8 KiB erase sectors to the core-neutral [`SecurityStateJournal`].

use efr32s2_hal::flash::{Efr32s2Flash, FLASH_PAGE_SIZE, FlashError, MAX_FLASH_CAPACITY};
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SLOT_SIZE, SecurityStateJournal};

pub const FLASH_START: u32 = 0x0000_0000;
pub const FLASH_SIZE: usize = ha_connect_zbt1::INTERNAL_FLASH_CAPACITY;
pub const BOOTLOADER_SIZE: usize = 16 * 1024;
pub const APPLICATION_START: u32 = FLASH_START + BOOTLOADER_SIZE as u32;
pub const APPLICATION_END: u32 = 0x000F_C000;
pub const PERSISTENCE_PARTITION_START: u32 = APPLICATION_END;
pub const PERSISTENCE_PARTITION_SIZE: usize = 16 * 1024;
pub const PERSISTENCE_PARTITION_END: u32 =
    PERSISTENCE_PARTITION_START + PERSISTENCE_PARTITION_SIZE as u32;
pub const SECURITY_SECTOR_SIZE: usize = 8 * 1024;
pub const SECURITY_SLOT_SIZE: usize = SECURITY_JOURNAL_SLOT_SIZE;

const SECTOR_A: u32 = 0;
const SECTOR_B: u32 = SECURITY_SECTOR_SIZE as u32;

const _: () = assert!(FLASH_START == 0);
const _: () = assert!(FLASH_SIZE == 1024 * 1024);
const _: () = assert!(FLASH_SIZE <= MAX_FLASH_CAPACITY);
const _: () = assert!(BOOTLOADER_SIZE == 16 * 1024);
const _: () = assert!(APPLICATION_START as usize == BOOTLOADER_SIZE);
const _: () = assert!(APPLICATION_START == ha_connect_zbt1::BOOTLOADER_SIZE);
const _: () = assert!(APPLICATION_START < APPLICATION_END);
const _: () = assert!(APPLICATION_END == PERSISTENCE_PARTITION_START);
const _: () = assert!(PERSISTENCE_PARTITION_END as usize == FLASH_SIZE);
const _: () = assert!(PERSISTENCE_PARTITION_SIZE == SECURITY_SECTOR_SIZE * 2);
const _: () = assert!(SECURITY_SECTOR_SIZE == FLASH_PAGE_SIZE);
const _: () = assert!(SECURITY_SECTOR_SIZE.is_multiple_of(SECURITY_SLOT_SIZE));

/// Bounds every core-journal access to this product's protected final 16 KiB.
pub struct PartitionFlash {
    flash: Efr32s2Flash,
}

impl PartitionFlash {
    const fn new(flash: Efr32s2Flash) -> Self {
        Self { flash }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, FlashError> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= PERSISTENCE_PARTITION_SIZE)
            .ok_or(FlashError::OutOfBounds)?;
        PERSISTENCE_PARTITION_START
            .checked_add(offset)
            .ok_or(FlashError::OutOfBounds)
    }
}

impl ErrorType for PartitionFlash {
    type Error = FlashError;
}

impl ReadNorFlash for PartitionFlash {
    const READ_SIZE: usize = Efr32s2Flash::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.flash
            .read(Self::physical_offset(offset, bytes.len())?, bytes)
    }

    fn capacity(&self) -> usize {
        PERSISTENCE_PARTITION_SIZE
    }
}

impl NorFlash for PartitionFlash {
    const WRITE_SIZE: usize = Efr32s2Flash::WRITE_SIZE;
    const ERASE_SIZE: usize = Efr32s2Flash::ERASE_SIZE;

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
        self.flash
            .write(Self::physical_offset(offset, bytes.len())?, bytes)
    }
}

/// The generic core journal bound to this product's two 8 KiB sectors.
pub type SecurityStore = SecurityStateJournal<PartitionFlash, { SECURITY_SECTOR_SIZE }>;

/// Construct the product-selected generic journal from an exclusive raw board
/// flash token. Sector offsets are relative to `PartitionFlash`, never board
/// addresses.
pub(crate) fn security_journal(flash: Efr32s2Flash) -> SecurityStore {
    SecurityStore::new_with_sector_size(PartitionFlash::new(flash), SECTOR_A, SECTOR_B)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn partition_offsets_map_only_into_the_top_16_kib() {
        assert_eq!(PartitionFlash::physical_offset(0, 4), Ok(0x000F_C000));
        assert_eq!(
            PartitionFlash::physical_offset(SECTOR_B, 4),
            Ok(0x000F_E000)
        );
        assert_eq!(
            PartitionFlash::physical_offset((PERSISTENCE_PARTITION_SIZE - 4) as u32, 4),
            Ok(0x000F_FFFC)
        );
        assert_eq!(
            PartitionFlash::physical_offset((PERSISTENCE_PARTITION_SIZE - 3) as u32, 4),
            Err(FlashError::OutOfBounds)
        );
        assert_eq!(
            PartitionFlash::physical_offset(u32::MAX, 1),
            Err(FlashError::OutOfBounds)
        );
    }

    #[test]
    fn application_never_overlaps_the_bootloader_or_persistence() {
        assert_eq!(APPLICATION_START, 0x4000);
        assert_eq!(APPLICATION_END, PERSISTENCE_PARTITION_START);
        assert_eq!(PERSISTENCE_PARTITION_END as usize, 1024 * 1024);
    }
}
