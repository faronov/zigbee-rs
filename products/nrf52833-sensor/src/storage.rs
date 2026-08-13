//! Product-owned nRF52833 flash partitions and Zigbee security persistence
//! wiring.
//!
//! Same crash-safe two-sector `SecurityStateJournal` mechanism as
//! `products/nrf52840-sensor`, relocated for the nRF52833's 512 KiB flash:
//! the journal owns the last 8 KiB (pages 126-127, `0x0007_E000..0x0007_FFFF`)
//! and the application is limited to the 504 KiB below it by
//! `link/memory.x`. The `const` assertions below fail the build if the
//! partition ever stops being the top of flash or stops matching the NVMC
//! page size, which is what keeps a linker-script edit from silently
//! overwriting the frame counters.
//!
//! Target-gated: `embassy_nrf::nvmc` only builds for the real Cortex-M
//! target (see the dependency comment in `Cargo.toml`).

#![cfg(target_os = "none")]

use embassy_nrf::nvmc::{Error, Nvmc, PAGE_SIZE};
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

/// nRF52833 (xxAA) internal flash: 512 KiB.
const FLASH_CAPACITY: usize = 512 * 1024;
/// Start of the crash-safe security journal partition: the last 8 KiB of
/// the 512 KiB flash (pages 126-127), reserved in `link/memory.x`.
pub const SECURITY_PARTITION_START: u32 = 0x0007_E000;
pub const SECURITY_PARTITION_SIZE: usize = SECURITY_JOURNAL_SECTOR_SIZE * 2;
const SECURITY_SECTOR_A: u32 = 0;
const SECURITY_SECTOR_B: u32 = SECURITY_JOURNAL_SECTOR_SIZE as u32;

const _: () = assert!(PAGE_SIZE == SECURITY_JOURNAL_SECTOR_SIZE);
const _: () =
    assert!(SECURITY_PARTITION_START as usize + SECURITY_PARTITION_SIZE == FLASH_CAPACITY);

pub struct SecurityFlash<'d> {
    nvmc: Nvmc<'d>,
}

impl<'d> SecurityFlash<'d> {
    pub fn new(nvmc: Nvmc<'d>) -> Self {
        Self { nvmc }
    }

    fn physical_offset(offset: u32, length: usize) -> Result<u32, Error> {
        (offset as usize)
            .checked_add(length)
            .filter(|end| *end <= SECURITY_PARTITION_SIZE)
            .ok_or(Error::OutOfBounds)?;
        SECURITY_PARTITION_START
            .checked_add(offset)
            .ok_or(Error::OutOfBounds)
    }
}

impl ErrorType for SecurityFlash<'_> {
    type Error = Error;
}

impl ReadNorFlash for SecurityFlash<'_> {
    const READ_SIZE: usize = Nvmc::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        let physical = Self::physical_offset(offset, bytes.len())?;
        self.nvmc.read(physical, bytes)
    }

    fn capacity(&self) -> usize {
        SECURITY_PARTITION_SIZE
    }
}

impl NorFlash for SecurityFlash<'_> {
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

pub type SecurityStore<'d> = SecurityStateJournal<SecurityFlash<'d>>;

pub fn security_store(nvmc: Nvmc<'_>) -> SecurityStore<'_> {
    SecurityStateJournal::new(
        SecurityFlash::new(nvmc),
        SECURITY_SECTOR_A,
        SECURITY_SECTOR_B,
    )
}
