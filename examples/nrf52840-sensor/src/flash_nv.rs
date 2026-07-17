//! nRF52840 NVMC adapter for the runtime security-state journal.
//!
//! The journal owns record layout, generation selection, CRC validation, and
//! atomic commit. This module only exposes the last two 4 KiB flash pages as
//! bounded read/program/erase operations.

use core::cell::RefCell;

use embassy_nrf::nvmc::{Nvmc, PAGE_SIZE};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use zigbee_runtime::security_journal::{SecurityJournalStorage, SECURITY_JOURNAL_SECTOR_SIZE};
use zigbee_runtime::security_store::SecurityStoreError;

/// Last two pages of the nRF52840's 1 MiB flash.
pub const SECURITY_PAGE_A: u32 = 0x000F_E000;
pub const SECURITY_PAGE_B: u32 = 0x000F_F000;

const SECURITY_FLASH_END: u32 = 0x0010_0000;
const NVMC_WRITE_SIZE: u32 = 4;

const _: () = assert!(PAGE_SIZE == SECURITY_JOURNAL_SECTOR_SIZE);
const _: () = assert!(SECURITY_PAGE_A + SECURITY_JOURNAL_SECTOR_SIZE as u32 == SECURITY_PAGE_B);
const _: () = assert!(SECURITY_PAGE_B + SECURITY_JOURNAL_SECTOR_SIZE as u32 == SECURITY_FLASH_END);

/// NVMC access restricted to the two pages reserved by `memory.x`.
///
/// `RefCell` is needed because Embassy's `ReadNorFlash` implementation takes
/// `&mut self`, while `SecurityJournalStorage::read` intentionally takes
/// `&self`.
pub struct Nrf52840SecurityFlash<'d> {
    nvmc: RefCell<Nvmc<'d>>,
}

impl<'d> Nrf52840SecurityFlash<'d> {
    pub fn new(nvmc: Nvmc<'d>) -> Self {
        Self {
            nvmc: RefCell::new(nvmc),
        }
    }

    fn checked_range(address: u32, length: usize) -> Result<(), SecurityStoreError> {
        let length = u32::try_from(length).map_err(|_| SecurityStoreError::Hardware)?;
        let end = address
            .checked_add(length)
            .ok_or(SecurityStoreError::Hardware)?;
        if address < SECURITY_PAGE_A || end > SECURITY_FLASH_END {
            return Err(SecurityStoreError::Hardware);
        }
        Ok(())
    }
}

impl SecurityJournalStorage for Nrf52840SecurityFlash<'_> {
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), SecurityStoreError> {
        Self::checked_range(address, output.len())?;
        self.nvmc
            .borrow_mut()
            .read(address, output)
            .map_err(|_| SecurityStoreError::Hardware)
    }

    fn program(&mut self, address: u32, data: &[u8]) -> Result<(), SecurityStoreError> {
        Self::checked_range(address, data.len())?;
        if address % NVMC_WRITE_SIZE != 0 || data.len() % NVMC_WRITE_SIZE as usize != 0 {
            return Err(SecurityStoreError::Hardware);
        }
        self.nvmc
            .borrow_mut()
            .write(address, data)
            .map_err(|_| SecurityStoreError::Hardware)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), SecurityStoreError> {
        if address != SECURITY_PAGE_A && address != SECURITY_PAGE_B {
            return Err(SecurityStoreError::Hardware);
        }
        self.nvmc
            .borrow_mut()
            .erase(address, address + PAGE_SIZE as u32)
            .map_err(|_| SecurityStoreError::Hardware)
    }
}
