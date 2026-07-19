//! TLSR8258 flash adapter for the shared crash-safe security journal.

use zigbee_runtime::security_journal::{SecurityJournalStorage, SecurityStateJournal};
use zigbee_runtime::security_store::SecurityStoreError;

const SECURITY_SECTOR_A: u32 = 0x0007_4000;
const SECURITY_SECTOR_B: u32 = 0x0007_5000;

pub struct Tlsr8258SecurityFlash;

impl SecurityJournalStorage for Tlsr8258SecurityFlash {
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), SecurityStoreError> {
        if tlsr8258_hal::flash::read_bytes(address, output) {
            Ok(())
        } else {
            Err(SecurityStoreError::Hardware)
        }
    }

    fn program(&mut self, address: u32, data: &[u8]) -> Result<(), SecurityStoreError> {
        tlsr8258_hal::flash::program(address, data).map_err(|_| SecurityStoreError::Hardware)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), SecurityStoreError> {
        tlsr8258_hal::flash::erase_sector(address).map_err(|_| SecurityStoreError::Hardware)
    }
}

pub const fn security_store() -> SecurityStateJournal<Tlsr8258SecurityFlash> {
    SecurityStateJournal::new(Tlsr8258SecurityFlash, SECURITY_SECTOR_A, SECURITY_SECTOR_B)
}
