//! Crash-safe Zigbee security journal for PHY62x2 flash.

use zigbee_runtime::security_journal::{SecurityJournalStorage, SecurityStateJournal};
use zigbee_runtime::security_store::SecurityStoreError;

#[cfg(all(feature = "phy6222", feature = "phy6252"))]
compile_error!("select exactly one of the phy6222 or phy6252 features");
#[cfg(not(any(feature = "phy6222", feature = "phy6252")))]
compile_error!("select exactly one of the phy6222 or phy6252 features");

#[cfg(feature = "phy6222")]
const SECURITY_SECTOR_A: u32 = 0x0007_e000;
#[cfg(feature = "phy6222")]
const SECURITY_SECTOR_B: u32 = 0x0007_f000;

#[cfg(feature = "phy6252")]
const SECURITY_SECTOR_A: u32 = 0x0003_e000;
#[cfg(feature = "phy6252")]
const SECURITY_SECTOR_B: u32 = 0x0003_f000;

pub struct Phy62x2SecurityFlash;

impl SecurityJournalStorage for Phy62x2SecurityFlash {
    fn read(&self, address: u32, output: &mut [u8]) -> Result<(), SecurityStoreError> {
        phy6222_hal::flash::read(address, output).map_err(|_| SecurityStoreError::Hardware)
    }

    fn program(&mut self, address: u32, data: &[u8]) -> Result<(), SecurityStoreError> {
        crate::time_driver::run_flash_operation(|| phy6222_hal::flash::write(address, data))
            .map_err(|_| SecurityStoreError::Hardware)
    }

    fn erase_sector(&mut self, address: u32) -> Result<(), SecurityStoreError> {
        crate::time_driver::run_flash_operation(|| phy6222_hal::flash::erase_sector(address))
            .map_err(|_| SecurityStoreError::Hardware)
    }
}

pub const fn create_security_store() -> SecurityStateJournal<Phy62x2SecurityFlash> {
    SecurityStateJournal::new(Phy62x2SecurityFlash, SECURITY_SECTOR_A, SECURITY_SECTOR_B)
}
