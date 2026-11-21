//! Product-owned partitions and the security-journal wiring.
//!
//! The partitions mirror `link/memory-*.x` (asserted below and by
//! `examples/cc26x2-router/tools/verify-layout.py`): the router child/APS
//! journals and the security journal sit directly below the CCFG sector at
//! the top of flash, with the Green Power Proxy Table window below them.
//! Every role image links the same map.

use cc26x2_hal::chip::{FLASH_SIZE, SECTOR_SIZE};
use cc26x2_hal::flash::{CCFG_SECTOR, FlashPartition};
use zigbee_runtime::security_journal::{SECURITY_JOURNAL_SLOT_SIZE, SecurityStateJournal};

pub const PERSISTENCE_PARTITION_SIZE: usize = 2 * SECTOR_SIZE;
pub const PERSISTENCE_PARTITION_END: u32 = CCFG_SECTOR;
pub const PERSISTENCE_PARTITION_START: u32 =
    PERSISTENCE_PARTITION_END - PERSISTENCE_PARTITION_SIZE as u32;
pub const APS_TABLE_PARTITION_SIZE: usize = 2 * SECTOR_SIZE;
pub const APS_TABLE_PARTITION_START: u32 =
    PERSISTENCE_PARTITION_START - APS_TABLE_PARTITION_SIZE as u32;
pub const CHILD_TABLE_PARTITION_SIZE: usize = 2 * SECTOR_SIZE;
pub const CHILD_TABLE_PARTITION_START: u32 =
    APS_TABLE_PARTITION_START - CHILD_TABLE_PARTITION_SIZE as u32;
/// Green Power Proxy Table window: the minimum two-sector journal. One
/// 1 KiB slot holds the whole 8-entry table, so an 8 KiB sector takes 8
/// full records, or one full record and 224 32-byte frame-counter records,
/// between rollovers. Reserved in every image, used by `green-power` builds.
pub const GP_PROXY_TABLE_PARTITION_SIZE: usize = 2 * SECTOR_SIZE;
pub const GP_PROXY_TABLE_PARTITION_START: u32 =
    CHILD_TABLE_PARTITION_START - GP_PROXY_TABLE_PARTITION_SIZE as u32;
/// End of the application image window.
pub const APPLICATION_END: u32 = GP_PROXY_TABLE_PARTITION_START;

pub const SECURITY_SECTOR_SIZE: usize = SECTOR_SIZE;

const _: () = assert!(SECURITY_SECTOR_SIZE.is_multiple_of(SECURITY_JOURNAL_SLOT_SIZE));
const _: () = assert!(CCFG_SECTOR as usize == FLASH_SIZE - SECTOR_SIZE);
#[cfg(feature = "zbdongle-p")]
const _: () = {
    assert!(FLASH_SIZE == 352 * 1024);
    assert!(APPLICATION_END == 0x46000);
    assert!(GP_PROXY_TABLE_PARTITION_START == 0x46000);
    assert!(CHILD_TABLE_PARTITION_START == 0x4A000);
    assert!(PERSISTENCE_PARTITION_START == 0x52000);
};
#[cfg(feature = "cc2652p7-dongle")]
const _: () = {
    assert!(FLASH_SIZE == 704 * 1024);
    assert!(APPLICATION_END == 0x9E000);
    assert!(GP_PROXY_TABLE_PARTITION_START == 0x9E000);
    assert!(CHILD_TABLE_PARTITION_START == 0xA2000);
    assert!(PERSISTENCE_PARTITION_START == 0xAA000);
};

pub type SecurityFlash = FlashPartition<PERSISTENCE_PARTITION_START, PERSISTENCE_PARTITION_SIZE>;
/// The generic journal on this product's two 8 KiB sectors.
pub type SecurityStore = SecurityStateJournal<SecurityFlash, { SECURITY_SECTOR_SIZE }>;

/// Build the security journal from the exclusive raw flash handle.
pub(crate) fn security_journal(flash: cc26x2_hal::flash::Flash) -> SecurityStore {
    let _owned = flash;
    // SAFETY: the security window is disjoint from the application and the
    // router journals (asserted above and in the linker script); the single
    // flash handle is consumed here.
    let partition = unsafe { SecurityFlash::new_unchecked() };
    SecurityStore::new_with_sector_size(partition, 0, SECURITY_SECTOR_SIZE as u32)
}
