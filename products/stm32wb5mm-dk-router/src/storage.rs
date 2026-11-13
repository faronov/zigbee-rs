//! The three router journals on internal flash below the CPU2 secure area.

use stm32wb_hal::flash::{FlashError, FlashWindow, secure_flash_start};
use stm32wb_hal::word_flash::WordPerDoubleWord;
use zigbee_runtime::aps_table_store::ApsTableJournal;
use zigbee_runtime::child_store::ChildTableJournal;
use zigbee_runtime::security_journal::SecurityStateJournal;

use crate::layout::*;

pub type ChildTableFlash = WordPerDoubleWord<FlashWindow<CHILD_TABLE_START, JOURNAL_PHYSICAL_SIZE>>;
pub type ApsTableFlash = WordPerDoubleWord<FlashWindow<APS_TABLE_START, JOURNAL_PHYSICAL_SIZE>>;
pub type SecurityFlash = WordPerDoubleWord<FlashWindow<SECURITY_START, JOURNAL_PHYSICAL_SIZE>>;

pub type ChildStore = ChildTableJournal<ChildTableFlash, JOURNAL_SECTOR_SIZE>;
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, JOURNAL_SECTOR_SIZE>;
pub type SecurityStore = SecurityStateJournal<SecurityFlash>;

const SECTOR_A: u32 = 0;
const SECTOR_B: u32 = JOURNAL_SECTOR_SIZE as u32;

/// Why the journals could not be opened.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StorageError {
    /// The FUS-reported secure area starts below the product's storage
    /// (a larger wireless stack was installed): touching it is refused.
    SecureAreaOverlap {
        secure_start: u32,
    },
    Flash(FlashError),
}

/// The router's journals: children, APS tables, security state.
pub struct RouterStores {
    pub children: ChildStore,
    pub aps_tables: ApsTableStore,
    pub security: SecurityStore,
}

/// Open the three journals. Call once.
pub fn open() -> Result<RouterStores, StorageError> {
    let secure_start = secure_flash_start();
    if secure_start < NV_END {
        return Err(StorageError::SecureAreaOverlap { secure_start });
    }
    // SAFETY: the three windows are disjoint from each other and from the
    // application (`link/memory.x` asserts it), they are minted once here,
    // and all journals run synchronously in the single application task.
    let (children, aps, security) = unsafe {
        (
            FlashWindow::<CHILD_TABLE_START, JOURNAL_PHYSICAL_SIZE>::steal(),
            FlashWindow::<APS_TABLE_START, JOURNAL_PHYSICAL_SIZE>::steal(),
            FlashWindow::<SECURITY_START, JOURNAL_PHYSICAL_SIZE>::steal(),
        )
    };
    Ok(RouterStores {
        children: ChildStore::new_with_sector_size(
            children.map_err(StorageError::Flash)?.into_word_flash(),
            SECTOR_A,
            SECTOR_B,
        ),
        aps_tables: ApsTableStore::new_with_sector_size(
            aps.map_err(StorageError::Flash)?.into_word_flash(),
            SECTOR_A,
            SECTOR_B,
        ),
        security: SecurityStateJournal::new(
            security.map_err(StorageError::Flash)?.into_word_flash(),
            SECTOR_A,
            SECTOR_B,
        ),
    })
}
