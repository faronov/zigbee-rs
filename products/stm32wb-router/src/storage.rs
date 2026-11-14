//! The router journals and the settings journal on internal flash below the
//! CPU2 secure area.

use stm32wb_hal::flash::{FlashError, FlashWindow, secure_flash_start};
use stm32wb_hal::word_flash::WordPerDoubleWord;
use zigbee_runtime::aps_table_store::ApsTableJournal;
use zigbee_runtime::child_store::ChildTableJournal;
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::settings_journal::SettingsJournal;

use crate::layout::*;

pub type ChildTableFlash = WordPerDoubleWord<FlashWindow<CHILD_TABLE_START, JOURNAL_PHYSICAL_SIZE>>;
pub type ApsTableFlash = WordPerDoubleWord<FlashWindow<APS_TABLE_START, JOURNAL_PHYSICAL_SIZE>>;
pub type SecurityFlash = WordPerDoubleWord<FlashWindow<SECURITY_START, JOURNAL_PHYSICAL_SIZE>>;
pub type SettingsFlash = WordPerDoubleWord<FlashWindow<SETTINGS_START, SETTINGS_PHYSICAL_SIZE>>;

pub type ChildStore = ChildTableJournal<ChildTableFlash, JOURNAL_SECTOR_SIZE>;
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, JOURNAL_SECTOR_SIZE>;
pub type SecurityStore = SecurityStateJournal<SecurityFlash>;
pub type SettingsStore = SettingsJournal<SettingsFlash, SETTINGS_SECTOR_SIZE>;
#[cfg(feature = "green-power")]
pub type GpProxyTableFlash =
    WordPerDoubleWord<FlashWindow<GP_PROXY_TABLE_START, JOURNAL_PHYSICAL_SIZE>>;
/// Green Power Proxy Table journal, same geometry as the router journals.
#[cfg(feature = "green-power")]
pub type GpProxyTableStore =
    zigbee_runtime::gp_store::GpProxyJournal<GpProxyTableFlash, JOURNAL_SECTOR_SIZE>;

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

/// The router's journals: children, APS tables, security state, and the
/// application settings.
pub struct RouterStores {
    pub children: ChildStore,
    pub aps_tables: ApsTableStore,
    pub security: SecurityStore,
    pub settings: SettingsStore,
    #[cfg(feature = "green-power")]
    pub gp_proxy_table: GpProxyTableStore,
}

/// Open the four journals. Call once.
pub fn open() -> Result<RouterStores, StorageError> {
    let secure_start = secure_flash_start();
    if secure_start < NV_END {
        return Err(StorageError::SecureAreaOverlap { secure_start });
    }
    // SAFETY: the four windows are disjoint from each other and from the
    // application (`link/memory.x` asserts it), they are minted once here,
    // and all journals run synchronously in the single application task.
    let (children, aps, security, settings) = unsafe {
        (
            FlashWindow::<CHILD_TABLE_START, JOURNAL_PHYSICAL_SIZE>::steal(),
            FlashWindow::<APS_TABLE_START, JOURNAL_PHYSICAL_SIZE>::steal(),
            FlashWindow::<SECURITY_START, JOURNAL_PHYSICAL_SIZE>::steal(),
            FlashWindow::<SETTINGS_START, SETTINGS_PHYSICAL_SIZE>::steal(),
        )
    };
    // SAFETY: as above; the GP window is disjoint from every other window.
    #[cfg(feature = "green-power")]
    let gp = unsafe { FlashWindow::<GP_PROXY_TABLE_START, JOURNAL_PHYSICAL_SIZE>::steal() };
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
        settings: SettingsJournal::new(
            settings.map_err(StorageError::Flash)?.into_word_flash(),
            0,
            SETTINGS_SECTOR_SIZE as u32,
        ),
        #[cfg(feature = "green-power")]
        gp_proxy_table: GpProxyTableStore::new(
            gp.map_err(StorageError::Flash)?.into_word_flash(),
            SECTOR_A,
            SECTOR_B,
        ),
    })
}
