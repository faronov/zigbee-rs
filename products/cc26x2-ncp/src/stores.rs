//! The NCP's NV journals, each bound to its [`crate::layout`] window, and
//! the opening policy.
//!
//! Opening policy (before the stack reads anything):
//!
//! * no valid security record (blank flash, a previous Z-Stack NV or any
//!   foreign bytes): every NCP window is erased — foreign bytes are never
//!   interpreted;
//! * a valid security record of another role image (e.g. the router):
//!   `select_role_tag` clears its keys while keeping the outgoing frame
//!   counter limits, and the role's other journals are erased;
//! * a valid NCP journal is preserved.
//!
//! A power loss during an erase repeats the same path on the next boot.

use cc26x2_hal::flash::{Flash, FlashPartition};
use embedded_storage::nor_flash::NorFlash;
use zigbee_runtime::aps_table_store::ApsTableJournal;
use zigbee_runtime::child_store::ChildTableJournal;
use zigbee_runtime::gp_store::GpProxyJournal;
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};
use zigbee_runtime::settings_journal::SettingsJournal;
use zigbee_runtime::trust_center_stream::{
    TRUST_CENTER_STREAM_MIN_SECTOR_SIZE, TrustCenterStreamJournal,
};

use crate::layout::{
    APS_TABLES, CHILDREN, GREEN_POWER, IDENTITY, NV_SIZE, NV_START, SECURITY, TRUST_CENTER,
};

pub type SecurityFlash = FlashPartition<{ SECURITY.start }, { SECURITY.size }>;
pub type TrustCenterFlash = FlashPartition<{ TRUST_CENTER.start }, { TRUST_CENTER.size }>;
pub type ChildFlash = FlashPartition<{ CHILDREN.start }, { CHILDREN.size }>;
pub type GreenPowerFlash = FlashPartition<{ GREEN_POWER.start }, { GREEN_POWER.size }>;
pub type ApsTableFlash = FlashPartition<{ APS_TABLES.start }, { APS_TABLES.size }>;
pub type IdentityFlash = FlashPartition<{ IDENTITY.start }, { IDENTITY.size }>;
type NvFlash = FlashPartition<NV_START, NV_SIZE>;

/// NWK/APS reservations, keys and replay floors: 2 x 16 KiB.
pub type SecurityStore = SecurityStateJournal<SecurityFlash, { SECURITY.sector }>;
/// Trust Center devices (256), streamed across 4 KiB chunk slots of
/// 2 x 32 KiB logical sectors.
pub type TrustCenterStore = TrustCenterStreamJournal<TrustCenterFlash, { TRUST_CENTER.sector }>;
/// 64 children in 1 KiB slots (`ncp-capacity`).
pub type ChildStore = ChildTableJournal<ChildFlash, { CHILDREN.sector }>;
/// GP combo journal (Proxy Table 8 + Sink Table 16).
pub type GreenPowerStore = GpProxyJournal<GreenPowerFlash, { GREEN_POWER.sector }>;
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, { APS_TABLES.sector }>;
/// IEEE override and persisted settings (reserved; not used yet).
pub type IdentityStore = SettingsJournal<IdentityFlash, { IDENTITY.sector }>;

const _: () = assert!(SecurityStore::NCP_SECTOR_FITS);
const _: () = assert!(TRUST_CENTER_STREAM_MIN_SECTOR_SIZE <= TRUST_CENTER.sector);

/// How the journals were found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Opened {
    /// A valid NCP journal set.
    Existing,
    /// Blank or foreign NV: every window erased.
    Fresh,
    /// Another role's journal: keys cleared (counter limits kept), its
    /// other journals erased.
    RoleChanged,
}

pub struct Stores {
    pub security: SecurityStore,
    pub trust_center: TrustCenterStore,
    pub children: ChildStore,
    pub green_power: GreenPowerStore,
    pub aps_tables: ApsTableStore,
}

/// Open every NCP journal from the exclusive raw flash handle.
pub fn open(flash: Flash, role_tag: u16) -> Result<(Stores, Opened), SecurityStoreError> {
    let _owned = flash;
    // SAFETY: the windows are disjoint from each other and from the
    // application (layout.rs and the linker script assert it); the single
    // flash handle is consumed here, and `nv` is dropped before the
    // journals are used.
    let mut security = SecurityStore::new_with_sector_size(
        unsafe { SecurityFlash::new_unchecked() },
        0,
        SECURITY.sector as u32,
    );
    let mut opened = Opened::Existing;
    if security.load()?.is_none() {
        let mut nv = unsafe { NvFlash::new_unchecked() };
        nv.erase(0, NV_SIZE as u32)
            .map_err(|_| SecurityStoreError::Hardware)?;
        // Re-read the erased journal.
        let _ = security.storage_mut();
        opened = Opened::Fresh;
    }
    if security.select_role_tag(role_tag)? {
        let mut nv = unsafe { NvFlash::new_unchecked() };
        // Everything but the identity and security windows.
        nv.erase(APS_TABLES.start - NV_START, SECURITY.start - NV_START)
            .map_err(|_| SecurityStoreError::Hardware)?;
        opened = Opened::RoleChanged;
    }
    let stores = unsafe {
        Stores {
            security,
            trust_center: TrustCenterStore::new(
                TrustCenterFlash::new_unchecked(),
                0,
                TRUST_CENTER.sector as u32,
            ),
            children: ChildStore::new_with_sector_size(
                ChildFlash::new_unchecked(),
                0,
                CHILDREN.sector as u32,
            ),
            green_power: GreenPowerStore::new(
                GreenPowerFlash::new_unchecked(),
                0,
                GREEN_POWER.sector as u32,
            ),
            aps_tables: ApsTableStore::new_with_sector_size(
                ApsTableFlash::new_unchecked(),
                0,
                APS_TABLES.sector as u32,
            ),
        }
    };
    Ok((stores, opened))
}

const _: () = {
    // The role-change erase range is contiguous: APS, GP, children, TC.
    assert!(APS_TABLES.end() == GREEN_POWER.start);
    assert!(GREEN_POWER.end() == CHILDREN.start);
    assert!(CHILDREN.end() == TRUST_CENTER.start);
    assert!(TRUST_CENTER.end() == SECURITY.start);
    assert!(IDENTITY.end() == APS_TABLES.start);
};
