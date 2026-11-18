//! The NCP's NV journals, each bounded to its `zbnv` window
//! ([`crate::layout::Nv`]) of the active layout.

use crate::flash::FlashWindow;
use zigbee_runtime::aps_table_store::ApsTableJournal;
use zigbee_runtime::child_store::ChildTableJournal;
use zigbee_runtime::gp_store::GpProxyJournal;
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::settings_journal::SettingsJournal;
use zigbee_runtime::trust_center_store::TrustCenterDeviceJournal;

use crate::layout::{ACTIVE, Nv, SECTOR_SIZE};

const SECTOR: usize = SECTOR_SIZE as usize;

pub type SecurityFlash = FlashWindow<{ ACTIVE.nv(Nv::SECURITY.0) }, { Nv::SECURITY.1 as usize }>;
pub type TrustCenterFlash =
    FlashWindow<{ ACTIVE.nv(Nv::TRUST_CENTER.0) }, { Nv::TRUST_CENTER.1 as usize }>;
pub type ChildFlash = FlashWindow<{ ACTIVE.nv(Nv::CHILDREN.0) }, { Nv::CHILDREN.1 as usize }>;
pub type GreenPowerFlash =
    FlashWindow<{ ACTIVE.nv(Nv::GREEN_POWER.0) }, { Nv::GREEN_POWER.1 as usize }>;
pub type ApsTableFlash =
    FlashWindow<{ ACTIVE.nv(Nv::APS_TABLES.0) }, { Nv::APS_TABLES.1 as usize }>;
pub type IdentityFlash = FlashWindow<{ ACTIVE.nv(Nv::IDENTITY.0) }, { Nv::IDENTITY.1 as usize }>;

/// NWK/APS reservations, keys and replay floors: 2 × 16 KiB sectors.
pub type SecurityStore = SecurityStateJournal<SecurityFlash, { Nv::SECURITY_SECTOR }>;
/// Trust Center devices: today's 4 KiB journal at the start of each 32 KiB
/// half (the rest is reserved for streamed snapshots).
pub type TrustCenterStore = TrustCenterDeviceJournal<TrustCenterFlash>;
pub type ChildStore = ChildTableJournal<ChildFlash, SECTOR>;
/// GP Proxy + Sink tables: 2 × 8 KiB sectors.
pub type GreenPowerStore = GpProxyJournal<GreenPowerFlash, { Nv::GREEN_POWER_SECTOR }>;
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, SECTOR>;
/// IEEE override and persisted settings.
pub type IdentityStore = SettingsJournal<IdentityFlash, SECTOR>;

pub fn security_store() -> SecurityStore {
    SecurityStore::new_with_sector_size(SecurityFlash::new(), 0, Nv::SECURITY_SECTOR as u32)
}

pub fn trust_center_store() -> TrustCenterStore {
    TrustCenterStore::new(TrustCenterFlash::new(), 0, Nv::TRUST_CENTER.1 / 2)
}

pub fn child_store() -> ChildStore {
    ChildStore::new_with_sector_size(ChildFlash::new(), 0, SECTOR as u32)
}

pub fn green_power_store() -> GreenPowerStore {
    GreenPowerStore::new(GreenPowerFlash::new(), 0, Nv::GREEN_POWER_SECTOR as u32)
}

pub fn aps_table_store() -> ApsTableStore {
    ApsTableStore::new_with_sector_size(ApsTableFlash::new(), 0, SECTOR as u32)
}

pub fn identity_store() -> IdentityStore {
    IdentityStore::new(IdentityFlash::new(), 0, SECTOR as u32)
}
