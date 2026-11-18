//! The NCP's NV journals, each bounded to its `zbnv` window
//! ([`crate::layout::Nv`]) of the active layout.

use crate::flash::FlashWindow;
use zigbee_runtime::aps_table_store::ApsTableJournal;
use zigbee_runtime::child_store::ChildTableJournal;
use zigbee_runtime::security_journal::SecurityStateJournal;
use zigbee_runtime::settings_journal::SettingsJournal;
use zigbee_runtime::trust_center_stream::{
    TRUST_CENTER_STREAM_MIN_SECTOR_SIZE, TrustCenterStreamJournal,
};

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
/// Trust Center devices (256): snapshots streamed across 4 KiB chunk slots
/// of 2 × 32 KiB logical sectors (spec 8.9). A single-slot record written by
/// the previous `TrustCenterDeviceJournal` at the start of a half is read
/// and migrated on the next store.
pub type TrustCenterStore = TrustCenterStreamJournal<TrustCenterFlash, { Nv::TRUST_CENTER_SECTOR }>;
/// 64 children in 1 024-byte slots (`ncp-capacity`), 2 × 4 KiB.
pub type ChildStore = ChildTableJournal<ChildFlash, SECTOR>;

// The full replay snapshot (256 APS keys, 128 NWK floors) fits a 16 KiB
// security sector with room for appends, and a full 256-device Trust Center
// snapshot fits one 32 KiB half.
const _: () = assert!(SecurityStore::NCP_SECTOR_FITS);
const _: () = assert!(TRUST_CENTER_STREAM_MIN_SECTOR_SIZE <= Nv::TRUST_CENTER_SECTOR);
const _: () = assert!(2 * Nv::TRUST_CENTER_SECTOR == Nv::TRUST_CENTER.1 as usize);
// The GP journal window (`GreenPowerFlash`, 2 × 8 KiB) is reserved for the
// GP combo sink; this NCP build does not run Green Power yet.
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, SECTOR>;
/// IEEE override and persisted settings.
pub type IdentityStore = SettingsJournal<IdentityFlash, SECTOR>;

pub fn security_store() -> SecurityStore {
    SecurityStore::new_with_sector_size(SecurityFlash::new(), 0, Nv::SECURITY_SECTOR as u32)
}

pub fn trust_center_store() -> TrustCenterStore {
    TrustCenterStore::new(TrustCenterFlash::new(), 0, Nv::TRUST_CENTER_SECTOR as u32)
}

pub fn child_store() -> ChildStore {
    ChildStore::new_with_sector_size(ChildFlash::new(), 0, SECTOR as u32)
}

pub fn aps_table_store() -> ApsTableStore {
    ApsTableStore::new_with_sector_size(ApsTableFlash::new(), 0, SECTOR as u32)
}

pub fn identity_store() -> IdentityStore {
    IdentityStore::new(IdentityFlash::new(), 0, SECTOR as u32)
}
