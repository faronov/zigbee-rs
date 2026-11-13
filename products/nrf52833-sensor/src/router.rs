//! Router-role persistence, policy and profile for the nRF52833-DK.
//!
//! The router keeps its child table and APS binding/group tables in two
//! product-owned journals at `0x7A000..0x7E000`, directly below the security
//! journal, and its Green Power Proxy Table journal at `0x78000..0x7A000`.
//! Every role image (sensor SED, End Device, Router) links the same 480 KiB
//! application window (`link/memory.x`), so a reflash to another role
//! can never overwrite another role's persistence.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;

/// nRF52833 NVMC erase page.
pub const FLASH_PAGE_SIZE: usize = 4096;
/// End of the application window (`link/memory.x` `FLASH`).
pub const APPLICATION_END: u32 = 0x0007_8000;
/// Router-role Green Power Proxy Table journal: two 4 KiB pages. Reserved in
/// every image; only `green-power` builds open it.
pub const GP_PROXY_TABLE_PARTITION_START: u32 = APPLICATION_END;
pub const GP_PROXY_TABLE_PARTITION_SIZE: usize = 2 * FLASH_PAGE_SIZE;
/// Router-role child-table journal: two 4 KiB pages.
pub const CHILD_TABLE_PARTITION_START: u32 =
    GP_PROXY_TABLE_PARTITION_START + GP_PROXY_TABLE_PARTITION_SIZE as u32;
pub const CHILD_TABLE_PARTITION_SIZE: usize = 2 * FLASH_PAGE_SIZE;
/// Router-role APS binding/group journal: two 4 KiB pages.
pub const APS_TABLE_PARTITION_START: u32 =
    CHILD_TABLE_PARTITION_START + CHILD_TABLE_PARTITION_SIZE as u32;
pub const APS_TABLE_PARTITION_SIZE: usize = 2 * FLASH_PAGE_SIZE;
/// First byte of the protected security journal (`storage.rs`).
pub const SECURITY_PARTITION_START: u32 = 0x0007_E000;

const _: () = assert!(APPLICATION_END == 480 * 1024);
const _: () = assert!(CHILD_TABLE_PARTITION_START == 0x0007_A000);
const _: () = assert!(GP_PROXY_TABLE_PARTITION_START.is_multiple_of(FLASH_PAGE_SIZE as u32));
const _: () = assert!(
    APS_TABLE_PARTITION_START + APS_TABLE_PARTITION_SIZE as u32 == SECURITY_PARTITION_START
);
const _: () = assert!(CHILD_TABLE_PARTITION_START.is_multiple_of(FLASH_PAGE_SIZE as u32));
const _: () = assert!(APS_TABLE_PARTITION_START.is_multiple_of(FLASH_PAGE_SIZE as u32));

/// Same bounded receive slice as the end device; 5..60 s commissioning
/// retry; failed secured rejoins are retried on that backoff and never reset
/// the network on their own (only Leave or a user reset does).
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

pub type RouterProfile = DeviceProfile<RangeExtender>;

/// HA Range Extender: Basic + Identify on a routing, child-capable node.
pub const fn router_profile() -> RouterProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    )
}

#[cfg(target_os = "none")]
mod target {
    use super::*;
    use crate::partition::NvmcPartition;
    use embassy_nrf::nvmc::{Nvmc, PAGE_SIZE};
    use zigbee_runtime::aps_table_store::ApsTableJournal;
    use zigbee_runtime::child_store::ChildTableJournal;

    const _: () = assert!(PAGE_SIZE == FLASH_PAGE_SIZE);
    const _: () = assert!(crate::storage::SECURITY_PARTITION_START == SECURITY_PARTITION_START);

    pub type ChildTableFlash =
        NvmcPartition<CHILD_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE>;
    pub type ApsTableFlash = NvmcPartition<APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE>;
    /// Child-table journal on two 4 KiB NVMC pages.
    pub type ChildStore = ChildTableJournal<ChildTableFlash, FLASH_PAGE_SIZE>;
    /// APS binding/group journal on two 4 KiB NVMC pages.
    pub type ApsTableStore = ApsTableJournal<ApsTableFlash, FLASH_PAGE_SIZE>;

    const SECTOR_A: u32 = 0;
    const SECTOR_B: u32 = FLASH_PAGE_SIZE as u32;

    /// Split the unique NVMC handle into the router journals.
    ///
    /// The handle is returned for [`crate::storage::security_store`], whose
    /// partition view is bounded to `0x7E000..0x80000`; the three windows
    /// are disjoint by the const assertions above.
    pub fn router_stores(nvmc: Nvmc<'_>) -> (Nvmc<'_>, ChildStore, ApsTableStore) {
        // SAFETY: the child and APS windows are disjoint from each other,
        // from the security partition and from the application, and this
        // function consumes the only safe NVMC handle before minting them.
        // All three users run synchronously in the single application task.
        let (children, aps) = unsafe { (ChildTableFlash::steal(), ApsTableFlash::steal()) };
        (
            nvmc,
            ChildStore::new_with_sector_size(children, SECTOR_A, SECTOR_B),
            ApsTableStore::new_with_sector_size(aps, SECTOR_A, SECTOR_B),
        )
    }

    #[cfg(feature = "green-power")]
    pub type GpProxyTableFlash =
        NvmcPartition<GP_PROXY_TABLE_PARTITION_START, GP_PROXY_TABLE_PARTITION_SIZE>;
    /// Green Power Proxy Table journal on two 4 KiB NVMC pages.
    #[cfg(feature = "green-power")]
    pub type GpProxyTableStore =
        zigbee_runtime::gp_store::GpProxyJournal<GpProxyTableFlash, FLASH_PAGE_SIZE>;

    /// Mint the GP Proxy Table journal; the NVMC handle is passed through
    /// exactly as by [`router_stores`].
    #[cfg(feature = "green-power")]
    pub fn gp_proxy_table_store(nvmc: Nvmc<'_>) -> (Nvmc<'_>, GpProxyTableStore) {
        // SAFETY: the GP window is disjoint from the child, APS and security
        // windows and from the application (const and linker assertions),
        // and it is driven from the same single application task.
        let gp = unsafe { GpProxyTableFlash::steal() };
        (nvmc, GpProxyTableStore::new(gp, SECTOR_A, SECTOR_B))
    }
}

#[cfg(target_os = "none")]
pub use target::{ApsTableFlash, ApsTableStore, ChildStore, ChildTableFlash, router_stores};
#[cfg(all(target_os = "none", feature = "green-power"))]
pub use target::{GpProxyTableFlash, GpProxyTableStore, gp_proxy_table_store};

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::profile::ApplicationProfile;

    #[test]
    fn router_policy_is_valid_and_bounded() {
        assert!(ROUTER_POLICY.is_valid());
        assert!(ROUTER_POLICY.max_receive_slice_us <= 20_000);
    }

    #[test]
    fn router_advertises_a_range_extender() {
        let profile = router_profile();
        assert_eq!(profile.device_id(), DeviceId::RANGE_EXTENDER);
        assert_eq!(profile.expected_report_clusters(), 0);
    }

    #[test]
    fn router_journals_fill_the_window_below_the_security_journal() {
        assert_eq!(GP_PROXY_TABLE_PARTITION_START, 0x0007_8000);
        assert_eq!(
            GP_PROXY_TABLE_PARTITION_START as usize + GP_PROXY_TABLE_PARTITION_SIZE,
            CHILD_TABLE_PARTITION_START as usize
        );
        assert_eq!(CHILD_TABLE_PARTITION_START, 0x0007_A000);
        assert_eq!(APS_TABLE_PARTITION_START, 0x0007_C000);
        assert_eq!(
            APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE,
            SECURITY_PARTITION_START as usize
        );
    }
}
