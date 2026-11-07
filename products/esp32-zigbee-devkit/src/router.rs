//! Router-role persistence, policy and profile for the ESP32-C6/H2 devkits.
//!
//! The router keeps its child table and APS binding/group tables in two
//! product-owned journals in the first 16 KiB of `zbnv`
//! (`0x3F0000..0x3F4000`, see [`crate::layout`]); the security journal stays
//! at `0x3FE000..0x400000`. The end-device roles never open the router
//! journals.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;

/// 20 ms receive slices; 5..60 s commissioning retry; failed secured
/// rejoins retry on that backoff and never reset the network on their own.
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
pub use stores::{
    ApsTableFlash, ApsTableStore, ChildStore, ChildTableFlash, FlashWindow, router_stores,
};

#[cfg(target_os = "none")]
mod stores {
    use zigbee_runtime::aps_table_store::ApsTableJournal;
    use zigbee_runtime::child_store::ChildTableJournal;

    pub use crate::flash_window::FlashWindow;

    use crate::layout::{APS_TABLE_OFFSET, APS_TABLE_SIZE, CHILD_TABLE_OFFSET, CHILD_TABLE_SIZE};

    const SECTOR: usize = crate::layout::SECTOR_SIZE as usize;

    pub type ChildTableFlash = FlashWindow<CHILD_TABLE_OFFSET, { CHILD_TABLE_SIZE as usize }>;
    pub type ApsTableFlash = FlashWindow<APS_TABLE_OFFSET, { APS_TABLE_SIZE as usize }>;
    /// Child-table journal on two 4 KiB flash sectors.
    pub type ChildStore = ChildTableJournal<ChildTableFlash, SECTOR>;
    /// APS binding/group journal on two 4 KiB flash sectors.
    pub type ApsTableStore = ApsTableJournal<ApsTableFlash, SECTOR>;

    const SECTOR_A: u32 = 0;
    const SECTOR_B: u32 = SECTOR as u32;

    /// The router's child-table and APS journals. Both windows are bounded
    /// and disjoint from each other, from the security journal and from the
    /// OTA slots (`layout` asserts).
    pub fn router_stores() -> (ChildStore, ApsTableStore) {
        (
            ChildStore::new_with_sector_size(ChildTableFlash::new(), SECTOR_A, SECTOR_B),
            ApsTableStore::new_with_sector_size(ApsTableFlash::new(), SECTOR_A, SECTOR_B),
        )
    }
}

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
}
