//! Router-role persistence, policy and profile for the ZBT-1.
//!
//! The router keeps its child table and APS binding/group tables in two
//! product-owned journals at `0xF4000..0xFC000` (see `journal.rs`); the
//! end-device roles link the same application window and never touch them.

use efr32s2_hal::flash::{Efr32s2Flash, FLASH_PAGE_SIZE, FlashPartition};
use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::aps_table_store::ApsTableJournal;
use zigbee_runtime::child_store::ChildTableJournal;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;
use crate::journal::{
    APS_TABLE_PARTITION_SIZE, APS_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE,
    CHILD_TABLE_PARTITION_START,
};

/// Same bounded receive slice as the end device; 5..60 s commissioning
/// retry; three failed secured rejoins fall back to fresh commissioning.
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 3,
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

pub type ChildTableFlash = FlashPartition<CHILD_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE>;
pub type ApsTableFlash = FlashPartition<APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE>;
/// Child-table journal on two 8 KiB MSC pages.
pub type ChildStore = ChildTableJournal<ChildTableFlash, FLASH_PAGE_SIZE>;
/// APS binding/group journal on two 8 KiB MSC pages.
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, FLASH_PAGE_SIZE>;

const SECTOR_A: u32 = 0;
const SECTOR_B: u32 = FLASH_PAGE_SIZE as u32;

/// Split the unique internal-flash handle into the router journals.
///
/// The handle is returned for [`crate::storage::security_store`], whose
/// partition view is bounded to `0xFC000..0x100000`; the three windows are
/// disjoint by the const assertions in `journal.rs`.
pub fn router_stores(flash: Efr32s2Flash) -> (Efr32s2Flash, ChildStore, ApsTableStore) {
    // SAFETY: the child and APS windows are disjoint from each other, from
    // the security partition and from the application (journal.rs asserts),
    // and this function consumes the only safe flash handle before minting
    // them.
    let (children, aps) = unsafe {
        (
            ChildTableFlash::new_unchecked(),
            ApsTableFlash::new_unchecked(),
        )
    };
    (
        flash,
        ChildStore::new_with_sector_size(children, SECTOR_A, SECTOR_B),
        ApsTableStore::new_with_sector_size(aps, SECTOR_A, SECTOR_B),
    )
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
