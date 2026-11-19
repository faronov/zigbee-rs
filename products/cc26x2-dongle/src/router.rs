//! Router-role persistence, policy and profile.
//!
//! Child-table and APS binding/group journals live in two product-owned
//! partitions directly below the security journal (see `journal.rs`); the
//! Green Power Proxy Table window below them is reserved for a later GP
//! proxy build.

use cc26x2_hal::flash::{FLASH_PAGE_SIZE, FlashPartition};
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

/// 20 ms receive slices; 5..60 s commissioning retry; failed secured
/// rejoins are retried on that backoff and never reset the network on
/// their own (only Leave does).
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

pub type ChildTableFlash = FlashPartition<CHILD_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE>;
pub type ApsTableFlash = FlashPartition<APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE>;
/// Child-table journal on two 8 KiB sectors.
pub type ChildStore = ChildTableJournal<ChildTableFlash, FLASH_PAGE_SIZE>;
/// APS binding/group journal on two 8 KiB sectors.
pub type ApsTableStore = ApsTableJournal<ApsTableFlash, FLASH_PAGE_SIZE>;

/// Mint the router journals; the flash handle is passed on to
/// [`crate::storage::security_store`].
pub fn router_stores(
    flash: cc26x2_hal::flash::Flash,
) -> (cc26x2_hal::flash::Flash, ChildStore, ApsTableStore) {
    // SAFETY: the child and APS windows are disjoint from each other, from
    // the security partition and from the application (journal.rs and the
    // linker script assert it); the single flash handle is consumed by the
    // caller after these are minted.
    let (children, aps) = unsafe {
        (
            ChildTableFlash::new_unchecked(),
            ApsTableFlash::new_unchecked(),
        )
    };
    (
        flash,
        ChildStore::new_with_sector_size(children, 0, FLASH_PAGE_SIZE as u32),
        ApsTableStore::new_with_sector_size(aps, 0, FLASH_PAGE_SIZE as u32),
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
