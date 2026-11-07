//! Router-role persistence, policy and profile (size probe).
//!
//! **The router does not fit the EFR32MG1P132F256** (see
//! `examples/efr32mg1-sensor/src/router.rs` for the measured numbers); this
//! module keeps the role template complete so the linker, not a silent
//! table shrink, rejects it.
//!
//! The router would keep its child table and APS binding/group tables in two
//! router-only journals at `0x33000..0x37000`, carved from the top of the
//! application window by `link/router-tables.x`. The sleepy sensor and the
//! End Device keep the full `0x4000..0x37000` window and never touch them;
//! all roles share the security journal at `0x37000`.

use efr32mg1_hal::flash::Efr32mg1Flash;
use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::aps_table_store::{APS_TABLE_JOURNAL_SECTOR_SIZE, ApsTableJournal};
use zigbee_runtime::child_store::{CHILD_JOURNAL_SECTOR_SIZE, ChildTableJournal};
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;
use crate::storage::{PartitionFlash, SECURITY_PARTITION_START};

/// Child-table journal: two 4 KiB sectors (two 2 KiB MSC pages each).
pub const CHILD_TABLE_PARTITION_START: u32 = 0x0003_3000;
pub const CHILD_TABLE_PARTITION_SIZE: usize = CHILD_JOURNAL_SECTOR_SIZE * 2;
/// APS binding/group journal: two 4 KiB sectors.
pub const APS_TABLE_PARTITION_START: u32 = 0x0003_5000;
pub const APS_TABLE_PARTITION_SIZE: usize = APS_TABLE_JOURNAL_SECTOR_SIZE * 2;
/// End of the router image's application window (`link/router-tables.x`).
pub const ROUTER_APPLICATION_END: u32 = CHILD_TABLE_PARTITION_START;

const _: () = assert!(
    CHILD_TABLE_PARTITION_START as usize + CHILD_TABLE_PARTITION_SIZE
        == APS_TABLE_PARTITION_START as usize
);
const _: () = assert!(
    APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE
        == SECURITY_PARTITION_START as usize
);
const _: () = assert!(CHILD_TABLE_PARTITION_START >= 0x0000_4000);
const _: () =
    assert!(CHILD_TABLE_PARTITION_START.is_multiple_of(
        <Efr32mg1Flash as embedded_storage::nor_flash::NorFlash>::ERASE_SIZE as u32
    ));

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

pub type ChildTableFlash = PartitionFlash<CHILD_TABLE_PARTITION_START, CHILD_TABLE_PARTITION_SIZE>;
pub type ApsTableFlash = PartitionFlash<APS_TABLE_PARTITION_START, APS_TABLE_PARTITION_SIZE>;
pub type ChildStore = ChildTableJournal<ChildTableFlash>;
pub type ApsTableStore = ApsTableJournal<ApsTableFlash>;

/// Mint the router journals from the unique internal-flash handle.
///
/// The handle is returned for [`crate::storage::split_flash`]; the router
/// windows are disjoint from the security and application-NV partitions
/// (const assertions above) and from the router image (linker script).
pub fn router_stores(flash: Efr32mg1Flash) -> (Efr32mg1Flash, ChildStore, ApsTableStore) {
    // SAFETY: the child and APS windows are disjoint from each other and
    // from every other product partition, and this function consumes the
    // only safe flash handle before minting them.
    let (children, aps) = unsafe { (ChildTableFlash::new(), ApsTableFlash::new()) };
    (
        flash,
        ChildStore::new(children, 0, CHILD_JOURNAL_SECTOR_SIZE as u32),
        ApsTableStore::new(aps, 0, APS_TABLE_JOURNAL_SECTOR_SIZE as u32),
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

    #[test]
    fn router_journals_sit_below_the_security_journal() {
        assert_eq!(ROUTER_APPLICATION_END, 0x0003_3000);
        assert_eq!(
            APS_TABLE_PARTITION_START as usize + APS_TABLE_PARTITION_SIZE,
            SECURITY_PARTITION_START as usize
        );
    }
}
