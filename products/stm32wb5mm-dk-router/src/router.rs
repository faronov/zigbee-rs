//! Router policy and profile for the STM32WB5MM-DK.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;

/// Same bounded receive slice and retry policy as the other routers. The
/// coprocessor sends Imm-Acks and holds indirect frames itself, so the slice
/// only bounds how long button and display work may wait.
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
