//! Router policy and the plain Range Extender profile of the STM32WB
//! routers.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_mac::MacDriver;
use zigbee_runtime::builder::DeviceBuilder;
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;

/// Same bounded receive slice and retry policy as the other routers. The
/// coprocessor sends Imm-Acks and holds indirect frames itself, so the slice
/// only bounds how long button and indicator work may wait.
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

/// HA Range Extender: Basic + Identify on a routing, child-capable node.
pub type RangeExtenderProfile = DeviceProfile<RangeExtender>;

pub const fn range_extender_profile() -> RangeExtenderProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    )
}

/// Declare the Range Extender endpoint on the device builder.
pub fn declare_range_extender<M: MacDriver>(
    builder: DeviceBuilder<M>,
    profile: &RangeExtenderProfile,
) -> DeviceBuilder<M> {
    builder.endpoint(
        profile.endpoint(),
        profile.profile_id(),
        profile.device_id(),
        |endpoint| profile.configure_endpoint(endpoint),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn router_policy_is_valid_and_bounded() {
        assert!(ROUTER_POLICY.is_valid());
        assert!(ROUTER_POLICY.max_receive_slice_us <= 20_000);
    }

    #[test]
    fn plain_router_is_a_range_extender_without_reports() {
        let profile = range_extender_profile();
        assert_eq!(profile.device_id(), DeviceId::RANGE_EXTENDER);
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.expected_report_clusters(), 0);
    }
}
