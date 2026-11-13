//! Router policy and profile for the STM32WB5MM-DK.

use router_app::RouterPolicy;
use zigbee_mac::MacDriver;
use zigbee_runtime::builder::DeviceBuilder;
#[cfg(not(any(feature = "sensors", feature = "sound")))]
use zigbee_runtime::profile::{ApplicationProfile, DeviceProfile, RangeExtender};
#[cfg(not(any(feature = "sensors", feature = "sound")))]
use {crate::ENDPOINT, zigbee_aps::PROFILE_HOME_AUTOMATION, zigbee_zcl::DeviceId};

/// Same bounded receive slice and retry policy as the other routers. The
/// coprocessor sends Imm-Acks and holds indirect frames itself, so the slice
/// only bounds how long button and display work may wait.
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

/// HA Range Extender: Basic + Identify on a routing, child-capable node.
#[cfg(not(any(feature = "sensors", feature = "sound")))]
pub type RouterProfile = DeviceProfile<RangeExtender>;

/// Range Extender plus the on-board sensor endpoints
/// ([`crate::sensor_profile`]).
#[cfg(any(feature = "sensors", feature = "sound"))]
pub type RouterProfile = crate::sensor_profile::DkProfile;

#[cfg(not(any(feature = "sensors", feature = "sound")))]
pub const fn router_profile() -> RouterProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::RANGE_EXTENDER,
        RangeExtender,
    )
}

#[cfg(any(feature = "sensors", feature = "sound"))]
pub fn router_profile() -> RouterProfile {
    crate::sensor_profile::DkProfile::new()
}

/// Declare every endpoint of `profile` on the device builder.
pub fn declare_endpoints<M: MacDriver>(
    builder: DeviceBuilder<M>,
    profile: &RouterProfile,
) -> DeviceBuilder<M> {
    #[cfg(not(any(feature = "sensors", feature = "sound")))]
    return builder.endpoint(
        profile.endpoint(),
        profile.profile_id(),
        profile.device_id(),
        |endpoint| profile.configure_endpoint(endpoint),
    );
    #[cfg(any(feature = "sensors", feature = "sound"))]
    profile.declare_endpoints(builder)
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
        assert_eq!(profile.device_id(), zigbee_zcl::DeviceId::RANGE_EXTENDER);
        assert_eq!(profile.endpoint(), crate::ENDPOINT);
        #[cfg(not(feature = "sensors"))]
        assert_eq!(profile.expected_report_clusters(), 0);
        #[cfg(feature = "sensors")]
        assert_eq!(profile.expected_report_clusters(), 2);
    }
}
