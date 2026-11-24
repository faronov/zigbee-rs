//! Router role: policy and profile. The journals are the shared ones of
//! [`crate::stores`] (security, children, APS tables, Green Power), so a
//! reflash between the NCP and the router keeps the frame-counter
//! reservations and drops the other role's tables.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

/// Endpoint of the HA Range Extender profile.
pub const ENDPOINT: u8 = 1;

/// 20 ms receive slices (same as the other EFR32 routers); 5..60 s
/// commissioning retry; failed secured rejoins are retried on that backoff
/// and never reset the network on their own.
pub static ROUTER_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

/// After a Leave the router has no button: re-arm Network Steering after
/// 5 min (longer than ZHA's 254 s permit window).
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;

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
        assert_eq!(router_profile().device_id(), DeviceId::RANGE_EXTENDER);
    }
}
