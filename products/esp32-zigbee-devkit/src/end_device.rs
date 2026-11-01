//! Always-on (mains-powered, RX-on-when-idle) End Device role.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;

/// 20 ms receive slices; 15..60 s commissioning retry; three failed secured
/// rejoins fall back to fresh commissioning.
pub static END_DEVICE_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 15_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 3,
};

pub type EndDeviceProfile = DeviceProfile<RangeExtender>;

/// Basic + Identify with the Simple Sensor device ID: a non-routing
/// endpoint that does not advertise the HA Range Extender router type.
pub const fn end_device_profile() -> EndDeviceProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::SIMPLE_SENSOR,
        RangeExtender,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::profile::ApplicationProfile;

    #[test]
    fn end_device_policy_is_valid_and_bounded() {
        assert!(END_DEVICE_POLICY.is_valid());
        assert!(END_DEVICE_POLICY.max_receive_slice_us <= 20_000);
    }

    #[test]
    fn end_device_exposes_no_reported_cluster() {
        let profile = end_device_profile();
        assert_eq!(profile.device_id(), DeviceId::SIMPLE_SENSOR);
        assert_eq!(profile.expected_report_clusters(), 0);
    }
}
