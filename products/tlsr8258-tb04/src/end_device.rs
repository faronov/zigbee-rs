//! Always-on (mains-powered, RX-on-when-idle) non-routing End Device role.
//!
//! Identity-only endpoint: the TB-04 fits no measurement hardware, so the
//! End Device exposes Basic and Identify only and has no reported cluster.

use router_app::RouterPolicy;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

pub const ENDPOINT: u8 = 1;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const END_DEVICE_MODEL: &str = "TLSR8258-ED";
pub const DATE_CODE: &str = "20261012";
pub const SW_BUILD: &str = "0.1.0";

/// Added to byte 0 of the factory EUI-64. Distinct from the sensor (`0x33`)
/// and router (`0x52`) offsets so one board never presents one IEEE address
/// in two roles.
pub const DEVICE_EUI_OFFSET: u8 = 0x45; // 'E' for End Device

/// 20 ms receive slices; 15..60 s commissioning retry; failed secured
/// rejoins retry on that backoff and never reset the network on their own
/// (the repository-wide always-on End Device policy).
pub static END_DEVICE_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: 20_000,
    join_retry_initial_ms: 15_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
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

    #[test]
    fn end_device_identity_is_distinct_from_the_other_roles() {
        assert_ne!(DEVICE_EUI_OFFSET, 0x33, "sensor offset");
        assert_ne!(DEVICE_EUI_OFFSET, 0x52, "router offset");
        assert_eq!(crate::role::END_DEVICE, 0x5202);
    }
}
