//! Router policy and profile for the STM32WB5MM-DK.
//!
//! The bounded [`ROUTER_POLICY`] and the plain Range Extender come from the
//! shared STM32WB router product.

pub use stm32wb_router_product::policy::ROUTER_POLICY;
use zigbee_mac::MacDriver;
use zigbee_runtime::builder::DeviceBuilder;

/// HA Range Extender: Basic + Identify on a routing, child-capable node.
#[cfg(not(any(feature = "sensors", feature = "sound")))]
pub type RouterProfile = stm32wb_router_product::policy::RangeExtenderProfile;

/// Range Extender plus the on-board sensor endpoints
/// ([`crate::sensor_profile`]).
#[cfg(any(feature = "sensors", feature = "sound"))]
pub type RouterProfile = crate::sensor_profile::DkProfile;

#[cfg(not(any(feature = "sensors", feature = "sound")))]
pub const fn router_profile() -> RouterProfile {
    stm32wb_router_product::policy::range_extender_profile()
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
    return stm32wb_router_product::policy::declare_range_extender(builder, profile);
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
