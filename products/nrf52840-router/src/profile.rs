//! Home Automation basic End Device profile selected by this product.

use crate::ENDPOINT;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

pub type AlwaysOnEndDeviceProfile = DeviceProfile<RangeExtender>;

/// A mains-powered non-routing endpoint: Basic and Identify only.
///
/// `RangeExtender` provides that cluster composition, while the Simple Sensor
/// device ID avoids advertising the HA Range Extender router device type.
pub fn always_on_end_device_profile() -> AlwaysOnEndDeviceProfile {
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
    use zigbee_mac::mock::MockMac;
    use zigbee_runtime::ZigbeeDevice;
    use zigbee_runtime::profile::ApplicationProfile;
    use zigbee_nwk::DeviceType;
    use zigbee_runtime::role::EndDevice;

    #[test]
    fn product_exposes_only_basic_and_identify_servers() {
        let profile = always_on_end_device_profile();
        let device: ZigbeeDevice<_, EndDevice> =
            ZigbeeDevice::builder(MockMac::new([1, 2, 3, 4, 5, 6, 7, 8]))
                .device_type(DeviceType::EndDevice)
                .endpoint(
                    profile.endpoint(),
                    profile.profile_id(),
                    profile.device_id(),
                    |endpoint| profile.configure_endpoint(endpoint),
                )
                .build();
        let descriptor = device.bdb().zdo().find_endpoint(ENDPOINT).unwrap();

        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::SIMPLE_SENSOR);
        assert_eq!(descriptor.input_clusters.as_slice(), &[0x0000, 0x0003]);
        assert!(descriptor.output_clusters.is_empty());
        assert_eq!(profile.expected_report_clusters(), 0);
    }
}
