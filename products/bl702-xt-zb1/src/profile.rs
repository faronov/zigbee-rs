//! Complete Zigbee profile selected by the XT-ZB1 sensor product.

use crate::ENDPOINT;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{
    BatteryDescriptor, DeviceProfile, EnvironmentalReporting, TemperatureHumidityBattery,
    TemperatureRange,
};
use zigbee_zcl::DeviceId;

pub type SensorProfile = DeviceProfile<TemperatureHumidityBattery>;

const TEMPERATURE_RANGE: TemperatureRange = TemperatureRange {
    min_centi_celsius: -4_000,
    max_centi_celsius: 12_500,
};

// Preserve the former PowerConfigCluster defaults: no battery form factor,
// cell count, or rated cell voltage has been established for the XT-ZB1.
const BATTERY: BatteryDescriptor = BatteryDescriptor {
    size: 0xff,
    quantity: 0,
    rated_voltage_100mv: 0,
};

pub fn sensor_profile() -> SensorProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        TemperatureHumidityBattery::new(
            TEMPERATURE_RANGE,
            BATTERY,
            EnvironmentalReporting::default(),
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::profile::ApplicationProfile;

    #[test]
    fn profile_preserves_endpoint_identity_and_cluster_contract() {
        let profile = sensor_profile();
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::TEMPERATURE_SENSOR);
        assert_eq!(profile.expected_report_clusters(), 3);
    }
}
