//! Typed endpoint profile selected by the PHY62x2 sensor product.

use crate::ENDPOINT;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{
    BatteryDescriptor, DeviceProfile, EnvironmentalReporting, TemperatureHumidityBattery,
    TemperatureRange,
};
use zigbee_zcl::DeviceId;
use zigbee_zcl::foundation::reporting::MAX_REPORT_CONFIGS;

pub type SensorProfile = DeviceProfile<TemperatureHumidityBattery>;

const APPLICATION_ENDPOINT_COUNT: usize = 1;
const SERVER_CLUSTER_COUNT: usize = 5;
const DEFAULT_REPORT_CONFIG_COUNT: usize =
    TemperatureHumidityBattery::EXPECTED_REPORT_CLUSTER_IDS.len();

const _: () = {
    assert!(APPLICATION_ENDPOINT_COUNT <= zigbee_runtime::MAX_ENDPOINTS);
    assert!(SERVER_CLUSTER_COUNT <= zigbee_runtime::MAX_CLUSTERS_PER_ENDPOINT);
    assert!(DEFAULT_REPORT_CONFIG_COUNT <= MAX_REPORT_CONFIGS);
};

const TEMPERATURE_RANGE: TemperatureRange = TemperatureRange {
    min_centi_celsius: -4_000,
    max_centi_celsius: 12_500,
};

const BATTERY: BatteryDescriptor = BatteryDescriptor {
    size: 4,
    quantity: 2,
    rated_voltage_100mv: 15,
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
    fn profile_preserves_endpoint_and_reporting_contract() {
        let profile = sensor_profile();
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::TEMPERATURE_SENSOR);
        assert_eq!(profile.expected_report_clusters(), 3);
        assert_eq!(zigbee_runtime::MAX_ENDPOINTS, 2);
        assert_eq!(MAX_REPORT_CONFIGS, 4);
    }
}
