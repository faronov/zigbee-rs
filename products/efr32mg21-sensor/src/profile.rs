//! Complete non-OTA Zigbee profile selected by this product.

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
    fn profile_identity_and_reporting_contract_are_stable() {
        let profile = sensor_profile();
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::TEMPERATURE_SENSOR);
        assert_eq!(profile.expected_report_clusters(), 3);
        assert_eq!(TEMPERATURE_RANGE.min_centi_celsius, -4_000);
        assert_eq!(TEMPERATURE_RANGE.max_centi_celsius, 12_500);
        assert_eq!(
            BATTERY,
            BatteryDescriptor {
                size: 4,
                quantity: 2,
                rated_voltage_100mv: 15,
            }
        );
        assert_eq!(
            EnvironmentalReporting::default(),
            EnvironmentalReporting {
                temperature_min_secs: 60,
                temperature_max_secs: 300,
                temperature_change_centi_celsius: 50,
                humidity_min_secs: 60,
                humidity_max_secs: 300,
                humidity_change_centi_percent: 100,
                battery_min_secs: 300,
                battery_max_secs: 3_600,
                battery_change_half_percent: 4,
            }
        );
    }
}
