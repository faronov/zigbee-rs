//! Typed Zigbee endpoint profile selected by the CC2340 sensor product.
//!
//! [`DeviceProfile`] owns all cluster instances and reporting defaults. The
//! composition root therefore has no raw cluster arrays or duplicated cluster
//! identifiers.

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

// The development image reports a labelled synthetic fixed 3-V source until
// BATMON/ADC support exists. Its physical battery form factor is unknown.
const BATTERY: BatteryDescriptor = BatteryDescriptor {
    size: 0xFF,
    quantity: 1,
    rated_voltage_100mv: 30,
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
    fn profile_has_the_product_endpoint_and_three_report_clusters() {
        let profile = sensor_profile();
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::TEMPERATURE_SENSOR);
        assert_eq!(profile.expected_report_clusters(), 3);
    }
}
