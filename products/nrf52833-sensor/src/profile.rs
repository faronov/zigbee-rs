//! Complete Zigbee profile selected by this firmware product.
//!
//! Identical composition to `products/nrf52840-sensor`: the shared
//! [`TemperatureHumidityBattery`] archetype from `zigbee-runtime`, with the
//! same temperature range and 2x AAA battery descriptor the nRF52833
//! firmware reported before the product/board split, and the same
//! `EnvironmentalReporting` defaults. With the `sensor-bme280` feature a
//! Pressure Measurement cluster is added via
//! [`TemperatureHumidityBattery::with_pressure`], which selects the
//! distinct [`TemperatureHumidityPressureBattery`] component type so a
//! build without that feature never links the Pressure cluster at all.

use crate::ENDPOINT;
use zigbee_aps::PROFILE_HOME_AUTOMATION;
#[cfg(feature = "sensor-bme280")]
use zigbee_runtime::profile::PressureRange;
#[cfg(feature = "sensor-bme280")]
use zigbee_runtime::profile::TemperatureHumidityPressureBattery;
use zigbee_runtime::profile::{
    BatteryDescriptor, DeviceProfile, EnvironmentalReporting, TemperatureHumidityBattery,
    TemperatureRange,
};
use zigbee_zcl::DeviceId;

#[cfg(not(feature = "sensor-bme280"))]
pub type SensorProfile = DeviceProfile<TemperatureHumidityBattery>;
#[cfg(feature = "sensor-bme280")]
pub type SensorProfile = DeviceProfile<TemperatureHumidityPressureBattery>;

/// Temperature range: -40.00..125.00 °C, matching the original firmware's
/// `TemperatureCluster::new(-4000, 12500)`.
const TEMPERATURE_RANGE: TemperatureRange = TemperatureRange {
    min_centi_celsius: -4_000,
    max_centi_celsius: 12_500,
};

/// 2x AAA battery pack descriptor, matching the original firmware's
/// `set_battery_size(4)` / `set_battery_quantity(2)` /
/// `set_battery_rated_voltage(15)`.
const BATTERY: BatteryDescriptor = BatteryDescriptor {
    size: 4,
    quantity: 2,
    rated_voltage_100mv: 15,
};

/// BME280 pressure range in the existing Pressure cluster's whole-hPa units.
#[cfg(feature = "sensor-bme280")]
const PRESSURE_RANGE: PressureRange = PressureRange {
    min_tenth_kpa: 300,
    max_tenth_kpa: 1_100,
};

/// Convert the BME280 driver's Pascal reading to the existing Pressure
/// cluster representation (whole hPa).
#[cfg(feature = "sensor-bme280")]
pub fn pressure_pa_to_zcl(pressure_pa: u32) -> i16 {
    (pressure_pa / 100) as i16
}

pub fn sensor_profile() -> SensorProfile {
    let environment = TemperatureHumidityBattery::new(
        TEMPERATURE_RANGE,
        BATTERY,
        EnvironmentalReporting::default(),
    );
    #[cfg(feature = "sensor-bme280")]
    let environment = environment.with_pressure(PRESSURE_RANGE);

    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::TEMPERATURE_SENSOR,
        environment,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::profile::ApplicationProfile;

    #[test]
    fn sensor_profile_has_expected_identity() {
        let profile = sensor_profile();
        assert_eq!(profile.endpoint(), ENDPOINT);
        assert_eq!(profile.profile_id(), PROFILE_HOME_AUTOMATION);
        assert_eq!(profile.device_id(), DeviceId::TEMPERATURE_SENSOR);
        #[cfg(not(feature = "sensor-bme280"))]
        assert_eq!(profile.expected_report_clusters(), 3);
        #[cfg(feature = "sensor-bme280")]
        assert_eq!(profile.expected_report_clusters(), 4);
    }

    #[cfg(feature = "sensor-bme280")]
    #[test]
    fn bme280_pressure_uses_pressure_cluster_units() {
        assert_eq!(pressure_pa_to_zcl(30_000), 300);
        assert_eq!(pressure_pa_to_zcl(101_325), 1_013);
        assert_eq!(pressure_pa_to_zcl(110_000), 1_100);
        assert_eq!(PRESSURE_RANGE.min_tenth_kpa, 300);
        assert_eq!(PRESSURE_RANGE.max_tenth_kpa, 1_100);
    }
}
