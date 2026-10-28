//! Archetype-neutral measurement contract between [`crate::SensorApp`] and
//! the product's fitted sensors.
//!
//! The lifecycle only decides *when* to sample (cold start, the periodic
//! sample interval, or a user report request). *What* is sampled, and how
//! readings map onto the profile's clusters, belongs to the product:
//!
//! - every [`EnvironmentSource`] is a [`SensorMeasurement`] for every
//!   [`EnvironmentalSensorProfile`] through a blanket implementation that
//!   keeps the original temperature/humidity(/pressure) behavior;
//! - other archetypes, such as the plant sensor, implement
//!   [`SensorMeasurement`] for their own source type and profile.

use zigbee_runtime::profile::{
    ApplicationProfile, BatteryMeasurement, DeviceProfile, PlantSensor,
    TemperatureHumidityMeasurement,
};

use crate::diagnostics::{DiagnosticEvent, Diagnostics};
use crate::environment::{EnvironmentSource, EnvironmentalSensorProfile};

/// Why the lifecycle requested a measurement.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SampleTrigger {
    /// First measurement after boot, before default reporting starts, so the
    /// coordinator's interview reads real values.
    ColdStart,
    /// The policy's `sample_interval_ms` elapsed while joined.
    Periodic,
    /// A user action asked for an immediate report.
    UserRequest,
}

impl SampleTrigger {
    /// Whether the caller expects every fitted measurement to be refreshed
    /// now, rather than on a product-specific slower cadence.
    pub const fn refresh_all(self) -> bool {
        !matches!(self, SampleTrigger::Periodic)
    }
}

/// Profile capability every sleepy sensor archetype shares: publishing the
/// separately sampled [`crate::BatterySource`] reading.
pub trait SensorProfile: ApplicationProfile {
    fn update_battery(&mut self, measurement: BatteryMeasurement);
}

impl<P: EnvironmentalSensorProfile> SensorProfile for P {
    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        EnvironmentalSensorProfile::update_battery(self, measurement);
    }
}

impl SensorProfile for DeviceProfile<PlantSensor> {
    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.component_mut().update_battery(measurement);
    }
}

/// A plant sensor with an OTA Upgrade client endpoint.
#[cfg(feature = "ota")]
impl<F> SensorProfile for zigbee_runtime::profile::WithOta<DeviceProfile<PlantSensor>, F>
where
    F: zigbee_runtime::firmware_writer::FirmwareWriter,
{
    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.inner_mut().component_mut().update_battery(measurement);
    }
}

/// Product-owned sampling of the fitted sensors into profile `P`.
///
/// Implementations update the profile's cluster values in place and record
/// readings and failures through `diagnostics`. A failed read must leave the
/// previous cluster value untouched.
#[allow(async_fn_in_trait)]
pub trait SensorMeasurement<P> {
    async fn measure<D: Diagnostics>(
        &mut self,
        profile: &mut P,
        trigger: SampleTrigger,
        diagnostics: &mut D,
    );
}

/// The environmental archetype: sample temperature/humidity (and optional
/// pressure) on every trigger, exactly as before the measurement contract
/// was introduced.
impl<P, E> SensorMeasurement<P> for E
where
    P: EnvironmentalSensorProfile,
    E: EnvironmentSource,
{
    async fn measure<D: Diagnostics>(
        &mut self,
        profile: &mut P,
        _trigger: SampleTrigger,
        diagnostics: &mut D,
    ) {
        match self.sample().await {
            Ok(reading) => {
                diagnostics.record(DiagnosticEvent::Environment(reading));
                profile.update_environment(TemperatureHumidityMeasurement {
                    temperature_centi_celsius: reading.temperature_centi_celsius,
                    humidity_centi_percent: reading.humidity_centi_percent,
                });
                if let Some(pressure) = reading.pressure_tenth_kpa {
                    profile.update_pressure(pressure);
                }
            }
            Err(_) => diagnostics.record(DiagnosticEvent::EnvironmentReadFailed),
        }
    }
}

#[cfg(all(test, feature = "ota"))]
mod tests {
    use super::*;
    use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};
    use zigbee_runtime::ota::{OtaConfig, OtaManager};
    use zigbee_runtime::profile::{
        BatteryDescriptor, PlantSensorReporting, TemperatureRange, WithOta,
    };
    use zigbee_zcl::DeviceId;

    struct NullWriter;

    impl FirmwareWriter for NullWriter {
        fn erase_slot(&mut self) -> Result<(), FirmwareError> {
            Ok(())
        }
        fn write_block(&mut self, _offset: u32, _data: &[u8]) -> Result<(), FirmwareError> {
            Ok(())
        }
        fn verify(&mut self, _size: u32, _hash: Option<&[u8]>) -> Result<(), FirmwareError> {
            Ok(())
        }
        fn activate(&mut self) -> Result<(), FirmwareError> {
            Ok(())
        }
        fn slot_size(&self) -> u32 {
            0
        }
        fn abort(&mut self) -> Result<(), FirmwareError> {
            Ok(())
        }
    }

    fn requires_sensor_profile<P: SensorProfile>(profile: &mut P) {
        profile.update_battery(BatteryMeasurement {
            voltage_100mv: 29,
            percentage_remaining: 180,
        });
    }

    #[test]
    fn plant_sensor_with_ota_is_a_sensor_profile() {
        let plant = DeviceProfile::new(
            1,
            0x0104,
            DeviceId::SIMPLE_SENSOR,
            PlantSensor::new(
                TemperatureRange {
                    min_centi_celsius: -4_000,
                    max_centi_celsius: 8_500,
                },
                0xFFFE,
                PlantSensorReporting::default(),
            )
            .with_battery(BatteryDescriptor {
                size: 10,
                quantity: 1,
                rated_voltage_100mv: 30,
            }),
        );
        let ota = OtaManager::new(
            NullWriter,
            OtaConfig {
                endpoint: 1,
                ..OtaConfig::default()
            },
        );
        let mut profile = WithOta::new(plant, ota).expect("same endpoint");
        requires_sensor_profile(&mut profile);
        assert!(profile.inner().component().has_battery());
    }
}
