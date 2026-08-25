//! Environmental measurement source and profile-update abstractions.

use zigbee_runtime::profile::{
    ApplicationProfile, BatteryMeasurement, DeviceProfile, ProfileComponent,
    TemperatureHumidityBattery, TemperatureHumidityMeasurement, TemperatureHumidityPressureBattery,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnvironmentReading {
    pub temperature_centi_celsius: i16,
    pub humidity_centi_percent: u16,
    /// Pressure Measurement cluster units (whole hPa).
    pub pressure_tenth_kpa: Option<i16>,
}

/// A fitted asynchronous environmental sensor.
#[allow(async_fn_in_trait)]
pub trait EnvironmentSource {
    type Error;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error>;
}

/// Synchronous sensor contract used by constrained or polling-only HALs.
pub trait BlockingEnvironmentSource {
    type Error;

    fn sample(&mut self) -> Result<EnvironmentReading, Self::Error>;
}

/// Zero-allocation adapter that exposes a blocking sensor through the shared
/// async application contract.
pub struct BlockingEnvironment<T>(T);

impl<T> BlockingEnvironment<T> {
    pub const fn new(inner: T) -> Self {
        Self(inner)
    }

    pub fn into_inner(self) -> T {
        self.0
    }
}

impl<T: BlockingEnvironmentSource> EnvironmentSource for BlockingEnvironment<T> {
    type Error = T::Error;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        self.0.sample()
    }
}

/// Profile component that accepts environmental and battery measurements.
pub trait EnvironmentSink {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement);
    fn update_battery(&mut self, measurement: BatteryMeasurement);
    fn update_pressure(&mut self, _tenth_kpa: i16) {}
}

impl EnvironmentSink for TemperatureHumidityBattery {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        TemperatureHumidityBattery::update_environment(self, measurement);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        TemperatureHumidityBattery::update_battery(self, measurement);
    }
}

impl EnvironmentSink for TemperatureHumidityPressureBattery {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        TemperatureHumidityPressureBattery::update_environment(self, measurement);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        TemperatureHumidityPressureBattery::update_battery(self, measurement);
    }

    fn update_pressure(&mut self, tenth_kpa: i16) {
        TemperatureHumidityPressureBattery::update_pressure(self, tenth_kpa);
    }
}

/// Application profile understood by this environmental-sensor archetype.
///
/// This is deliberately narrower than a universal "sensor behavior" trait:
/// temperature and humidity remain the archetype's required measurements.
pub trait EnvironmentalSensorProfile: ApplicationProfile {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement);
    fn update_battery(&mut self, measurement: BatteryMeasurement);
    fn update_pressure(&mut self, tenth_kpa: i16);
}

impl<C> EnvironmentalSensorProfile for DeviceProfile<C>
where
    C: ProfileComponent + EnvironmentSink,
{
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.component_mut().update_environment(measurement);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.component_mut().update_battery(measurement);
    }

    fn update_pressure(&mut self, tenth_kpa: i16) {
        self.component_mut().update_pressure(tenth_kpa);
    }
}

#[cfg(feature = "ota")]
impl<P, F> EnvironmentalSensorProfile for zigbee_runtime::profile::WithOta<P, F>
where
    P: EnvironmentalSensorProfile,
    F: zigbee_runtime::firmware_writer::FirmwareWriter,
{
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.inner_mut().update_environment(measurement);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.inner_mut().update_battery(measurement);
    }

    fn update_pressure(&mut self, tenth_kpa: i16) {
        self.inner_mut().update_pressure(tenth_kpa);
    }
}

#[cfg(feature = "ota")]
impl<P, F> EnvironmentalSensorProfile for zigbee_runtime::profile::OptionalOta<P, F>
where
    P: EnvironmentalSensorProfile,
    F: zigbee_runtime::firmware_writer::FirmwareWriter,
{
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement) {
        self.inner_mut().update_environment(measurement);
    }

    fn update_battery(&mut self, measurement: BatteryMeasurement) {
        self.inner_mut().update_battery(measurement);
    }

    fn update_pressure(&mut self, tenth_kpa: i16) {
        self.inner_mut().update_pressure(tenth_kpa);
    }
}
