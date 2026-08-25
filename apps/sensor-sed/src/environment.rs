//! Platform-independent environmental measurement source and sink abstractions.
//!
//! [`EnvironmentSource`] is whatever fitted hardware produces a reading.
//! Concrete on-chip and I2C adapters stay in platform/board-facing crates.
//!
//! [`EnvironmentSink`] is the profile component the reading is written to.
//! Implementing it for the shared `zigbee-runtime` archetypes lets the
//! application update temperature/humidity/battery — and, when the product
//! selected a pressure-capable component, pressure — without knowing which
//! archetype the product chose.

use zigbee_runtime::profile::{
    BatteryMeasurement, TemperatureHumidityBattery, TemperatureHumidityMeasurement,
    TemperatureHumidityPressureBattery,
};

/// One complete environmental sample.
///
/// `pressure_tenth_kpa` is `None` for sources with no pressure channel; the
/// application then never calls [`EnvironmentSink::update_pressure`], which
/// keeps a product without a Pressure Measurement cluster unaffected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EnvironmentReading {
    pub temperature_centi_celsius: i16,
    pub humidity_centi_percent: u16,
    /// In the Pressure Measurement cluster's own units (whole hPa).
    pub pressure_tenth_kpa: Option<i16>,
}

/// A fitted environmental sensor.
///
/// `async fn` in a trait is deliberate: embedded executors commonly run this
/// lifecycle on one thread, and requiring `Send` would wrongly reject sensor
/// drivers that hold a borrowed bus.
#[allow(async_fn_in_trait)]
pub trait EnvironmentSource {
    /// Take one reading. `None` means the read failed and the previously
    /// reported cluster values must be left untouched.
    async fn sample(&mut self) -> Option<EnvironmentReading>;

    /// Log a successful reading.
    ///
    /// Each source owns its own log line so the exact diagnostic text
    /// (and the set of fields that are meaningful for that part) is
    /// preserved rather than flattened into one generic format string.
    fn log_reading(&self, reading: &EnvironmentReading);
}

/// Profile component that accepts environmental and battery measurements.
pub trait EnvironmentSink {
    fn update_environment(&mut self, measurement: TemperatureHumidityMeasurement);
    fn update_battery(&mut self, measurement: BatteryMeasurement);
    /// Only called when the source actually produced a pressure channel.
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
