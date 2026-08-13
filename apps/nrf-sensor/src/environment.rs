//! Environmental measurement source and sink abstractions.
//!
//! [`EnvironmentSource`] is whatever hardware actually produces a reading:
//! the on-chip `TEMP` peripheral ([`OnChipTemperature`], chip-generic and
//! therefore provided here), or a board-wired external I2C part
//! (BME280/SHT31), which stays in the composition root because its bus type
//! is board-specific.
//!
//! [`EnvironmentSink`] is the profile component the reading is written to.
//! Implementing it for the shared `zigbee-runtime` archetypes lets the
//! application update temperature/humidity/battery — and, when the product
//! selected a pressure-capable component, pressure — without knowing which
//! archetype the product chose.

use defmt::info;
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
/// `async fn` in a trait is deliberate here: this application runs on a
/// single-threaded `embassy-executor` thread-mode executor, so the returned
/// futures never need a `Send` bound, and the desugared
/// `impl Future + Send` form would wrongly constrain external I2C sensor
/// drivers holding a `&mut` bus.
#[allow(async_fn_in_trait)]
pub trait EnvironmentSource {
    /// Take one reading. `None` means the read failed and the previously
    /// reported cluster values must be left untouched.
    async fn sample(&mut self) -> Option<EnvironmentReading>;

    /// Log a successful reading.
    ///
    /// Each source owns its own `defmt` line so the exact diagnostic text
    /// (and the set of fields that are meaningful for that part) is
    /// preserved rather than flattened into one generic format string.
    fn log_reading(&self, reading: &EnvironmentReading);

    /// Log a failed read. Sources that cannot fail never call this.
    fn log_failure(&self) {
        defmt::warn!("Environmental sensor read failed");
    }
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

/// The nRF52 on-chip `TEMP` peripheral, plus the synthetic humidity ramp
/// the original firmware reported when no humidity part is fitted.
///
/// Both the fixed-point conversion (`raw * 100 / 4`) and the 50.00..59.90 %
/// humidity ramp are carried over unchanged, so a build with no external
/// sensor reports exactly what it did before.
pub struct OnChipTemperature<'d> {
    temp: embassy_nrf::temp::Temp<'d>,
    humidity_tick: u32,
}

impl<'d> OnChipTemperature<'d> {
    pub const fn new(temp: embassy_nrf::temp::Temp<'d>) -> Self {
        Self {
            temp,
            humidity_tick: 0,
        }
    }
}

impl EnvironmentSource for OnChipTemperature<'_> {
    async fn sample(&mut self) -> Option<EnvironmentReading> {
        let raw_temp = self.temp.read().await;
        let temperature_centi_celsius = (raw_temp.to_bits() * 100 / 4) as i16;
        self.humidity_tick = self.humidity_tick.wrapping_add(1);
        let humidity_centi_percent = 5000u16 + ((self.humidity_tick % 100) as u16).wrapping_mul(10);
        Some(EnvironmentReading {
            temperature_centi_celsius,
            humidity_centi_percent,
            pressure_tenth_kpa: None,
        })
    }

    fn log_reading(&self, reading: &EnvironmentReading) {
        info!(
            "T={}.{:02}°C H={}.{:02}% (on-chip)",
            reading.temperature_centi_celsius / 100,
            (reading.temperature_centi_celsius % 100).unsigned_abs(),
            reading.humidity_centi_percent / 100,
            reading.humidity_centi_percent % 100
        );
    }
}
