//! Nordic environmental-sensor adapters for the shared SED lifecycle.

use core::convert::Infallible;

use sensor_sed_app::{EnvironmentReading, EnvironmentSource};

/// The nRF52 on-chip `TEMP` peripheral, plus the synthetic humidity ramp
/// reported by the existing no-external-sensor firmware.
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
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        let raw_temp = self.temp.read().await;
        let temperature_centi_celsius = (raw_temp.to_bits() * 100 / 4) as i16;
        self.humidity_tick = self.humidity_tick.wrapping_add(1);
        let humidity_centi_percent = 5000u16 + ((self.humidity_tick % 100) as u16).wrapping_mul(10);
        Ok(EnvironmentReading {
            temperature_centi_celsius,
            humidity_centi_percent,
            pressure_tenth_kpa: None,
        })
    }
}
