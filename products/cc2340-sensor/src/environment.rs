//! Environmental source for the LP-EM-CC2340R5 bring-up board.
//!
//! The LaunchPad has no ambient sensor fitted, so temperature is the BATMON
//! *die* temperature (it tracks the board, slightly above ambient while the
//! radio is busy) and humidity stays an explicitly synthetic constant until
//! an I2C sensor adapter is fitted and validated.

use sensor_sed_app::{EnvironmentReading, EnvironmentSource};

use crate::BatmonNotReady;

pub const SYNTHETIC_HUMIDITY_CENTI_PERCENT: u16 = 5_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnvironmentSourceDisposition {
    DieTemperatureSyntheticHumidity,
}

pub const fn source_disposition() -> EnvironmentSourceDisposition {
    EnvironmentSourceDisposition::DieTemperatureSyntheticHumidity
}

/// Die temperature in 0.01 °C, `None` before the first measurement.
pub trait DieTemperature {
    fn temperature_centi_c(&self) -> Option<i16>;
}

#[cfg(target_os = "none")]
impl DieTemperature for lp_em_cc2340r5::batmon::Batmon {
    fn temperature_centi_c(&self) -> Option<i16> {
        lp_em_cc2340r5::batmon::Batmon::temperature_centi_c(self)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct BatmonEnvironment<T> {
    sensor: T,
}

impl<T: DieTemperature> BatmonEnvironment<T> {
    pub const fn new(sensor: T) -> Self {
        Self { sensor }
    }

    pub fn reading(&self) -> Result<EnvironmentReading, BatmonNotReady> {
        let temperature_centi_celsius = self.sensor.temperature_centi_c().ok_or(BatmonNotReady)?;
        Ok(EnvironmentReading {
            temperature_centi_celsius,
            humidity_centi_percent: SYNTHETIC_HUMIDITY_CENTI_PERCENT,
            pressure_tenth_kpa: None,
        })
    }
}

impl<T: DieTemperature> EnvironmentSource for BatmonEnvironment<T> {
    type Error = BatmonNotReady;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        self.reading()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Fake(Option<i16>);

    impl DieTemperature for Fake {
        fn temperature_centi_c(&self) -> Option<i16> {
            self.0
        }
    }

    #[test]
    fn die_temperature_is_reported_with_synthetic_humidity() {
        assert_eq!(
            BatmonEnvironment::new(Fake(Some(2_441))).reading(),
            Ok(EnvironmentReading {
                temperature_centi_celsius: 2_441,
                humidity_centi_percent: 5_000,
                pressure_tenth_kpa: None,
            })
        );
    }

    #[test]
    fn missing_measurement_is_a_read_failure() {
        assert_eq!(
            BatmonEnvironment::new(Fake(None)).reading(),
            Err(BatmonNotReady)
        );
    }
}
