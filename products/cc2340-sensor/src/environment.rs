//! Explicitly synthetic environmental source used until an ADC/I2C sensor is
//! fitted and validated on the LP-EM-CC2340R5.

use core::convert::Infallible;

use sensor_sed_app::{EnvironmentReading, EnvironmentSource};

pub const SYNTHETIC_TEMPERATURE_CENTI_CELSIUS: i16 = 2_250;
pub const SYNTHETIC_HUMIDITY_CENTI_PERCENT: u16 = 5_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EnvironmentSourceDisposition {
    SyntheticFixedUntilSensorAdapter,
}

pub const fn source_disposition() -> EnvironmentSourceDisposition {
    EnvironmentSourceDisposition::SyntheticFixedUntilSensorAdapter
}

#[derive(Debug, Default, Clone, Copy)]
pub struct SyntheticEnvironment;

impl SyntheticEnvironment {
    pub const fn new() -> Self {
        Self
    }

    pub const fn reading() -> EnvironmentReading {
        EnvironmentReading {
            temperature_centi_celsius: SYNTHETIC_TEMPERATURE_CENTI_CELSIUS,
            humidity_centi_percent: SYNTHETIC_HUMIDITY_CENTI_PERCENT,
            pressure_tenth_kpa: None,
        }
    }
}

impl EnvironmentSource for SyntheticEnvironment {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        Ok(Self::reading())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_is_explicitly_synthetic() {
        assert_eq!(
            source_disposition(),
            EnvironmentSourceDisposition::SyntheticFixedUntilSensorAdapter
        );
    }

    #[test]
    fn fixed_reading_uses_zcl_environment_units() {
        assert_eq!(
            SyntheticEnvironment::reading(),
            EnvironmentReading {
                temperature_centi_celsius: 2_250,
                humidity_centi_percent: 5_000,
                pressure_tenth_kpa: None,
            }
        );
    }
}
