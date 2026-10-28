//! Environmental source for the LP-EM-CC2340R5 bring-up board.
//!
//! The LaunchPad has no ambient sensor fitted, so temperature is the BATMON
//! *die* temperature (it tracks the board, slightly above ambient while the
//! radio is busy) and humidity is an explicitly synthetic, slowly drifting
//! wave until an I2C sensor adapter is fitted and validated.

use sensor_sed_app::{EnvironmentReading, EnvironmentSource};
use zigbee_runtime::synthetic_sensor::SyntheticSensor;

use crate::BatmonNotReady;

/// Centre of the synthetic humidity wave.
pub const SYNTHETIC_HUMIDITY_CENTI_PERCENT: u16 = 5_000;
/// Peak deviation of the synthetic humidity wave (±15.00 %RH).
pub const SYNTHETIC_HUMIDITY_DEVIATION_CENTI_PERCENT: u16 = 1_500;
/// Samples spent on each of the wave's 16 steps. With the 30 s sample period
/// humidity moves one step (3.75 %RH) every 5 min, a full cycle every 80 min,
/// so the drift costs at most one extra report per 5 min.
pub const SYNTHETIC_HUMIDITY_SAMPLES_PER_STEP: u32 = 10;

const SYNTHETIC_HUMIDITY: SyntheticSensor = SyntheticSensor::new(
    0,
    0,
    SYNTHETIC_HUMIDITY_CENTI_PERCENT,
    SYNTHETIC_HUMIDITY_DEVIATION_CENTI_PERCENT,
);

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
    samples: u32,
}

impl<T: DieTemperature> BatmonEnvironment<T> {
    pub const fn new(sensor: T) -> Self {
        Self { sensor, samples: 0 }
    }

    pub fn reading(&mut self) -> Result<EnvironmentReading, BatmonNotReady> {
        let temperature_centi_celsius = self.sensor.temperature_centi_c().ok_or(BatmonNotReady)?;
        let step = self.samples / SYNTHETIC_HUMIDITY_SAMPLES_PER_STEP;
        self.samples = self.samples.wrapping_add(1);
        Ok(EnvironmentReading {
            temperature_centi_celsius,
            humidity_centi_percent: SYNTHETIC_HUMIDITY.sample(step).humidity_centipercent,
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
                // The wave starts at step 5 of 16 (+25 % of the deviation).
                humidity_centi_percent: 5_375,
                pressure_tenth_kpa: None,
            })
        );
    }

    #[test]
    fn synthetic_humidity_drifts_one_step_per_period_within_bounds() {
        let mut environment = BatmonEnvironment::new(Fake(Some(2_000)));
        let mut humidity = [0u16; 32];
        for value in &mut humidity {
            for _ in 0..SYNTHETIC_HUMIDITY_SAMPLES_PER_STEP {
                *value = environment.reading().unwrap().humidity_centi_percent;
            }
        }
        assert_eq!(&humidity[..4], &[5_375, 5_750, 6_125, 6_500]);
        assert_eq!(humidity[..16], humidity[16..]);
        assert_eq!(humidity.iter().min(), Some(&3_500));
        assert_eq!(humidity.iter().max(), Some(&6_500));
    }

    #[test]
    fn humidity_holds_within_a_step() {
        let mut environment = BatmonEnvironment::new(Fake(Some(2_000)));
        let first = environment.reading().unwrap().humidity_centi_percent;
        for _ in 1..SYNTHETIC_HUMIDITY_SAMPLES_PER_STEP {
            assert_eq!(environment.reading().unwrap().humidity_centi_percent, first);
        }
    }

    #[test]
    fn missing_measurement_is_a_read_failure() {
        assert_eq!(
            BatmonEnvironment::new(Fake(None)).reading(),
            Err(BatmonNotReady)
        );
    }
}
