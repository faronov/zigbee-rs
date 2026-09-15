//! Product-selected synthetic environment source.
//!
//! No physical environmental sensor is fitted or hardware-proven on the
//! current XT-ZB1 product, so the source remains explicitly synthetic.

use core::convert::Infallible;

use sensor_sed_app::{BlockingEnvironmentSource, EnvironmentReading};

#[derive(Debug, Default)]
pub struct SyntheticEnvironment {
    sequence: u32,
}

impl SyntheticEnvironment {
    pub const fn new() -> Self {
        Self { sequence: 0 }
    }
}

impl BlockingEnvironmentSource for SyntheticEnvironment {
    type Error = Infallible;

    fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        Ok(EnvironmentReading {
            temperature_centi_celsius: 2_250 + (sequence % 20) as i16,
            humidity_centi_percent: 5_000 + (sequence.wrapping_mul(7) % 100) as u16,
            pressure_tenth_kpa: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_sequence_matches_the_existing_sensor_values() {
        let mut sensor = SyntheticEnvironment::new();
        assert_eq!(
            sensor.sample(),
            Ok(EnvironmentReading {
                temperature_centi_celsius: 2_250,
                humidity_centi_percent: 5_000,
                pressure_tenth_kpa: None,
            })
        );
        assert_eq!(
            sensor.sample(),
            Ok(EnvironmentReading {
                temperature_centi_celsius: 2_251,
                humidity_centi_percent: 5_007,
                pressure_tenth_kpa: None,
            })
        );
    }
}
