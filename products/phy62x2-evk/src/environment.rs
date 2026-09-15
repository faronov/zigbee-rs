//! Blocking synthetic environment source used by the current bring-up image.

use core::convert::Infallible;
use sensor_sed_app::{BlockingEnvironmentSource, EnvironmentReading};
use zigbee_runtime::synthetic_sensor::SyntheticSensor;

const SENSOR: SyntheticSensor = SyntheticSensor::new(2_250, 75, 5_000, 300);

#[derive(Debug, Default, Clone, Copy)]
pub struct SyntheticEnvironment {
    sample: u32,
}

impl SyntheticEnvironment {
    pub const fn new() -> Self {
        Self { sample: 0 }
    }
}

impl BlockingEnvironmentSource for SyntheticEnvironment {
    type Error = Infallible;

    fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        let reading = SENSOR.sample(self.sample);
        self.sample = self.sample.wrapping_add(1);
        Ok(EnvironmentReading {
            temperature_centi_celsius: reading.temperature_centidegrees,
            humidity_centi_percent: reading.humidity_centipercent,
            pressure_tenth_kpa: None,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_preserves_the_existing_initial_synthetic_reading() {
        let mut source = SyntheticEnvironment::new();
        assert_eq!(
            source.sample(),
            Ok(EnvironmentReading {
                temperature_centi_celsius: 2_175,
                humidity_centi_percent: 5_075,
                pressure_tenth_kpa: None,
            })
        );
    }
}
