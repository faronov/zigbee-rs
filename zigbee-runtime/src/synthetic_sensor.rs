//! Deterministic temperature/humidity readings for cross-platform testing.

use crate::ClusterRef;
use zigbee_zcl::clusters::humidity;
use zigbee_zcl::clusters::temperature;
use zigbee_zcl::data_types::ZclValue;
use zigbee_zcl::{ClusterId, ZclStatus};

const DEVIATION_WAVE_PERCENT: [i16; 16] = [
    -100, -75, -50, -25, 0, 25, 50, 75, 100, 75, 50, 25, 0, -25, -50, -75,
];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyntheticSensorReading {
    pub temperature_centidegrees: i16,
    pub humidity_centipercent: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyntheticSensor {
    base_temperature_centidegrees: i16,
    temperature_deviation_centidegrees: u16,
    base_humidity_centipercent: u16,
    humidity_deviation_centipercent: u16,
}

impl SyntheticSensor {
    pub const fn new(
        base_temperature_centidegrees: i16,
        temperature_deviation_centidegrees: u16,
        base_humidity_centipercent: u16,
        humidity_deviation_centipercent: u16,
    ) -> Self {
        Self {
            base_temperature_centidegrees,
            temperature_deviation_centidegrees,
            base_humidity_centipercent,
            humidity_deviation_centipercent,
        }
    }

    /// Return a deterministic sample. Humidity is phase-shifted so the two
    /// measurements do not move in lockstep.
    pub fn sample(&self, index: u32) -> SyntheticSensorReading {
        let temperature_percent =
            DEVIATION_WAVE_PERCENT[index as usize % DEVIATION_WAVE_PERCENT.len()];
        let humidity_percent =
            DEVIATION_WAVE_PERCENT[(index as usize + 5) % DEVIATION_WAVE_PERCENT.len()];

        let temperature = i32::from(self.base_temperature_centidegrees)
            + scaled_deviation(self.temperature_deviation_centidegrees, temperature_percent);
        let humidity = i32::from(self.base_humidity_centipercent)
            + scaled_deviation(self.humidity_deviation_centipercent, humidity_percent);

        SyntheticSensorReading {
            // 0x8000 is the ZCL "unknown" temperature sentinel.
            temperature_centidegrees: temperature.clamp(i16::MIN as i32 + 1, i16::MAX as i32)
                as i16,
            humidity_centipercent: humidity.clamp(0, 10_000) as u16,
        }
    }
}

fn scaled_deviation(maximum: u16, percent: i16) -> i32 {
    i32::from(maximum) * i32::from(percent) / 100
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ApplySyntheticReadingError {
    TemperatureClusterMissing,
    HumidityClusterMissing,
    AttributeUpdateFailed(ZclStatus),
}

/// Apply a generated reading to the standard Temperature and Humidity
/// Measurement server clusters on an endpoint.
pub fn apply_synthetic_reading(
    clusters: &mut [ClusterRef<'_>],
    endpoint: u8,
    reading: SyntheticSensorReading,
) -> Result<(), ApplySyntheticReadingError> {
    let mut temperature_found = false;
    let mut humidity_found = false;

    for cluster_ref in clusters {
        if cluster_ref.endpoint != endpoint {
            continue;
        }

        match cluster_ref.cluster.cluster_id() {
            ClusterId::TEMPERATURE => {
                cluster_ref
                    .cluster
                    .attributes_mut()
                    .set_raw(
                        temperature::ATTR_MEASURED_VALUE,
                        ZclValue::I16(reading.temperature_centidegrees),
                    )
                    .map_err(ApplySyntheticReadingError::AttributeUpdateFailed)?;
                temperature_found = true;
            }
            ClusterId::HUMIDITY => {
                cluster_ref
                    .cluster
                    .attributes_mut()
                    .set_raw(
                        humidity::ATTR_MEASURED_VALUE,
                        ZclValue::U16(reading.humidity_centipercent),
                    )
                    .map_err(ApplySyntheticReadingError::AttributeUpdateFailed)?;
                humidity_found = true;
            }
            _ => {}
        }
    }

    if !temperature_found {
        return Err(ApplySyntheticReadingError::TemperatureClusterMissing);
    }
    if !humidity_found {
        return Err(ApplySyntheticReadingError::HumidityClusterMissing);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_zcl::clusters::Cluster;
    use zigbee_zcl::clusters::humidity::HumidityCluster;
    use zigbee_zcl::clusters::temperature::TemperatureCluster;

    #[test]
    fn samples_are_deterministic_and_phase_shifted() {
        let sensor = SyntheticSensor::new(2_150, 100, 5_000, 400);

        assert_eq!(
            sensor.sample(0),
            SyntheticSensorReading {
                temperature_centidegrees: 2_050,
                humidity_centipercent: 5_100,
            }
        );
        assert_eq!(
            sensor.sample(8),
            SyntheticSensorReading {
                temperature_centidegrees: 2_250,
                humidity_centipercent: 4_900,
            }
        );
        assert_eq!(sensor.sample(0), sensor.sample(16));
    }

    #[test]
    fn samples_clamp_to_zcl_ranges() {
        let sensor = SyntheticSensor::new(i16::MIN + 1, u16::MAX, 0, u16::MAX);
        let low = sensor.sample(0);
        let high = sensor.sample(8);

        assert_eq!(low.temperature_centidegrees, i16::MIN + 1);
        assert_eq!(low.humidity_centipercent, 10_000);
        assert_eq!(high.temperature_centidegrees, i16::MAX);
        assert_eq!(high.humidity_centipercent, 0);
    }

    #[test]
    fn reading_updates_standard_measurement_clusters() {
        let mut temperature = TemperatureCluster::new(-4_000, 12_500);
        let mut humidity = HumidityCluster::new(0, 10_000);
        let mut clusters = [
            ClusterRef {
                endpoint: 1,
                cluster: &mut temperature,
            },
            ClusterRef {
                endpoint: 1,
                cluster: &mut humidity,
            },
        ];
        let reading = SyntheticSensorReading {
            temperature_centidegrees: 2_225,
            humidity_centipercent: 5_125,
        };

        apply_synthetic_reading(&mut clusters, 1, reading).unwrap();

        assert_eq!(
            temperature
                .attributes()
                .get(temperature::ATTR_MEASURED_VALUE),
            Some(&ZclValue::I16(2_225))
        );
        assert_eq!(
            humidity.attributes().get(humidity::ATTR_MEASURED_VALUE),
            Some(&ZclValue::U16(5_125))
        );
    }
}
