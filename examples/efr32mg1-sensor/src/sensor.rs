//! Recoverable SHT3x discovery for the production sensor.

use embassy_time::{Duration, Timer};
use sensor_sed_app::{EnvironmentReading, EnvironmentSource};

pub type Sht3x = zigbee_sht3x::Sht3x<efr32mg1_tradfri::SensorI2c>;

pub struct Sensor {
    i2c: Option<efr32mg1_tradfri::SensorI2c>,
    sht: Option<Sht3x>,
}

impl Sensor {
    pub const fn new(i2c: efr32mg1_tradfri::SensorI2c) -> Self {
        Self {
            i2c: Some(i2c),
            sht: None,
        }
    }

    async fn probe(&mut self) -> bool {
        let Some(mut i2c) = self.i2c.take() else {
            return self.sht.is_some();
        };

        for address in [
            zigbee_sht3x::PRIMARY_ADDRESS,
            zigbee_sht3x::SECONDARY_ADDRESS,
        ] {
            let mut sensor = zigbee_sht3x::Sht3x::new(i2c, address);
            if sensor.soft_reset().is_ok() {
                Timer::after(Duration::from_millis(2)).await;
                if sensor.read_status().is_ok() {
                    self.sht = Some(sensor);
                    return true;
                }
            }

            i2c = sensor.release();
        }

        self.i2c = Some(i2c);
        false
    }

    async fn read_active(&mut self) -> Option<zigbee_sht3x::Measurement> {
        let measurement = {
            let sensor = self.sht.as_mut()?;
            if sensor.start_measurement().is_err() {
                None
            } else {
                Timer::after(Duration::from_millis(20)).await;
                sensor.read_measurement().ok()
            }
        };
        if measurement.is_none() {
            let sensor = self.sht.take()?;
            self.i2c = Some(sensor.release());
        }
        measurement
    }

    pub async fn sample(&mut self) -> Option<zigbee_sht3x::Measurement> {
        if let Some(measurement) = self.read_active().await {
            return Some(measurement);
        }
        if !self.probe().await {
            return None;
        }
        self.read_active().await
    }
}

impl EnvironmentSource for Sensor {
    type Error = ();

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        let measurement = Sensor::sample(self).await.ok_or(())?;
        Ok(EnvironmentReading {
            temperature_centi_celsius: measurement.temperature_centi_celsius,
            humidity_centi_percent: measurement.humidity_centi_percent,
            pressure_tenth_kpa: None,
        })
    }
}
