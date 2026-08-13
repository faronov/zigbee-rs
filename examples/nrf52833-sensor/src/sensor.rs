//! Recoverable external I2C environmental-sensor wiring.
//!
//! Uses the shared `zigbee-bme280` / `zigbee-sht3x` drivers over the
//! board's async TWISPI0 bus, following the same recoverable
//! probe-then-read pattern as `examples/nrf52840-sensor/src/sensor.rs`
//! (from which this is the PCA10100 board-typed counterpart: only the
//! board and product crate names differ).
//!
//! Only one of `sensor-bme280` / `sensor-sht31` is expected to be enabled;
//! if both are, BME280 takes priority.

#[cfg(feature = "sensor-bme280")]
#[allow(unused_imports)] // `Measurement` is part of this module's public API
pub use bme280_sensor::{Measurement, Sensor};
#[cfg(all(feature = "sensor-sht31", not(feature = "sensor-bme280")))]
#[allow(unused_imports)] // `Measurement` is part of this module's public API
pub use sht31_sensor::{Measurement, Sensor};

#[cfg(feature = "sensor-bme280")]
mod bme280_sensor {
    use embassy_time::Delay;
    use nrf52833_dk::SensorI2c;
    use nrf52833_sensor_product::profile::pressure_pa_to_zcl;
    use zigbee_bme280::asynch::Bme280;
    use zigbee_bme280::{Chip, Oversampling, SamplingConfig, PRIMARY_ADDRESS};

    /// x8 oversampling on all three channels.
    const SAMPLING: SamplingConfig = SamplingConfig {
        temperature: Oversampling::X8,
        pressure: Oversampling::X8,
        humidity: Oversampling::X8,
    };

    pub struct Measurement {
        pub temperature_centi_celsius: i16,
        pub humidity_centi_percent: u16,
        /// In the Pressure cluster's whole-hPa representation.
        pub pressure_hpa: i16,
    }

    pub struct Sensor<'d> {
        i2c: Option<SensorI2c<'d>>,
        bme: Option<Bme280<SensorI2c<'d>>>,
    }

    impl<'d> Sensor<'d> {
        pub const fn new(i2c: SensorI2c<'d>) -> Self {
            Self {
                i2c: Some(i2c),
                bme: None,
            }
        }

        async fn probe(&mut self) -> bool {
            let Some(i2c) = self.i2c.take() else {
                return self.bme.is_some();
            };
            let mut sensor = Bme280::new(i2c, PRIMARY_ADDRESS);
            // Reject a BMP280 on this address the same way the original
            // firmware's explicit chip-id check did: this profile always
            // reports humidity, which a BMP280 does not have.
            let ready = matches!(sensor.init(&mut Delay).await, Ok(Chip::Bme280))
                && sensor.set_sampling(SAMPLING).await.is_ok();
            if ready {
                self.bme = Some(sensor);
                true
            } else {
                self.i2c = Some(sensor.release());
                false
            }
        }

        async fn read_active(&mut self) -> Option<Measurement> {
            let reading = {
                let sensor = self.bme.as_mut()?;
                sensor.measure_forced(&mut Delay).await.ok()
            };
            match reading {
                Some(reading) => Some(Measurement {
                    temperature_centi_celsius: reading.temperature_centi_celsius as i16,
                    humidity_centi_percent: reading.humidity_centi_percent.unwrap_or(0),
                    pressure_hpa: pressure_pa_to_zcl(reading.pressure_pa),
                }),
                None => {
                    let sensor = self.bme.take()?;
                    self.i2c = Some(sensor.release());
                    None
                }
            }
        }

        pub async fn sample(&mut self) -> Option<Measurement> {
            if let Some(measurement) = self.read_active().await {
                return Some(measurement);
            }
            if !self.probe().await {
                return None;
            }
            self.read_active().await
        }
    }
    impl nrf_sensor_app::EnvironmentSource for Sensor<'_> {
        async fn sample(&mut self) -> Option<nrf_sensor_app::EnvironmentReading> {
            let measurement = Sensor::sample(self).await?;
            Some(nrf_sensor_app::EnvironmentReading {
                temperature_centi_celsius: measurement.temperature_centi_celsius,
                humidity_centi_percent: measurement.humidity_centi_percent,
                pressure_tenth_kpa: Some(measurement.pressure_hpa),
            })
        }

        fn log_reading(&self, reading: &nrf_sensor_app::EnvironmentReading) {
            defmt::info!(
                "T={}.{:02}°C H={}.{:02}% P={}hPa",
                reading.temperature_centi_celsius / 100,
                (reading.temperature_centi_celsius % 100).unsigned_abs(),
                reading.humidity_centi_percent / 100,
                reading.humidity_centi_percent % 100,
                reading.pressure_tenth_kpa.unwrap_or(0),
            );
        }
    }
}

#[cfg(all(feature = "sensor-sht31", not(feature = "sensor-bme280")))]
mod sht31_sensor {
    use embassy_time::{Duration, Timer};
    use nrf52833_dk::SensorI2c;
    use zigbee_sht3x::asynch::Sht3x;
    use zigbee_sht3x::PRIMARY_ADDRESS;

    pub struct Measurement {
        pub temperature_centi_celsius: i16,
        pub humidity_centi_percent: u16,
    }

    pub struct Sensor<'d> {
        i2c: Option<SensorI2c<'d>>,
        sht: Option<Sht3x<SensorI2c<'d>>>,
    }

    impl<'d> Sensor<'d> {
        pub const fn new(i2c: SensorI2c<'d>) -> Self {
            Self {
                i2c: Some(i2c),
                sht: None,
            }
        }

        async fn probe(&mut self) -> bool {
            let Some(i2c) = self.i2c.take() else {
                return self.sht.is_some();
            };
            let mut sensor = Sht3x::new(i2c, PRIMARY_ADDRESS);
            if sensor.soft_reset().await.is_err() {
                self.i2c = Some(sensor.release());
                return false;
            }
            Timer::after(Duration::from_millis(2)).await;
            if sensor.read_status().await.is_err() {
                self.i2c = Some(sensor.release());
                return false;
            }
            self.sht = Some(sensor);
            true
        }

        async fn read_active(&mut self) -> Option<Measurement> {
            let reading = {
                let sensor = self.sht.as_mut()?;
                Self::measure_with_retry(sensor).await
            };
            match reading {
                Some(reading) => Some(reading),
                None => {
                    let sensor = self.sht.take()?;
                    self.i2c = Some(sensor.release());
                    None
                }
            }
        }

        /// Trigger a single-shot measurement and read it back, retrying up
        /// to 3 times on an I2C-level read error (the sensor NACKs a read
        /// issued before its conversion completes). A CRC mismatch is not
        /// retried.
        async fn measure_with_retry(sensor: &mut Sht3x<SensorI2c<'d>>) -> Option<Measurement> {
            if sensor.start_measurement().await.is_err() {
                return None;
            }
            Timer::after(Duration::from_millis(20)).await;
            for attempt in 0..3u8 {
                match sensor.read_measurement().await {
                    Ok(measurement) => {
                        return Some(Measurement {
                            temperature_centi_celsius: measurement.temperature_centi_celsius,
                            humidity_centi_percent: measurement.humidity_centi_percent,
                        });
                    }
                    Err(zigbee_sht3x::Error::I2c(_)) if attempt < 2 => {
                        Timer::after(Duration::from_millis(5)).await;
                    }
                    Err(_) => return None,
                }
            }
            None
        }

        pub async fn sample(&mut self) -> Option<Measurement> {
            if let Some(measurement) = self.read_active().await {
                return Some(measurement);
            }
            if !self.probe().await {
                return None;
            }
            self.read_active().await
        }
    }
    impl nrf_sensor_app::EnvironmentSource for Sensor<'_> {
        async fn sample(&mut self) -> Option<nrf_sensor_app::EnvironmentReading> {
            let measurement = Sensor::sample(self).await?;
            Some(nrf_sensor_app::EnvironmentReading {
                temperature_centi_celsius: measurement.temperature_centi_celsius,
                humidity_centi_percent: measurement.humidity_centi_percent,
                pressure_tenth_kpa: None,
            })
        }

        fn log_reading(&self, reading: &nrf_sensor_app::EnvironmentReading) {
            defmt::info!(
                "T={}.{:02}°C H={}.{:02}%",
                reading.temperature_centi_celsius / 100,
                (reading.temperature_centi_celsius % 100).unsigned_abs(),
                reading.humidity_centi_percent / 100,
                reading.humidity_centi_percent % 100,
            );
        }
    }
}
