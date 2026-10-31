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
    use embassy_time::{Delay, Duration, Timer};
    use nrf52833_dk::SensorI2c;
    use nrf52833_sensor_product::profile::pressure_pa_to_zcl;
    use zigbee_bme280::asynch::Bme280;
    use zigbee_bme280::{Chip, Config, Filter, I2cInterface, Oversampling, PRIMARY_ADDRESS};
    use zigbee_sensor::asynch::Sensor as _;

    /// x8 oversampling on all three channels.
    const CONFIG: Config = Config {
        temperature: Oversampling::X8,
        pressure: Oversampling::X8,
        humidity: Oversampling::X8,
        filter: Filter::Off,
    };

    pub struct Measurement {
        pub temperature_centi_celsius: i16,
        pub humidity_centi_percent: u16,
        /// In the Pressure cluster's whole-hPa representation.
        pub pressure_hpa: i16,
    }

    pub struct Sensor<'d> {
        i2c: Option<SensorI2c<'d>>,
        bme: Option<Bme280<I2cInterface<SensorI2c<'d>>>>,
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
            let mut sensor = Bme280::new_i2c(i2c, PRIMARY_ADDRESS).with_config(CONFIG);
            // Reject a BMP280 on this address the same way the original
            // firmware's explicit chip-id check did: this profile always
            // reports humidity, which a BMP280 does not have.
            let ready = match sensor.soft_reset().await {
                Ok(wait) => {
                    Timer::after(Duration::from_micros(wait.into())).await;
                    matches!(sensor.init().await, Ok(Chip::Bme280))
                }
                Err(_) => false,
            };
            if ready {
                self.bme = Some(sensor);
                true
            } else {
                self.i2c = Some(sensor.release().i2c);
                false
            }
        }

        async fn read_active(&mut self) -> Option<Measurement> {
            let reading = {
                let sensor = self.bme.as_mut()?;
                match sensor.measure(&mut Delay).await {
                    // The wait is the datasheet maximum; allow one late finish.
                    Err(zigbee_sensor::Error::NotReady) => {
                        Timer::after(Duration::from_millis(2)).await;
                        sensor.read_measurement().await.ok()
                    }
                    reading => reading.ok(),
                }
            };
            match reading {
                Some(reading) => Some(Measurement {
                    temperature_centi_celsius: reading.temperature as i16,
                    humidity_centi_percent: reading.humidity.unwrap_or(0),
                    pressure_hpa: pressure_pa_to_zcl(reading.pressure),
                }),
                None => {
                    let sensor = self.bme.take()?;
                    self.i2c = Some(sensor.release().i2c);
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
        type Error = ();

        async fn sample(&mut self) -> Result<nrf_sensor_app::EnvironmentReading, Self::Error> {
            let measurement = Sensor::sample(self).await.ok_or(())?;
            Ok(nrf_sensor_app::EnvironmentReading {
                temperature_centi_celsius: measurement.temperature_centi_celsius,
                humidity_centi_percent: measurement.humidity_centi_percent,
                pressure_tenth_kpa: Some(measurement.pressure_hpa),
            })
        }
    }
}

#[cfg(all(feature = "sensor-sht31", not(feature = "sensor-bme280")))]
mod sht31_sensor {
    use embassy_time::{Duration, Timer};
    use nrf52833_dk::SensorI2c;
    use zigbee_sensor::asynch::Sensor as _;
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
            let Ok(wait) = sensor.soft_reset().await else {
                self.i2c = Some(sensor.release());
                return false;
            };
            Timer::after(Duration::from_micros(wait.into())).await;
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
            let Ok(wait) = sensor.start_measurement().await else {
                return None;
            };
            Timer::after(Duration::from_micros(wait.into())).await;
            for attempt in 0..3u8 {
                match sensor.read_measurement().await {
                    Ok(measurement) => {
                        return Some(Measurement {
                            temperature_centi_celsius: measurement.temperature as i16,
                            humidity_centi_percent: measurement.humidity,
                        });
                    }
                    Err(zigbee_sensor::Error::Bus(_)) if attempt < 2 => {
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
        type Error = ();

        async fn sample(&mut self) -> Result<nrf_sensor_app::EnvironmentReading, Self::Error> {
            let measurement = Sensor::sample(self).await.ok_or(())?;
            Ok(nrf_sensor_app::EnvironmentReading {
                temperature_centi_celsius: measurement.temperature_centi_celsius,
                humidity_centi_percent: measurement.humidity_centi_percent,
                pressure_tenth_kpa: None,
            })
        }
    }
}
