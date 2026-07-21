//! SHT3x discovery for the production sensor.

use embassy_time::{Duration, Timer};

use crate::platform;

pub type Sht3x = zigbee_sht3x::Sht3x<efr32mg1_tradfri::SensorI2c>;

pub async fn probe(mut i2c: efr32mg1_tradfri::SensorI2c) -> Sht3x {
    loop {
        for address in [
            zigbee_sht3x::PRIMARY_ADDRESS,
            zigbee_sht3x::SECONDARY_ADDRESS,
        ] {
            let mut sensor = zigbee_sht3x::Sht3x::new(i2c, address);
            if sensor.soft_reset().is_ok() {
                Timer::after(Duration::from_millis(2)).await;
                if sensor.read_status().is_ok() {
                    return sensor;
                }
            }
            i2c = sensor.release();
        }

        for _ in 0..2 {
            platform::led_on();
            Timer::after(Duration::from_millis(100)).await;
            platform::led_off();
            Timer::after(Duration::from_millis(100)).await;
        }
        Timer::after(Duration::from_millis(2_600)).await;
    }
}
