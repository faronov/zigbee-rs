#![no_std]
#![no_main]
#![feature(impl_trait_in_assoc_type)]

#[path = "../shared/fault.rs"]
mod fault;
#[path = "../shared/platform.rs"]
mod platform;
#[path = "../shared/time_driver.rs"]
mod time_driver;
#[path = "../shared/vectors.rs"]
mod vectors;

use cortex_m as _;
use embassy_time::{Duration, Timer};

async fn probe(
    mut i2c: efr32mg1_tradfri::SensorI2c,
) -> zigbee_sht3x::Sht3x<efr32mg1_tradfri::SensorI2c> {
    loop {
        for address in [
            zigbee_sht3x::PRIMARY_ADDRESS,
            zigbee_sht3x::SECONDARY_ADDRESS,
        ] {
            rtt_target::rprintln!("[EFR32][diag-sht] PROBE address=0x{:02X}", address);
            let mut sensor = zigbee_sht3x::Sht3x::new(i2c, address);
            if sensor.soft_reset().is_ok() {
                Timer::after(Duration::from_millis(2)).await;
                match sensor.read_status() {
                    Ok(status) => {
                        rtt_target::rprintln!(
                            "[EFR32][diag-sht] SHT_FOUND address=0x{:02X} \
                             status=0x{:04X} crc=ok",
                            address,
                            status.raw
                        );
                        return sensor;
                    }
                    Err(error) => rtt_target::rprintln!(
                        "[EFR32][diag-sht] STATUS_ERROR address=0x{:02X} error={:?}",
                        address,
                        error
                    ),
                }
            }
            i2c = sensor.release();
        }
        rtt_target::rprintln!("[EFR32][diag-sht] SHT_NOT_FOUND retry_ms=3000");
        Timer::after(Duration::from_secs(3)).await;
    }
}

#[embassy_executor::task]
async fn run(i2c: efr32mg1_tradfri::SensorI2c) -> ! {
    rtt_target::rprintln!(
        "[EFR32][diag-sht] I2C_READY controller=I2C0 sda=PC10 scl=PC11 loc=15 hz={}",
        efr32mg1_tradfri::SENSOR_I2C_HZ
    );
    let mut sensor = probe(i2c).await;
    let mut samples = 0u32;
    let mut errors = 0u32;
    loop {
        match sensor.start_measurement() {
            Ok(()) => {
                Timer::after(Duration::from_millis(20)).await;
                match sensor.read_measurement() {
                    Ok(measurement) => {
                        samples = samples.wrapping_add(1);
                        rtt_target::rprintln!(
                            "[EFR32][diag-sht] MEAS_OK seq={} errors={} address=0x{:02X} \
                             temp_centi_c={} humidity_centi_percent={} crc=ok",
                            samples,
                            errors,
                            sensor.address(),
                            measurement.temperature_centi_celsius,
                            measurement.humidity_centi_percent
                        );
                        platform::led_on();
                        Timer::after(Duration::from_millis(80)).await;
                        platform::led_off();
                    }
                    Err(error) => {
                        errors = errors.wrapping_add(1);
                        rtt_target::rprintln!(
                            "[EFR32][diag-sht] MEAS_READ_ERROR seq={} errors={} error={:?}",
                            samples,
                            errors,
                            error
                        );
                    }
                }
            }
            Err(error) => {
                errors = errors.wrapping_add(1);
                rtt_target::rprintln!(
                    "[EFR32][diag-sht] MEAS_START_ERROR seq={} errors={} error={:?}",
                    samples,
                    errors,
                    error
                );
            }
        }
        Timer::after(Duration::from_secs(2)).await;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    platform::init_small!("diag-sht");
    time_driver::init();
    let i2c = match efr32mg1_tradfri::sensor_i2c() {
        Ok(i2c) => i2c,
        Err(error) => {
            rtt_target::rprintln!("[EFR32][diag-sht] I2C_FATAL error={:?}", error);
            platform::halt()
        }
    };
    static EXECUTOR: static_cell::StaticCell<embassy_executor::Executor> =
        static_cell::StaticCell::new();
    EXECUTOR
        .init(embassy_executor::Executor::new())
        .run(|spawner| spawner.must_spawn(run(i2c)))
}
