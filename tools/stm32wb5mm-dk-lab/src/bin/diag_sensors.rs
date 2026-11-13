//! I2C3 sensors: bus scan, then STTS22H temperature, ISM330DHCX
//! acceleration and VL53L0X distance, five samples each, then all three
//! once per second. Pass = all three IDs probe and the values are plausible.

#![no_std]
#![no_main]

#[path = "../shared/lab.rs"]
#[macro_use]
mod lab;

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_time::Timer;
use embedded_hal::i2c::I2c as _;
use embedded_hal_bus::i2c::RefCellDevice;
use stm32wb5mm_dk::sensors::{self, ISM330DHCX_ADDRESS, STTS22H_ADDRESS};
use zigbee_lsm6dso::Lsm6dso;
use zigbee_sensor::Error;
use zigbee_sensor::blocking::Sensor;
use zigbee_stts22h::Stts22h;
use zigbee_vl53l0x::Vl53l0x;

async fn wait_us(us: u32) {
    Timer::after_micros(u64::from(us)).await;
}

/// Start, wait, and read until the result is ready (bounded).
async fn sample<S: Sensor>(sensor: &mut S) -> Result<S::Measurement, Error<S::BusError>> {
    let wait = sensor.start_measurement()?;
    wait_us(wait).await;
    for _ in 0..50 {
        match sensor.read_measurement() {
            Err(Error::NotReady) => Timer::after_millis(2).await,
            Err(Error::Pending(us)) => wait_us(us).await,
            other => return other,
        }
    }
    Err(Error::Timeout)
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = lab::init();
    let mut vcom = lab::Vcom::new(p.USART1, p.PB6);
    say!(
        vcom,
        "[DK][i2c] READY i2c3 scl=PB13 sda=PB11 {} Hz",
        sensors::I2C3_HZ
    );
    let bus = RefCell::new(sensors::i2c3(p.I2C3, p.PB13, p.PB11));

    {
        let mut dev = RefCellDevice::new(&bus);
        let mut found = 0u32;
        for address in 0x08u8..0x78 {
            let mut byte = [0u8];
            if dev.read(address, &mut byte).is_ok() {
                found += 1;
                say!(vcom, "[DK][i2c] ACK 0x{address:02X}");
            }
        }
        say!(vcom, "[DK][i2c] SCAN found={found}");
    }

    let mut temp = Stts22h::new(RefCellDevice::new(&bus), STTS22H_ADDRESS);
    let temp_ok = match temp.probe().and_then(|()| temp.configure()) {
        Ok(()) => {
            say!(vcom, "[DK][stts22h] PROBE ok");
            true
        }
        Err(error) => {
            say!(vcom, "[DK][stts22h] PROBE_FAIL {error:?}");
            false
        }
    };

    let mut imu = Lsm6dso::new_i2c(RefCellDevice::new(&bus), ISM330DHCX_ADDRESS);
    let imu_ok = match imu.probe() {
        Ok(variant) => match imu.configure() {
            Ok(wait) => {
                say!(
                    vcom,
                    "[DK][imu] PROBE ok variant={variant:?} wait={wait} us"
                );
                wait_us(wait).await;
                true
            }
            Err(error) => {
                say!(vcom, "[DK][imu] CONFIG_FAIL {error:?}");
                false
            }
        },
        Err(error) => {
            say!(vcom, "[DK][imu] PROBE_FAIL {error:?}");
            false
        }
    };

    let mut tof = Vl53l0x::new(RefCellDevice::new(&bus));
    let mut tof_ok = false;
    match tof.probe() {
        Ok(()) => match tof.begin_init() {
            Ok(wait) => {
                wait_us(wait).await;
                for _ in 0..20 {
                    match tof.finish_init() {
                        Ok(()) => {
                            tof_ok = true;
                            break;
                        }
                        Err(Error::NotReady) => wait_us(zigbee_vl53l0x::CAL_WAIT_US).await,
                        Err(error) => {
                            say!(vcom, "[DK][tof] INIT_FAIL {error:?}");
                            break;
                        }
                    }
                }
                say!(
                    vcom,
                    "[DK][tof] INIT ok={tof_ok} budget={} us",
                    tof.timing_budget_us()
                );
            }
            Err(error) => say!(vcom, "[DK][tof] BEGIN_FAIL {error:?}"),
        },
        Err(error) => say!(vcom, "[DK][tof] PROBE_FAIL {error:?}"),
    }

    for n in 0..20u32 {
        if temp_ok {
            match sample(&mut temp).await {
                Ok(m) => say!(vcom, "[DK][stts22h] n={n} T={} cC", m.temperature),
                Err(error) => say!(vcom, "[DK][stts22h] n={n} FAIL {error:?}"),
            }
        }
        if imu_ok {
            match sample(&mut imu).await {
                Ok(m) => say!(
                    vcom,
                    "[DK][imu] n={n} accel mg x={} y={} z={} gyro={:?}",
                    m.accel.x,
                    m.accel.y,
                    m.accel.z,
                    m.gyro
                ),
                Err(error) => say!(vcom, "[DK][imu] n={n} FAIL {error:?}"),
            }
        }
        if tof_ok {
            match sample(&mut tof).await {
                Ok(m) => say!(
                    vcom,
                    "[DK][tof] n={n} d={} mm status={} signal={} kcps",
                    m.distance_mm,
                    m.range_status,
                    m.signal_kcps
                ),
                Err(error) => say!(vcom, "[DK][tof] n={n} FAIL {error:?}"),
            }
        }
        Timer::after_millis(1000).await;
    }
    say!(vcom, "[DK][i2c] DONE");
    lab::halt()
}
