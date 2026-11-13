//! I2C3 sensor task: STTS22H temperature, ISM330DHCX motion/inclination
//! and VL53L0X distance, published to [`shared::SENSORS`] for the router
//! loop to apply to the ZCL profile.
//!
//! The IMU is sampled every [`IMU_PERIOD_MS`] so that short movements set
//! occupancy; temperature and distance every [`SLOW_PERIOD_TICKS`] IMU
//! samples. A part that fails to probe, or fails [`MAX_FAILURES`] samples
//! in a row, reads as `None` and is re-initialised every
//! [`RETRY_PERIOD_TICKS`]. I2C transfers are blocking (100 kHz, a few
//! bytes each); the ToF integration wait is asynchronous.

use core::cell::RefCell;

use embassy_time::{Instant, Timer};
use embedded_hal_bus::i2c::RefCellDevice;
use stm32wb5mm_dk::sensors::{ISM330DHCX_ADDRESS, STTS22H_ADDRESS, SensorBus};
use stm32wb5mm_dk_router_product::measure::{self, MotionDetector};
use zigbee_lsm6dso::{I2cInterface, Lsm6dso};
use zigbee_sensor::Error;
use zigbee_sensor::blocking::Sensor;
use zigbee_stts22h::Stts22h;
use zigbee_vl53l0x::Vl53l0x;

use crate::shared;

const IMU_PERIOD_MS: u64 = 200;
/// Temperature and distance: once per second.
const SLOW_PERIOD_TICKS: u32 = 5;
/// Re-probe a missing part every 30 s.
const RETRY_PERIOD_TICKS: u32 = 150;
/// Consecutive failed samples before a part counts as missing.
const MAX_FAILURES: u8 = 3;
/// Bounded result polling of one measurement.
const MAX_POLLS: u8 = 50;

type Device<'a> = RefCellDevice<'a, SensorBus>;

/// Per-part health: initialised, and consecutive sample failures.
#[derive(Default)]
struct Health {
    ready: bool,
    failures: u8,
}

impl Health {
    /// Value to publish: a fresh reading, `None` once the part counts as
    /// missing, else (a transient failure) the previous value.
    fn publish<T>(&self, fresh: Option<T>, previous: Option<T>) -> Option<T> {
        if fresh.is_some() || !self.ready {
            fresh
        } else {
            previous
        }
    }

    fn record<T>(&mut self, result: Result<T, ()>) -> Option<T> {
        match result {
            Ok(value) => {
                self.failures = 0;
                Some(value)
            }
            Err(()) => {
                self.failures = self.failures.saturating_add(1);
                if self.failures >= MAX_FAILURES {
                    self.ready = false;
                }
                None
            }
        }
    }
}

#[embassy_executor::task]
pub async fn run(bus: SensorBus) -> ! {
    let bus = RefCell::new(bus);
    let mut temperature = Stts22h::new(RefCellDevice::new(&bus), STTS22H_ADDRESS);
    let mut imu = Lsm6dso::new_i2c(RefCellDevice::new(&bus), ISM330DHCX_ADDRESS);
    let mut tof = Vl53l0x::new(RefCellDevice::new(&bus));
    let (mut t_health, mut imu_health, mut tof_health) =
        (Health::default(), Health::default(), Health::default());
    let mut motion = MotionDetector::new();

    let mut tick = 0u32;
    loop {
        if tick.is_multiple_of(RETRY_PERIOD_TICKS) {
            if !t_health.ready {
                t_health.ready = init_temperature(&mut temperature);
            }
            if !imu_health.ready {
                imu_health.ready = init_imu(&mut imu).await;
            }
            if !tof_health.ready {
                tof_health.ready = init_tof(&mut tof).await;
            }
            if tick == 0 {
                log::info!(
                    "[WB] sensors: stts22h={} imu={} tof={}",
                    t_health.ready,
                    imu_health.ready,
                    tof_health.ready
                );
            }
        }

        let accel = if imu_health.ready {
            imu_health.record(
                sample(&mut imu)
                    .await
                    .map(|m| [m.accel.x, m.accel.y, m.accel.z]),
            )
        } else {
            None
        };
        let now = Instant::now().as_millis();
        let hold_ms = u64::from(shared::MOTION_HOLD_SECS.get()) * 1_000;
        let occupied = accel.map(|a| motion.update(a, now, hold_ms));
        let inclination = accel.and_then(|[x, y, z]| measure::inclination_centi_deg(x, y, z));

        let slow = tick.is_multiple_of(SLOW_PERIOD_TICKS);
        let temperature_centi_c = if slow && t_health.ready {
            t_health.record(
                sample(&mut temperature)
                    .await
                    .map(|m| m.temperature.clamp(i16::MIN.into(), i16::MAX.into()) as i16),
            )
        } else {
            None
        };
        let distance_mm = if slow && tof_health.ready {
            tof_health.record(
                sample(&mut tof)
                    .await
                    .map(|m| measure::distance_mm(m.range_status == 0, m.distance_mm)),
            )
        } else {
            None
        };

        shared::SENSORS.update(|s| {
            if accel.is_some() || !imu_health.ready {
                s.occupied = occupied;
                // `None` also while the board accelerates.
                s.inclination_centi_deg = inclination;
            }
            if slow {
                s.temperature_centi_c =
                    t_health.publish(temperature_centi_c, s.temperature_centi_c);
                s.distance_mm = tof_health.publish(distance_mm, s.distance_mm);
            }
        });
        tick = tick.wrapping_add(1);
        Timer::after_millis(IMU_PERIOD_MS).await;
    }
}

fn init_temperature(sensor: &mut Stts22h<Device<'_>>) -> bool {
    sensor
        .probe()
        .and_then(|()| sensor.configure())
        .inspect_err(|error| log::warn!("[WB] STTS22H: {error:?}"))
        .is_ok()
}

async fn init_imu(sensor: &mut Lsm6dso<I2cInterface<Device<'_>>>) -> bool {
    match sensor.probe().and_then(|_| sensor.configure()) {
        Ok(wait_us) => {
            Timer::after_micros(u64::from(wait_us)).await;
            true
        }
        Err(error) => {
            log::warn!("[WB] ISM330DHCX: {error:?}");
            false
        }
    }
}

async fn init_tof(sensor: &mut Vl53l0x<Device<'_>>) -> bool {
    let wait_us = match sensor.probe().and_then(|()| sensor.begin_init()) {
        Ok(wait_us) => wait_us,
        Err(error) => {
            log::warn!("[WB] VL53L0X: {error:?}");
            return false;
        }
    };
    Timer::after_micros(u64::from(wait_us)).await;
    for _ in 0..20 {
        match sensor.finish_init() {
            Ok(()) => return true,
            Err(Error::NotReady) => {
                Timer::after_micros(u64::from(zigbee_vl53l0x::CAL_WAIT_US)).await
            }
            Err(error) => {
                log::warn!("[WB] VL53L0X init: {error:?}");
                return false;
            }
        }
    }
    log::warn!("[WB] VL53L0X calibration timed out");
    false
}

/// Start one measurement, wait for it and read it (bounded).
async fn sample<S: Sensor>(sensor: &mut S) -> Result<S::Measurement, ()> {
    let wait_us = sensor.start_measurement().map_err(drop)?;
    Timer::after_micros(u64::from(wait_us)).await;
    for _ in 0..MAX_POLLS {
        match sensor.read_measurement() {
            Err(Error::NotReady) => Timer::after_millis(2).await,
            Err(Error::Pending(us)) => Timer::after_micros(u64::from(us)).await,
            other => return other.map_err(drop),
        }
    }
    Err(())
}
