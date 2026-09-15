//! ESP32-H2 capabilities for the shared sleepy-sensor application.

use core::convert::Infallible;

use esp_hal::gpio::Output;
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, SensorStatus, StatusSink,
    Supervisor,
};

use crate::chip_temperature::H2TemperatureSensor;

/// GPIO8 status LED, active low.
pub struct ActiveLowStatus<'d> {
    led: Output<'d>,
}

impl<'d> ActiveLowStatus<'d> {
    pub const fn new(led: Output<'d>) -> Self {
        Self { led }
    }
}

impl StatusSink for ActiveLowStatus<'_> {
    fn set(&mut self, status: SensorStatus) {
        let on = match status {
            SensorStatus::Off => false,
            SensorStatus::Joining { on }
            | SensorStatus::Identifying { on }
            | SensorStatus::Reporting { on }
            | SensorStatus::Resetting { on } => on,
            SensorStatus::Joined { active } => active,
            SensorStatus::Ota | SensorStatus::Fault => true,
        };
        if on {
            self.led.set_low();
        } else {
            self.led.set_high();
        }
    }
}

/// H2 die temperature plus the example's existing synthetic humidity source.
pub struct H2Environment {
    sensor: H2TemperatureSensor,
    humidity_tick: u32,
}

impl H2Environment {
    pub const fn new(sensor: H2TemperatureSensor) -> Self {
        Self {
            sensor,
            humidity_tick: 0,
        }
    }
}

impl EnvironmentSource for H2Environment {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        self.humidity_tick = self.humidity_tick.wrapping_add(1);
        Ok(EnvironmentReading {
            temperature_centi_celsius: self.sensor.read_centi_celsius(),
            humidity_centi_percent: 5_000 + ((self.humidity_tick % 100) as u16) * 10,
            pressure_tenth_kpa: None,
        })
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct EspSupervisor;

impl Supervisor for EspSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        esp_hal::system::software_reset()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct EspDiagnostics;

impl Diagnostics for EspDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        esp_println::println!("[ESP32-H2] {:?}", event);
    }
}
