//! ESP32-C6 capabilities for the shared sleepy-sensor application.

use core::convert::Infallible;

use esp_hal::tsens::TemperatureSensor;
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, Supervisor,
};

/// C6 die temperature plus the example's existing synthetic humidity source.
pub struct C6Environment<'d> {
    sensor: TemperatureSensor<'d>,
    humidity_tick: u32,
}

impl<'d> C6Environment<'d> {
    pub fn new(sensor: TemperatureSensor<'d>) -> Self {
        sensor.power_down();
        Self {
            sensor,
            humidity_tick: 0,
        }
    }
}

impl EnvironmentSource for C6Environment<'_> {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        self.sensor.power_up();
        esp_hal::delay::Delay::new().delay_micros(300);
        let raw = self.sensor.get_temperature();
        self.sensor.power_down();
        // Preserve the existing fixed-point conversion used by this example.
        let temperature_centi_celsius =
            ((raw.raw_value as i32) * 4_386 - (raw.offset as i32) * 278_800 - 205_200) / 100;
        self.humidity_tick = self.humidity_tick.wrapping_add(1);

        Ok(EnvironmentReading {
            temperature_centi_celsius: temperature_centi_celsius as i16,
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
        esp_println::println!("[ESP32-C6] {:?}", event);
    }
}
