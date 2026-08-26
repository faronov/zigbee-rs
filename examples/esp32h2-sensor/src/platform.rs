//! ESP32-H2 capabilities for the shared sleepy-sensor application.

use core::convert::Infallible;

use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::{Input, Output};
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, SensorStatus, SleepDepth,
    StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_mac::esp::EspMac;

use crate::chip_temperature::H2TemperatureSensor;

const BUTTON_POLL_MS: u32 = 10;
const HELD_POLL_MS: u32 = 25;

fn elapsed_ms(later: Instant, earlier: Instant) -> u32 {
    later
        .saturating_duration_since(earlier)
        .as_millis()
        .min(u32::MAX as u64) as u32
}

/// Active-only timer/button wait.
///
/// The current ESP radio and polling time driver cannot perform the atomic
/// quiesce/sleep/restore operation required by deeper SensorApp wait modes.
pub struct ActiveWake<'d> {
    button: Input<'d>,
}

impl<'d> ActiveWake<'d> {
    pub const fn new(button: Input<'d>) -> Self {
        Self { button }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeError {
    UnsupportedDepth(SleepDepth),
}

impl<'button, 'radio> WakeController<EspMac<'radio>> for ActiveWake<'button> {
    type Mark = Instant;
    type Error = WakeError;

    fn mark(&self) -> Self::Mark {
        Instant::now()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark + Duration::from_millis(duration_ms as u64)
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        elapsed_ms(later, earlier)
    }

    async fn wait(
        &mut self,
        _mac: &mut EspMac<'radio>,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        if request.sleep_depth != SleepDepth::Active {
            return Err(WakeError::UnsupportedDepth(request.sleep_depth));
        }

        let started = Instant::now();
        loop {
            if self.button.is_low() {
                return Ok(WakeReason::Button);
            }

            let elapsed = elapsed_ms(Instant::now(), started);
            if elapsed >= request.timeout_ms {
                return Ok(WakeReason::Timer);
            }

            let delay_ms = (request.timeout_ms - elapsed).min(BUTTON_POLL_MS);
            Timer::after(Duration::from_millis(delay_ms as u64)).await;
        }
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        let started = Instant::now();
        while self.button.is_low() {
            if elapsed_ms(Instant::now(), started) >= duration_ms {
                return true;
            }
            Timer::after(Duration::from_millis(HELD_POLL_MS as u64)).await;
        }
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(duration_ms as u64)).await;
    }
}

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
