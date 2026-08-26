//! ESP32-C6 capabilities for the shared sleepy-sensor application.

use core::convert::Infallible;

use embassy_time::{Duration, Instant, Timer};
use esp_hal::gpio::Input;
use esp_hal::tsens::TemperatureSensor;
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, SleepDepth, Supervisor,
    WaitRequest, WakeController, WakeReason,
};
use zigbee_mac::esp::EspMac;

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

/// C6 die temperature plus the example's existing synthetic humidity source.
pub struct C6Environment<'d> {
    sensor: TemperatureSensor<'d>,
    humidity_tick: u32,
}

impl<'d> C6Environment<'d> {
    pub const fn new(sensor: TemperatureSensor<'d>) -> Self {
        Self {
            sensor,
            humidity_tick: 0,
        }
    }
}

impl EnvironmentSource for C6Environment<'_> {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        let raw = self.sensor.get_temperature();
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
