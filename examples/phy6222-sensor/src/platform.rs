//! PHY62x2 adapters for the shared sleepy-sensor lifecycle.

use embassy_time::{Duration, Instant, Timer};
use phy62x2_evk::{StatusLed, UserButton};
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, SensorStatus, SleepDepth, StatusSink, Supervisor, WaitRequest,
    WakeController, WakeReason,
};
use zigbee_mac::phy6222::Phy6222Mac;

const BUTTON_SAMPLE_MS: u32 = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeError {
    RetentionUnsupported,
}

pub struct PhyWakeController {
    button: UserButton,
}

impl PhyWakeController {
    pub const fn new(button: UserButton) -> Self {
        Self { button }
    }

    async fn wait_for_button_or_timer(&self, timeout_ms: u32) -> WakeReason {
        let started = Instant::now();
        loop {
            if self.button.is_pressed() {
                return WakeReason::Button;
            }
            let elapsed = Instant::now()
                .saturating_duration_since(started)
                .as_millis()
                .min(u64::from(u32::MAX)) as u32;
            if elapsed >= timeout_ms {
                return WakeReason::Timer;
            }
            let remaining = timeout_ms - elapsed;
            Timer::after(Duration::from_millis(u64::from(
                remaining.min(BUTTON_SAMPLE_MS),
            )))
            .await;
        }
    }
}

impl WakeController<Phy6222Mac> for PhyWakeController {
    type Mark = Instant;
    type Error = WakeError;

    fn mark(&self) -> Self::Mark {
        Instant::now()
    }

    fn add_ms(mark: Self::Mark, duration_ms: u32) -> Self::Mark {
        mark + Duration::from_millis(u64::from(duration_ms))
    }

    fn elapsed_ms(later: Self::Mark, earlier: Self::Mark) -> u32 {
        later
            .saturating_duration_since(earlier)
            .as_millis()
            .min(u64::from(u32::MAX)) as u32
    }

    async fn wait(
        &mut self,
        mac: &mut Phy6222Mac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        let restore_radio = match request.sleep_depth {
            SleepDepth::Active => false,
            SleepDepth::Idle => {
                mac.radio_sleep();
                true
            }
            SleepDepth::Retention => return Err(WakeError::RetentionUnsupported),
        };

        let reason = self.wait_for_button_or_timer(request.timeout_ms).await;
        if restore_radio {
            mac.radio_wake();
        }
        Ok(reason)
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        let started = Instant::now();
        while self.button.is_pressed() {
            if Self::elapsed_ms(Instant::now(), started) >= duration_ms {
                return true;
            }
            Timer::after(Duration::from_millis(u64::from(BUTTON_SAMPLE_MS))).await;
        }
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

pub struct PhyStatus {
    led: StatusLed,
}

impl PhyStatus {
    pub const fn new(led: StatusLed) -> Self {
        Self { led }
    }

    pub fn fault(&mut self) {
        self.set(SensorStatus::Fault);
    }
}

impl StatusSink for PhyStatus {
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
        self.led.set_on(on);
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct PhySupervisor;

impl Supervisor for PhySupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct PhyDiagnostics;

impl Diagnostics for PhyDiagnostics {
    fn record(&mut self, _event: DiagnosticEvent) {}
}
