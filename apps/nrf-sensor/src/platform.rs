//! Nordic GPIO, clock, reset, and atomic radio-wait adapters.

use embassy_futures::select::{Either, select};
use embassy_nrf::gpio;
use embassy_time::{Duration, Instant, Timer};
use sensor_sed_app::{
    SensorStatus, SleepDepth, StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_mac::MacError;
use zigbee_mac::nrf::NrfMac;

pub type SensorMac =
    NrfMac<'static, embassy_nrf::peripherals::RADIO, embassy_nrf::peripherals::RNG>;

pub struct NrfWakeController {
    button: gpio::Input<'static>,
}

impl NrfWakeController {
    pub const fn new(button: gpio::Input<'static>) -> Self {
        Self { button }
    }
}

impl WakeController<SensorMac> for NrfWakeController {
    type Mark = Instant;
    type Error = MacError;

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
        mac: &mut SensorMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => {}
            SleepDepth::Idle => mac.enter_low_power_idle()?,
            SleepDepth::Retention => return Err(MacError::Unsupported),
        }

        let reason = match select(
            self.button.wait_for_falling_edge(),
            Timer::after(Duration::from_millis(u64::from(request.timeout_ms))),
        )
        .await
        {
            Either::First(_) => WakeReason::Button,
            Either::Second(_) => WakeReason::Timer,
        };
        // NrfMac deliberately keeps RADIO disabled here. Its next normal
        // Embassy RX/TX operation guarantees the complete DISABLED-to-active
        // transition, satisfying WakeController's lazy-readiness contract.
        Ok(reason)
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        matches!(
            select(
                self.button.wait_for_rising_edge(),
                Timer::after(Duration::from_millis(u64::from(duration_ms))),
            )
            .await,
            Either::Second(_)
        )
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

/// Timer-only Nordic wait adapter for boards without a usable application
/// button.
#[derive(Debug, Default, Clone, Copy)]
pub struct NrfTimerWakeController;

impl WakeController<SensorMac> for NrfTimerWakeController {
    type Mark = Instant;
    type Error = MacError;

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
        mac: &mut SensorMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => {}
            SleepDepth::Idle => mac.enter_low_power_idle()?,
            SleepDepth::Retention => return Err(MacError::Unsupported),
        }
        Timer::after(Duration::from_millis(u64::from(request.timeout_ms))).await;
        Ok(WakeReason::Timer)
    }

    async fn button_held_for(&mut self, _duration_ms: u32) -> bool {
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

/// Polarity-aware Nordic semantic status LED.
pub struct NrfPolarityStatus<const ACTIVE_LOW: bool> {
    led: gpio::Output<'static>,
}

impl<const ACTIVE_LOW: bool> NrfPolarityStatus<ACTIVE_LOW> {
    pub const fn new(led: gpio::Output<'static>) -> Self {
        Self { led }
    }

    fn set_on(&mut self, on: bool) {
        if on == ACTIVE_LOW {
            self.led.set_low();
        } else {
            self.led.set_high();
        }
    }
}

impl<const ACTIVE_LOW: bool> StatusSink for NrfPolarityStatus<ACTIVE_LOW> {
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
        self.set_on(on);
    }
}

/// Source-compatible active-low status adapter for the Nordic DKs.
pub type NrfStatus = NrfPolarityStatus<true>;

#[derive(Debug, Default, Clone, Copy)]
pub struct NrfSupervisor;

impl Supervisor for NrfSupervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}
