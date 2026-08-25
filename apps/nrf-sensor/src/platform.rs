//! Nordic GPIO, clock, reset, and radio-power adapters.

use embassy_futures::select::{Either, select};
use embassy_nrf::gpio;
use embassy_time::{Duration, Instant, Timer};
use sensor_sed_app::{LifecyclePlatform, RadioPower, WakeReason};
use zigbee_mac::MacError;
use zigbee_mac::nrf::NrfMac;

pub type SensorMac =
    NrfMac<'static, embassy_nrf::peripherals::RADIO, embassy_nrf::peripherals::RNG>;

pub struct NrfPlatform {
    led: gpio::Output<'static>,
    button: gpio::Input<'static>,
}

impl NrfPlatform {
    pub const fn new(led: gpio::Output<'static>, button: gpio::Input<'static>) -> Self {
        Self { led, button }
    }
}

impl LifecyclePlatform for NrfPlatform {
    type Instant = Instant;

    fn now(&self) -> Self::Instant {
        Instant::now()
    }

    fn add_millis(instant: Self::Instant, duration_ms: u64) -> Self::Instant {
        instant + Duration::from_millis(duration_ms)
    }

    fn elapsed_millis(later: Self::Instant, earlier: Self::Instant) -> u64 {
        later.saturating_duration_since(earlier).as_millis()
    }

    async fn wait_for_wake(&mut self, timeout_ms: u64) -> WakeReason {
        match select(
            self.button.wait_for_falling_edge(),
            Timer::after(Duration::from_millis(timeout_ms)),
        )
        .await
        {
            Either::First(_) => WakeReason::Button,
            Either::Second(_) => WakeReason::Timer,
        }
    }

    async fn button_held_for(&mut self, duration_ms: u64) -> bool {
        matches!(
            select(
                self.button.wait_for_rising_edge(),
                Timer::after(Duration::from_millis(duration_ms)),
            )
            .await,
            Either::Second(_)
        )
    }

    async fn delay_ms(&mut self, duration_ms: u64) {
        Timer::after(Duration::from_millis(duration_ms)).await;
    }

    fn led_on(&mut self) {
        self.led.set_low();
    }

    fn led_off(&mut self) {
        self.led.set_high();
    }

    fn led_toggle(&mut self) {
        self.led.toggle();
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct NrfRadioPower;

impl RadioPower<SensorMac> for NrfRadioPower {
    fn prepare_for_sleep(&mut self, mac: &mut SensorMac) -> Result<(), MacError> {
        mac.enter_low_power_idle()
    }
}
