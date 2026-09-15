//! BRD4181A capability adapters for the shared sleepy-sensor lifecycle.
//!
//! `Idle` below is deliberately narrow: the radio block is gated while the
//! Embassy Cortex-M executor waits with WFE for SysTick or PD2. SysTick remains
//! active at 1 kHz, no low-frequency wake source is selected, and this module
//! does **not** enter or claim EFR32 EM2. `Retention` is rejected before any
//! wait because no qualified restore sequence exists for this platform.

use core::{
    convert::Infallible,
    future::poll_fn,
    sync::atomic::{AtomicBool, Ordering},
    task::{Poll, Waker},
};

use cortex_m::interrupt::Mutex;
use efr32mg21_devkit::{Button0, Led0};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, SensorStatus, SleepDepth,
    StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_mac::efr32s2::Efr32s2Mac;

use crate::vectors;

const HELD_POLL_MS: u32 = 25;

static BUTTON_PENDING: AtomicBool = AtomicBool::new(false);
static BUTTON_WAKER: Mutex<core::cell::RefCell<Option<Waker>>> =
    Mutex::new(core::cell::RefCell::new(None));

/// Enable BRD4181A's PD2/GPIO_EVEN wake input after the pin is configured.
pub fn enable_button_interrupt() {
    BUTTON_PENDING.store(false, Ordering::Release);
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::GpioEven);
    // SAFETY: the board has configured exclusive PD2/EXTI2 ownership and the
    // crate provides the sole GPIO_EVEN handler.
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::GpioEven) };
}

/// ISR entry called by the vector table.
pub fn gpio_even_irq() {
    if efr32mg21_devkit::service_button0_interrupt() {
        BUTTON_PENDING.store(true, Ordering::Release);
        cortex_m::interrupt::free(|critical| {
            if let Some(waker) = BUTTON_WAKER.borrow(critical).borrow().as_ref() {
                waker.wake_by_ref();
            }
        });
    }
}

fn take_button_edge() -> bool {
    BUTTON_PENDING.swap(false, Ordering::AcqRel)
}

async fn wait_for_button_edge() {
    poll_fn(|context| {
        if take_button_edge() {
            return Poll::Ready(());
        }

        cortex_m::interrupt::free(|critical| {
            *BUTTON_WAKER.borrow(critical).borrow_mut() = Some(context.waker().clone());
        });

        // Close the edge-arriving-between-test-and-waker-registration race.
        if take_button_edge() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await
}

fn clear_button_waker() {
    cortex_m::interrupt::free(|critical| {
        *BUTTON_WAKER.borrow(critical).borrow_mut() = None;
    });
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeError {
    RetentionUnsupported,
}

/// PD2 button and truthful Active/Idle wait implementation.
pub struct Brd4181aWake {
    button: Button0,
}

impl Brd4181aWake {
    pub const fn new(button: Button0) -> Self {
        Self { button }
    }

    async fn timer_or_button(&mut self, timeout_ms: u32) -> WakeReason {
        if take_button_edge() || self.button.take_interrupt() {
            return WakeReason::Button;
        }

        let selected = select(
            wait_for_button_edge(),
            Timer::after(Duration::from_millis(u64::from(timeout_ms))),
        )
        .await;
        clear_button_waker();

        if matches!(selected, Either::First(()))
            || take_button_edge()
            || self.button.take_interrupt()
        {
            WakeReason::Button
        } else {
            WakeReason::Timer
        }
    }
}

impl WakeController<Efr32s2Mac> for Brd4181aWake {
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
        mac: &mut Efr32s2Mac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => Ok(self.timer_or_button(request.timeout_ms).await),
            SleepDepth::Idle => {
                // Radio-off light CPU idle only. The 1 kHz SysTick remains on;
                // this is not EM2.
                mac.radio_sleep();
                let reason = self.timer_or_button(request.timeout_ms).await;
                mac.radio_wake();
                Ok(reason)
            }
            // Reject before timer registration or radio state changes.
            SleepDepth::Retention => Err(WakeError::RetentionUnsupported),
        }
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        if !self.button.is_pressed() {
            return false;
        }

        let started = Instant::now();
        loop {
            if !self.button.is_pressed() {
                return false;
            }
            if started.elapsed().as_millis() >= u64::from(duration_ms) {
                return true;
            }
            Timer::after(Duration::from_millis(u64::from(HELD_POLL_MS))).await;
        }
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

/// PB0 active-high implementation of the shared semantic status.
pub struct Pb0Status {
    led: Led0,
}

impl Pb0Status {
    pub const fn new(led: Led0) -> Self {
        Self { led }
    }
}

impl StatusSink for Pb0Status {
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
            self.led.on();
        } else {
            self.led.off();
        }
    }
}

/// Explicitly synthetic temperature/humidity source for platform bring-up.
#[derive(Debug, Default)]
pub struct SyntheticEnvironment {
    sequence: u32,
}

impl SyntheticEnvironment {
    pub const fn new() -> Self {
        Self { sequence: 0 }
    }
}

impl EnvironmentSource for SyntheticEnvironment {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        Ok(EnvironmentReading {
            temperature_centi_celsius: 2_250 + ((sequence % 50) as i16 - 25),
            humidity_centi_percent: 5_000 + ((sequence % 100) as u16) * 10,
            pressure_tenth_kpa: None,
        })
    }
}

/// Cortex-M reset supervision; this board composition makes no watchdog claim.
#[derive(Debug, Default, Clone, Copy)]
pub struct Efr32Supervisor;

impl Supervisor for Efr32Supervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        cortex_m::peripheral::SCB::sys_reset()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct RttDiagnostics;

impl Diagnostics for RttDiagnostics {
    fn record(&mut self, event: DiagnosticEvent) {
        log::info!("[SENSOR] {event:?}");
    }
}

pub async fn signal_boot(led: &mut Led0) {
    for _ in 0..2 {
        led.on();
        Timer::after(Duration::from_millis(100)).await;
        led.off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;
}

#[inline(never)]
pub fn halt() -> ! {
    efr32mg21_devkit::emergency_led_on();
    loop {
        cortex_m::asm::wfi();
    }
}
