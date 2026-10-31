//! BRD4183A capability adapters for the shared sleepy-sensor lifecycle.
//!
//! Sleep depths:
//! - `Active` (fast poll): radio state untouched; Embassy races the BTN0
//!   edge against an RTCC timer while the executor idles with WFE (EM1).
//! - `Idle`: radio idled through `SoftMacCore::sleep`, then the same wait.
//! - `Retention` (slow poll): radio idled, VCOM flushed, EM2 until RTCC CC1
//!   or BTN0; `SoftMacCore::wake` then runs `Radio::resume` (RAC/FRC state
//!   is lost in EM2; the HFXO restarts on demand) before any traffic.

use core::{
    cell::RefCell,
    convert::Infallible,
    future::poll_fn,
    sync::atomic::{AtomicBool, Ordering},
    task::{Poll, Waker},
};

use cortex_m::interrupt::Mutex;
use efr32mg22_devkit::{Button0, Led0};
use efr32s2_hal::{power, rtcc, vectors::Interrupt};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use sensor_sed_app::{
    DiagnosticEvent, Diagnostics, EnvironmentReading, EnvironmentSource, SensorStatus, SleepDepth,
    StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_mac::efr32s2::Efr32s2SoftMac;

const HELD_POLL_MS: u32 = 25;

static BUTTON_PENDING: AtomicBool = AtomicBool::new(false);
static BUTTON_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));

/// Enable the BTN0 (PB0, EXTI0 → GPIO_EVEN) wake input after the board
/// configured the pin.
pub fn enable_button_interrupt() {
    BUTTON_PENDING.store(false, Ordering::Release);
    cortex_m::peripheral::NVIC::unpend(Interrupt::GPIO_EVEN);
    // SAFETY: the board token exclusively owns PB0/EXTI0 and this crate
    // provides the sole GPIO_EVEN handler.
    unsafe { cortex_m::peripheral::NVIC::unmask(Interrupt::GPIO_EVEN) };
}

/// GPIO_EVEN ISR body.
pub fn gpio_even_irq() {
    if efr32mg22_devkit::service_button0_interrupt() {
        BUTTON_PENDING.store(true, Ordering::Release);
        cortex_m::interrupt::free(|cs| {
            if let Some(waker) = BUTTON_WAKER.borrow(cs).borrow().as_ref() {
                waker.wake_by_ref();
            }
        });
    }
}

fn button_edge_pending() -> bool {
    BUTTON_PENDING.load(Ordering::Acquire)
}

fn take_button_edge() -> bool {
    BUTTON_PENDING.swap(false, Ordering::AcqRel)
}

async fn wait_for_button_edge() {
    poll_fn(|context| {
        if take_button_edge() {
            return Poll::Ready(());
        }
        cortex_m::interrupt::free(|cs| {
            *BUTTON_WAKER.borrow(cs).borrow_mut() = Some(context.waker().clone());
        });
        // Close the edge-between-test-and-registration race.
        if take_button_edge() {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await
}

fn clear_button_waker() {
    cortex_m::interrupt::free(|cs| {
        BUTTON_WAKER.borrow(cs).borrow_mut().take();
    });
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WakeError {
    Radio,
    Power,
}

/// BTN0 plus RTCC/EM2 wait implementation.
pub struct Mg22Wake {
    button: Button0,
}

impl Mg22Wake {
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

    fn retention_wait(
        &mut self,
        mac: &mut Efr32s2SoftMac,
        timeout_ms: u32,
    ) -> Result<WakeReason, WakeError> {
        if take_button_edge() || self.button.take_interrupt() {
            return Ok(WakeReason::Button);
        }
        clear_button_waker();
        mac.sleep().map_err(|_| WakeError::Radio)?;
        crate::vcom_log::flush();

        let slept = power::sleep_em2(rtcc::ms_to_ticks(timeout_ms.max(1)), button_edge_pending);
        // Restore the radio on every path before reporting a sleep error.
        let woke = mac.wake();
        slept.map_err(|_| WakeError::Power)?;
        woke.map_err(|_| WakeError::Radio)?;

        if take_button_edge() || self.button.take_interrupt() {
            Ok(WakeReason::Button)
        } else {
            Ok(WakeReason::Timer)
        }
    }
}

impl WakeController<Efr32s2SoftMac> for Mg22Wake {
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
        mac: &mut Efr32s2SoftMac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            SleepDepth::Active => Ok(self.timer_or_button(request.timeout_ms).await),
            SleepDepth::Idle => {
                mac.sleep().map_err(|_| WakeError::Radio)?;
                let reason = self.timer_or_button(request.timeout_ms).await;
                mac.wake().map_err(|_| WakeError::Radio)?;
                Ok(reason)
            }
            SleepDepth::Retention => self.retention_wait(mac, request.timeout_ms),
        }
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        let started = Instant::now();
        while self.button.is_pressed() {
            if started.elapsed().as_millis() >= u64::from(duration_ms) {
                return true;
            }
            Timer::after(Duration::from_millis(u64::from(HELD_POLL_MS))).await;
        }
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

/// LED0 (PB1, active high) semantic status.
pub struct Led0Status {
    led: Led0,
}

impl Led0Status {
    pub const fn new(led: Led0) -> Self {
        Self { led }
    }
}

impl StatusSink for Led0Status {
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

/// EFR32 die temperature (EMU_TEMP, ±1–2 °C class, not ambient) and an
/// explicitly synthetic humidity.
#[derive(Debug, Default)]
pub struct DieTemperature {
    sequence: u32,
}

impl DieTemperature {
    pub const fn new() -> Self {
        Self { sequence: 0 }
    }
}

impl EnvironmentSource for DieTemperature {
    type Error = Infallible;

    async fn sample(&mut self) -> Result<EnvironmentReading, Self::Error> {
        let sequence = self.sequence;
        self.sequence = self.sequence.wrapping_add(1);
        Ok(EnvironmentReading {
            temperature_centi_celsius: power::die_temperature_centi_celsius(),
            humidity_centi_percent: 5_000 + ((sequence % 100) as u16) * 10,
            pressure_tenth_kpa: None,
        })
    }
}

/// Reset supervision; this composition makes no watchdog claim.
#[derive(Debug, Default, Clone, Copy)]
pub struct Efr32Supervisor;

impl Supervisor for Efr32Supervisor {
    fn heartbeat(&mut self) {}

    fn max_wait_ms(&self) -> Option<u32> {
        None
    }

    fn reset(&mut self) -> ! {
        crate::vcom_log::flush();
        cortex_m::peripheral::SCB::sys_reset()
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct VcomDiagnostics;

impl Diagnostics for VcomDiagnostics {
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
}

#[inline(never)]
pub fn halt() -> ! {
    efr32mg22_devkit::emergency_led_on();
    loop {
        cortex_m::asm::wfi();
    }
}
