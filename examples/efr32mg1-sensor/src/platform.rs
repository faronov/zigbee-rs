//! TRÅDFRI board startup, status LED, and PB13 wake handling.
//!
//! Consumes typed [`BoardResources`](efr32mg1_tradfri::resources::BoardResources)
//! tokens passed from the composition root (`main.rs`). PA0 is claimed as a
//! direct GPIO LED (not PWM). Product code separately selects the external
//! flash owner for OTA.

use core::{
    cell::RefCell,
    future::poll_fn,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    task::{Poll, Waker},
};

use cortex_m::interrupt::Mutex;
use efr32mg1_hal::pm;
use efr32mg1_tradfri::resources::{ButtonToken, Pa0Output};
use efr32mg1_tradfri::{Button, Led};
use embassy_futures::select::{Either, select};
use embassy_time::{Duration, Instant, Timer};
use sensor_sed_app::{
    SensorStatus, SleepDepth, StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use zigbee_mac::efr32::Efr32Mac;

use crate::{time_driver, vectors};

/// LED handle — a ZST proxy to the PA0 hardware register.
/// Hardware initialization is performed by `Pa0Output::into_led()` during `init()`.
static LED: Led = Led::new();
/// Button handle — a ZST proxy to the PB13 hardware register.
/// Hardware initialization is performed by `ButtonToken::into_button()` during `init()`.
static BUTTON: Button = Button::new();
static BUTTON_EDGE_PENDING: AtomicBool = AtomicBool::new(false);
static BUTTON_WAKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
static STACK_CANARY_LIMIT: AtomicU32 = AtomicU32::new(0);

const STACK_CANARY: u32 = 0xE2F3_A4B5;
const STACK_CANARY_HEADROOM: usize = 256;

unsafe extern "C" {
    static _stack_end: u8;
    static _stack_start: u8;
}

#[unsafe(no_mangle)]
pub extern "C" fn GPIO_ODD() {
    if BUTTON.take_interrupt() {
        BUTTON_EDGE_PENDING.store(true, Ordering::Release);
        let waker = cortex_m::interrupt::free(|cs| BUTTON_WAKER.borrow(cs).borrow_mut().take());
        if let Some(waker) = waker {
            waker.wake();
        }
    }
}

/// Initialize platform services (RTT, clocks, time driver) and claim
/// exclusive board resources: PA0 as GPIO LED and PB13 button with interrupt.
///
/// The consumed PA0 token enforces at the type level that TIMER0 PWM cannot
/// also be configured through the typed board-resource path.
pub fn init(pa0: Pa0Output, button: ButtonToken) {
    let channels = rtt_target::rtt_init! {
        up: {
            0: {
                size: 128,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
        down: {
            0: {
                size: 16,
                mode: rtt_target::ChannelMode::NoBlockSkip,
                name: "Terminal"
            }
        }
    };
    rtt_target::set_print_channel(channels.up.0);

    // PA0 → direct GPIO LED (excludes TIMER0 PWM on this pin).
    // Token consumption initializes hardware; the static LED handle is a
    // ZST that proxies the same memory-mapped GPIO register.
    let _led = pa0.into_led();
    LED.off();

    // PB13 → button with interrupt.
    let _btn = button.into_button();
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::GpioOdd);
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::GpioOdd) };

    // System clocks and time driver
    if efr32mg1_tradfri::init_clocks().is_err() {
        halt_with_led();
    }
    time_driver::init();
}

pub fn init_stack_watermark() {
    let stack_end = core::ptr::addr_of!(_stack_end) as usize;
    let limit =
        (cortex_m::register::msp::read() as usize).saturating_sub(STACK_CANARY_HEADROOM) & !3;
    if limit <= stack_end {
        halt_with_led();
    }

    let mut cursor = stack_end;
    while cursor < limit {
        unsafe { core::ptr::write_volatile(cursor as *mut u32, STACK_CANARY) };
        cursor += core::mem::size_of::<u32>();
    }
    STACK_CANARY_LIMIT.store(limit as u32, Ordering::Release);
}

pub fn stack_high_water_bytes() -> usize {
    let limit = STACK_CANARY_LIMIT.load(Ordering::Acquire) as usize;
    if limit == 0 {
        return 0;
    }

    let stack_end = core::ptr::addr_of!(_stack_end) as usize;
    let stack_start = core::ptr::addr_of!(_stack_start) as usize;
    let mut cursor = stack_end;
    while cursor < limit {
        let word = unsafe { core::ptr::read_volatile(cursor as *const u32) };
        if word != STACK_CANARY {
            break;
        }
        cursor += core::mem::size_of::<u32>();
    }
    stack_start.saturating_sub(cursor)
}

pub async fn signal_boot() {
    for _ in 0..3 {
        led_on();
        Timer::after(Duration::from_millis(100)).await;
        led_off();
        Timer::after(Duration::from_millis(100)).await;
    }
    Timer::after(Duration::from_millis(500)).await;
}

#[inline(always)]
pub fn button_is_pressed() -> bool {
    BUTTON.is_pressed()
}

#[inline(always)]
pub fn button_edge_pending() -> bool {
    BUTTON_EDGE_PENDING.load(Ordering::Acquire)
}

#[inline(always)]
pub fn take_button_edge() -> bool {
    BUTTON_EDGE_PENDING.swap(false, Ordering::AcqRel)
}

async fn wait_for_button_edge() {
    poll_fn(|context| {
        cortex_m::interrupt::free(|critical_section| {
            if BUTTON_EDGE_PENDING.load(Ordering::Acquire) {
                Poll::Ready(())
            } else {
                let mut slot = BUTTON_WAKER.borrow(critical_section).borrow_mut();
                if slot
                    .as_ref()
                    .is_none_or(|waker| !waker.will_wake(context.waker()))
                {
                    *slot = Some(context.waker().clone());
                }
                Poll::Pending
            }
        })
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
    UnsupportedSleepDepth,
    Power,
    Clock,
}

/// RTCC-backed atomic wait for the shared sensor lifecycle.
///
/// Fast polling uses `Active`: HFXO and radio ownership remain live while
/// Embassy races the PB13 edge against RTCC CC0. Steady-state polling uses
/// `Retention`: the radio is quiesced, RTCC CC1 wakes EM2, and the Series-1
/// DCDC gate, HFXO, and radio are all restored before a successful return.
#[derive(Debug, Default, Clone, Copy)]
pub struct Efr32WakeController;

impl Efr32WakeController {
    pub const fn new() -> Self {
        Self
    }

    async fn active_wait(timeout_ms: u32) -> WakeReason {
        if take_button_edge() {
            return WakeReason::Button;
        }

        let selected = select(
            wait_for_button_edge(),
            Timer::after(Duration::from_millis(u64::from(timeout_ms))),
        )
        .await;
        clear_button_waker();
        let edge_pending = take_button_edge();
        if matches!(selected, Either::First(_)) || edge_pending {
            WakeReason::Button
        } else {
            WakeReason::Timer
        }
    }

    fn retention_wait(mac: &mut Efr32Mac, timeout_ms: u32) -> Result<WakeReason, WakeError> {
        if take_button_edge() {
            return Ok(WakeReason::Button);
        }

        clear_button_waker();
        mac.radio_sleep();
        cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::FrcPri);

        let ticks = pm::ms_to_ticks(timeout_ms.max(1), pm::LFRCO_HZ).max(1);
        let sleep_result = pm::sleep_for_ticks_polled_until(ticks, button_edge_pending)
            .map_err(|_| WakeError::Power);

        // `sleep_for_ticks_polled_until` applies the DCDC LNHS workaround
        // before every EM2 entry. Repeat it after wake, as the proven native
        // runtime does on every loop, before restoring the high-frequency
        // clock and radio.
        let dcdc_result = pm::apply_dcdc_lnhs_workaround().map_err(|_| WakeError::Power);
        let clock_result = efr32mg1_tradfri::init_clocks().map_err(|_| WakeError::Clock);
        if clock_result.is_ok() {
            mac.radio_wake();
        }

        sleep_result?;
        dcdc_result?;
        clock_result?;

        if take_button_edge() {
            Ok(WakeReason::Button)
        } else {
            Ok(WakeReason::Timer)
        }
    }
}

impl WakeController<Efr32Mac> for Efr32WakeController {
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
        mac: &mut Efr32Mac,
        request: WaitRequest,
    ) -> Result<WakeReason, Self::Error> {
        match request.sleep_depth {
            // Do not touch the MAC here: preserving its live direct-RX state
            // is the EFR32 fast window. SensorApp follows this wait with its
            // bounded four-round parent-poll drain.
            SleepDepth::Active => Ok(Self::active_wait(request.timeout_ms).await),
            SleepDepth::Retention => Self::retention_wait(mac, request.timeout_ms),
            SleepDepth::Idle => Err(WakeError::UnsupportedSleepDepth),
        }
    }

    async fn button_held_for(&mut self, duration_ms: u32) -> bool {
        let started = Instant::now();
        while button_is_pressed() {
            if started.elapsed().as_millis() >= u64::from(duration_ms) {
                return true;
            }
            Timer::after(Duration::from_millis(50)).await;
        }
        false
    }

    async fn delay_ms(&mut self, duration_ms: u32) {
        Timer::after(Duration::from_millis(u64::from(duration_ms))).await;
    }
}

/// PA0 active-high semantic status adapter.
#[derive(Debug, Default, Clone, Copy)]
pub struct Efr32Status;

impl Efr32Status {
    pub const fn new() -> Self {
        Self
    }
}

impl StatusSink for Efr32Status {
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
            led_on();
        } else {
            led_off();
        }
    }
}

/// Cortex-M system-reset supervisor. No watchdog is fitted in this product.
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

#[inline(always)]
pub fn led_on() {
    LED.on();
}

#[inline(always)]
pub fn led_off() {
    LED.off();
}

pub fn halt_with_led() -> ! {
    LED.on();
    loop {
        cortex_m::asm::nop();
    }
}

/// Emergency halt before board resources are fully initialized.
/// Configures PA0 directly via HAL as a last-resort indicator.
pub fn halt_with_led_raw() -> ! {
    use efr32mg1_hal::gpio::{Mode, Pin, Port};
    let pin = Pin::new(Port::A, 0);
    pin.configure(Mode::PushPull, false);
    pin.set_high();
    loop {
        cortex_m::asm::nop();
    }
}
