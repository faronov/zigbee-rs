//! TRÅDFRI board startup, status LED, and PB13 wake handling.
//!
//! Consumes typed [`BoardResources`](efr32mg1_tradfri::resources::BoardResources)
//! tokens passed from the composition root (`main.rs`). PA0 is claimed as a
//! direct GPIO LED (not PWM). Product code separately selects the external
//! flash owner for OTA.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32mg1_tradfri::resources::{ButtonToken, Pa0Output};
use efr32mg1_tradfri::{Button, Led};
use embassy_time::{Duration, Timer};

use crate::{time_driver, vectors};

/// LED handle — a ZST proxy to the PA0 hardware register.
/// Hardware initialization is performed by `Pa0Output::into_led()` during `init()`.
static LED: Led = Led::new();
/// Button handle — a ZST proxy to the PB13 hardware register.
/// Hardware initialization is performed by `ButtonToken::into_button()` during `init()`.
static BUTTON: Button = Button::new();
static BUTTON_EDGE_PENDING: AtomicBool = AtomicBool::new(false);

#[unsafe(no_mangle)]
pub extern "C" fn GPIO_ODD() {
    if BUTTON.take_interrupt() {
        BUTTON_EDGE_PENDING.store(true, Ordering::Release);
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
                size: 64,
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

#[inline(always)]
pub fn led_on() {
    LED.on();
}

#[inline(always)]
pub fn led_off() {
    LED.off();
}

#[inline(always)]
pub fn led_is_on() -> bool {
    LED.is_on()
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
