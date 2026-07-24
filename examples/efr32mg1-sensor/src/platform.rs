//! TRÅDFRI board startup, status LED, and PB13 wake handling.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32mg1_tradfri::{Button, Led};
use embassy_time::{Duration, Timer};

use crate::{time_driver, vectors};

static LED: Led = Led::new();
static BUTTON: Button = Button::new();
static BUTTON_EDGE_PENDING: AtomicBool = AtomicBool::new(false);

#[unsafe(no_mangle)]
pub extern "C" fn GPIO_ODD() {
    if BUTTON.take_interrupt() {
        BUTTON_EDGE_PENDING.store(true, Ordering::Release);
    }
}

pub fn init() {
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

    LED.init();
    LED.off();
    BUTTON.init();
    cortex_m::peripheral::NVIC::unpend(vectors::Interrupt::GpioOdd);
    unsafe { cortex_m::peripheral::NVIC::unmask(vectors::Interrupt::GpioOdd) };

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
    loop {
        LED.on();
        cortex_m::asm::nop();
    }
}
