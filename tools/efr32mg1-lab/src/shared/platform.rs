#![allow(dead_code, unused_imports, unused_macros)]

use efr32mg1_tradfri::Led;

static LED: Led = Led::new();

macro_rules! init_small {
    ($profile:expr) => {{
        let channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 256,
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
        $crate::platform::finish_init($profile);
    }};
}

macro_rules! init_large {
    ($profile:expr) => {{
        let channels = rtt_target::rtt_init! {
            up: {
                0: {
                    size: 4096,
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
        $crate::platform::finish_init($profile);
    }};
}

pub(crate) use init_large;
pub(crate) use init_small;

pub(crate) fn finish_init(profile: &str) {
    let _ = &crate::vectors::__INTERRUPTS;
    LED.init();
    LED.off();
    match efr32mg1_tradfri::init_clocks() {
        Ok(()) => rtt_target::rprintln!(
            "[EFR32][{}] CLOCK_READY hclk={} ctune={}",
            profile,
            efr32mg1_tradfri::HCLK_HZ,
            efr32mg1_tradfri::HFXO_CTUNE
        ),
        Err(error) => {
            rtt_target::rprintln!("[EFR32][{}] CLOCK_FATAL {:?}", profile, error);
            halt()
        }
    }
}

pub fn led_on() {
    LED.on();
}

pub fn led_off() {
    LED.off();
}

pub fn halt() -> ! {
    loop {
        cortex_m::asm::nop();
    }
}
