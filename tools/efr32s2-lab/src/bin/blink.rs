//! Gate 1: HFXO + TIMER0 timing + LED0 + VCOM log.
//!
//! Expected VCOM output: banner, `HFXO ok, HCLK 38400000 Hz`, the factory
//! EUI-64, then one `tick n` line per second while LED0 (PB1) toggles.

#![no_std]
#![no_main]

#[path = "../shared/mod.rs"]
mod shared;

use efr32s2_hal::delay;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut lab = shared::init("blink");
    let mut n = 0u32;
    loop {
        let t0 = delay::cycles();
        shared::delay_ms(1000);
        let dt = delay::cycles().wrapping_sub(t0);
        lab.led.toggle();
        log!("tick {} ({} us by TIMER0)", n, delay::cycles_to_us(dt));
        n += 1;
    }
}
