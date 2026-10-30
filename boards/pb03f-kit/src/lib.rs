//! Physical wiring of the Ai-Thinker PB-03F-Kit.
//!
//! PB-03F module: PHY6252 (QFN32), 512 KiB SPI flash (JEDEC 0x1340C4,
//! measured over SWD), 64 KiB SRAM. Kit wiring:
//!
//! | Function        | Pin | HAL GPIO index |
//! |-----------------|-----|----------------|
//! | RGB LED red     | P7  | 4              |
//! | RGB LED green   | P11 | 7              |
//! | RGB LED blue    | P18 | 12             |
//! | Cool-white LED  | P34 | 22             |
//! | Warm-white LED  | P0  | 0              |
//! | User button     | P15 | 9              |
//! | Supply sense (ADC, driven high) | P14 | 8 |
//! | UART0 TX (CH340)| P9  | 5              |
//! | UART0 RX (CH340)| P10 | 6              |
//! | SWDIO / SWCLK   | P2 / P3 | 2 / 3 (left on SWD; never muxed here) |
//!
//! P14 has no kit function (LEDs, button, UART, SWD); keep its header pin
//! unconnected. The board drives it high only while the ADC samples it, so
//! the reading is the module supply (THB2 `ADC_PIN_USE_OUT` method). It idles
//! as an input with pull-down.
//!
//! No sensor or I2C bus is fitted. Products select persistence, identity and
//! boot layout; this crate only owns the fitted hardware.

#![no_std]

#[cfg(feature = "log")]
pub mod console_log;
mod flash;
mod pins;
mod resources;

pub use flash::{INTERNAL_FLASH_CAPACITY, InternalFlash};
pub use phy6222_hal::adc::AdcError;
pub use phy6222_hal::{time, vectors};
pub use resources::{
    BoardError, Console, Led, Leds, Resources, SupplyMonitor, SupplyMonitorToken, UserButton,
};
