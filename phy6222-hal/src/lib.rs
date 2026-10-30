//! Pure-Rust HAL for the PHY6222 family.
//!
//! No vendor SDK, no binary blobs — all hardware access through direct
//! register writes transcribed from the PHYPLUS PHY62xx SDK sources
//! (pvvx/phy62x2-SDK, pvvx/THB2).
//!
//! # Peripherals
//! - [`gpio`] — GPIO output/input, IOMUX pin mux, AON pull-up/down
//! - [`i2c`] — DesignWare I2C master, 100kHz/400kHz, polling mode
//! - [`adc`] — ADC single-shot battery voltage measurement
//! - [`flash`] — SPIF flash controller: read (XIP), write, sector erase
//! - [`clock`] — HCLK source query and DLL 48 MHz switch
//! - [`uart`] — UART0 blocking transmitter for diagnostics
//! - `time` (feature `time-driver`) — SysTick Embassy time driver
//! - [`Peripherals`] — exclusive ownership tokens for stateful peripherals
//!
//! Boot layout, flash size, and ROM services vary by chip. Applications must
//! select and validate a concrete device layout rather than assuming PHY6222
//! and PHY6252 images are interchangeable.

#![no_std]

pub mod adc;
pub mod clock;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod peripherals;
pub mod regs;
pub mod sleep;
#[cfg(feature = "time-driver")]
pub mod time;
pub mod uart;
pub mod vectors;

pub use peripherals::Peripherals;
