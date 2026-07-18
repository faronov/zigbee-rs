//! Pure-Rust HAL for the PHY6222 family.
//!
//! No vendor SDK, no binary blobs — all hardware access through direct
//! register writes derived from the open-source PHY6222 SDK.
//!
//! # Peripherals
//! - [`gpio`] — GPIO output/input, IOMUX pin mux, AON pull-up/down
//! - [`i2c`] — DesignWare I2C master, 100kHz/400kHz, polling mode
//! - [`adc`] — ADC single-shot battery voltage measurement
//! - [`flash`] — SPIF flash controller: read (XIP), write, sector erase
//!
//! Boot layout, flash size, and ROM services vary by chip. Applications must
//! select and validate a concrete device layout rather than assuming PHY6222
//! and PHY6252 images are interchangeable.

#![no_std]

pub mod adc;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod regs;
pub mod sleep;
