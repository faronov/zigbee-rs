//! Pure-Rust HAL for PHY6222/PHY6226/PHY6252 SoC.
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
//! # Chip variants
//! - **PHY6222**: BLE 5.4, 512KB flash, 64KB SRAM, QFN32/QFN24
//! - **PHY6226**: Zigbee 3.0 variant (same silicon, different ROM)
//! - **PHY6252**: 256KB flash variant

#![no_std]

pub mod regs;
pub mod gpio;
pub mod i2c;
pub mod adc;
pub mod flash;
