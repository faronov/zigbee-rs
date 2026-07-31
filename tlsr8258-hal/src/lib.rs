//! Pure-Rust HAL for the Telink TLSR8258.
//!
//! The application owns startup, linker layout, stacks, and interrupt
//! vectors. This crate owns only reusable chip operations and marks routines
//! that must execute from SRAM with the `.ram_code` input section.

#![no_std]
// Most register-driving items are tc32-only but their constants and pure
// helpers remain visible to host tests. They are intentionally unused in a
// normal host library build.
#![cfg_attr(not(target_arch = "tc32"), allow(dead_code))]

pub mod adc;
pub mod aes;
pub mod capture;
pub mod clocks;
#[cfg(target_arch = "tc32")]
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod irq;
pub mod mmio;
pub mod peripherals;
pub mod pm;
pub mod pwm;
pub mod radio;
pub mod reset;
pub mod rng;
pub mod spi;
pub mod timer;
pub mod uart;
pub mod watchdog;
