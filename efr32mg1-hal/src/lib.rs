//! Pure-Rust EFR32MG1 peripheral support.

#![no_std]

#[cfg(test)]
#[macro_use]
extern crate std;

pub mod adc;
pub mod clock;
pub mod flash;
pub mod gpio;
pub mod i2c;
pub mod pm;
pub mod pwm;
mod routing;
pub mod spi;
