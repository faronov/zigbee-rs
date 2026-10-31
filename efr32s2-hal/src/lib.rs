//! Pure-Rust peripheral support for EFR32 Series 2 (EFR32MG21, EFR32MG22).
//!
//! Exactly one chip feature must be enabled: `efr32mg21` or `efr32mg22`.
//! Register addresses and field layouts are checked against the GSDK 4.5.0
//! device headers; see `docs/book/src/platform-guides/efr32.md`.

#![no_std]
// MMIO helpers and register constants are only reachable on the target;
// host builds compile them for the unit tests.
#![cfg_attr(not(all(target_arch = "arm", target_os = "none")), allow(dead_code))]

#[cfg(all(feature = "efr32mg21", feature = "efr32mg22"))]
compile_error!("efr32s2-hal: enable only one of `efr32mg21` / `efr32mg22`");
#[cfg(not(any(feature = "efr32mg21", feature = "efr32mg22")))]
compile_error!("efr32s2-hal: enable one chip feature (`efr32mg21` or `efr32mg22`)");

pub mod chip;
pub mod clock;
pub mod delay;
pub mod devinfo;
pub mod flash;
pub mod gecko_bootloader;
pub mod gpio;
pub mod i2c;
pub mod peripherals;
pub mod power;
pub mod radio;
pub mod rtcc;
#[cfg(feature = "time-driver")]
pub mod time_driver;
pub mod usart;
pub mod vectors;
