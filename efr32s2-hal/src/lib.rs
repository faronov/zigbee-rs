//! Pure-Rust peripheral support for EFR32 Series 2 (EFR32MG21, EFR32MG22,
//! EFR32MG24).
//!
//! Exactly one chip feature must be enabled: `efr32mg21`, `efr32mg22` or
//! `efr32mg24` (the latter through its PA variant, `efr32mg24-pa10` or
//! `efr32mg24-pa20`).
//! Register addresses and field layouts are checked against the GSDK 4.5.0
//! device headers; see `docs/book/src/platform-guides/efr32.md`.

#![no_std]
// MMIO helpers and register constants are only reachable on the target;
// host builds compile them for the unit tests.
#![cfg_attr(not(all(target_arch = "arm", target_os = "none")), allow(dead_code))]

#[cfg(any(
    all(feature = "efr32mg21", feature = "efr32mg22"),
    all(feature = "efr32mg21", feature = "efr32mg24"),
    all(feature = "efr32mg22", feature = "efr32mg24"),
))]
compile_error!("efr32s2-hal: enable only one of `efr32mg21` / `efr32mg22` / `efr32mg24`");
#[cfg(not(any(feature = "efr32mg21", feature = "efr32mg22", feature = "efr32mg24")))]
compile_error!("efr32s2-hal: enable one chip feature (`efr32mg21`, `efr32mg22` or `efr32mg24`)");
#[cfg(all(
    feature = "efr32mg24",
    not(any(feature = "efr32mg24-pa10", feature = "efr32mg24-pa20"))
))]
compile_error!(
    "efr32s2-hal: EFR32MG24 needs its PA variant (`efr32mg24-pa10` or `efr32mg24-pa20`)"
);
#[cfg(all(feature = "efr32mg24-pa10", feature = "efr32mg24-pa20"))]
compile_error!("efr32s2-hal: enable only one of `efr32mg24-pa10` / `efr32mg24-pa20`");
#[cfg(all(feature = "hfxo-39mhz", not(feature = "efr32mg24")))]
compile_error!("efr32s2-hal: `hfxo-39mhz` is only supported on EFR32MG24");
#[cfg(all(feature = "usart0-host", not(feature = "rt"), target_os = "none"))]
compile_error!("efr32s2-hal: `usart0-host` firmware needs the `rt` feature");

pub mod chip;
pub mod clock;
pub mod crypto;
pub mod delay;
pub mod devinfo;
pub mod flash;
pub mod gecko_bootloader;
pub mod gpio;
pub mod i2c;
#[cfg(feature = "efr32mg22")]
pub mod iadc;
pub mod peripherals;
pub mod power;
pub mod radio;
pub mod reset;
pub mod rtcc;
#[cfg(feature = "efr32mg24")]
pub mod se;
#[cfg(feature = "time-driver")]
pub mod time_driver;
#[cfg(feature = "efr32mg22")]
pub mod trng;
pub mod usart;
pub mod vectors;
pub mod watchdog;
