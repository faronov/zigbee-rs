//! Platform layer: boot vectors/startup, raw MMIO, clocks, Timer0, GPIO
//! LEDs, and linker-symbol-derived layout checks.
//!
//! Everything that only makes sense on real TLSR8258 silicon (assembly
//! vectors, volatile MMIO, linker-symbol externs) is gated behind
//! `#[cfg(target_arch = "tc32")]`, which is true for the `tc32-unknown-none-elf`
//! target and false for ordinary host targets used by `cargo test`. This
//! keeps `cargo test` working with the plain host toolchain, without any
//! custom test runner.

pub mod gpio;
#[cfg(target_arch = "tc32")]
pub mod linker;
#[cfg(target_arch = "tc32")]
pub mod vectors;

#[cfg(target_arch = "tc32")]
pub use tlsr8258_hal::flash;
pub use tlsr8258_hal::{clocks, mmio, timer};
