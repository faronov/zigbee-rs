//! Pure-Rust HAL for the Telink TLSR8258.
//!
//! The application owns startup, linker layout, stacks, and interrupt
//! vectors. This crate owns only reusable chip operations and marks routines
//! that must execute from SRAM with the `.ram_code` input section.

#![no_std]

pub mod clocks;
#[cfg(target_arch = "tc32")]
pub mod flash;
pub mod mmio;
pub mod radio;
pub mod timer;
