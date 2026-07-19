//! Board support for the TLSR8258 TB-04 module.

#![no_std]

pub mod leds;
#[cfg(target_arch = "tc32")]
pub mod storage;
