//! Board support for the TLSR8258 TB-04 module.

#![no_std]

pub mod leds;
pub mod resources;

pub const ONBOARD_FLASH_CAPACITY: usize = 512 * 1024;
