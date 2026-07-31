//! Board support for the TLSR8258 TB-04 module.

#![no_std]
#![cfg_attr(not(target_arch = "tc32"), allow(dead_code))]

pub mod leds;
pub mod resources;

pub const ONBOARD_FLASH_CAPACITY: usize = 512 * 1024;
