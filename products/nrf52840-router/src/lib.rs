//! Product configuration for the nRF52840-DK always-on Zigbee End Device.
//!
//! The product owns its Zigbee identity, Range Extender profile, bounded
//! always-on scheduling policy, semantic LED1 mapping, and the crash-safe
//! security journal at the top of flash. Physical DK wiring remains in
//! `boards/nrf52840-dk`; commissioning and runtime lifecycle remain in
//! `apps/router`.

#![no_std]

pub mod policy;
pub mod profile;
pub mod status;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "nRF52840-AlwaysOn-ED";
pub const DATE_CODE: &str = "20260405";
pub const SW_BUILD: &str = "0.1.0";

pub const ENDPOINT: u8 = 1;
