//! Product configuration for the nRF52840-DK Zigbee sensor firmware.
//!
//! Owns manufacturer/model identity, the flash memory layout and the
//! crash-safe security persistence partition (`storage`), the battery
//! chemistry policy (`battery`), and the concrete Zigbee profile
//! (`profile`) built from shared `zigbee-runtime` archetypes. See
//! `boards/nrf52840-dk` for the physical wiring this product selects.

#![no_std]

pub mod battery;
pub mod profile;
#[cfg(target_os = "none")]
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "nRF52840-Sensor";
pub const DATE_CODE: &str = "20260401";
pub const SW_BUILD: &str = "0.1.0";

pub const ENDPOINT: u8 = 1;
