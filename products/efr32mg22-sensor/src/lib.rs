//! Product policy for the BRD4183A EFR32MG22 environmental sensor.
//!
//! Physical PB1/PB0/VCOM wiring stays in `efr32mg22-devkit`; this crate
//! selects Zigbee identity, the typed endpoint profile, lifecycle timing,
//! the flash layout and the security-journal opening policy.

#![no_std]

mod journal;
pub mod policy;
pub mod profile;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "EFR32MG22-Sensor";
pub const DATE_CODE: &str = "20261031";
pub const SW_BUILD: &str = "0.1.0";
pub const APPLICATION_VERSION: u8 = 1;
pub const ENDPOINT: u8 = 1;
