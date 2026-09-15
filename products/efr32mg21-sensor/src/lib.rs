//! Product policy for the BRD4181A EFR32MG21 environmental sensor.
//!
//! Physical PB0/PD2 wiring stays in `efr32mg21-devkit`; this crate selects
//! Zigbee identity, the typed endpoint profile, lifecycle timing, and the
//! destructive legacy-NV-to-security-journal migration policy.

#![no_std]

mod journal;
pub mod policy;
pub mod profile;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "EFR32MG21-Sensor";
pub const DATE_CODE: &str = "20260402";
pub const SW_BUILD: &str = "0.1.0";
pub const APPLICATION_VERSION: u8 = 1;
pub const ENDPOINT: u8 = 1;
