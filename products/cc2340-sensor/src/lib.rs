//! Product policy for the LP-EM-CC2340R5 Zigbee sensor.
//!
//! The board crate owns physical wiring. This crate owns Zigbee identity,
//! the typed endpoint profile, sleepy-sensor lifecycle policy, deliberately
//! synthetic bring-up measurements, and the protected flash partition used
//! for durable security state.

#![no_std]

pub mod battery;
pub mod environment;
pub mod policy;
pub mod profile;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "CC2340-Sensor";
pub const DATE_CODE: &str = "20260402";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;
