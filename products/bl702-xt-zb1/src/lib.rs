//! Product policy for the BL702 XT-ZB1 Zigbee sensor.

#![no_std]

pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "XT-ZB1 Sensor";
pub const DATE_CODE: &str = "20260402";
pub const SW_BUILD: &str = "0.1.0";
pub const ENDPOINT: u8 = 1;
