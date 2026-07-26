//! Product configuration for the EFR32MG1P TRADFRI sensor firmware.

#![no_std]

pub mod battery;
pub mod ota;
pub mod profile;
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "EFR32MG1-Sensor";
pub const DATE_CODE: &str = "20260402";

pub const ENDPOINT: u8 = 1;
pub const OTA_MANUFACTURER_CODE: u16 = 0x1049;
pub const OTA_IMAGE_TYPE: u16 = 0x0002;
