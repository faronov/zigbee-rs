//! Product configuration for the nRF52833-DK Zigbee sensor firmware.
//!
//! Owns manufacturer/model identity, the flash memory layout and the
//! crash-safe security persistence partition (`storage`), the battery
//! chemistry policy (`battery`), and the concrete Zigbee profile
//! (`profile`) built from shared `zigbee-runtime` archetypes. See
//! `boards/nrf52833-dk` for the physical wiring this product selects and
//! `apps/nrf-sensor` for the shared application lifecycle it runs.
//!
//! The only intentional differences from `products/nrf52840-sensor` are the
//! ones the silicon forces:
//!
//! | | nRF52840 product | nRF52833 product |
//! |-|------------------|------------------|
//! | Flash | 1 MiB | 512 KiB |
//! | RAM | 256 KiB | 128 KiB |
//! | Application flash | 1016 KiB | 504 KiB |
//! | Security journal | `0x000F_E000` | `0x0007_E000` |
//! | Model string | `nRF52840-Sensor` | `nRF52833-Sensor` |
//!
//! Endpoint, clusters, reporting defaults, battery curve, and the whole
//! commissioning lifecycle are identical.

#![no_std]

pub mod battery;
pub mod policy;
pub mod profile;
#[cfg(target_os = "none")]
pub mod storage;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "nRF52833-Sensor";
pub const DATE_CODE: &str = "20260401";
pub const SW_BUILD: &str = "0.1.0";

pub const ENDPOINT: u8 = 1;
