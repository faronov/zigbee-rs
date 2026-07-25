//! Shared board support for ESP32-C6 and ESP32-H2 Zigbee development boards.
//!
//! * [`layout`] — the 4 MiB partition table these boards are flashed with.
//! * [`storage`] — log-structured Zigbee NV in the last 8 KiB of `zbnv`.
//! * [`otadata`] — ESP-IDF boot-slot selection records.
//! * [`ota`] — [`zigbee_runtime::firmware_writer::FirmwareWriter`] that stages
//!   an OTA payload into the inactive application slot.
//!
//! Everything except the flash back end is chip independent, so both examples
//! share it; the only per-chip value is the expected image chip ID, selected by
//! the `esp32c6`/`esp32h2` features.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "esp32c6", feature = "esp32h2"))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");

pub mod esp_image;
pub mod firmware;
pub mod layout;
pub mod ota;
pub mod otadata;
pub mod sha256;
#[cfg(target_os = "none")]
pub mod storage;
