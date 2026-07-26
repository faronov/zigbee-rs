//! Product configuration for the ESP32-C6/H2 Zigbee sensor firmware.
//!
//! Builds on the physical chip flash exposed by `esp32-zigbee-devkit` (the
//! board crate) and owns everything that crate deliberately does not:
//!
//! * [`storage`] — the Zigbee security-state journal, at the last 8 KiB of
//!   the physical flash chip (unchanged from before any partition table
//!   existed).
//! * [`migration`] — a one-time, crash-safe migration of that window from the
//!   legacy `LogStructuredNv` format to the security-state journal, so
//!   already-joined devices keep their network and never reuse a frame counter
//!   across the switch.
//! * [`profile`] — the endpoint/cluster profile this product selects, with
//!   OTA composed in only where a backend for it exists.
//!
//! ESP32-C6 additionally owns a two-slot OTA partition table and firmware
//! writer, gated behind the `esp32c6` feature:
//!
//! * [`layout`] — the 4 MiB partition table these boards are flashed with.
//! * [`otadata`] — ESP-IDF boot-slot selection records.
//! * [`esp_image`] — ESP application image header/chip-ID validation.
//! * [`sha256`] — SHA-256, used to verify a staged image before activation.
//! * [`ota`] — [`zigbee_runtime::firmware_writer::FirmwareWriter`] that stages
//!   an OTA payload into the inactive application slot.
//! * [`firmware`] — firmware version helpers shared by the OTA and Basic
//!   clusters.
//!
//! ESP32-H2 has no OTA backend: it keeps the default single-app partition
//! table and [`profile::SensorProfile`] never advertises the OTA Upgrade
//! client cluster on that build.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "esp32c6", feature = "esp32h2"))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");

pub mod migration;
pub mod profile;
#[cfg(target_os = "none")]
pub mod storage;

#[cfg(feature = "esp32c6")]
pub mod esp_image;
#[cfg(feature = "esp32c6")]
pub mod firmware;
#[cfg(feature = "esp32c6")]
pub mod layout;
#[cfg(feature = "esp32c6")]
pub mod ota;
#[cfg(feature = "esp32c6")]
pub mod otadata;
#[cfg(feature = "esp32c6")]
pub mod sha256;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const DATE_CODE: &str = "20260403";
/// Endpoint hosting every application and OTA cluster.
pub const ENDPOINT: u8 = 1;

#[cfg(feature = "esp32c6")]
pub const MODEL: &str = "ESP32-C6-Sensor";
#[cfg(feature = "esp32h2")]
pub const MODEL: &str = "ESP32-H2-Sensor";

/// Manufacturer code advertised in `QueryNextImageRequest` and stamped into
/// the OTA container by `tools/create-ota.py`. ZHA matches images on this
/// pair, so the two must stay in sync.
#[cfg(feature = "esp32c6")]
pub const OTA_MANUFACTURER_CODE: u16 = 0x1234;
/// Image type: 0x0001 = ESP32-C6 sensor.
#[cfg(feature = "esp32c6")]
pub const OTA_IMAGE_TYPE: u16 = 0x0001;
/// Hardware version reported to the OTA server.
#[cfg(feature = "esp32c6")]
pub const OTA_HARDWARE_VERSION: u16 = 1;
