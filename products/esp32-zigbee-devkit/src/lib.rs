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
//! Both supported chips own a two-slot OTA partition table and firmware
//! writer:
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
//! ESP32-C6 and ESP32-H2 use distinct OTA image types so a coordinator cannot
//! offer an application image for the wrong chip.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "esp32c6", feature = "esp32h2"))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");

pub mod migration;
pub mod profile;
#[cfg(target_os = "none")]
pub mod storage;

#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod esp_image;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod firmware;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod layout;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod ota;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod ota_transport;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod otadata;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
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
pub const OTA_MANUFACTURER_CODE: u16 = 0x1234;
/// OTA image type assigned to the selected chip.
#[cfg(feature = "esp32c6")]
pub const OTA_IMAGE_TYPE: u16 = 0x0001;
/// Image type: 0x0002 = ESP32-H2 sensor.
#[cfg(feature = "esp32h2")]
pub const OTA_IMAGE_TYPE: u16 = 0x0002;
/// Hardware version reported to the OTA server.
pub const OTA_HARDWARE_VERSION: u16 = 1;
