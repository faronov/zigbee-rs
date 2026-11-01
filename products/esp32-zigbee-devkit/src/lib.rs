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
//! * [`chip_temperature`] — ESP-IDF-equivalent TSENS range selection and
//!   per-chip eFuse calibration for the on-die temperature source.
//!
//! Both supported chips own a two-slot OTA partition table and firmware
//! writer:
//!
//! * [`layout`] — the 4 MiB partition table these boards are flashed with.
//! * [`otadata`] — ESP-IDF boot-slot selection records.
//! * [`esp_image`] — ESP application image header/chip-ID validation.
//! * [`sha256`] — SHA-256, used to verify a staged image before activation.
//! * [`ota`] — [`zigbee_runtime::firmware_writer::FirmwareWriter`] that stages
//!   an OTA payload into the inactive application slot, plus the
//!   pending-verification boot check, confirmation and rollback.
//! * [`ota_boot`] — when a freshly installed image counts as proven on the
//!   network, and the deadline after which it is rolled back.
//! * [`firmware`] — firmware version helpers shared by the OTA and Basic
//!   clusters.
//!
//! ESP32-C6 and ESP32-H2 use distinct OTA image types so a coordinator cannot
//! offer an application image for the wrong chip.
//!
//! Besides the sleepy sensor, the product owns two mains roles that share the
//! same partition table and security journal (separated by [`role`] tags):
//!
//! * [`end_device`] — always-on (RX-on-when-idle) End Device policy/profile.
//! * [`router`] — parent Router policy/profile plus the child-table and APS
//!   binding/group journals in the first 16 KiB of `zbnv`.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "esp32c6", feature = "esp32h2"))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");

pub mod chip_temperature;
#[cfg(feature = "roles")]
pub mod end_device;
#[cfg(any(target_os = "none", test))]
pub mod executor;
pub mod migration;
#[cfg(feature = "sed")]
pub mod policy;
pub mod profile;
#[cfg(feature = "roles")]
pub mod router;
pub mod sleep;
#[cfg(target_os = "none")]
pub mod storage;
#[cfg(any(target_os = "none", test))]
pub mod time_driver;
#[cfg(all(feature = "sed", any(target_os = "none", test)))]
pub mod wake;

#[cfg(test)]
#[path = "../../../vendor/esp-radio/src/ieee802154/suspend.rs"]
mod radio_suspend_tests;

#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod esp_image;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod firmware;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod layout;
#[cfg(any(feature = "esp32c6", feature = "esp32h2"))]
pub mod ota;
#[cfg(all(feature = "sed", any(feature = "esp32c6", feature = "esp32h2")))]
pub mod ota_boot;
#[cfg(all(feature = "sed", any(feature = "esp32c6", feature = "esp32h2")))]
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

/// Model identifiers of the mains role images.
#[cfg(feature = "esp32c6")]
pub const ROUTER_MODEL: &str = "ESP32-C6-Router";
#[cfg(feature = "esp32c6")]
pub const END_DEVICE_MODEL: &str = "ESP32-C6-ED";
#[cfg(feature = "esp32h2")]
pub const ROUTER_MODEL: &str = "ESP32-H2-Router";
#[cfg(feature = "esp32h2")]
pub const END_DEVICE_MODEL: &str = "ESP32-H2-ED";

/// Security-journal role tags (`SecurityStateJournal::select_role_tag`).
///
/// All role images share one security journal so outgoing frame counters
/// stay monotonic across a reflash; the tag stops one role from resuming
/// another role's network state. The original sensor image predates the
/// tags and writes none; leave the network before returning to it from a
/// tagged role.
pub mod role {
    pub const ROUTER: u16 = 0x5201;
    pub const END_DEVICE: u16 = 0x5202;
}

/// Mains roles re-arm commissioning on their own this long after a Leave
/// (the BOOT button re-arms it immediately).
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;

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
