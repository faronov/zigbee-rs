//! Shared board support for ESP32-C6 and ESP32-H2 Zigbee development boards.
//!
//! This crate exposes only physical chip resources: the on-die 4 MiB NOR
//! flash ([`flash`]) accessed through `esp_storage::FlashStorage`. It has no
//! dependency on `zigbee-runtime` and owns no partition layout, NV placement,
//! OTA policy, or firmware identity — those are product concerns. See
//! `products/esp32-zigbee-devkit` for the product crate that builds on this
//! board and selects them.
//!
//! The only per-chip difference at this layer is which `esp-storage` chip
//! feature is enabled, selected by the `esp32c6`/`esp32h2` features.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "esp32c6", feature = "esp32h2"))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");

#[cfg(target_os = "none")]
pub mod flash;
