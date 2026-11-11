//! Shared board support for ESP32-C6 and ESP32-H2 Zigbee development boards.
//!
//! This crate exposes only physical chip resources: the on-die 4 MiB NOR
//! flash ([`flash`]) accessed through `esp_storage::FlashStorage`, and the
//! WS2812 RGB LED fitted on GPIO8 of the ESP32-C6-DevKitC-1,
//! ESP32-H2-DevKitM-1 and Waveshare ESP32-H2-Zero ([`status_led`]; the RMT
//! driver and its `zigbee-ws2812` encoder behind feature `status-led`, so
//! images without it link no RMT code). It has no
//! dependency on `zigbee-runtime` and owns no partition layout, NV placement,
//! OTA policy, or firmware identity — those are product concerns. See
//! `products/esp32-zigbee-devkit` for the product crate that builds on this
//! board and selects them.
//!
//! The only per-chip difference at this layer is which `esp-storage` (and
//! `esp-hal`) chip feature is enabled, selected by the `esp32c6`/`esp32h2`
//! features.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "esp32c6", feature = "esp32h2"))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(all(feature = "waveshare-h2-zero", not(feature = "esp32h2")))]
compile_error!("the Waveshare ESP32-H2-Zero carries an ESP32-H2: enable esp32h2");

#[cfg(target_os = "none")]
pub mod flash;
pub mod status_led;

/// The physical boards this crate describes. They share the chip
/// resources above and differ only in fitted parts (today: the byte order
/// of the status LED, `status_led::color_order`). [`BOARD`] is the one
/// selected by the features.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Board {
    /// Espressif ESP32-C6-DevKitC-1.
    EspressifC6DevKitC1,
    /// Espressif ESP32-H2-DevKitM-1.
    EspressifH2DevKitM1,
    /// Waveshare ESP32-H2-Zero.
    WaveshareH2Zero,
}

/// The board this build is for: the Espressif devkit of the selected chip,
/// or the Waveshare ESP32-H2-Zero with feature `waveshare-h2-zero`.
pub const BOARD: Board = if cfg!(feature = "waveshare-h2-zero") {
    Board::WaveshareH2Zero
} else if cfg!(feature = "esp32c6") {
    Board::EspressifC6DevKitC1
} else {
    Board::EspressifH2DevKitM1
};
