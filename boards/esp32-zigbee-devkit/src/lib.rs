//! Shared board support for ESP32-C6 and ESP32-H2 Zigbee development boards.

#![no_std]

#[cfg(all(feature = "esp32c6", feature = "esp32h2"))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");
#[cfg(not(any(feature = "esp32c6", feature = "esp32h2")))]
compile_error!("select exactly one of the esp32c6 or esp32h2 features");

pub mod storage;
