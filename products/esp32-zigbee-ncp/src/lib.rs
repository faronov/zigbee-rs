//! ESP32-H2 ZRHP network co-processor product.
//!
//! The NCP runs a zigbee-rs coordinator, Trust Center and Green Power combo
//! and speaks ZRHP v1 to a host
//! (`docs/book/src/advanced/host-protocol.md`). This crate owns what is
//! product-specific:
//!
//! * identity ([`MANUFACTURER`], [`MODEL`], [`CHIP`], the security-journal
//!   [`ROLE_TAG`]);
//! * both partition tables and the `zbnv` journal placement ([`layout`]);
//! * the fitted-flash check ([`flash_size`]) and the flash windows
//!   (`flash`, target only);
//! * the host transport and its UART policy ([`transport`]);
//! * the NV journals bound to their windows (`stores`, target only).
//!
//! The coordinator stack, the ZRHP protocol and the NCP command dispatch
//! live in the shared crates; the composition root is
//! `examples/esp32h2-ncp`.

#![cfg_attr(not(test), no_std)]

#[cfg(all(target_os = "none", not(feature = "esp32h2")))]
compile_error!("the ZRHP NCP product supports the ESP32-H2 only (enable esp32h2)");
#[cfg(all(feature = "thread-br-h2", feature = "waveshare-h2-zero"))]
compile_error!("select one board");

#[cfg(target_os = "none")]
pub mod flash;
pub mod flash_size;
pub mod layout;
#[cfg(target_os = "none")]
pub mod stores;
pub mod transport;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "ESP32-H2-NCP";
/// `SYS_NEGOTIATE.chip`: 1 = ESP32-H2.
pub const CHIP: u8 = 1;
/// `SYS_GET_INFO.board`.
pub const BOARD_NAME: &str = if cfg!(feature = "thread-br-h2") {
    "esp-thread-br-zigbee-gw"
} else if cfg!(feature = "waveshare-h2-zero") {
    "waveshare-esp32-h2-zero"
} else {
    "esp32-h2-devkitm-1"
};
/// `SYS_GET_INFO.product`.
pub const PRODUCT_NAME: &str = "zigbee-rs-ncp";

/// Security-journal role tag of the coordinator NCP. The devkit images use
/// 0x5201 (router) and 0x5202 (end device); the NCP journal lives in its own
/// `zbnv`, and the tag additionally stops any other role from resuming it.
pub const ROLE_TAG: u16 = 0x5203;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_strings_fit_zrhp_fields() {
        // ZRHP strings are bounded to 32 bytes (spec 3.3).
        assert!(MODEL.len() <= 32 && BOARD_NAME.len() <= 32 && PRODUCT_NAME.len() <= 32);
        assert_ne!(ROLE_TAG, 0x5201);
        assert_ne!(ROLE_TAG, 0x5202);
    }
}
