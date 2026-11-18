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
//! * the NV journals bound to their windows (`stores`, target only);
//! * PTA coexistence on the Thread BR board (`pta`, feature `pta`).
//!
//! The coordinator stack, the ZRHP protocol and the NCP command dispatch
//! live in the shared crates; the composition root is
//! `examples/esp32h2-ncp`.

#![cfg_attr(not(test), no_std)]

#[cfg(all(target_os = "none", not(any(feature = "esp32h2", feature = "esp32c6"))))]
compile_error!("select the chip: esp32h2 or esp32c6");
#[cfg(all(feature = "esp32h2", feature = "esp32c6"))]
compile_error!("select exactly one chip: esp32h2 or esp32c6");
#[cfg(all(
    feature = "esp32c6",
    any(feature = "thread-br-h2", feature = "waveshare-h2-zero")
))]
compile_error!("the selected board carries an ESP32-H2");
#[cfg(all(feature = "thread-br-h2", feature = "waveshare-h2-zero"))]
compile_error!("select one board");

#[cfg(target_os = "none")]
pub mod flash;
pub mod flash_size;
#[cfg(target_os = "none")]
pub mod host;
pub mod layout;
#[cfg(feature = "pta")]
pub mod pta;
#[cfg(target_os = "none")]
pub mod stores;
pub mod transport;

pub const MANUFACTURER: &str = "Zigbee-RS";
#[cfg(not(feature = "esp32c6"))]
pub const MODEL: &str = "ESP32-H2-NCP";
#[cfg(feature = "esp32c6")]
pub const MODEL: &str = "ESP32-C6-NCP";
/// `SYS_NEGOTIATE.chip`: 1 = ESP32-H2, 2 = ESP32-C6.
pub const CHIP: u8 = if cfg!(feature = "esp32c6") { 2 } else { 1 };
/// `SYS_GET_INFO.board`.
pub const BOARD_NAME: &str = if cfg!(feature = "thread-br-h2") {
    "esp-thread-br-zigbee-gw"
} else if cfg!(feature = "waveshare-h2-zero") {
    "waveshare-esp32-h2-zero"
} else if cfg!(feature = "esp32c6") {
    "esp32-c6-devkit"
} else {
    "esp32-h2-devkitm-1"
};
/// `SYS_GET_INFO.product`.
pub const PRODUCT_NAME: &str = "zigbee-rs-ncp";

/// Coordinator lifecycle policy: 2 ms receive slices so the host link is
/// serviced at least every ~2 ms (ZRHP `ack_delay` 1 ms, spec 5.8).
#[cfg(target_os = "none")]
pub static POLICY: router_app::RouterPolicy = router_app::RouterPolicy {
    max_receive_slice_us: 2_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

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
