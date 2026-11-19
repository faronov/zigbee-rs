//! ZRHP network co-processor product for CC2652P7 / CC2652P USB dongles.
//!
//! The NCP runs a zigbee-rs coordinator, Trust Center and Green Power
//! combo and speaks ZRHP v1 to a host
//! (`docs/book/src/advanced/host-protocol.md`) over the dongle's UART.
//! This crate owns what is product-specific:
//!
//! * identity ([`MANUFACTURER`], [`MODEL`], [`CHIP`], [`ROLE_TAG`]);
//! * the NV journal placement ([`layout`], `link/memory-<chip>.x`);
//! * the journals and their opening policy ([`stores`]);
//! * the host transport and its UART policy ([`transport`]);
//! * the CCFG ([`ccfg`]): the serial ROM bootloader stays enabled — it is
//!   the NCP's firmware-update path (no A/B slots, see the platform guide).
//!
//! The coordinator stack, the ZRHP protocol and the NCP dispatch live in
//! the shared crates; the composition root is `examples/cc26x2-ncp`.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "zbdongle-p", feature = "cc2652p7-dongle"))]
compile_error!("cc26x2-ncp-product: enable exactly one board variant");
#[cfg(feature = "zbdongle-p")]
compile_error!(
    "the full-capacity NCP (160 KiB of NV journals) does not fit the CC2652P's \
     352 KiB flash; see docs/book/src/platform-guides/cc26x2.md (ZRHP network co-processor)"
);

pub mod ccfg;
pub mod layout;
pub mod stores;
pub mod transport;

pub const MANUFACTURER: &str = "Zigbee-RS";
#[cfg(feature = "cc2652p7-dongle")]
pub const MODEL: &str = "CC2652P7-NCP";
#[cfg(feature = "zbdongle-p")]
pub const MODEL: &str = "CC2652P-NCP";
/// `SYS_NEGOTIATE.chip` (spec 7.1): 10 = CC2652P, 11 = CC2652P7.
pub const CHIP: u8 = if cfg!(feature = "zbdongle-p") { 10 } else { 11 };
/// `SYS_GET_INFO.board`.
pub const BOARD_NAME: &str = cc26x2_dongle::BOARD_NAME;
/// `SYS_GET_INFO.product`.
pub const PRODUCT_NAME: &str = "zigbee-rs-ncp";
/// Default `phyTransmitPower` (+5 dBm, standard PA; see the router
/// product for the ETSI rationale).
pub const TX_POWER_DBM: i8 = 5;
/// Security-journal role tag of the coordinator NCP (same as the ESP NCP;
/// the router image uses 0x5201).
pub const ROLE_TAG: u16 = 0x5203;

/// Coordinator lifecycle policy: 2 ms receive slices so the host link is
/// serviced at least every ~2 ms (ZRHP `ack_delay` 1 ms, spec 5.8).
pub static POLICY: router_app::RouterPolicy = router_app::RouterPolicy {
    max_receive_slice_us: 2_000,
    join_retry_initial_ms: 5_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_strings_fit_zrhp_fields() {
        assert!(MODEL.len() <= 32 && BOARD_NAME.len() <= 32 && PRODUCT_NAME.len() <= 32);
        assert_ne!(ROLE_TAG, 0x5201);
    }
}
