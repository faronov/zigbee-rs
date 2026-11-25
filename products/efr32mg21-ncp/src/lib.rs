//! zigbee-rs ZRHP network co-processor product for 64 KiB EFR32MG21
//! modules: the Tuya ZS3L (EFR32MG21A020F768IM32) on the host UART of an
//! ESP32-S3 panel running zigbee-rs-hub (`examples/efr32mg21-ncp`).
//!
//! The coordinator, Trust Center and Green Power combo of
//! `apps/ncp-coordinator` at the **tiny** capacity tier
//! (`ncp-coordinator-app/tiny`, `zigbee-runtime/ncp-capacity-tiny`: 48
//! Trust Center devices, 16 children, a 4-frame link window), see
//! `docs/book/src/advanced/tiny-ncp.md`.
//!
//! This crate owns what is product-specific:
//!
//! * identity ([`MANUFACTURER`], [`MODEL`], [`CHIP`], [`ROLE_TAG`]);
//! * the memory map ([`layout`], `link/memory-768k.x` / `memory-512k.x`):
//!   the resident **Gecko bootloader is kept** in `0x0..0x4000` and is the
//!   NCP's update path (GBL over XMODEM at 115 200, see [`gecko`]); the NV
//!   journals sit at the top of flash, outside every GBL;
//! * the journals and their opening policy ([`stores`]);
//! * the host transport and its UART policy ([`transport`]);
//! * the Gecko `ApplicationProperties` record ([`gecko`]);
//! * the fatal-fault policy ([`fatal`]): a poisoned Secure Engine (the
//!   TRNG, `efr32s2-hal/se-trng`) is reported and ends in a controlled
//!   reset that records its cause.
//!
//! Status: host-tested product policy for a feasibility composition: the
//! tiny NCP's static RAM exceeds the 64 KiB SRAM, so no image links (see
//! `docs/book/src/advanced/tiny-ncp.md`). No ZS3L has run anything from it;
//! the xG21 Secure Engine TRNG is not proven on silicon.

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "flash-768k", feature = "flash-512k"))]
compile_error!("efr32mg21-ncp-product: enable exactly one flash size");
#[cfg(not(any(feature = "flash-768k", feature = "flash-512k")))]
compile_error!("efr32mg21-ncp-product: enable a flash size (`flash-768k` or `flash-512k`)");

pub mod fatal;
pub mod gecko;
pub mod layout;
pub mod stores;
pub mod transport;

pub use tuya_zs3l as board;

pub const MANUFACTURER: &str = "Zigbee-RS";
pub const MODEL: &str = "ZS3L-NCP";
/// `SYS_NEGOTIATE.chip` (spec 7.1): 4 = EFR32MG21.
pub const CHIP: u8 = 4;
/// `SYS_GET_INFO.board`.
pub const BOARD_NAME: &str = board::BOARD_NAME;
/// `SYS_GET_INFO.product`.
pub const PRODUCT_NAME: &str = "zigbee-rs-ncp";
/// Default `phyTransmitPower`: 10 dBm, inside the HAL's validated xG21
/// range (`efr32s2_hal::radio::TX_POWER_RANGE_DBM`, up to 20 dBm); the
/// ZS3L's certified limit is not published.
pub const TX_POWER_DBM: i8 = 10;
/// Security-journal role tag of the coordinator NCP (same as the other
/// zigbee-rs NCPs).
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
        assert_eq!(ROLE_TAG, 0x5203);
    }

    #[test]
    fn default_tx_power_is_inside_the_validated_hal_range() {
        let (min, max) = efr32s2_hal::radio::TX_POWER_RANGE_DBM;
        assert!((min..=max).contains(&TX_POWER_DBM));
        const { assert!(TX_POWER_DBM <= board::PA_MAX_DBM) };
    }

    #[test]
    fn the_tiny_tier_is_selected() {
        assert_eq!(zigbee_runtime::child_store::MAX_PERSISTED_CHILDREN, 32);
        assert_eq!(ncp_coordinator_app::ncp::WINDOW, 4);
    }
}
