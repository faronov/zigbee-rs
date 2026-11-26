//! zigbee-rs products for EFR32MG24 boards, one role per image:
//!
//! * `ncp` — the ZRHP network co-processor: a zigbee-rs coordinator, Trust
//!   Center and Green Power combo speaking ZRHP v1
//!   (`docs/book/src/advanced/host-protocol.md`) over the board's USART
//!   without flow control (`examples/efr32mg24-ncp`);
//! * `router` — an HA Range Extender with the Green Power Proxy Basic
//!   (`examples/efr32mg24-router`).
//!
//! This crate owns what is product-specific:
//!
//! * identity ([`MANUFACTURER`], [`MODEL`], [`CHIP`], [`ROLE_TAG`]);
//! * the memory map ([`layout`], `link/memory.x`): the resident **Gecko
//!   bootloader is kept** in `0x0800_0000..0x0800_6000` and is the NCP's
//!   update path (GBL over XMODEM at 115 200, see [`gecko`]); the NV
//!   journals sit at the top of flash, outside every GBL;
//! * the journals and their opening policy ([`stores`]), shared by both
//!   roles so a role change keeps the frame-counter reservations;
//! * the NCP host transport and its UART policy (`transport`);
//! * the router policy and profile (`router`);
//! * the Gecko `ApplicationProperties` record ([`gecko`]);
//! * the fatal-fault policy ([`fatal`]): a poisoned Secure Engine is
//!   reported and ends in a controlled reset that records its cause.
//!
//! `FW_*` (A/B slots) is not built in; see the platform guide for why the
//! Gecko path is the default and what an A/B layout would cost.
//!
//! The board is selected by feature: `rf-bm-mg24b1` (lab module),
//! `sonoff-dongle-m` or `wstk-brd4187` (Silicon Labs BRD4187 radio board on
//! a BRD4001A WSTK, the board the xG24 radio was brought up on).

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "ncp", feature = "router"))]
compile_error!("efr32mg24-product: enable exactly one role (`ncp` or `router`)");
#[cfg(not(any(feature = "ncp", feature = "router")))]
compile_error!("efr32mg24-product: enable a role (`ncp` or `router`)");
#[cfg(any(
    all(feature = "rf-bm-mg24b1", feature = "sonoff-dongle-m"),
    all(feature = "rf-bm-mg24b1", feature = "wstk-brd4187"),
    all(feature = "sonoff-dongle-m", feature = "wstk-brd4187"),
))]
compile_error!("efr32mg24-product: enable exactly one board variant");
#[cfg(not(any(
    feature = "rf-bm-mg24b1",
    feature = "sonoff-dongle-m",
    feature = "wstk-brd4187"
)))]
compile_error!(
    "efr32mg24-product: enable a board variant (`rf-bm-mg24b1`, `sonoff-dongle-m` or `wstk-brd4187`)"
);

pub mod fatal;
pub mod gecko;
pub mod layout;
#[cfg(feature = "router")]
pub mod router;
pub mod stores;
#[cfg(feature = "ncp")]
pub mod transport;

#[cfg(feature = "wstk-brd4187")]
pub use efr32mg24_brd4187 as board;
#[cfg(feature = "rf-bm-mg24b1")]
pub use rf_bm_mg24b1 as board;
#[cfg(feature = "sonoff-dongle-m")]
pub use sonoff_dongle_m as board;

pub const MANUFACTURER: &str = "Zigbee-RS";
#[cfg(all(feature = "rf-bm-mg24b1", feature = "ncp"))]
pub const MODEL: &str = "RF-BM-MG24B1-NCP";
#[cfg(all(feature = "sonoff-dongle-m", feature = "ncp"))]
pub const MODEL: &str = "Dongle-M-NCP";
#[cfg(all(feature = "rf-bm-mg24b1", feature = "router"))]
pub const MODEL: &str = "RF-BM-MG24B1-Router";
#[cfg(all(feature = "sonoff-dongle-m", feature = "router"))]
pub const MODEL: &str = "Dongle-M-Router";
#[cfg(all(feature = "wstk-brd4187", feature = "ncp"))]
pub const MODEL: &str = "BRD4187-NCP";
#[cfg(all(feature = "wstk-brd4187", feature = "router"))]
pub const MODEL: &str = "BRD4187-Router";
/// `SYS_NEGOTIATE.chip` (spec 7.1): 12 = EFR32MG24.
pub const CHIP: u8 = 12;
/// `SYS_GET_INFO.board`.
pub const BOARD_NAME: &str = board::BOARD_NAME;
/// `SYS_GET_INFO.product`.
pub const PRODUCT_NAME: &str = "zigbee-rs-ncp";
/// Default `phyTransmitPower`: 0 dBm on every board. The 20 dBm PA at
/// 39 MHz (`efr32s2_hal::radio::TX_POWER_RANGE_DBM` = `(-24, 20)`) is
/// hardware-verified on the BRD4187A only; the RF-BM-MG24B1 (10 dBm PA)
/// and the Dongle-M (38.4 MHz crystal) report `(0, 0)` and their radio
/// returns `Unsupported`. A higher product default waits for the Dongle-M's
/// RF path and TX power limit (platform guide, hardware gates).
pub const TX_POWER_DBM: i8 = 0;
/// Security-journal role tag of the coordinator NCP (same as the ESP and
/// CC26x2 NCPs).
#[cfg(feature = "ncp")]
pub const ROLE_TAG: u16 = 0x5203;
/// Security-journal role tag of the router (same as the other zigbee-rs
/// routers).
#[cfg(feature = "router")]
pub const ROLE_TAG: u16 = 0x5201;

/// Coordinator lifecycle policy: 2 ms receive slices so the host link is
/// serviced at least every ~2 ms (ZRHP `ack_delay` 1 ms, spec 5.8).
#[cfg(feature = "ncp")]
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
        #[cfg(feature = "ncp")]
        assert_eq!(ROLE_TAG, 0x5203);
        #[cfg(feature = "router")]
        assert_eq!(ROLE_TAG, 0x5201);
    }

    #[test]
    fn default_tx_power_is_inside_the_validated_hal_range() {
        let (min, max) = efr32s2_hal::radio::TX_POWER_RANGE_DBM;
        assert!((min..=max).contains(&TX_POWER_DBM));
    }
}
