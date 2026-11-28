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
//! `sonoff-dongle-m`, `wstk-brd4187` (Silicon Labs BRD4187 radio board on
//! a BRD4001A WSTK, where the 20 dBm xG24 radio was brought up) or
//! `ek2703a` (xG24 Explorer Kit, where the 10 dBm radio was brought up).

#![cfg_attr(not(test), no_std)]

#[cfg(all(feature = "ncp", feature = "router"))]
compile_error!("efr32mg24-product: enable exactly one role (`ncp` or `router`)");
#[cfg(not(any(feature = "ncp", feature = "router")))]
compile_error!("efr32mg24-product: enable a role (`ncp` or `router`)");
#[cfg(any(
    all(feature = "rf-bm-mg24b1", feature = "sonoff-dongle-m"),
    all(feature = "rf-bm-mg24b1", feature = "wstk-brd4187"),
    all(feature = "rf-bm-mg24b1", feature = "ek2703a"),
    all(feature = "sonoff-dongle-m", feature = "wstk-brd4187"),
    all(feature = "sonoff-dongle-m", feature = "ek2703a"),
    all(feature = "wstk-brd4187", feature = "ek2703a"),
))]
compile_error!("efr32mg24-product: enable exactly one board variant");
#[cfg(not(any(
    feature = "rf-bm-mg24b1",
    feature = "sonoff-dongle-m",
    feature = "wstk-brd4187",
    feature = "ek2703a"
)))]
compile_error!(
    "efr32mg24-product: enable a board variant (`rf-bm-mg24b1`, `sonoff-dongle-m`, `wstk-brd4187` or `ek2703a`)"
);

pub mod fatal;
pub mod gecko;
pub mod layout;
#[cfg(feature = "ncp")]
pub mod platform;
#[cfg(feature = "router")]
pub mod router;
pub mod stores;
#[cfg(feature = "ncp")]
pub mod transport;

#[cfg(feature = "wstk-brd4187")]
pub use efr32mg24_brd4187 as board;
#[cfg(feature = "ek2703a")]
pub use efr32mg24_ek2703a as board;
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
#[cfg(all(feature = "ek2703a", feature = "ncp"))]
pub const MODEL: &str = "EK2703A-NCP";
#[cfg(all(feature = "ek2703a", feature = "router"))]
pub const MODEL: &str = "EK2703A-Router";
/// `SYS_NEGOTIATE.chip` (spec 7.1): 12 = EFR32MG24.
pub const CHIP: u8 = 12;
/// `SYS_GET_INFO.board`.
pub const BOARD_NAME: &str = board::BOARD_NAME;
/// `SYS_GET_INFO.product`.
pub const PRODUCT_NAME: &str = "zigbee-rs-ncp";
/// Default `phyTransmitPower`, until the host sets another with
/// `NWK_SET_TX_POWER` (the NCP: [`platform::Mg24Platform`]; volatile, the
/// host replays it after a reset).
///
/// - Dongle-M: +8 dBm on the 20 dBm PA (`TX_POWER_RANGE_DBM` `(-24,
///   20)`; the 7.9 dBm table entry, an exact step), by user decision. The
///   stock image's PA configuration is GSDK's default one (highest PA,
///   3300 mV, 10 us ramp), its runtime power comes from the EZSP host.
///   Certification limit of the product unknown. In the EU the binding
///   limit for 802.15.4 is EN 300 328's 10 dBm/MHz EIRP power spectral
///   density: about 11.5 dBm EIRP for the 2 Mchip/s O-QPSK spectrum (about
///   +6.5 dBm conducted with the ~5 dBi antenna; the PSD scales with
///   antenna gain), so +8 dBm is about 1.4 dB over that estimate and +6 dBm
///   would comply by it. The host can lower or raise the power
///   (`NWK_SET_TX_POWER`); the regional limit is the user's responsibility
///   (platform guide, "Transmit power").
/// - Every other board: 0 dBm. At 39 MHz the 20 dBm PA is
///   hardware-verified on the BRD4187A and the 10 dBm PA (`(-30, 10)`) on
///   the EK2703A; a higher default waits for each board's RF path and TX
///   power limit (platform guide, hardware gates).
#[cfg(feature = "sonoff-dongle-m")]
pub const TX_POWER_DBM: i8 = 8;
#[cfg(not(feature = "sonoff-dongle-m"))]
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
        // Every board names its crystal, so its radio is supported.
        const {
            assert!(
                efr32s2_hal::radio::TX_POWER_RANGE_DBM.1 > efr32s2_hal::radio::TX_POWER_RANGE_DBM.0
            )
        };
        const { assert!(TX_POWER_DBM <= board::PA_MAX_DBM) };
        // The default is a PA table step, so a host reading it back and
        // setting it again changes nothing.
        assert_eq!(
            efr32s2_hal::radio::tx_power_step_dbm(TX_POWER_DBM),
            TX_POWER_DBM
        );
    }

    /// Dongle-M: +8 dBm by user decision (about 1.4 dB over the EU PSD
    /// estimate with a 5 dBi antenna; platform guide, "Transmit power").
    #[test]
    #[cfg(feature = "sonoff-dongle-m")]
    fn dongle_m_default_is_8_dbm() {
        assert_eq!(TX_POWER_DBM, 8);
    }
}
