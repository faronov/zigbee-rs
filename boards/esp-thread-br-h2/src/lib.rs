//! Espressif Thread Border Router / Zigbee Gateway board, ESP32-H2 side.
//!
//! The board's ESP32-H2-MINI-1 (2 MB `-H2S` or 4 MB `-H4S` flash) is wired
//! to the ESP32-S3 host (schematic v1.3, "Interface" sheet). This crate
//! states the physical wiring and builds the host UART on it; baud-rate policy, the protocol and which
//! transport an image uses are product decisions.
//!
//! | Signal | ESP32-S3 | ESP32-H2 |
//! |---|---|---|
//! | host → H2 (U0RXD) | GPIO18 | GPIO23 |
//! | H2 → host (U0TXD) | GPIO17 | GPIO24 |
//! | H2 RTS → S3 CTS (reserved 0 Ω R19) | GPIO5 | GPIO4 |
//! | S3 RTS → H2 CTS (reserved 0 Ω R20) | GPIO4 | GPIO5 |
//! | PTA request / grant / priority / tx_line | 12 / 13 / 10 / 11 | 0 / 1 / 2 / 3 |
//! | RCP RESET / BOOT (host-driven) | 7 / 8 | EN / GPIO9 |
//!
//! RTS/CTS run over the schematic's "Reserved" 0 Ω links R19/R20 (feature
//! `rtscts-reserved-lines`, the Thread BR profile's default since ZRHP
//! decision N10). They are fitted on the hwlab board (verified both ways,
//! 2026-11-19); H2 GPIO4/5 (MTCK/MTDI) are then UART pins, not pad JTAG
//! (USB-JTAG stays). The H2's CTS input has a pull-down, so a host that
//! does not drive its RTS never blocks the H2's transmitter.

#![cfg_attr(not(test), no_std)]

/// UART wiring as fitted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HostUartWiring {
    /// UART instance of the Zigbee chip.
    pub uart: u8,
    pub tx_gpio: u8,
    pub rx_gpio: u8,
    /// `Some` only when both RTS and CTS are wired.
    pub rts_cts_gpio: Option<(u8, u8)>,
}

/// PTA (external 802.15.4/Wi-Fi coexistence) lines of the Zigbee chip.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PtaWiring {
    pub request_gpio: u8,
    pub grant_gpio: u8,
    pub priority_gpio: u8,
    pub tx_line_gpio: u8,
}

/// Thread BR board, H2 side.
pub const THREAD_BR_H2_UART: HostUartWiring = HostUartWiring {
    uart: 0,
    tx_gpio: 24,
    rx_gpio: 23,
    rts_cts_gpio: if cfg!(feature = "rtscts-reserved-lines") {
        Some((4, 5))
    } else {
        None
    },
};

/// Thread BR board PTA wiring (H2 side; the H2 is the 4-wire follower,
/// driven by `esp32-zigbee-ncp-product`'s `pta::Pta`, feature `pta`).
pub const THREAD_BR_H2_PTA: PtaWiring = PtaWiring {
    request_gpio: 0,
    grant_gpio: 1,
    priority_gpio: 2,
    tx_line_gpio: 3,
};

/// The configured host UART of the Thread BR board's H2 (UART0 on
/// GPIO24/23; with `rtscts-reserved-lines` RTS on GPIO4 and CTS, pulled
/// down, on GPIO5, both through the GPIO matrix). Whether flow control is
/// enabled is the UART configuration's (product) decision.
#[cfg(target_os = "none")]
pub fn thread_br_h2_uart(
    uart0: esp_hal::peripherals::UART0<'static>,
    tx: esp_hal::peripherals::GPIO24<'static>,
    rx: esp_hal::peripherals::GPIO23<'static>,
    #[cfg(feature = "rtscts-reserved-lines")] rts: esp_hal::peripherals::GPIO4<'static>,
    #[cfg(feature = "rtscts-reserved-lines")] cts: esp_hal::peripherals::GPIO5<'static>,
    config: esp_hal::uart::Config,
) -> Result<esp_hal::uart::Uart<'static, esp_hal::Blocking>, esp_hal::uart::ConfigError> {
    let uart = esp_hal::uart::Uart::new(uart0, config)?
        .with_tx(tx)
        .with_rx(rx);
    // The pull-down is set first and the signal frozen, so `with_cts` keeps
    // it: an undriven host RTS reads as "ready" instead of stalling TX.
    #[cfg(feature = "rtscts-reserved-lines")]
    let uart = {
        use esp_hal::gpio::{Input, InputConfig, Pull};
        let cts = Input::new(cts, InputConfig::default().with_pull(Pull::Down));
        uart.with_rts(rts).with_cts(cts.peripheral_input())
    };
    Ok(uart)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn thread_br_uart_is_uart0_on_the_documented_pins() {
        assert_eq!(THREAD_BR_H2_UART.uart, 0);
        assert_eq!(
            (THREAD_BR_H2_UART.tx_gpio, THREAD_BR_H2_UART.rx_gpio),
            (24, 23)
        );
        assert_eq!(
            THREAD_BR_H2_UART.rts_cts_gpio.is_some(),
            cfg!(feature = "rtscts-reserved-lines")
        );
    }
}
