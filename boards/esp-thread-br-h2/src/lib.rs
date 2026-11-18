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
//! | H2 RTS → S3 CTS (reserved R19, optional mod) | GPIO5 | GPIO4 |
//! | S3 RTS → H2 CTS (reserved R20, optional mod) | GPIO4 | GPIO5 |
//! | PTA request / grant / priority / tx_line | 12 / 13 / 10 / 11 | 0 / 1 / 2 / 3 |
//! | RCP RESET / BOOT (host-driven) | 7 / 8 | EN / GPIO9 |
//!
//! RTS/CTS exist only with the optional board modification on the reserved
//! 0 Ω links (feature `rtscts-reserved-lines`); H2 GPIO4/5 are then lost as
//! pad-JTAG pins. Whether R19/R20 are fitted on a delivered board, and the
//! H2 module's flash size (2 or 4 MB), are hardware gate H8.

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

/// Thread BR board PTA wiring (H2 side). Wired on the board; no driver in
/// the MAC yet.
pub const THREAD_BR_H2_PTA: PtaWiring = PtaWiring {
    request_gpio: 0,
    grant_gpio: 1,
    priority_gpio: 2,
    tx_line_gpio: 3,
};

/// The configured host UART of the Thread BR board's H2 (UART0 on
/// GPIO24/23, RTS on GPIO4 and CTS on GPIO5 with the board mod).
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
    #[cfg(feature = "rtscts-reserved-lines")]
    let uart = uart.with_rts(rts).with_cts(cts);
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
