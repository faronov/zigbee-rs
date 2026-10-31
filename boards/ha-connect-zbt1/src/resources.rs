//! Exclusive Home Assistant Connect ZBT-1 resources.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32s2_hal::{
    clock::ClockControl,
    gpio::{Disabled, Level, Pin, Pull},
    peripherals::Peripherals,
    radio::RadioToken,
    rtcc::RtccToken,
    usart::Usart,
};

use crate::{
    Efr32s2Flash, HostUart, UART_BAUD, UART_CTS_PIN, UART_CTS_PORT, UART_RTS_PIN, UART_RTS_PORT,
    UART_RX_PIN, UART_RX_PORT, UART_TX_PIN, UART_TX_PORT,
};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Complete singleton resource set for the ZBT-1.
pub struct BoardResources {
    pub clocks: ClockControl,
    pub host_uart: HostUartToken,
    /// 802.15.4 radio (needs the HFXO from [`crate::init_clocks`]).
    pub radio: RadioToken,
    /// LFRCO-clocked RTCC: EM2 wake and time base.
    pub rtcc: RtccToken,
    /// Raw, exclusive internal-flash controller. The product bounds it to
    /// partitions above the resident bootloader.
    pub flash: Efr32s2Flash,
}

impl BoardResources {
    /// Acquire all supported board resources once.
    pub fn take() -> Option<Self> {
        if TAKEN
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return None;
        }

        let Some(mut peripherals) = Peripherals::take() else {
            TAKEN.store(false, Ordering::Release);
            return None;
        };

        // SAFETY: BoardResources is a singleton and PA6/PA5/PB1/PB0 are the
        // distinct USART0 pins wired to the CP2102N.
        let (tx, rx, cts, rts) = unsafe {
            (
                peripherals.gpio.claim_pin(UART_TX_PORT, UART_TX_PIN).ok()?,
                peripherals.gpio.claim_pin(UART_RX_PORT, UART_RX_PIN).ok()?,
                peripherals
                    .gpio
                    .claim_pin(UART_CTS_PORT, UART_CTS_PIN)
                    .ok()?,
                peripherals
                    .gpio
                    .claim_pin(UART_RTS_PORT, UART_RTS_PIN)
                    .ok()?,
            )
        };

        Some(Self {
            clocks: peripherals.clocks,
            host_uart: HostUartToken {
                usart: peripherals.usart0,
                tx,
                rx,
                cts,
                rts,
            },
            radio: peripherals.radio,
            rtcc: peripherals.rtcc,
            flash: peripherals.flash,
        })
    }
}

/// Exclusive USART0 + PA6/PA5/PB1/PB0 ownership.
pub struct HostUartToken {
    usart: Usart,
    tx: Pin<Disabled>,
    rx: Pin<Disabled>,
    cts: Pin<Disabled>,
    rts: Pin<Disabled>,
}

impl HostUartToken {
    /// Start a 115200 8N1 TX-only UART to the CP2102N with RTS asserted
    /// (active low). Requires the HFXO ([`crate::init_clocks`]).
    pub fn into_host_uart(self) -> HostUart {
        let rts = self.rts.into_push_pull(Level::Low);
        let cts = self.cts.into_input(Pull::None);
        let rx = self.rx.into_input(Pull::Up);
        let uart = self
            .usart
            .into_uart_tx(self.tx.into_push_pull(Level::High), UART_BAUD);
        HostUart::new(uart, rts, cts, rx)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn board_resources_are_singleton() {
        assert!(BoardResources::take().is_some());
        assert!(BoardResources::take().is_none());
    }
}
