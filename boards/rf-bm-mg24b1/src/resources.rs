//! Exclusive RF-BM-MG24B1 dev-board resources.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32s2_hal::{
    clock::ClockControl,
    crypto::Crypto,
    gpio::{Disabled, Level, Pin},
    peripherals::Peripherals,
    radio::RadioToken,
    rtcc::RtccToken,
    usart::{UartTx, Usart},
    watchdog::WdogToken,
};

use crate::{
    Efr32s2Flash, UART_CTS_PIN, UART_CTS_PORT, UART_RTS_PIN, UART_RTS_PORT, UART_RX_PIN,
    UART_RX_PORT, UART_TX_PIN, UART_TX_PORT,
};

static TAKEN: AtomicBool = AtomicBool::new(false);

/// Complete singleton resource set.
pub struct BoardResources {
    pub clocks: ClockControl,
    pub host_uart: HostUartToken,
    /// 802.15.4 radio (needs the HFXO). The xG24 radio tables are not
    /// captured yet: `RadioToken::init` returns `Unsupported`.
    pub radio: RadioToken,
    /// LFRCO-clocked SYSRTC0 (the HAL's `rtcc` module): time base.
    pub rtcc: RtccToken,
    pub wdog: WdogToken,
    /// Raw, exclusive internal-flash controller; the product bounds it to
    /// its partitions.
    pub flash: Efr32s2Flash,
    /// Secure Engine AES-128 service (`efr32s2_hal::crypto`). The product
    /// installs it as the MAC's only AES provider.
    pub crypto: Crypto,
}

impl BoardResources {
    /// Acquire all board resources once.
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
        // SAFETY: singleton; PA05..PA08 are distinct pins wired only to the
        // host header. RTS/CTS are claimed so nothing else drives them.
        let (tx, rx, rts, cts) = unsafe {
            (
                peripherals.gpio.claim_pin(UART_TX_PORT, UART_TX_PIN).ok()?,
                peripherals.gpio.claim_pin(UART_RX_PORT, UART_RX_PIN).ok()?,
                peripherals
                    .gpio
                    .claim_pin(UART_RTS_PORT, UART_RTS_PIN)
                    .ok()?,
                peripherals
                    .gpio
                    .claim_pin(UART_CTS_PORT, UART_CTS_PIN)
                    .ok()?,
            )
        };
        Some(Self {
            clocks: peripherals.clocks,
            host_uart: HostUartToken {
                usart: peripherals.usart0,
                tx,
                rx,
                rts,
                cts,
            },
            radio: peripherals.radio,
            rtcc: peripherals.rtcc,
            wdog: peripherals.wdog,
            flash: peripherals.flash,
            crypto: peripherals.crypto,
        })
    }
}

/// Exclusive USART0 + PA05/PA06 (+ the unused RTS/CTS pins).
pub struct HostUartToken {
    usart: Usart,
    tx: Pin<Disabled>,
    rx: Pin<Disabled>,
    rts: Pin<Disabled>,
    cts: Pin<Disabled>,
}

impl HostUartToken {
    /// Interrupt-driven 8N1 host link without flow control (RTS/CTS left
    /// disabled). Requires the HFXO.
    #[cfg(feature = "host-uart")]
    pub fn open(
        self,
        baud: u32,
    ) -> Result<efr32s2_hal::usart::host::HostUart, efr32s2_hal::usart::host::HostUartError> {
        let _unused_flow_control = (self.rts, self.cts);
        efr32s2_hal::usart::host::HostUart::open(
            self.usart,
            self.tx.into_push_pull(Level::High),
            self.rx.into_input(efr32s2_hal::gpio::Pull::Up),
            baud,
        )
    }

    /// Blocking transmit-only UART (lab logs). Requires the HFXO.
    pub fn into_uart_tx(self, baud: u32) -> UartTx {
        let _unused = (self.rx, self.rts, self.cts);
        self.usart
            .into_uart_tx(self.tx.into_push_pull(Level::High), baud)
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
