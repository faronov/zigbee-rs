//! Exclusive Dongle-M (radio side) resources.

use core::sync::atomic::{AtomicBool, Ordering};

use efr32s2_hal::{
    clock::ClockControl,
    gpio::{Disabled, Level, Pin},
    peripherals::Peripherals,
    radio::RadioToken,
    rtcc::RtccToken,
    usart::{UartTx, Usart},
    watchdog::WdogToken,
};

use crate::{Efr32s2Flash, UART_RX_PIN, UART_RX_PORT, UART_TX_PIN, UART_TX_PORT};

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
        // SAFETY: singleton; PC01/PC02 are the distinct pins wired to the
        // ESP32 UART (community manifests, unverified).
        let (tx, rx) = unsafe {
            (
                peripherals.gpio.claim_pin(UART_TX_PORT, UART_TX_PIN).ok()?,
                peripherals.gpio.claim_pin(UART_RX_PORT, UART_RX_PIN).ok()?,
            )
        };
        Some(Self {
            clocks: peripherals.clocks,
            host_uart: HostUartToken {
                usart: peripherals.usart0,
                tx,
                rx,
            },
            radio: peripherals.radio,
            rtcc: peripherals.rtcc,
            wdog: peripherals.wdog,
            flash: peripherals.flash,
        })
    }
}

/// Exclusive USART0 + PC01/PC02 (the link to the ESP32, no RTS/CTS).
pub struct HostUartToken {
    usart: Usart,
    tx: Pin<Disabled>,
    rx: Pin<Disabled>,
}

impl HostUartToken {
    /// Interrupt-driven 8N1 host link without flow control. Requires the
    /// HFXO.
    #[cfg(feature = "host-uart")]
    pub fn open(
        self,
        baud: u32,
    ) -> Result<efr32s2_hal::usart::host::HostUart, efr32s2_hal::usart::host::HostUartError> {
        efr32s2_hal::usart::host::HostUart::open(
            self.usart,
            self.tx.into_push_pull(Level::High),
            self.rx.into_input(efr32s2_hal::gpio::Pull::Up),
            baud,
        )
    }

    /// Blocking transmit-only UART (lab logs). Requires the HFXO.
    pub fn into_uart_tx(self, baud: u32) -> UartTx {
        let _unused = self.rx;
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
