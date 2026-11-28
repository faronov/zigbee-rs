//! ZRHP host transport: UART0 through the dongle's USB-UART bridge
//! (spec 4.1), interrupt-driven receive into an 8 KiB ring (at least the
//! 8-frame `compact` window x 656 bytes) and an interrupt-fed 2 KiB
//! transmit ring (`cc26x2-hal` `uart-host-rings`).
//!
//! No RTS/CTS: the ZBDongle-P's optional flow-control DIP switch shares the
//! bridge's RTS with the RESET circuit and is not verified, so ZRHP credits
//! are the only flow control. The product default is 460 800 baud (spec
//! 4.4, no flow control). Flash erase/program masks interrupts for
//! milliseconds; bytes the 32-byte UART FIFO cannot hold meanwhile are
//! lost and counted as line faults, and ARQ recovers the frame.

use cc26x2_hal::uart::{Uart0, UartError};
use ncp_coordinator_app::transport::{HostTransport, LineFaults, TransportInfo, UartChangeError};

/// Product default line rate (volatile `SYS_SET_UART` changes revert to it
/// on reboot).
pub const UART_DEFAULT_BAUD: u32 = 460_800;

/// Rates `SYS_SET_UART` accepts: the spec's no-flow-control rates plus
/// 115 200 (the rate of the dongle's ROM bootloader tooling, for hosts or
/// bridges that cannot run faster). RTS/CTS is never accepted.
pub const fn uart_rate_supported(baud: u32) -> bool {
    matches!(baud, 115_200 | 460_800 | 921_600)
}

pub const fn check_uart_change(baud: u32, flow_control: bool) -> Result<(), UartChangeError> {
    if flow_control {
        return Err(UartChangeError::Unsupported);
    }
    if !uart_rate_supported(baud) {
        return Err(UartChangeError::InvalidRate);
    }
    Ok(())
}

/// The host link.
pub struct Host {
    uart: Uart0,
    overruns: u32,
    line_errors: u32,
}

impl Host {
    /// Take the opened UART and enable its receive interrupt.
    pub fn new(mut uart: Uart0) -> Self {
        uart.enable_rx_interrupt();
        Self {
            overruns: uart.rx_overruns(),
            line_errors: uart.rx_line_errors(),
            uart,
        }
    }
}

impl HostTransport for Host {
    fn read(&mut self, buf: &mut [u8]) -> usize {
        self.uart.read(buf)
    }

    fn write(&mut self, data: &[u8]) -> usize {
        self.uart.try_write(data)
    }

    fn tx_backlog(&self) -> usize {
        self.uart.tx_pending()
    }

    fn take_line_faults(&mut self) -> LineFaults {
        let overruns = self.uart.rx_overruns();
        let line_errors = self.uart.rx_line_errors();
        let new_overruns = overruns.wrapping_sub(self.overruns);
        let new_errors = line_errors.wrapping_sub(self.line_errors);
        self.overruns = overruns;
        self.line_errors = line_errors;
        LineFaults {
            errors: new_overruns.saturating_add(new_errors),
            overruns: new_overruns,
        }
    }

    fn info(&self) -> TransportInfo {
        TransportInfo {
            kind: 2,
            baud: self.uart.baud(),
            flow_control: false,
        }
    }

    fn check_uart(&self, baud: u32, flow_control: bool) -> Result<(), UartChangeError> {
        check_uart_change(baud, flow_control)
    }

    fn apply_uart(&mut self, baud: u32, flow_control: bool) -> Result<(), UartChangeError> {
        check_uart_change(baud, flow_control)?;
        self.uart.set_baud(baud).map_err(|error| match error {
            UartError::Baud => UartChangeError::InvalidRate,
            UartError::Busy | UartError::Power => UartChangeError::Busy,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uart_policy() {
        assert!(check_uart_change(UART_DEFAULT_BAUD, false).is_ok());
        assert!(check_uart_change(115_200, false).is_ok());
        assert!(check_uart_change(921_600, false).is_ok());
        assert_eq!(
            check_uart_change(2_000_000, false),
            Err(UartChangeError::InvalidRate)
        );
        assert_eq!(
            check_uart_change(921_600, true),
            Err(UartChangeError::Unsupported)
        );
    }
}
