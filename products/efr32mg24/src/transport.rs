//! ZRHP host transport: USART0 to the host MCU (RF-BM-MG24B1: the
//! ESP32-S3 on PA05/PA06; Dongle-M: the ESP32 on PC01/PC02), spec 4.1,
//! an LDMA-fed 8 KiB receive ring and an interrupt-driven 2 KiB transmit
//! ring (`efr32s2-hal` `usart0-host`).
//!
//! No RTS/CTS: the Dongle-M wires none, and the dev board's optional pins
//! stay unused so both boards behave the same. ZRHP credits are the only
//! flow control. The product default is 460 800 baud (spec 4.4, no flow
//! control); the Gecko bootloader keeps 115 200. The WSTK BRD4187 variant
//! runs at the board controller's VCOM rate (`efr32mg24_brd4187::VCOM_BAUD`,
//! 115 200) and accepts no other: measured, 460 800 and 921 600 fail on the
//! BRD4001A VCOM (framing errors on both ends). The LDMA keeps receiving
//! while interrupts are held off (the radio's Imm-Ack interrupt runs about
//! 650 µs; a flash page erase stalls instruction fetches for about 12 ms),
//! so the two-byte USART buffer no longer overruns. Overruns (a reader more
//! than 6 KiB behind stalls the DMA) and framing errors are still counted
//! as line faults, and ARQ recovers the frame.

use efr32s2_hal::usart::host::{HostUart, HostUartError};
use ncp_coordinator_app::transport::{HostTransport, LineFaults, TransportInfo, UartChangeError};

/// Product default line rate (volatile `SYS_SET_UART` changes revert to it
/// on reboot).
#[cfg(not(feature = "wstk-brd4187"))]
pub const UART_DEFAULT_BAUD: u32 = 460_800;
/// WSTK: the board controller's fixed VCOM rate.
#[cfg(feature = "wstk-brd4187")]
pub const UART_DEFAULT_BAUD: u32 = crate::board::VCOM_BAUD;
/// Line rate of the resident Gecko bootloader (XMODEM GBL upload).
pub const BOOTLOADER_BAUD: u32 = 115_200;

/// Rates `SYS_SET_UART` accepts: the spec's no-flow-control rates plus
/// 115 200 (the stock Dongle-M rate and the bootloader's). RTS/CTS is never
/// accepted.
#[cfg(not(feature = "wstk-brd4187"))]
pub const fn uart_rate_supported(baud: u32) -> bool {
    matches!(baud, 115_200 | 460_800 | 921_600)
}

/// WSTK: only the board controller's VCOM rate; a switch to any other rate
/// would strand the link until the next reset.
#[cfg(feature = "wstk-brd4187")]
pub const fn uart_rate_supported(baud: u32) -> bool {
    baud == crate::board::VCOM_BAUD
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
    uart: HostUart,
    overruns: u32,
    line_errors: u32,
}

impl Host {
    pub fn new(uart: HostUart) -> Self {
        Self {
            overruns: uart.rx_overruns(),
            line_errors: uart.rx_line_errors(),
            uart,
        }
    }
}

impl Host {
    /// Nothing queued and the transmitter idle (drain before a reset).
    pub fn tx_idle(&self) -> bool {
        self.uart.tx_idle()
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
            HostUartError::Baud => UartChangeError::InvalidRate,
            HostUartError::Busy => UartChangeError::Busy,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn uart_policy() {
        assert!(check_uart_change(UART_DEFAULT_BAUD, false).is_ok());
        assert!(check_uart_change(BOOTLOADER_BAUD, false).is_ok());
        #[cfg(not(feature = "wstk-brd4187"))]
        assert!(check_uart_change(921_600, false).is_ok());
        #[cfg(feature = "wstk-brd4187")]
        assert_eq!(
            check_uart_change(921_600, false),
            Err(UartChangeError::InvalidRate)
        );
        assert_eq!(
            check_uart_change(2_000_000, false),
            Err(UartChangeError::InvalidRate)
        );
        assert_eq!(
            check_uart_change(460_800, true),
            Err(UartChangeError::Unsupported)
        );
    }

    #[test]
    fn supported_rates_are_within_one_percent_at_pclk() {
        let pclk = efr32s2_hal::usart::PCLK_HZ as u64;
        for baud in [115_200u32, 460_800, 921_600] {
            let div = efr32s2_hal::usart::clkdiv(pclk as u32, baud) as u64;
            // fclk / (16 * (1 + CLKDIV / 256))
            let actual = pclk * 256 / (16 * (256 + div));
            let error_ppm = (actual as i64 - baud as i64).abs() * 1_000_000 / baud as i64;
            assert!(error_ppm < 10_000, "{baud}: {actual} ({error_ppm} ppm)");
        }
    }
}
