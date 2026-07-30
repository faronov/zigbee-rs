//! Blocking BL702 UART.

use core::fmt;
use core::hint::spin_loop;

use crate::clock::{Clocks, Peripheral, configure_uart_fclk, enable_and_reset};
use crate::gpio::{Alternate, Disabled, Drive, FUNCTION_UART, Pin, Pull};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::Uart0;

const GLB_BASE: u32 = 0x4000_0000;
const UART0_BASE: u32 = 0x4000_a000;
const UART_SIG_SEL: u32 = GLB_BASE + 0x0c0;
const UART_TX_CONFIG: u32 = UART0_BASE;
const UART_RX_CONFIG: u32 = UART0_BASE + 0x004;
const UART_BIT_PERIOD: u32 = UART0_BASE + 0x008;
const UART_FIFO_CONFIG_0: u32 = UART0_BASE + 0x080;
const UART_FIFO_CONFIG_1: u32 = UART0_BASE + 0x084;
const UART_FIFO_WRITE: u32 = UART0_BASE + 0x088;

const FIFO_TIMEOUT: u32 = 1_000_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    BaudOutOfRange,
    Route,
    Pin(crate::gpio::ConfigError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WriteError {
    Timeout,
}

/// UART baud-period register value rounded as in the Bouffalo SDK.
pub const fn baud_period(clock_hz: u32, baud: u32) -> Option<u16> {
    if baud == 0 {
        return None;
    }
    let divisor = (clock_hz + baud / 2) / baud;
    if divisor == 0 || divisor > 65_536 {
        None
    } else {
        Some((divisor - 1) as u16)
    }
}

/// Blocking UART0 with 8-N-1 framing.
pub struct Uart0Tx<const TX: u8, const RX: u8> {
    _token: Uart0,
    _tx: Pin<TX, Alternate>,
    _rx: Pin<RX, Alternate>,
}

impl<const TX: u8, const RX: u8> Uart0Tx<TX, RX> {
    pub fn new(
        token: Uart0,
        tx: Pin<TX, Disabled>,
        rx: Pin<RX, Disabled>,
        clocks: Clocks,
        baud: u32,
    ) -> Result<Self, ConfigError> {
        if !valid_route(TX, RX) {
            return Err(ConfigError::Route);
        }
        let period = baud_period(clocks.fclk_hz(), baud).ok_or(ConfigError::BaudOutOfRange)?;
        let tx = tx
            .into_alternate(FUNCTION_UART, Pull::Up, Drive::Milliamp9_6)
            .map_err(ConfigError::Pin)?;
        let rx = rx
            .into_alternate(FUNCTION_UART, Pull::Up, Drive::Milliamp9_6)
            .map_err(ConfigError::Pin)?;

        enable_and_reset(Peripheral::Uart0);
        configure_uart_fclk(0);
        select_signal(TX, 2);
        select_signal(RX, 3);

        write32(UART_TX_CONFIG, 0);
        write32(UART_RX_CONFIG, 0);
        write32(
            UART_BIT_PERIOD,
            u32::from(period) | (u32::from(period) << 16),
        );
        write32(UART_FIFO_CONFIG_0, 0x0c);
        write32(UART_FIFO_CONFIG_0, 0);
        write32(UART_FIFO_CONFIG_1, 0x0f0f_0000);
        // 8 data bits, one stop bit, TX freerun; RX 8 data bits.
        write32(UART_TX_CONFIG, 0x0000_0f05);
        write32(UART_RX_CONFIG, 0x0000_0701);

        Ok(Self {
            _token: token,
            _tx: tx,
            _rx: rx,
        })
    }

    pub fn write_byte(&mut self, byte: u8) -> Result<(), WriteError> {
        for _ in 0..FIFO_TIMEOUT {
            if read32(UART_FIFO_CONFIG_1) & 0xff != 0 {
                write32(UART_FIFO_WRITE, u32::from(byte));
                return Ok(());
            }
            spin_loop();
        }
        Err(WriteError::Timeout)
    }

    pub fn write_all(&mut self, bytes: &[u8]) -> Result<(), WriteError> {
        for &byte in bytes {
            self.write_byte(byte)?;
        }
        Ok(())
    }
}

impl<const TX: u8, const RX: u8> fmt::Write for Uart0Tx<TX, RX> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        self.write_all(text.as_bytes()).map_err(|_| fmt::Error)
    }
}

fn select_signal(pin: u8, function: u8) {
    let shift = signal_shift(pin);
    #[cfg(target_arch = "riscv32")]
    riscv::interrupt::free(|| {
        rmw(UART_SIG_SEL, 0xf << shift, u32::from(function) << shift);
    });

    #[cfg(not(target_arch = "riscv32"))]
    rmw(UART_SIG_SEL, 0xf << shift, u32::from(function) << shift);
}

const fn valid_route(tx: u8, rx: u8) -> bool {
    tx & 7 != rx & 7
}

const fn signal_shift(pin: u8) -> u32 {
    (pin as u32 & 7) * 4
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proven_two_mbaud_period_is_fifteen() {
        assert_eq!(baud_period(32_000_000, 2_000_000), Some(15));
    }

    #[test]
    fn invalid_baud_is_rejected() {
        assert_eq!(baud_period(32_000_000, 0), None);
        assert_eq!(baud_period(32_000_000, 1), None);
    }

    #[test]
    fn monitor_pins_select_distinct_uart_signal_slots() {
        assert!(valid_route(14, 15));
        assert_eq!(signal_shift(14), 24);
        assert_eq!(signal_shift(15), 28);
    }
}
