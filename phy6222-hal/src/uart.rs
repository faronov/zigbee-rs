//! UART0 blocking transmitter (DesignWare 16550, 4-byte register stride).
//!
//! Register sequence follows the PHYPLUS SDK `uart_hw_init`
//! (`components/driver/uart/uart.c`). Only the transmit path is provided;
//! it is intended for lab diagnostics and boot logs, not for a production
//! command interface.

use crate::peripherals::Uart0Token;
use crate::regs::*;

const RBR_THR_DLL: u32 = AP_UART0_BASE;
const IER_DLM: u32 = AP_UART0_BASE + 0x04;
const FCR: u32 = AP_UART0_BASE + 0x08;
const LCR: u32 = AP_UART0_BASE + 0x0C;
const MCR: u32 = AP_UART0_BASE + 0x10;
const LSR: u32 = AP_UART0_BASE + 0x14;
const USR: u32 = AP_UART0_BASE + 0x7C;

const LSR_TEMT: u32 = 0x40;
const USR_BUSY: u32 = 0x01;
/// HAL logical index of P27, the SDK's dummy UART pad. Boards must not
/// use P27 for anything that a brief pull-up and UART mux would disturb.
const DUMMY_RX_PIN: u8 = 18;
const USR_TFNF: u32 = 0x02;

/// UART0 pin selection, as HAL logical GPIO indices.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    pub tx_pin: u8,
    pub rx_pin: Option<u8>,
    pub baud: u32,
}

/// Divisor used by the SDK: `((pclk >> 4) + baud / 2) / baud`.
pub const fn divisor(pclk_hz: u32, baud: u32) -> u32 {
    ((pclk_hz >> 4) + (baud >> 1)) / baud
}

pub struct Uart0 {
    _token: Uart0Token,
}

impl Uart0 {
    /// Configure UART0 8N1 with FIFOs enabled. The divisor is derived from
    /// the current HCLK, so call this after the board's clock init.
    pub fn new(token: Uart0Token, config: Config) -> Self {
        reg_write(PCR_SW_CLK, reg_read(PCR_SW_CLK) | MOD_UART0_BIT);
        // SDK `hal_clk_reset(MOD_UART0)`. Required, not cosmetic: the
        // DesignWare UART ignores LCR/divisor writes while `USR.BUSY` is
        // set, and the boot ROM leaves UART0 configured (divisor 17 for
        // 115200 at 32 MHz). Without the reset the divisor below is dropped
        // and the console runs at 48/32 x the requested baud (observed on a
        // PB-03F: ~176 kBd instead of 115.2 kBd).
        reg_write(PCR_SW_RESET0, reg_read(PCR_SW_RESET0) & !MOD_UART0_BIT);
        reg_write(PCR_SW_RESET0, reg_read(PCR_SW_RESET0) | MOD_UART0_BIT);

        // Park the UART input on the SDK's dummy pad (P27, pulled up) while
        // the line settings change: an unrouted or noisy RX input reads as
        // a start bit, keeps `USR.BUSY` set and makes the core drop the
        // LCR/divisor writes (SDK `uart_hw_init` does the same).
        crate::gpio::clear_fmux(config.tx_pin);
        if let Some(rx) = config.rx_pin {
            crate::gpio::clear_fmux(rx);
        }
        crate::gpio::set_pull(DUMMY_RX_PIN, crate::gpio::Pull::StrongPullUp);
        crate::gpio::set_fmux(DUMMY_RX_PIN, FMUX_UART0_RX);
        let mut guard = 10_000u32;
        while reg_read(USR) & USR_BUSY != 0 && guard > 0 {
            let _ = reg_read(RBR_THR_DLL);
            guard -= 1;
        }

        let dll = divisor(crate::clock::hclk_hz(), config.baud);
        reg_write(LCR, 0);
        reg_write(MCR, 0);
        reg_write(LCR, 0x80);
        reg_write(IER_DLM, (dll >> 8) & 0xFF);
        reg_write(RBR_THR_DLL, dll & 0xFF);
        reg_write(LCR, 0x03);
        // FIFO enable, reset RX/TX FIFOs.
        reg_write(FCR, 0x07);
        reg_write(IER_DLM, 0);

        crate::gpio::set_pull(config.tx_pin, crate::gpio::Pull::StrongPullUp);
        crate::gpio::set_fmux(config.tx_pin, FMUX_UART0_TX);
        if let Some(rx) = config.rx_pin {
            crate::gpio::set_pull(rx, crate::gpio::Pull::StrongPullUp);
            crate::gpio::set_fmux(rx, FMUX_UART0_RX);
        }
        crate::gpio::clear_fmux(DUMMY_RX_PIN);
        crate::gpio::set_pull(DUMMY_RX_PIN, crate::gpio::Pull::PullDown);
        Self { _token: token }
    }

    pub fn write_byte(&mut self, byte: u8) {
        while reg_read(USR) & USR_TFNF == 0 {}
        reg_write(RBR_THR_DLL, u32::from(byte));
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.write_byte(b);
        }
    }

    /// Wait until the shift register is empty.
    pub fn flush(&mut self) {
        while reg_read(LSR) & LSR_TEMT == 0 {}
    }
}

impl core::fmt::Write for Uart0 {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for b in s.bytes() {
            if b == b'\n' {
                self.write_byte(b'\r');
            }
            self.write_byte(b);
        }
        Ok(())
    }
}

// SDK divisor formula sanity (evaluated at compile time; the HAL is
// target-only and has no host test harness).
const _: () = assert!(divisor(48_000_000, 115_200) == 26);
const _: () = assert!(divisor(32_000_000, 115_200) == 17);
