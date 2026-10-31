//! Polled asynchronous (UART) mode of the Series 2 USART.
//!
//! Register offsets are identical on xG21 and xG22; only the peripheral base
//! addresses and GPIO route offsets differ (see [`crate::chip`]).

use crate::chip::{BusClock, base, route};
use crate::gpio::{Pin, PushPull};

const EN: u32 = 0x04;
const CTRL: u32 = 0x08;
const FRAME: u32 = 0x0C;
const CMD: u32 = 0x14;
const STATUS: u32 = 0x18;
const CLKDIV: u32 = 0x1C;
const TXDATA: u32 = 0x3C;

const CMD_RXDIS: u32 = 1 << 1;
const CMD_TXEN: u32 = 1 << 2;
const CMD_CLEARTX: u32 = 1 << 11;
const CMD_CLEARRX: u32 = 1 << 10;
const STATUS_TXC: u32 = 1 << 5;
const STATUS_TXBL: u32 = 1 << 6;
/// 8 data bits, no parity, 1 stop bit.
const FRAME_8N1: u32 = 0x1005;

/// GPIO `USARTn_ROUTEEN` bit and route-register offsets relative to it.
const ROUTEEN_TXPEN: u32 = 1 << 4;
const TXROUTE_FROM_ROUTEEN: u32 = 0x18;

/// USART PCLK after [`crate::clock::ClockControl::configure_hfxo`].
pub const PCLK_HZ: u32 = 38_400_000;

/// Which USART instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Instance {
    Usart0,
    Usart1,
}

impl Instance {
    const fn base(self) -> u32 {
        match self {
            Self::Usart0 => base::USART0,
            Self::Usart1 => base::USART1,
        }
    }
    const fn routeen(self) -> u32 {
        base::GPIO
            + match self {
                Self::Usart0 => route::USART0_ROUTEEN,
                Self::Usart1 => route::USART1_ROUTEEN,
            }
    }
    const fn clock(self) -> BusClock {
        match self {
            Self::Usart0 => BusClock::USART0,
            Self::Usart1 => BusClock::USART1,
        }
    }
}

/// Exclusive ownership of one USART block.
pub struct Usart {
    instance: Instance,
}

impl Usart {
    pub(crate) const fn new(instance: Instance) -> Self {
        Self { instance }
    }

    pub const fn instance(&self) -> Instance {
        self.instance
    }

    /// Configure a TX-only 8N1 UART on `tx`.
    pub fn into_uart_tx(self, tx: Pin<PushPull>, baud: u32) -> UartTx {
        let instance = self.instance;
        crate::clock::enable_bus_clock(instance.clock());
        let base = instance.base();
        let routeen = instance.routeen();
        unsafe {
            write(base + EN, 1);
            write(base + CMD, CMD_RXDIS | CMD_CLEARTX | CMD_CLEARRX);
            write(base + CTRL, 0); // async, 16x oversampling
            write(base + FRAME, FRAME_8N1);
            write(base + CLKDIV, clkdiv(PCLK_HZ, baud));
            write(routeen + TXROUTE_FROM_ROUTEEN, route_value(&tx));
            write(routeen, read(routeen) | ROUTEEN_TXPEN);
            write(base + CMD, CMD_TXEN);
        }
        UartTx {
            usart: self,
            _tx: tx,
        }
    }
}

/// Route register value: `PORT` in bits 1:0, `PIN` in bits 19:16.
pub(crate) fn route_value<M>(pin: &Pin<M>) -> u32 {
    (pin.port() as u32) | ((pin.number() as u32) << 16)
}

/// Register value of `CLKDIV` for 16x oversampling, computed exactly like
/// GSDK `USART_BaudrateAsyncSet`: `(round(32 * fclk / (16 * baud)) - 32) * 8`.
/// The resulting baud rate is `fclk / (16 * (1 + CLKDIV / 256))`.
pub const fn clkdiv(fclk: u32, baud: u32) -> u32 {
    let ovs_baud = 16 * baud as u64;
    let div = (32 * fclk as u64 + ovs_baud / 2) / ovs_baud;
    ((div.saturating_sub(32) * 8) as u32) & 0x007F_FFF8
}

/// Blocking UART transmitter.
pub struct UartTx {
    usart: Usart,
    _tx: Pin<PushPull>,
}

impl UartTx {
    #[inline]
    pub fn write_byte(&mut self, byte: u8) {
        let base = self.usart.instance.base();
        unsafe {
            while read(base + STATUS) & STATUS_TXBL == 0 {}
            write(base + TXDATA, byte as u32);
        }
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            self.write_byte(byte);
        }
    }

    /// Wait until the shift register is empty.
    pub fn flush(&mut self) {
        let base = self.usart.instance.base();
        unsafe { while read(base + STATUS) & STATUS_TXC == 0 {} }
    }
}

impl core::fmt::Write for UartTx {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for byte in s.bytes() {
            if byte == b'\n' {
                self.write_byte(b'\r');
            }
            self.write_byte(byte);
        }
        Ok(())
    }
}

#[inline]
unsafe fn read(address: u32) -> u32 {
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[inline]
unsafe fn write(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clkdiv_matches_gsdk_for_115200() {
        // round(32 * 38.4e6 / (16 * 115200)) = 667 -> (667 - 32) * 8.
        assert_eq!(clkdiv(38_400_000, 115_200), 5080);
    }

    #[test]
    #[cfg(feature = "efr32mg22")]
    fn xg22_vcom_route_addresses() {
        assert_eq!(Instance::Usart1.base(), 0x4006_0000);
        assert_eq!(Instance::Usart1.routeen(), 0x4003_C63C);
        assert_eq!(
            Instance::Usart1.routeen() + TXROUTE_FROM_ROUTEEN,
            0x4003_C654
        );
    }

    #[test]
    #[cfg(feature = "efr32mg21")]
    fn xg21_route_addresses() {
        assert_eq!(Instance::Usart0.base(), 0x4005_8000);
        assert_eq!(Instance::Usart1.routeen(), 0x4003_C5CC);
    }
}
