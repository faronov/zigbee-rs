//! UART0 (PL011-compatible, `inc/hw_uart.h`, driverlib `uart.c`).
//!
//! Transmit is a blocking FIFO write (log output and host frames). Receive
//! is interrupt driven into a bounded single-producer/single-consumer ring
//! ([`RX_RING_LEN`] bytes); [`Uart0::read`] drains it. The `UART0_COMB`
//! interrupt handler is defined here; overruns of the hardware FIFO or the
//! ring are counted, never silently ignored.

use core::sync::atomic::{AtomicU8, AtomicU16, AtomicU32, Ordering};

use crate::gpio::{self, Dio, Pull};
use crate::mmio::{self, UART0_BASE};
use crate::prcm::{self, Domain, Gate};

const DR: u32 = UART0_BASE;
const RSR_ECR: u32 = UART0_BASE + 0x04;
const FR: u32 = UART0_BASE + 0x18;
const IBRD: u32 = UART0_BASE + 0x24;
const FBRD: u32 = UART0_BASE + 0x28;
const LCRH: u32 = UART0_BASE + 0x2C;
const CTL: u32 = UART0_BASE + 0x30;
const IFLS: u32 = UART0_BASE + 0x34;
const IMSC: u32 = UART0_BASE + 0x38;
const MIS: u32 = UART0_BASE + 0x40;
const ICR: u32 = UART0_BASE + 0x44;

const FR_TXFE: u32 = 1 << 7;
const FR_TXFF: u32 = 1 << 5;
const FR_RXFE: u32 = 1 << 4;
const FR_BUSY: u32 = 1 << 3;
const LCRH_WLEN8: u32 = 0x60;
const LCRH_FEN: u32 = 1 << 4;
const CTL_UARTEN: u32 = 1 << 0;
const CTL_TXE: u32 = 1 << 8;
const CTL_RXE: u32 = 1 << 9;
const CTL_RTSEN: u32 = 1 << 14;
const CTL_CTSEN: u32 = 1 << 15;
/// RX, RX timeout and overrun interrupts.
const INT_RX: u32 = (1 << 4) | (1 << 6) | (1 << 10);

/// Receive ring capacity (power of two).
pub const RX_RING_LEN: usize = 1024;

/// Ownership token for UART0.
pub struct Uart0Token(pub(crate) ());

/// Pin assignment for UART0.
pub struct Uart0Pins {
    pub tx: Dio,
    pub rx: Option<Dio>,
    /// RTS/CTS hardware flow control, when the board wires both.
    pub flow: Option<(Dio, Dio)>,
}

/// Line configuration error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UartError {
    /// The divisor is out of range for the 48 MHz UART clock.
    Baud,
    /// The SERIAL power domain did not come up.
    Power,
}

/// Divisors for `baud` at `clock` (driverlib `UARTConfigSetExpClk`).
pub const fn divisors(clock: u32, baud: u32) -> Option<(u32, u32)> {
    if baud == 0 || baud > clock / 16 {
        return None;
    }
    let div = (clock as u64 * 8 / baud as u64).div_ceil(2);
    let ibrd = (div / 64) as u32;
    if ibrd == 0 || ibrd > 0xFFFF {
        return None;
    }
    Some((ibrd, (div % 64) as u32))
}

/// UART0 driver.
pub struct Uart0 {
    _pins: Uart0Pins,
    baud: u32,
}

impl Uart0 {
    /// Power the SERIAL domain, mux the pins and start 8N1 at `baud`.
    pub fn new(_token: Uart0Token, pins: Uart0Pins, baud: u32) -> Result<Self, UartError> {
        let (ibrd, fbrd) = divisors(crate::MCU_CLOCK_HZ, baud).ok_or(UartError::Baud)?;
        if !prcm::is_on(Domain::Serial) && !prcm::power_on(Domain::Serial) {
            return Err(UartError::Power);
        }
        prcm::enable(Gate::Uart0);
        prcm::load();

        pins.tx.configure(gpio::PORT_UART0_TX, false, Pull::None);
        if let Some(rx) = &pins.rx {
            rx.configure(gpio::PORT_UART0_RX, true, Pull::Up);
        }
        if let Some((rts, cts)) = &pins.flow {
            rts.configure(gpio::PORT_UART0_RTS, false, Pull::None);
            cts.configure(gpio::PORT_UART0_CTS, true, Pull::None);
        }

        mmio::write(CTL, 0);
        mmio::write(IBRD, ibrd);
        mmio::write(FBRD, fbrd);
        mmio::write(LCRH, LCRH_WLEN8 | LCRH_FEN);
        // RX FIFO interrupt at 1/2 full; the timeout interrupt flushes the rest.
        mmio::write(IFLS, 2 << 3 | 2);
        mmio::write(ICR, 0x7F2);
        let mut ctl = CTL_UARTEN | CTL_TXE;
        if pins.rx.is_some() {
            ctl |= CTL_RXE;
        }
        if pins.flow.is_some() {
            ctl |= CTL_RTSEN | CTL_CTSEN;
        }
        mmio::write(CTL, ctl);
        Ok(Self { _pins: pins, baud })
    }

    pub const fn baud(&self) -> u32 {
        self.baud
    }

    /// Change the line rate after draining the transmitter
    /// (ZRHP `SYS_SET_UART`).
    pub fn set_baud(&mut self, baud: u32) -> Result<(), UartError> {
        let (ibrd, fbrd) = divisors(crate::MCU_CLOCK_HZ, baud).ok_or(UartError::Baud)?;
        self.flush();
        let ctl = mmio::read(CTL);
        mmio::write(CTL, ctl & !CTL_UARTEN);
        mmio::write(IBRD, ibrd);
        mmio::write(FBRD, fbrd);
        mmio::write(LCRH, LCRH_WLEN8 | LCRH_FEN);
        mmio::write(CTL, ctl);
        self.baud = baud;
        Ok(())
    }

    /// Blocking write of `bytes`.
    pub fn write(&mut self, bytes: &[u8]) {
        write_blocking(bytes);
    }

    /// Wait until the last bit has left the shift register.
    pub fn flush(&mut self) {
        prcm::wait(|| mmio::read(FR) & (FR_TXFE | FR_BUSY) == FR_TXFE);
    }

    /// Unmask the RX interrupts that fill the ring.
    pub fn enable_rx_interrupt(&mut self) {
        mmio::write(IMSC, INT_RX);
        #[cfg(all(target_arch = "arm", target_os = "none"))]
        unsafe {
            cortex_m::peripheral::NVIC::unmask(crate::vectors::Interrupt::UART0_COMB)
        };
    }

    /// Move received bytes from the ring into `out`; returns the count.
    pub fn read(&mut self, out: &mut [u8]) -> usize {
        RING.pop(out)
    }

    /// Bytes lost to hardware FIFO overruns or a full ring.
    pub fn rx_overruns(&self) -> u32 {
        RX_OVERRUNS.load(Ordering::Relaxed)
    }
}

/// Blocking FIFO write usable before a [`Uart0`] exists (panic/log path).
pub fn write_blocking(bytes: &[u8]) {
    for &b in bytes {
        while mmio::read(FR) & FR_TXFF != 0 {
            core::hint::spin_loop();
        }
        mmio::write(DR, u32::from(b));
    }
}

/// Wait (bounded) until UART0 has shifted out everything queued.
pub fn flush_blocking() {
    prcm::wait(|| mmio::read(FR) & (FR_TXFE | FR_BUSY) == FR_TXFE);
}

static RX_OVERRUNS: AtomicU32 = AtomicU32::new(0);
static RING: Ring = Ring::new();

struct Ring {
    buf: [AtomicU8; RX_RING_LEN],
    head: AtomicU16,
    tail: AtomicU16,
}

impl Ring {
    const fn new() -> Self {
        Self {
            buf: [const { AtomicU8::new(0) }; RX_RING_LEN],
            head: AtomicU16::new(0),
            tail: AtomicU16::new(0),
        }
    }

    /// Producer (interrupt).
    fn push(&self, byte: u8) -> bool {
        let head = self.head.load(Ordering::Relaxed);
        let tail = self.tail.load(Ordering::Acquire);
        if head.wrapping_sub(tail) as usize >= RX_RING_LEN {
            return false;
        }
        self.buf[head as usize % RX_RING_LEN].store(byte, Ordering::Relaxed);
        self.head.store(head.wrapping_add(1), Ordering::Release);
        true
    }

    /// Consumer (task).
    fn pop(&self, out: &mut [u8]) -> usize {
        let head = self.head.load(Ordering::Acquire);
        let mut tail = self.tail.load(Ordering::Relaxed);
        let mut n = 0;
        while tail != head && n < out.len() {
            out[n] = self.buf[tail as usize % RX_RING_LEN].load(Ordering::Relaxed);
            tail = tail.wrapping_add(1);
            n += 1;
        }
        self.tail.store(tail, Ordering::Release);
        n
    }
}

const _: () = assert!(RX_RING_LEN.is_power_of_two() && RX_RING_LEN <= 32768);

/// UART0 combined interrupt: drain the RX FIFO into the ring.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[unsafe(no_mangle)]
extern "C" fn UART0_COMB() {
    let mis = mmio::read(MIS);
    mmio::write(ICR, mis);
    if mis & (1 << 10) != 0 {
        RX_OVERRUNS.fetch_add(1, Ordering::Relaxed);
        mmio::write(RSR_ECR, 0);
    }
    while mmio::read(FR) & FR_RXFE == 0 {
        let byte = (mmio::read(DR) & 0xFF) as u8;
        if !RING.push(byte) {
            RX_OVERRUNS.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisors_match_driverlib() {
        // 115200 at 48 MHz: 26.0417 -> IBRD 26, FBRD 3.
        assert_eq!(divisors(48_000_000, 115_200), Some((26, 3)));
        // 460800: 6.5104 -> 6, 33.
        assert_eq!(divisors(48_000_000, 460_800), Some((6, 33)));
        // 921600: 3.2552 -> 3, 16.
        assert_eq!(divisors(48_000_000, 921_600), Some((3, 16)));
        assert_eq!(divisors(48_000_000, 0), None);
        assert_eq!(divisors(48_000_000, 4_000_000), None);
    }

    #[test]
    fn ring_is_bounded_and_ordered() {
        let ring = Ring::new();
        for i in 0..RX_RING_LEN {
            assert!(ring.push(i as u8));
        }
        assert!(!ring.push(0xAA));
        let mut out = [0u8; 4];
        assert_eq!(ring.pop(&mut out), 4);
        assert_eq!(out, [0, 1, 2, 3]);
        assert!(ring.push(0xAA));
    }
}
