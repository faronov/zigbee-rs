//! Interrupt-driven USART0 host link (feature `usart0-host`).
//!
//! Network co-processor images talk to their host over USART0 without
//! RTS/CTS. The `USART0_RX` handler moves every received byte into an
//! 8 KiB ring (at least eight 656-byte ZRHP frames, spec 4.4), and the
//! `USART0_TX` handler feeds the transmitter from a 2 KiB ring. Both rings
//! are single-producer/single-consumer: the handler is the only producer
//! (RX) or consumer (TX), the task the other side, so neither needs a
//! critical section.
//!
//! Loss model: the USART buffers two received bytes. Anything that keeps
//! the RX handler from running for longer than that (an MSC page erase
//! stalls flash fetches for milliseconds) overruns the hardware buffer.
//! Overruns (`IF.RXOF`), framing and parity errors are counted, never
//! hidden; the ZRHP link layer retransmits the damaged frame.
//!
//! Status: host-tested ring logic; the register sequence follows GSDK
//! 4.5.0 `em_usart.c` and is not yet hardware-proven on any die.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use super::{CLKDIV, CMD, CTRL, EN, FRAME, FRAME_8N1, Instance, STATUS, TXDATA};
use super::{ROUTEEN_TXPEN, TXROUTE_FROM_ROUTEEN, Usart, clkdiv, read, route_value, write};
use crate::gpio::{Input, Pin, PushPull};
#[cfg(all(target_arch = "arm", target_os = "none"))]
use crate::vectors::Interrupt;

/// Receive ring size (power of two).
pub const RX_RING: usize = [8192, 4096][cfg!(feature = "usart0-host-small") as usize];
/// Transmit ring size (power of two).
pub const TX_RING: usize = [2048, 1024][cfg!(feature = "usart0-host-small") as usize];
const _: () = assert!(RX_RING.is_power_of_two() && TX_RING.is_power_of_two());

const RXDATAX: u32 = 0x20;
const IF: u32 = 0x48;
const IEN: u32 = 0x4C;
const SET: u32 = 0x1000;
const CLR: u32 = 0x2000;

const CMD_RXEN: u32 = 1 << 0;
const CMD_RXDIS: u32 = 1 << 1;
const CMD_TXEN: u32 = 1 << 2;
const CMD_CLEARRX: u32 = 1 << 11;
const CMD_CLEARTX: u32 = 1 << 10;
const STATUS_TXC: u32 = 1 << 5;
const STATUS_TXBL: u32 = 1 << 6;
const STATUS_RXDATAV: u32 = 1 << 7;
const RXDATAX_PERR: u32 = 1 << 14;
const RXDATAX_FERR: u32 = 1 << 15;
const IEN_TXBL: u32 = 1 << 1;
const IEN_RXDATAV: u32 = 1 << 2;
const IF_RXOF: u32 = 1 << 4;
const ROUTEEN_RXPEN: u32 = 1 << 2;
const RXROUTE_FROM_ROUTEEN: u32 = 0x10;

/// Lowest and highest line rate the 16x-oversampled USART reaches within
/// 1 % at the board's PCLK (spec rates: 115 200, 460 800, 921 600).
pub const MIN_BAUD: u32 = 9_600;
pub const MAX_BAUD: u32 = 1_000_000;

/// Single-producer/single-consumer byte ring with free-running indices.
pub struct Ring<const N: usize> {
    buf: UnsafeCell<[u8; N]>,
    head: AtomicUsize,
    tail: AtomicUsize,
}

// SAFETY: one producer writes only `buf[head]` then publishes `head`; one
// consumer reads only `buf[tail]` below the published `head`, then
// publishes `tail`. The two never touch the same byte concurrently.
unsafe impl<const N: usize> Sync for Ring<N> {}

impl<const N: usize> Ring<N> {
    pub const fn new() -> Self {
        Self {
            buf: UnsafeCell::new([0; N]),
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    pub fn len(&self) -> usize {
        self.head
            .load(Ordering::Acquire)
            .wrapping_sub(self.tail.load(Ordering::Acquire))
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Producer side: append one byte; `false` when full.
    pub fn push(&self, byte: u8) -> bool {
        let head = self.head.load(Ordering::Relaxed);
        if head.wrapping_sub(self.tail.load(Ordering::Acquire)) >= N {
            return false;
        }
        // SAFETY: the slot is outside the consumer's readable range.
        unsafe { (*self.buf.get())[head % N] = byte };
        self.head.store(head.wrapping_add(1), Ordering::Release);
        true
    }

    /// Producer side: append as much of `data` as fits.
    pub fn push_slice(&self, data: &[u8]) -> usize {
        let mut count = 0;
        for &byte in data {
            if !self.push(byte) {
                break;
            }
            count += 1;
        }
        count
    }

    /// Consumer side: take one byte.
    pub fn pop(&self) -> Option<u8> {
        let tail = self.tail.load(Ordering::Relaxed);
        if self.head.load(Ordering::Acquire) == tail {
            return None;
        }
        // SAFETY: the slot is below the published head.
        let byte = unsafe { (*self.buf.get())[tail % N] };
        self.tail.store(tail.wrapping_add(1), Ordering::Release);
        Some(byte)
    }

    /// Consumer side: move up to `out.len()` bytes out.
    pub fn pop_slice(&self, out: &mut [u8]) -> usize {
        let mut count = 0;
        for slot in out.iter_mut() {
            match self.pop() {
                Some(byte) => *slot = byte,
                None => break,
            }
            count += 1;
        }
        count
    }

    /// Drop everything (only while the producer is stopped).
    fn clear(&self) {
        self.tail
            .store(self.head.load(Ordering::Acquire), Ordering::Release);
    }
}

impl<const N: usize> Default for Ring<N> {
    fn default() -> Self {
        Self::new()
    }
}

static RX: Ring<RX_RING> = Ring::new();
static TX: Ring<TX_RING> = Ring::new();
/// Bytes lost because the RX ring was full.
static RX_RING_DROPS: AtomicU32 = AtomicU32::new(0);
/// Hardware overruns (`IF.RXOF`).
static RX_OVERRUNS: AtomicU32 = AtomicU32::new(0);
/// Framing or parity errors (bytes dropped).
static RX_LINE_ERRORS: AtomicU32 = AtomicU32::new(0);

/// Why the host UART refused a request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostUartError {
    /// Rate outside [`MIN_BAUD`]..=[`MAX_BAUD`].
    Baud,
    /// The transmitter did not drain in time; retry.
    Busy,
}

/// The opened USART0 host link. Owns the USART and both pins.
pub struct HostUart {
    _usart: Usart,
    _tx: Pin<PushPull>,
    _rx: Pin<Input>,
    baud: u32,
}

impl HostUart {
    /// Open USART0 as an 8N1 UART on `tx`/`rx` (no flow control), enable
    /// the receiver interrupt and unmask both USART0 lines.
    pub fn open(
        usart: Usart,
        tx: Pin<PushPull>,
        rx: Pin<Input>,
        baud: u32,
    ) -> Result<Self, HostUartError> {
        if usart.instance() != Instance::Usart0 {
            // Only USART0 has handlers here (it is the only USART on xG24).
            return Err(HostUartError::Baud);
        }
        if !(MIN_BAUD..=MAX_BAUD).contains(&baud) {
            return Err(HostUartError::Baud);
        }
        crate::clock::enable_bus_clock(usart.instance().clock());
        let base = usart.instance().base();
        let routeen = usart.instance().routeen();
        RX.clear();
        TX.clear();
        unsafe {
            write(base + EN, 1);
            write(base + CMD, CMD_RXDIS | CMD_CLEARTX | CMD_CLEARRX);
            write(base + CTRL, 0); // async, 16x oversampling
            write(base + FRAME, FRAME_8N1);
            write(base + CLKDIV, clkdiv(super::PCLK_HZ, baud));
            write(routeen + TXROUTE_FROM_ROUTEEN, route_value(&tx));
            write(routeen + RXROUTE_FROM_ROUTEEN, route_value(&rx));
            write(routeen, read(routeen) | ROUTEEN_TXPEN | ROUTEEN_RXPEN);
            write(base + IF + CLR, u32::MAX);
            write(base + IEN, IEN_RXDATAV);
            write(base + CMD, CMD_RXEN | CMD_TXEN);
        }
        #[cfg(all(target_arch = "arm", target_os = "none"))]
        unsafe {
            cortex_m::peripheral::NVIC::unpend(Interrupt::USART0_RX);
            cortex_m::peripheral::NVIC::unpend(Interrupt::USART0_TX);
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART0_RX);
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART0_TX);
        }
        Ok(Self {
            _usart: usart,
            _tx: tx,
            _rx: rx,
            baud,
        })
    }

    /// Current line rate.
    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// Received bytes into `buf`; 0 when none. Never blocks.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        RX.pop_slice(buf)
    }

    /// Queue as much of `data` as fits; 0 when the ring is full. Never
    /// blocks.
    pub fn try_write(&mut self, data: &[u8]) -> usize {
        let count = TX.push_slice(data);
        if count != 0 {
            unsafe { write(base() + IEN + SET, IEN_TXBL) };
        }
        count
    }

    /// Nothing queued and the shift register empty.
    pub fn tx_idle(&self) -> bool {
        TX.is_empty() && unsafe { read(base() + STATUS) } & STATUS_TXC != 0
    }

    /// Hardware overruns since boot (wrapping).
    pub fn rx_overruns(&self) -> u32 {
        RX_OVERRUNS.load(Ordering::Relaxed)
    }

    /// Framing/parity errors plus bytes dropped on a full ring (wrapping).
    pub fn rx_line_errors(&self) -> u32 {
        RX_LINE_ERRORS
            .load(Ordering::Relaxed)
            .wrapping_add(RX_RING_DROPS.load(Ordering::Relaxed))
    }

    /// Change the line rate once the transmitter has drained (bounded
    /// wait); bytes in flight towards us at the switch may be lost.
    pub fn set_baud(&mut self, baud: u32) -> Result<(), HostUartError> {
        if !(MIN_BAUD..=MAX_BAUD).contains(&baud) {
            return Err(HostUartError::Baud);
        }
        let deadline = crate::delay::Deadline::after_us(50_000);
        while !self.tx_idle() {
            if deadline.expired() {
                return Err(HostUartError::Busy);
            }
        }
        unsafe { write(base() + CLKDIV, clkdiv(super::PCLK_HZ, baud)) };
        self.baud = baud;
        Ok(())
    }
}

const fn base() -> u32 {
    crate::chip::base::USART0
}

/// RX handler body: drain the hardware buffer into the ring.
#[cfg_attr(
    not(all(feature = "rt", target_arch = "arm", target_os = "none")),
    allow(dead_code)
)]
fn service_rx() {
    let base = base();
    unsafe {
        if read(base + IF) & IF_RXOF != 0 {
            write(base + IF + CLR, IF_RXOF);
            RX_OVERRUNS.fetch_add(1, Ordering::Relaxed);
        }
        while read(base + STATUS) & STATUS_RXDATAV != 0 {
            let word = read(base + RXDATAX);
            if word & (RXDATAX_FERR | RXDATAX_PERR) != 0 {
                RX_LINE_ERRORS.fetch_add(1, Ordering::Relaxed);
                continue;
            }
            if !RX.push(word as u8) {
                RX_RING_DROPS.fetch_add(1, Ordering::Relaxed);
            }
        }
        // RXDATAV is a level flag mirrored in IF; clear the latched copy.
        write(base + IF + CLR, IEN_RXDATAV);
    }
}

/// TX handler body: refill the transmitter, stop when the ring is empty.
#[cfg_attr(
    not(all(feature = "rt", target_arch = "arm", target_os = "none")),
    allow(dead_code)
)]
fn service_tx() {
    let base = base();
    unsafe {
        while read(base + STATUS) & STATUS_TXBL != 0 {
            match TX.pop() {
                Some(byte) => write(base + TXDATA, u32::from(byte)),
                None => {
                    write(base + IEN + CLR, IEN_TXBL);
                    break;
                }
            }
        }
    }
}

#[cfg(all(feature = "rt", target_arch = "arm", target_os = "none"))]
#[unsafe(no_mangle)]
extern "C" fn USART0_RX() {
    service_rx();
}

#[cfg(all(feature = "rt", target_arch = "arm", target_os = "none"))]
#[unsafe(no_mangle)]
extern "C" fn USART0_TX() {
    service_tx();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_wraps_and_reports_full() {
        let ring: Ring<8> = Ring::new();
        assert_eq!(ring.push_slice(&[1, 2, 3, 4, 5, 6, 7, 8, 9]), 8);
        assert!(!ring.push(10));
        let mut out = [0u8; 5];
        assert_eq!(ring.pop_slice(&mut out), 5);
        assert_eq!(out, [1, 2, 3, 4, 5]);
        assert_eq!(ring.push_slice(&[9, 10, 11, 12, 13, 14]), 5);
        let mut rest = [0u8; 16];
        assert_eq!(ring.pop_slice(&mut rest), 8);
        assert_eq!(&rest[..8], &[6, 7, 8, 9, 10, 11, 12, 13]);
        assert!(ring.is_empty());
        assert_eq!(ring.pop(), None);
    }

    #[test]
    fn ring_indices_survive_wraparound() {
        let ring: Ring<4> = Ring::new();
        ring.head.store(usize::MAX - 1, Ordering::Relaxed);
        ring.tail.store(usize::MAX - 1, Ordering::Relaxed);
        assert_eq!(ring.push_slice(&[1, 2, 3, 4, 5]), 4);
        assert_eq!(ring.len(), 4);
        let mut out = [0u8; 4];
        assert_eq!(ring.pop_slice(&mut out), 4);
        assert_eq!(out, [1, 2, 3, 4]);
    }

    #[test]
    fn register_offsets_match_gsdk() {
        // offsetof(USART_TypeDef, RXDATAX/IF/IEN) in efr32mg2{1,2,4}_usart.h.
        assert_eq!((RXDATAX, IF, IEN), (0x20, 0x48, 0x4C));
        // GPIO USARTROUTE: RXROUTE - ROUTEEN = 0x10 on every die.
        assert_eq!(RXROUTE_FROM_ROUTEEN, 0x10);
    }
}
