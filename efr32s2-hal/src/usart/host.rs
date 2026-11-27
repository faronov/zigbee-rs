//! USART0 host link (feature `usart0-host`): LDMA receiver,
//! interrupt-driven transmitter.
//!
//! Network co-processor images talk to their host over USART0 without
//! RTS/CTS.
//!
//! Receive: LDMA channel [`HOST_RX_CHANNEL`] copies every byte from
//! `RXDATA` into an 8 KiB ring, with no interrupt at all, so nothing that
//! holds the CPU or its interrupts off (the radio's Imm-Ack interrupt, an
//! MSC page erase that stalls instruction fetches for ~12 ms) can overrun
//! the two-byte USART buffer. The ring is four 2 KiB quarters, each filled by
//! one transfer descriptor behind a SYNC "gate" descriptor (`S0 T0 S1 T1 S2
//! T2 S3 T3`, back to `S0`): `S_k` closes the gate of quarter `k - 1` and
//! waits until the gate of quarter `k` is open; the reader reopens quarter
//! `k - 1` when it has read all of quarter `k`. Three gates start open, so
//! the DMA leads the reader by at most [`RX_LEAD`] bytes (at least eight
//! 656-byte ZRHP frames, spec 4.4) and never overwrites unread bytes. A
//! reader that falls that far behind stalls the DMA instead; the USART then
//! overruns and `IF.RXOF` counts it, as framing and parity errors are
//! counted (`IF.FERR`/`PERR`, polled by [`HostUart::read`]). The ZRHP link
//! layer retransmits a damaged frame.
//!
//! Transmit: the `USART0_TX` handler feeds the transmitter from a 2 KiB
//! single-producer/single-consumer ring; a late handler only delays output.
//!
//! Status: hardware-proven on the BRD4187A (EFR32MG24, WSTK VCOM at
//! 115 200) against a ZRHP host: a 35-minute soak with ZCL bursts and a
//! leave/rejoin had no overrun and no framing error. xG21/xG22 only build
//! and host-test it (no image uses it there). The register sequence follows
//! GSDK 4.5.0 `em_usart.c` / `em_ldma.c`.

use core::cell::UnsafeCell;
use core::sync::atomic::{AtomicU32, AtomicUsize, Ordering};

use super::{CLKDIV, CMD, CTRL, EN, FRAME, FRAME_8N1, Instance, STATUS, TXDATA};
use super::{ROUTEEN_TXPEN, TXROUTE_FROM_ROUTEEN, Usart, clkdiv, read, route_value, write};
use crate::gpio::{Input, Pin, PushPull};
pub use crate::ldma::HOST_RX_CHANNEL;
use crate::ldma::{self, Ldma};
#[cfg(all(target_arch = "arm", target_os = "none"))]
use crate::vectors::Interrupt;

/// Receive ring size (power of two, four LDMA transfers).
pub const RX_RING: usize = 8192;
/// Bytes per receive quarter (one LDMA transfer descriptor).
const QUARTER: usize = RX_RING / 4;
/// Most bytes the DMA writes ahead of the reader (three open quarters).
pub const RX_LEAD: usize = 3 * QUARTER;
/// Transmit ring size (power of two).
pub const TX_RING: usize = 2048;
const _: () = assert!(RX_RING.is_power_of_two() && TX_RING.is_power_of_two());
const _: () = assert!(QUARTER <= ldma::MAX_XFER);

const RXDATA: u32 = 0x24;
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
const IEN_TXBL: u32 = 1 << 1;
const IF_RXOF: u32 = 1 << 4;
const IF_PERR: u32 = 1 << 8;
const IF_FERR: u32 = 1 << 9;
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

static TX: Ring<TX_RING> = Ring::new();
/// Hardware overruns (`IF.RXOF`): the DMA was stalled at a closed gate.
static RX_OVERRUNS: AtomicU32 = AtomicU32::new(0);
/// Framing or parity errors (`IF.FERR`/`PERR` seen by a read).
static RX_LINE_ERRORS: AtomicU32 = AtomicU32::new(0);

/// The receive ring the LDMA writes.
#[repr(C, align(4))]
struct RxBuffer(UnsafeCell<[u8; RX_RING]>);
/// The gated descriptor chain (`S0 T0 .. S3 T3`).
#[repr(C, align(16))]
struct RxChain(UnsafeCell<[ldma::Descriptor; 8]>);
// SAFETY: only the [`HostUart`] owner touches them (the chain once, at
// open; the ring by volatile reads of bytes the DMA has published).
unsafe impl Sync for RxBuffer {}
unsafe impl Sync for RxChain {}
static RX_BUFFER: RxBuffer = RxBuffer(UnsafeCell::new([0; RX_RING]));
static RX_CHAIN: RxChain = RxChain(UnsafeCell::new([[0; 4]; 8]));

/// The gated ring for a chain at `chain` and a ring at `ring`: `S_k`
/// closes the gate of quarter `k - 1` (its `SYNCTRIG` bit) and waits for
/// the gate of quarter `k`; `T_k` copies one quarter from `rxdata`.
fn rx_chain(chain: u32, ring: u32, rxdata: u32) -> [ldma::Descriptor; 8] {
    let at = |i: usize| chain + 16 * (i % 8) as u32;
    core::array::from_fn(|i| {
        let k = i / 2;
        if i % 2 == 0 {
            let gate = 1u8 << k;
            ldma::sync(0, 1 << ((k + 3) % 4), gate, gate, at(i + 1))
        } else {
            ldma::p2m_bytes(rxdata, ring + (k * QUARTER) as u32, QUARTER, at(i + 1))
        }
    })
}

/// Ring offset the DMA writes next, from the channel's `DST` and `LINK`:
/// `DST` while it points into the ring; on a gate (a SYNC descriptor
/// carries no destination) the start of the quarter `LINK` names.
fn rx_head(dst: u32, link: u32, chain: u32, ring: u32) -> Option<usize> {
    if (ring..=ring + RX_RING as u32).contains(&dst) {
        return Some((dst - ring) as usize % RX_RING);
    }
    let index = (link & !3).wrapping_sub(chain) / 16;
    ((link & !3) >= chain && index < 8 && index % 2 == 1).then(|| (index as usize / 2) * QUARTER)
}

/// Reader side of the gated ring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RxReader {
    /// Quarter being read.
    quarter: usize,
    /// Bytes read from it.
    offset: usize,
}

impl RxReader {
    const fn new() -> Self {
        Self {
            quarter: 0,
            offset: 0,
        }
    }

    /// Ring offset of the next unread byte.
    fn position(&self) -> usize {
        self.quarter * QUARTER + self.offset
    }

    /// Unread bytes with the DMA at ring offset `head` (the lead is at most
    /// [`RX_LEAD`] < [`RX_RING`], so the difference is unambiguous).
    fn available(&self, head: usize) -> usize {
        (head + RX_RING - self.position()) % RX_RING
    }

    /// Account `n` bytes read; the `SYNCTRIG` bits to set: finishing quarter
    /// `q` reopens quarter `q - 1`, whose gate the DMA closed when it
    /// entered `q`.
    fn advance(&mut self, n: usize) -> u32 {
        let mut reopen = 0;
        self.offset += n;
        while self.offset >= QUARTER {
            self.offset -= QUARTER;
            reopen |= 1 << ((self.quarter + 3) % 4);
            self.quarter = (self.quarter + 1) % 4;
        }
        reopen
    }
}

/// Why the host UART refused a request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HostUartError {
    /// Rate outside [`MIN_BAUD`]..=[`MAX_BAUD`].
    Baud,
    /// The transmitter did not drain in time; retry.
    Busy,
}

/// The opened USART0 host link. Owns the USART, both pins and the LDMA.
pub struct HostUart {
    _usart: Usart,
    _ldma: Ldma,
    _tx: Pin<PushPull>,
    _rx: Pin<Input>,
    baud: u32,
    reader: RxReader,
}

/// `SYNCTRIG` bits of the four receive gates.
const GATES: u32 = 0x0F;
/// Gates open at start: quarters 0..=2.
const GATES_OPEN: u32 = 0x07;

impl HostUart {
    /// Open USART0 as an 8N1 UART on `tx`/`rx` (no flow control): the
    /// receiver through the LDMA (see the module documentation), the
    /// transmitter interrupt unmasked.
    pub fn open(
        usart: Usart,
        ldma: Ldma,
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
            // No receive interrupt: RXDATAV requests the LDMA.
            write(base + IEN, 0);
        }
        #[cfg(all(target_arch = "arm", target_os = "none"))]
        start_rx_dma(base + RXDATA)?;
        unsafe { write(base + CMD, CMD_RXEN | CMD_TXEN) };
        #[cfg(all(target_arch = "arm", target_os = "none"))]
        unsafe {
            cortex_m::peripheral::NVIC::mask(Interrupt::USART0_RX);
            cortex_m::peripheral::NVIC::unpend(Interrupt::USART0_TX);
            cortex_m::peripheral::NVIC::unmask(Interrupt::USART0_TX);
        }
        Ok(Self {
            _usart: usart,
            _ldma: ldma,
            _tx: tx,
            _rx: rx,
            baud,
            reader: RxReader::new(),
        })
    }

    /// Current line rate.
    pub fn baud(&self) -> u32 {
        self.baud
    }

    /// Received bytes into `buf`; 0 when none. Never blocks. Also counts
    /// the USART's overrun and framing/parity flags.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        poll_line_flags();
        let Some(head) = dma_head() else {
            return 0;
        };
        let count = self.reader.available(head).min(buf.len());
        if count == 0 {
            return 0;
        }
        // The bytes below `head` were written before the DMA advanced it.
        core::sync::atomic::fence(Ordering::Acquire);
        let start = self.reader.position();
        let ring = RX_BUFFER.0.get().cast::<u8>();
        for (i, slot) in buf[..count].iter_mut().enumerate() {
            // SAFETY: inside the ring; the DMA does not write unread bytes.
            *slot = unsafe { core::ptr::read_volatile(ring.add((start + i) % RX_RING)) };
        }
        let reopen = self.reader.advance(count);
        if reopen != 0 {
            #[cfg(all(target_arch = "arm", target_os = "none"))]
            unsafe {
                ldma::write(ldma::SYNCSWSET, reopen)
            };
        }
        count
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

    /// Reads that found a framing or parity error flagged (wrapping).
    pub fn rx_line_errors(&self) -> u32 {
        RX_LINE_ERRORS.load(Ordering::Relaxed)
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

/// Count and clear the USART's overrun and framing/parity flags.
fn poll_line_flags() {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        let flags = read(base() + IF) & (IF_RXOF | IF_FERR | IF_PERR);
        if flags != 0 {
            write(base() + IF + CLR, flags);
            if flags & IF_RXOF != 0 {
                RX_OVERRUNS.fetch_add(1, Ordering::Relaxed);
            }
            if flags & (IF_FERR | IF_PERR) != 0 {
                RX_LINE_ERRORS.fetch_add(1, Ordering::Relaxed);
            }
        }
    }
}

/// Ring offset the LDMA writes next (`None`: channel state not readable).
fn dma_head() -> Option<usize> {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    {
        let (chain, ring) = (RX_CHAIN.0.get() as u32, RX_BUFFER.0.get() as u32);
        let ch = ldma::ch(HOST_RX_CHANNEL);
        for _ in 0..4 {
            let (dst, link) = unsafe {
                (
                    ldma::read(ch + ldma::CH_DST),
                    ldma::read(ch + ldma::CH_LINK),
                )
            };
            if let Some(head) = rx_head(dst, link, chain, ring) {
                return Some(head);
            }
        }
        None
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    None
}

/// Build the gated chain and start the receive channel (USART receiver
/// still off). Follows GSDK `LDMA_Init` / `LDMA_StartTransfer`.
#[cfg(all(target_arch = "arm", target_os = "none"))]
fn start_rx_dma(rxdata: u32) -> Result<(), HostUartError> {
    use crate::chip::BusClock;
    let bit = 1 << HOST_RX_CHANNEL;
    let ch = ldma::ch(HOST_RX_CHANNEL);
    let (chain, ring) = (RX_CHAIN.0.get() as u32, RX_BUFFER.0.get() as u32);
    crate::clock::enable_bus_clock(BusClock::LDMA);
    crate::clock::enable_bus_clock(BusClock::LDMAXBAR);
    unsafe {
        core::ptr::write_volatile(RX_CHAIN.0.get(), rx_chain(chain, ring, rxdata));
        ldma::write(ldma::EN, 1);
        ldma::write(ldma::CHDIS, bit);
        let deadline = crate::delay::Deadline::after_us(1_000);
        while ldma::read(ldma::CHBUSY) & bit != 0 {
            if deadline.expired() {
                return Err(HostUartError::Busy);
            }
        }
        ldma::write(ldma::IEN + ldma::CLR, bit);
        ldma::write(ldma::IF + ldma::CLR, bit);
        ldma::write(ldma::REQDIS + ldma::CLR, bit);
        ldma::write(ldma::DBGHALT + ldma::CLR, bit);
        // Software gates only (no PRS on the four bits).
        ldma::write(ldma::SYNCHWEN + ldma::CLR, GATES | GATES << 16);
        ldma::write(ldma::SYNCSWCLR, GATES);
        ldma::write(ldma::SYNCSWSET, GATES_OPEN);
        core::ptr::write_volatile(
            (ldma::LDMAXBAR + ldma::xbar_reqsel(HOST_RX_CHANNEL)) as *mut u32,
            ldma::REQSEL_USART0_RXDATAV,
        );
        ldma::write(ch + ldma::CH_LOOP, 0);
        ldma::write(ch + ldma::CH_CFG, 0);
        ldma::write(ch + ldma::CH_LINK, chain);
        ldma::write(ldma::CHDONE + ldma::CLR, bit);
        // The descriptors are in RAM before the channel loads them.
        cortex_m::asm::dsb();
        ldma::write(ldma::LINKLOAD, bit);
    }
    Ok(())
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
        // offsetof(USART_TypeDef, RXDATA/IF/IEN) in efr32mg2{1,2,4}_usart.h.
        assert_eq!((RXDATA, IF, IEN), (0x24, 0x48, 0x4C));
        // USART_IF_RXOF/PERR/FERR.
        assert_eq!((IF_RXOF, IF_PERR, IF_FERR), (0x10, 0x100, 0x200));
        // GPIO USARTROUTE: RXROUTE - ROUTEEN = 0x10 on every die.
        assert_eq!(RXROUTE_FROM_ROUTEEN, 0x10);
    }

    const CHAIN: u32 = 0x2000_0100;
    const RING: u32 = 0x2000_1000;
    const RXDATA_ADDR: u32 = 0x4005_C024;

    #[test]
    fn the_chain_gates_each_quarter() {
        let chain = rx_chain(CHAIN, RING, RXDATA_ADDR);
        for k in 0..4u32 {
            let (s, t) = (chain[2 * k as usize], chain[2 * k as usize + 1]);
            // S_k closes the gate of quarter k - 1, waits for gate k, then T_k.
            let gate = 1u8 << k;
            let closes = 1u8 << ((k + 3) % 4);
            assert_eq!(s, ldma::sync(0, closes, gate, gate, CHAIN + 32 * k + 16));
            let next = CHAIN + (32 * (k + 1)) % 128;
            assert_eq!(
                t,
                ldma::p2m_bytes(RXDATA_ADDR, RING + k * QUARTER as u32, QUARTER, next)
            );
        }
        // T3 links back to S0.
        assert_eq!(chain[7][3], CHAIN | 2);
    }

    #[test]
    fn the_head_comes_from_dst_or_the_gate_link() {
        assert_eq!(rx_head(RING, 0, CHAIN, RING), Some(0));
        assert_eq!(rx_head(RING + 100, 0, CHAIN, RING), Some(100));
        assert_eq!(rx_head(RING + RX_RING as u32, 0, CHAIN, RING), Some(0));
        // On a gate: DST carries no ring address; LINK names T_k.
        for k in 0..4u32 {
            let link = (CHAIN + 32 * k + 16) | 2;
            assert_eq!(
                rx_head(0x0101, link, CHAIN, RING),
                Some(k as usize * QUARTER)
            );
        }
        // LINK at a gate (inside a transfer) or outside the chain: unknown.
        assert_eq!(rx_head(0x0101, CHAIN | 2, CHAIN, RING), None);
        assert_eq!(rx_head(0x0101, 0x2000_0000, CHAIN, RING), None);
    }

    /// The gated DMA as the reference manual describes it: a SYNC
    /// descriptor sets/clears its bits when loaded, then waits for the match.
    struct GatedDma {
        gates: u32,
        /// `Ok(k)`: at gate `S_k`; `Err((k, n))`: `n` bytes into `T_k`.
        at: Result<usize, (usize, usize)>,
        ring: [u8; RX_RING],
        written: usize,
        stalls: usize,
    }

    impl GatedDma {
        fn new() -> Self {
            let mut dma = Self {
                gates: GATES_OPEN,
                at: Ok(0),
                ring: [0; RX_RING],
                written: 0,
                stalls: 0,
            };
            dma.enter_gate(0);
            dma
        }

        fn enter_gate(&mut self, k: usize) {
            self.gates &= !(1 << ((k + 3) % 4));
            self.at = Ok(k);
        }

        /// One byte from the USART; `false`: stalled at a closed gate.
        fn receive(&mut self, byte: u8) -> bool {
            if let Ok(k) = self.at {
                if self.gates & (1 << k) == 0 {
                    self.stalls += 1;
                    return false;
                }
                self.at = Err((k, 0));
            }
            let Err((k, n)) = self.at else { unreachable!() };
            self.ring[k * QUARTER + n] = byte;
            self.written += 1;
            if n + 1 == QUARTER {
                self.enter_gate((k + 1) % 4);
            } else {
                self.at = Err((k, n + 1));
            }
            true
        }

        /// The channel's DST: the next byte's ring offset.
        fn head(&self) -> usize {
            match self.at {
                Ok(k) => k * QUARTER,
                Err((k, n)) => k * QUARTER + n,
            }
        }
    }

    #[test]
    fn the_reader_never_loses_or_repeats_a_byte() {
        let mut dma = GatedDma::new();
        let mut reader = RxReader::new();
        let (mut sent, mut read_total) = (0usize, 0usize);
        let mut seed = 0x1234_5678u32;
        let mut rand = |m: u32| {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (seed >> 8) % m
        };
        let mut out = [0u8; 900];
        for round in 0..20_000 {
            // Bursts up to 7 KiB arrive while the reader is busy elsewhere.
            let burst = rand(if round % 97 == 0 { 7_000 } else { 400 });
            for _ in 0..burst {
                if dma.receive(sent as u8) {
                    sent += 1;
                }
            }
            assert!(dma.written - read_total <= RX_LEAD, "lead");
            let want = 1 + rand(out.len() as u32) as usize;
            let n = reader.available(dma.head()).min(want);
            let start = reader.position();
            for (i, slot) in out[..n].iter_mut().enumerate() {
                *slot = dma.ring[(start + i) % RX_RING];
            }
            for (i, &byte) in out[..n].iter().enumerate() {
                assert_eq!(byte, (read_total + i) as u8, "byte {}", read_total + i);
            }
            read_total += n;
            dma.gates |= reader.advance(n);
        }
        assert!(read_total > 1_000_000 && dma.stalls > 0);
        // Drain: everything that was not refused arrives.
        loop {
            let n = reader.available(dma.head());
            if n == 0 {
                break;
            }
            read_total += n;
            dma.gates |= reader.advance(n);
        }
        assert_eq!(read_total, sent);
    }

    #[test]
    fn reading_quarters_reopens_the_one_behind() {
        let mut reader = RxReader::new();
        assert_eq!(reader.advance(QUARTER - 1), 0);
        // Finishing quarter 0 reopens quarter 3 (closed by S0 at start).
        assert_eq!(reader.advance(1), 1 << 3);
        // Two quarters at once: 1 and 2 done, reopen 0 and 1.
        assert_eq!(reader.advance(2 * QUARTER), 0b0011);
        assert_eq!(reader.position(), 3 * QUARTER);
        assert_eq!(reader.advance(QUARTER), 1 << 2);
        assert_eq!(reader.position(), 0);
    }
}
