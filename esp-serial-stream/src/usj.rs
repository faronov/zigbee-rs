//! Non-blocking byte stream over the USB-Serial-JTAG CDC-ACM port.
//!
//! * **Receive.** The `SERIAL_OUT_RECV_PKT` interrupt drains the 64-byte OUT
//!   FIFO into a [`USJ_RX_RING_BYTES`] ring. When the ring is full the
//!   interrupt stops draining and leaves bytes in the FIFO: the USB device
//!   then NAKs the host's next OUT packet, so back-pressure stalls the host
//!   write instead of losing bytes. [`UsjStream::read`] drains the FIFO
//!   again once there is room, because a held packet raises no new
//!   interrupt.
//! * **Transmit.** [`UsjStream::write`] puts bytes into the IN FIFO only
//!   while `SERIAL_IN_EP_DATA_FREE` reports room (never the esp-hal blocking
//!   `write`, which spins until a host reads). [`UsjStream::end_burst`]
//!   issues `WR_DONE` so the host sees a short packet, appending the
//!   configured filler first when the burst ended on a full packet
//!   ([`crate::usb_burst`]). With no host reading, the FIFO stays full and
//!   `write` returns 0; nothing here ever waits for the host.
//! * **Line hints.** DTR/RTS changes (host opened or closed the port) and
//!   USB bus resets are latched for the protocol above.
//!
//! The chip's DTR/RTS reset logic is left untouched
//! (`USB_SERIAL_JTAG_USB_UART_CHIP_RST_DIS` is not set): a host can always
//! hard-reset the chip.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};

use critical_section::Mutex;
use esp_hal::Blocking;
use esp_hal::handler;
use esp_hal::peripherals::USB_DEVICE;
use esp_hal::usb_serial_jtag::{UsbSerialJtag, UsbSerialJtagRx, UsbSerialJtagTx};

use crate::ring::{ByteRing, Consumer, Producer};
use crate::usb_burst::UsbBurst;

/// Receive ring size: at least `rx_window × 656` bytes for a 16-frame
/// window (10.3 KiB), rounded up to a power of two.
pub const USJ_RX_RING_BYTES: usize = 16 * 1024;

static RX_RING: ByteRing<USJ_RX_RING_BYTES> = ByteRing::new();
static RX_PRODUCER: Mutex<RefCell<Option<Producer<'static, USJ_RX_RING_BYTES>>>> =
    Mutex::new(RefCell::new(None));
/// Line events latched by the interrupt ([`LineEvents`] bits).
static LINE_EVENTS: AtomicU32 = AtomicU32::new(0);

/// Host-side line activity seen since the last [`UsjStream::take_line_events`].
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct LineEvents {
    /// DTR or RTS changed: a host opened or closed the port.
    pub control_lines_changed: bool,
    /// USB bus reset: the host (re)enumerated the device.
    pub bus_reset: bool,
}

const EVENT_LINES: u32 = 1 << 0;
const EVENT_BUS_RESET: u32 = 1 << 1;

/// Driver counters (saturating).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct UsjCounters {
    pub rx_bytes: u32,
    pub tx_bytes: u32,
    /// `write` calls that found the IN FIFO full (no host reading).
    pub tx_fifo_full: u32,
    /// Filler bytes added to end a burst with a short packet.
    pub fillers: u32,
}

/// The USB-Serial-JTAG byte stream.
pub struct UsjStream {
    _rx: UsbSerialJtagRx<'static, Blocking>,
    _tx: UsbSerialJtagTx<'static, Blocking>,
    rx: Consumer<'static, USJ_RX_RING_BYTES>,
    burst: UsbBurst,
    filler: Option<u8>,
    counters: UsjCounters,
}

impl UsjStream {
    /// Take over the USB-Serial-JTAG peripheral.
    ///
    /// `filler` is the byte appended before `WR_DONE` when a burst ends on a
    /// full 64-byte packet (`Some(0x00)` for COBS-framed protocols); `None`
    /// relies on the host accepting a zero-length packet.
    ///
    /// Bytes the ROM or bootloader left in the IN FIFO are flushed out once
    /// (non-blocking); the protocol above resynchronizes on them. Returns
    /// `None` if called twice (the receive ring is a singleton).
    pub fn new(usb: USB_DEVICE<'static>, filler: Option<u8>) -> Option<Self> {
        let (producer, consumer) = RX_RING.split()?;
        critical_section::with(|cs| *RX_PRODUCER.borrow_ref_mut(cs) = Some(producer));
        let mut usj = UsbSerialJtag::new(usb);
        usj.set_interrupt_handler(usj_interrupt);
        let (mut rx, tx) = usj.split();
        let regs = USB_DEVICE::regs();
        regs.int_clr().write(|w| {
            w.serial_out_recv_pkt().clear_bit_by_one();
            w.dtr_chg().clear_bit_by_one();
            w.rts_chg().clear_bit_by_one();
            w.usb_bus_reset().clear_bit_by_one()
        });
        regs.int_ena().modify(|_, w| {
            w.dtr_chg().set_bit();
            w.rts_chg().set_bit();
            w.usb_bus_reset().set_bit()
        });
        rx.listen_rx_packet_recv_interrupt();
        // Push out whatever an earlier stage left in the IN FIFO.
        regs.ep1_conf().modify(|_, w| w.wr_done().set_bit());
        Some(Self {
            _rx: rx,
            _tx: tx,
            rx: consumer,
            burst: UsbBurst::new(),
            filler,
            counters: UsjCounters::default(),
        })
    }

    /// Received bytes are waiting.
    pub fn read_ready(&mut self) -> bool {
        self.refill();
        !self.rx.is_empty()
    }

    /// Move received bytes into `buf`; never blocks, returns 0 when idle.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        self.refill();
        let count = self.rx.pop_slice(buf);
        self.counters.rx_bytes = self.counters.rx_bytes.saturating_add(count as u32);
        // Room was made: pull bytes the interrupt left in the FIFO.
        self.refill();
        count
    }

    /// Room for at least one byte in the IN FIFO.
    pub fn write_ready(&mut self) -> bool {
        in_fifo_free()
    }

    /// Put as much of `data` into the IN FIFO as fits now; never blocks.
    pub fn write(&mut self, data: &[u8]) -> usize {
        let regs = USB_DEVICE::regs();
        let mut written = 0;
        for byte in data {
            if !in_fifo_free() {
                break;
            }
            regs.ep1().write(|w| unsafe { w.rdwr_byte().bits(*byte) });
            written += 1;
        }
        if written < data.len() {
            self.counters.tx_fifo_full = self.counters.tx_fifo_full.saturating_add(1);
        }
        self.burst.written(written);
        self.counters.tx_bytes = self.counters.tx_bytes.saturating_add(written as u32);
        written
    }

    /// End a burst at a protocol frame boundary: issue `WR_DONE` so the
    /// host receives a short packet. Returns `false` (call again later)
    /// when a filler byte is due but the FIFO is full.
    pub fn end_burst(&mut self) -> bool {
        if !self.burst.is_open() {
            return true;
        }
        if self.burst.needs_filler()
            && let Some(filler) = self.filler
        {
            if self.write(&[filler]) == 0 {
                return false;
            }
            self.counters.fillers = self.counters.fillers.saturating_add(1);
        }
        USB_DEVICE::regs()
            .ep1_conf()
            .modify(|_, w| w.wr_done().set_bit());
        self.burst.flushed();
        true
    }

    /// Line events since the last call.
    pub fn take_line_events(&mut self) -> LineEvents {
        let bits = LINE_EVENTS.swap(0, Ordering::AcqRel);
        LineEvents {
            control_lines_changed: bits & EVENT_LINES != 0,
            bus_reset: bits & EVENT_BUS_RESET != 0,
        }
    }

    pub fn counters(&self) -> UsjCounters {
        self.counters
    }

    /// Discard buffered receive bytes (for example at session reset).
    pub fn discard_input(&mut self) {
        self.rx.clear();
        self.refill();
    }

    fn refill(&mut self) {
        critical_section::with(|cs| {
            if let Some(producer) = RX_PRODUCER.borrow_ref_mut(cs).as_mut() {
                drain_out_fifo(producer);
            }
        });
    }
}

fn in_fifo_free() -> bool {
    USB_DEVICE::regs()
        .ep1_conf()
        .read()
        .serial_in_ep_data_free()
        .bit_is_set()
}

/// Move OUT FIFO bytes into the ring while both have them/room for them.
#[inline(always)]
fn drain_out_fifo(producer: &mut Producer<'static, USJ_RX_RING_BYTES>) {
    let regs = USB_DEVICE::regs();
    while producer.free() > 0
        && regs
            .ep1_conf()
            .read()
            .serial_out_ep_data_avail()
            .bit_is_set()
    {
        let byte = regs.ep1().read().rdwr_byte().bits();
        let _ = producer.push(byte);
    }
}

#[handler]
fn usj_interrupt() {
    let regs = USB_DEVICE::regs();
    let status = regs.int_st().read();
    let mut events = 0;
    if status.dtr_chg().bit_is_set() || status.rts_chg().bit_is_set() {
        events |= EVENT_LINES;
    }
    if status.usb_bus_reset().bit_is_set() {
        events |= EVENT_BUS_RESET;
    }
    if events != 0 {
        LINE_EVENTS.fetch_or(events, Ordering::AcqRel);
    }
    regs.int_clr().write(|w| {
        w.serial_out_recv_pkt().clear_bit_by_one();
        w.dtr_chg().clear_bit_by_one();
        w.rts_chg().clear_bit_by_one();
        w.usb_bus_reset().clear_bit_by_one()
    });
    critical_section::with(|cs| {
        if let Some(producer) = RX_PRODUCER.borrow_ref_mut(cs).as_mut() {
            drain_out_fifo(producer);
        }
    });
}
