//! Non-blocking byte stream over a UART, with the receive path in DMA.
//!
//! The ZRHP transports without hardware flow control (the Thread Border
//! Router board's 460 800 baud S3↔H2 link) need a receiver that keeps
//! running while the CPU is stalled: flash erase and write run inside a
//! critical section with every interrupt masked, for tens to hundreds of
//! milliseconds, far longer than the 128-byte UART FIFO lasts. An interrupt
//! handler — IRAM-resident or not — cannot run then, so the receive path is
//! UHCI + GDMA into a [`DmaRxStreamBuf`]: the DMA engine fills internal SRAM
//! on its own and the main loop drains it later.
//!
//! * **Receive.** One long-lived UHCI receive transfer over
//!   [`UART_RX_DESCRIPTORS`] descriptors of [`UART_RX_CHUNK`] bytes. UHCI
//!   closes a descriptor at UART idle, so bytes become visible within one
//!   idle period. A small frame uses a whole descriptor, which is why there
//!   are many small ones. If every descriptor fills before the main loop
//!   drains them, the DMA stops and the UART FIFO overflows; the stream
//!   then counts `rx_overflow`, restarts the transfer and lets the ARQ
//!   above recover the lost frames.
//! * **Line errors.** Framing, parity and glitch errors and FIFO overflows
//!   are polled from the UART and counted; the protocol above treats each as
//!   a corrupted frame.
//! * **Transmit.** UHCI + GDMA from a [`UART_TX_BUFFER`]-byte buffer:
//!   [`UartDmaStream::write`] copies into the idle buffer and starts a
//!   transfer, and returns 0 while one is running. Nothing waits for the
//!   FIFO to drain.
//!
//! With RTS/CTS the same paths apply; hardware flow control only pauses a
//! frame, the ring never has to absorb a stall.

use esp_hal::Blocking;
use esp_hal::dma::DmaChannelFor;
use esp_hal::dma::{DmaDescriptor, DmaRxStreamBuf, DmaTxBuf};
use esp_hal::peripherals::UHCI0;
use esp_hal::uart::uhci::{self, AnyUhci, Uhci, UhciDmaRxTransfer, UhciDmaTxTransfer, UhciTx};
use esp_hal::uart::{self, CtsConfig, HwFlowControl, RtsConfig, Uart};

/// Receive buffer per descriptor.
pub const UART_RX_CHUNK: usize = 128;
/// Receive descriptors: 16 KiB in total, at least `rx_window × 656` bytes
/// for a 16-frame window, in chunks small enough that a burst of short
/// frames does not exhaust them during a flash stall.
pub const UART_RX_DESCRIPTORS: usize = 128;
/// Transmit buffer (one DMA transfer).
pub const UART_TX_BUFFER: usize = 1024;
/// RTS asserts (receiver full) at this RX FIFO level when flow control is on.
const RTS_THRESHOLD: u8 = 96;

/// Static memory for one [`UartDmaStream`]; place it in a `static`.
pub struct UartDmaMemory {
    rx_descriptors: [DmaDescriptor; UART_RX_DESCRIPTORS],
    rx: [u8; UART_RX_CHUNK * UART_RX_DESCRIPTORS],
    tx_descriptors: [DmaDescriptor; 1],
    tx: [u8; UART_TX_BUFFER],
}

impl Default for UartDmaMemory {
    fn default() -> Self {
        Self::new()
    }
}

impl UartDmaMemory {
    pub const fn new() -> Self {
        Self {
            rx_descriptors: [DmaDescriptor::EMPTY; UART_RX_DESCRIPTORS],
            rx: [0; UART_RX_CHUNK * UART_RX_DESCRIPTORS],
            tx_descriptors: [DmaDescriptor::EMPTY; 1],
            tx: [0; UART_TX_BUFFER],
        }
    }
}

/// UART line settings the stream applies.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LineConfig {
    pub baud: u32,
    /// RTS/CTS hardware flow control (only if the board wires both lines).
    pub flow_control: bool,
}

/// Driver counters (saturating).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct UartCounters {
    pub rx_bytes: u32,
    pub tx_bytes: u32,
    /// Framing, parity or glitch errors.
    pub rx_line_errors: u32,
    /// UART FIFO overflows and DMA restarts (bytes lost).
    pub rx_overflow: u32,
    pub tx_transfers: u32,
}

/// Which UART the stream drives (for polling its line-error flags while
/// the DMA owns the receiver).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartPort {
    Uart0,
    Uart1,
}

/// Errors of [`UartDmaStream::new`] and [`UartDmaStream::set_line`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UartStreamError {
    Config,
    Dma,
    /// A transmit transfer is still running; retry when idle.
    Busy,
}

enum Tx {
    Idle(UhciTx<'static, Blocking>, DmaTxBuf),
    Busy(UhciDmaTxTransfer<'static, Blocking, DmaTxBuf>),
    Lost,
}

/// The UART byte stream (UHCI GDMA in both directions).
pub struct UartDmaStream {
    rx: Option<UhciDmaRxTransfer<'static, Blocking, DmaRxStreamBuf>>,
    tx: Tx,
    port: UartPort,
    line: LineConfig,
    counters: UartCounters,
    pending_line_faults: u32,
}

impl UartDmaStream {
    /// Start the stream on a configured `uart` (pins already routed);
    /// `port` must name the same UART instance.
    pub fn new(
        uart: Uart<'static, Blocking>,
        port: UartPort,
        uhci0: UHCI0<'static>,
        dma: impl DmaChannelFor<AnyUhci<'static>>,
        memory: &'static mut UartDmaMemory,
        line: LineConfig,
    ) -> Result<Self, UartStreamError> {
        let mut uhci = Uhci::new(uart, uhci0, dma);
        uhci.set_uart_config(&uart_config(line))
            .map_err(|_| UartStreamError::Config)?;
        // A descriptor closes on UART idle (`idle_eof`) or after a chunk.
        uhci.apply_rx_config(&uhci::RxConfig::default().with_chunk_limit(UART_RX_CHUNK as u16))
            .map_err(|_| UartStreamError::Config)?;
        uhci.apply_tx_config(
            &uhci::TxConfig::default()
                .with_idle_eof(true)
                .with_len_eof(true),
        )
        .map_err(|_| UartStreamError::Config)?;
        let (rx, tx) = uhci.split();
        let rx_buf = DmaRxStreamBuf::new(&mut memory.rx_descriptors, &mut memory.rx)
            .map_err(|_| UartStreamError::Dma)?;
        let tx_buf = DmaTxBuf::new(&mut memory.tx_descriptors, &mut memory.tx)
            .map_err(|_| UartStreamError::Dma)?;
        let rx = rx.read(rx_buf).map_err(|_| UartStreamError::Dma)?;
        Ok(Self {
            rx: Some(rx),
            tx: Tx::Idle(tx, tx_buf),
            port,
            line,
            counters: UartCounters::default(),
            pending_line_faults: 0,
        })
    }

    /// Current line settings.
    pub fn line(&self) -> LineConfig {
        self.line
    }

    /// Received bytes are waiting.
    pub fn read_ready(&mut self) -> bool {
        self.service_rx();
        self.rx
            .as_ref()
            .is_some_and(|transfer| transfer.available_bytes() > 0)
    }

    /// Move received bytes into `buf`; never blocks, returns 0 when idle.
    pub fn read(&mut self, buf: &mut [u8]) -> usize {
        self.service_rx();
        let Some(transfer) = self.rx.as_mut() else {
            return 0;
        };
        let count = transfer.pop(buf);
        self.counters.rx_bytes = self.counters.rx_bytes.saturating_add(count as u32);
        count
    }

    /// The transmit buffer is free.
    pub fn write_ready(&mut self) -> bool {
        self.service_tx();
        matches!(self.tx, Tx::Idle(..))
    }

    /// Start transmitting up to [`UART_TX_BUFFER`] bytes of `data`; returns
    /// how many were taken (0 while the previous transfer runs).
    pub fn write(&mut self, data: &[u8]) -> usize {
        self.service_tx();
        if data.is_empty() {
            return 0;
        }
        let Tx::Idle(tx, mut buf) = core::mem::replace(&mut self.tx, Tx::Lost) else {
            return 0;
        };
        let count = data.len().min(UART_TX_BUFFER);
        buf.as_mut_slice()[..count].copy_from_slice(&data[..count]);
        buf.set_length(count);
        match tx.write(buf) {
            Ok(transfer) => {
                self.tx = Tx::Busy(transfer);
                self.counters.tx_bytes = self.counters.tx_bytes.saturating_add(count as u32);
                self.counters.tx_transfers = self.counters.tx_transfers.saturating_add(1);
                count
            }
            Err((_, tx, buf)) => {
                self.tx = Tx::Idle(tx, buf);
                0
            }
        }
    }

    /// Line faults (framing/parity/glitch errors and overflows) since the
    /// last call, for the protocol's corrupted-frame accounting.
    pub fn take_line_faults(&mut self) -> u32 {
        self.service_rx();
        core::mem::take(&mut self.pending_line_faults)
    }

    pub fn counters(&self) -> UartCounters {
        self.counters
    }

    /// Change baud rate and flow control. Only while no transmit transfer
    /// is running (`Busy` otherwise); bytes in flight at the old rate are
    /// the caller's concern (ZRHP switches after `apply_delay_ms`).
    pub fn set_line(&mut self, line: LineConfig) -> Result<(), UartStreamError> {
        self.service_tx();
        let Tx::Idle(tx, _) = &mut self.tx else {
            return Err(UartStreamError::Busy);
        };
        tx.apply_uart_config(&uart_config(line))
            .map_err(|_| UartStreamError::Config)?;
        self.line = line;
        Ok(())
    }

    fn service_tx(&mut self) {
        if let Tx::Busy(transfer) = &self.tx
            && transfer.is_done()
        {
            let Tx::Busy(transfer) = core::mem::replace(&mut self.tx, Tx::Lost) else {
                unreachable!()
            };
            // The DMA has handed every byte to the UART FIFO; stopping the
            // channel here never waits for the FIFO to drain.
            let (tx, buf) = transfer.cancel();
            self.tx = Tx::Idle(tx, buf);
        }
    }

    fn service_rx(&mut self) {
        let Some(transfer) = self.rx.as_mut() else {
            return;
        };
        let faults = take_line_faults(self.port);
        if faults.line_error {
            self.counters.rx_line_errors = self.counters.rx_line_errors.saturating_add(1);
            self.pending_line_faults = self.pending_line_faults.saturating_add(1);
        }
        let restart = faults.overflow;
        if restart {
            self.counters.rx_overflow = self.counters.rx_overflow.saturating_add(1);
            self.pending_line_faults = self.pending_line_faults.saturating_add(1);
        }
        // A finished receive transfer means the DMA ran out of descriptors
        // (or failed): restart it, the bytes it could not take are lost.
        if restart || transfer.is_done() {
            if !restart {
                self.counters.rx_overflow = self.counters.rx_overflow.saturating_add(1);
                self.pending_line_faults = self.pending_line_faults.saturating_add(1);
            }
            let transfer = self.rx.take().expect("checked above");
            let (rx, buf) = transfer.cancel();
            self.rx = rx.read(buf).ok();
        }
    }
}

struct LineFaults {
    line_error: bool,
    overflow: bool,
}

/// Read and clear the UART's raw error flags (framing, parity, glitch, RX
/// FIFO overflow) directly: the receive half is inside the DMA transfer.
fn take_line_faults(port: UartPort) -> LineFaults {
    macro_rules! poll {
        ($uart:ident) => {{
            let regs = esp_hal::peripherals::$uart::regs();
            let raw = regs.int_raw().read();
            let faults = LineFaults {
                line_error: raw.frm_err().bit_is_set()
                    || raw.parity_err().bit_is_set()
                    || raw.glitch_det().bit_is_set(),
                overflow: raw.rxfifo_ovf().bit_is_set(),
            };
            if faults.line_error || faults.overflow {
                regs.int_clr().write(|w| {
                    w.frm_err().clear_bit_by_one();
                    w.parity_err().clear_bit_by_one();
                    w.glitch_det().clear_bit_by_one();
                    w.rxfifo_ovf().clear_bit_by_one()
                });
            }
            faults
        }};
    }
    match port {
        UartPort::Uart0 => poll!(UART0),
        UartPort::Uart1 => poll!(UART1),
    }
}

fn uart_config(line: LineConfig) -> uart::Config {
    let hw_flow_ctrl = if line.flow_control {
        HwFlowControl {
            cts: CtsConfig::Enabled,
            rts: RtsConfig::Enabled(RTS_THRESHOLD),
        }
    } else {
        HwFlowControl::default()
    };
    uart::Config::default()
        .with_baudrate(line.baud)
        .with_hw_flow_ctrl(hw_flow_ctrl)
}
