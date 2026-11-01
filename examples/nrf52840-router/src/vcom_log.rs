//! `log` backend on the DK virtual COM port (UARTE0 TXD P0.06, 115200 8N1).
//!
//! The RADIO interrupt sends Imm-Acks within the 192 µs turnaround, so the
//! logger must never hold interrupts off for a UART transfer: `log!` only
//! formats one line into a stack buffer and copies it into [`PIPE`] (a few
//! microseconds in a critical section), dropping the whole line when the
//! pipe is full. The [`drain`] task moves the pipe to the UART by EasyDMA.
//!
//! Fatal paths ([`emergency`]) run with interrupts disabled and write the
//! remaining pipe plus their message with blocking register access.

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, pipe::Pipe};
use nrf52840_dk::VcomTx;

const PIPE_BYTES: usize = 2048;
const LINE_BYTES: usize = 192;

static PIPE: Pipe<CriticalSectionRawMutex, PIPE_BYTES> = Pipe::new();
static DROPPED_LINES: AtomicU32 = AtomicU32::new(0);

struct VcomLogger;

static LOGGER: VcomLogger = VcomLogger;

/// Fixed-capacity line; output beyond the capacity is truncated.
struct Line {
    bytes: [u8; LINE_BYTES],
    len: usize,
}

impl Line {
    const fn new() -> Self {
        Self {
            bytes: [0; LINE_BYTES],
            len: 0,
        }
    }

    fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // Keep two bytes for the line terminator.
        let room = LINE_BYTES - 2 - self.len;
        let take = s.len().min(room);
        self.bytes[self.len..self.len + take].copy_from_slice(&s.as_bytes()[..take]);
        self.len += take;
        Ok(())
    }
}

impl log::Log for VcomLogger {
    fn enabled(&self, _metadata: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        let ms = embassy_time::Instant::now().as_millis();
        let mut line = Line::new();
        let _ = write!(
            line,
            "{}.{:03} {} {}",
            ms / 1000,
            ms % 1000,
            level_tag(record.level()),
            record.args()
        );
        line.bytes[line.len] = b'\r';
        line.bytes[line.len + 1] = b'\n';
        line.len += 2;
        enqueue(line.as_bytes());
    }

    fn flush(&self) {}
}

/// Queue a whole line, or drop it (counted) when the pipe lacks room.
fn enqueue(bytes: &[u8]) {
    let queued = cortex_m::interrupt::free(|_| {
        PIPE.free_capacity() >= bytes.len() && PIPE.try_write(bytes) == Ok(bytes.len())
    });
    if !queued {
        DROPPED_LINES.fetch_add(1, Ordering::Relaxed);
    }
}

const fn level_tag(level: log::Level) -> &'static str {
    match level {
        log::Level::Error => "E",
        log::Level::Warn => "W",
        log::Level::Info => "I",
        log::Level::Debug => "D",
        log::Level::Trace => "T",
    }
}

/// Lines dropped because the pipe was full.
pub fn dropped_lines() -> u32 {
    DROPPED_LINES.load(Ordering::Relaxed)
}

/// Install the logger (once, before any `log!`).
pub fn init() {
    // SAFETY: called once from `main` before any other context uses `log`.
    unsafe { log::set_logger_racy(&LOGGER) }.ok();
    log::set_max_level(log::LevelFilter::Info);
}

/// Move queued log bytes to VCOM.
#[embassy_executor::task]
pub async fn drain(mut tx: VcomTx<'static>) {
    let mut chunk = [0u8; 64];
    loop {
        let count = PIPE.read(&mut chunk).await;
        let _ = tx.write(&chunk[..count]).await;
    }
}

// UARTE0 registers for the blocking fault path.
const UARTE0: usize = 0x4000_2000;
const TASKS_STARTTX: usize = UARTE0 + 0x008;
const TASKS_STOPTX: usize = UARTE0 + 0x00C;
const EVENTS_ENDTX: usize = UARTE0 + 0x120;
const EVENTS_TXSTOPPED: usize = UARTE0 + 0x158;
const ENABLE: usize = UARTE0 + 0x500;
const TXD_PTR: usize = UARTE0 + 0x544;
const TXD_MAXCNT: usize = UARTE0 + 0x548;
/// About 25 ms at 64 MHz: far longer than one 64-byte transfer.
const SPIN_LIMIT: u32 = 400_000;

fn reg_write(address: usize, value: u32) {
    // SAFETY: a UARTE0 register; only used with interrupts disabled.
    unsafe { core::ptr::write_volatile(address as *mut u32, value) }
}

fn reg_read(address: usize) -> u32 {
    // SAFETY: a UARTE0 register.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

fn wait_event(address: usize) {
    let mut spins = 0;
    while reg_read(address) == 0 && spins < SPIN_LIMIT {
        spins += 1;
    }
}

/// Blocking EasyDMA transfer; `bytes` must be in RAM.
fn blocking_write(bytes: &[u8]) {
    for chunk in bytes.chunks(64) {
        reg_write(EVENTS_ENDTX, 0);
        reg_write(TXD_PTR, chunk.as_ptr() as u32);
        reg_write(TXD_MAXCNT, chunk.len() as u32);
        reg_write(TASKS_STARTTX, 1);
        wait_event(EVENTS_ENDTX);
    }
}

/// Best-effort message from a fault path: abort the DMA in flight, then
/// write the queued log and `args` synchronously. Interrupts stay disabled.
pub fn emergency(args: core::fmt::Arguments<'_>) {
    cortex_m::interrupt::disable();
    // UARTE0 is enabled (8) once `drain` owns it; before that, no VCOM.
    if reg_read(ENABLE) != 8 {
        return;
    }
    reg_write(EVENTS_TXSTOPPED, 0);
    reg_write(TASKS_STOPTX, 1);
    wait_event(EVENTS_TXSTOPPED);
    let mut chunk = [0u8; 64];
    while let Ok(count) = PIPE.try_read(&mut chunk) {
        blocking_write(&chunk[..count]);
    }
    let mut line = Line::new();
    let _ = line.write_str("\r\n");
    let _ = line.write_fmt(args);
    let _ = line.write_str("\r\n");
    blocking_write(line.as_bytes());
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    emergency(format_args!("PANIC: {info}"));
    crate::roles::halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    emergency(format_args!(
        "HARDFAULT pc={:08X} lr={:08X}",
        frame.pc(),
        frame.lr()
    ));
    crate::roles::halt()
}
