//! `log` backend on a SEGGER RTT up-buffer (read over SWD by the probe).
//!
//! The ZBT-1's only other output is the CP2102N UART, which the host side
//! may own; RTT needs no pins. Writes never block: with no probe attached
//! the buffer fills and further output is dropped. Never called from an ISR
//! except the fault paths below.

use core::cell::RefCell;
use core::fmt::Write;
use core::ptr::{addr_of, addr_of_mut, read_volatile, write_volatile};

use cortex_m::interrupt::Mutex;

const SIZE: usize = 4 * 1024;

#[repr(C)]
struct Channel {
    name: *const u8,
    buf: *mut u8,
    size: u32,
    wr: u32,
    rd: u32,
    flags: u32,
}

#[repr(C)]
struct ControlBlock {
    id: [u8; 16],
    max_up: i32,
    max_down: i32,
    up: Channel,
    down: Channel,
}

const EMPTY: Channel = Channel {
    name: core::ptr::null(),
    buf: core::ptr::null_mut(),
    size: 0,
    wr: 0,
    rd: 0,
    flags: 0,
};

static mut BUF: [u8; SIZE] = [0; SIZE];
#[unsafe(no_mangle)]
static mut _SEGGER_RTT: ControlBlock = ControlBlock {
    id: [0; 16],
    max_up: 1,
    max_down: 1,
    up: EMPTY,
    down: EMPTY,
};

/// Exclusive writer token; present once [`init`] has published the block.
struct Rtt;

static RTT: Mutex<RefCell<Option<Rtt>>> = Mutex::new(RefCell::new(None));

impl Write for Rtt {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        // SAFETY: single producer (callers hold the RTT mutex); the probe
        // only advances `rd`.
        unsafe {
            let cb = addr_of_mut!(_SEGGER_RTT);
            let buf = addr_of_mut!(BUF).cast::<u8>();
            let mut wr = read_volatile(addr_of!((*cb).up.wr)) as usize;
            for &b in s.as_bytes() {
                let next = (wr + 1) % SIZE;
                if next == read_volatile(addr_of!((*cb).up.rd)) as usize {
                    break;
                }
                write_volatile(buf.add(wr), b);
                wr = next;
            }
            write_volatile(addr_of_mut!((*cb).up.wr), wr as u32);
        }
        Ok(())
    }
}

struct RttLogger;

static LOGGER: RttLogger = RttLogger;

impl log::Log for RttLogger {
    fn enabled(&self, _metadata: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        let ms = embassy_time::Instant::now().as_millis();
        cortex_m::interrupt::free(|cs| {
            if let Some(rtt) = RTT.borrow(cs).borrow_mut().as_mut() {
                let _ = writeln!(
                    rtt,
                    "{}.{:03} {} {}",
                    ms / 1000,
                    ms % 1000,
                    level_tag(record.level()),
                    record.args()
                );
            }
        });
    }

    fn flush(&self) {}
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

/// Publish the RTT control block and install the logger (once, first thing
/// in `main`).
pub fn init() {
    // SAFETY: runs once before any other RTT access; the ID is written last
    // so the probe never finds a partially initialised block.
    unsafe {
        let cb = addr_of_mut!(_SEGGER_RTT);
        (*cb).up.name = c"Terminal".as_ptr().cast();
        (*cb).up.buf = addr_of_mut!(BUF).cast();
        (*cb).up.size = SIZE as u32;
        (*cb).down.name = c"Terminal".as_ptr().cast();
        core::sync::atomic::compiler_fence(core::sync::atomic::Ordering::SeqCst);
        for (i, b) in b"SEGGER RTT\0\0\0\0\0\0".iter().enumerate() {
            write_volatile(addr_of_mut!((*cb).id[i]), *b);
        }
    }
    cortex_m::interrupt::free(|cs| *RTT.borrow(cs).borrow_mut() = Some(Rtt));
    // SAFETY: called once from `main` before anything logs.
    unsafe { log::set_logger_racy(&LOGGER) }.ok();
    log::set_max_level(log::LevelFilter::Info);
}

/// Best-effort message from a fault path. Skipped if the fault interrupted
/// an RTT write.
fn emergency(args: core::fmt::Arguments<'_>) {
    cortex_m::interrupt::free(|cs| {
        if let Ok(mut slot) = RTT.borrow(cs).try_borrow_mut()
            && let Some(rtt) = slot.as_mut()
        {
            let _ = rtt.write_fmt(args);
            let _ = rtt.write_str("\n");
        }
    });
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    emergency(format_args!("PANIC: {info}"));
    crate::halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    emergency(format_args!(
        "HARDFAULT pc={:08X} lr={:08X}",
        frame.pc(),
        frame.lr()
    ));
    crate::halt()
}
