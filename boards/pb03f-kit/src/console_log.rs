//! `log` backend on the kit's CH340 UART console.
//!
//! Diagnostic images only: records are formatted and written synchronously
//! (about 87 µs per character at 115200 Bd), so never log from an interrupt
//! handler or a timing-critical radio path.
//!
//! Interrupts stay enabled while a line is written: the UART is borrowed
//! out of its cell in a short critical section and put back afterwards.
//! The radio interrupt starts the 802.15.4 ACK in software, so masking it
//! for a whole log line would delay or lose ACKs. A record logged while the
//! UART is borrowed (an interrupt preempting a log call) is dropped.

use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;
use phy6222_hal::uart::Uart0;

static CONSOLE: Mutex<RefCell<Option<Uart0>>> = Mutex::new(RefCell::new(None));

struct ConsoleLogger;

static LOGGER: ConsoleLogger = ConsoleLogger;

impl log::Log for ConsoleLogger {
    fn enabled(&self, _metadata: &log::Metadata) -> bool {
        true
    }

    fn log(&self, record: &log::Record) {
        let ms = (crate::time::now_us() / 1000) as u32;
        with_console(|uart| {
            let _ = writeln!(
                uart,
                "{}.{:03} {} {}",
                ms / 1000,
                ms % 1000,
                record.level(),
                record.args()
            );
        });
    }

    fn flush(&self) {}
}

/// Install the UART console as the global `log` backend.
pub fn init(uart: Uart0, level: log::LevelFilter) {
    cortex_m::interrupt::free(|cs| {
        CONSOLE.borrow(cs).replace(Some(uart));
    });
    // SAFETY: thumbv6m has no pointer CAS, so `log` only offers the racy
    // setters. This runs once from the single-threaded composition root
    // before any task or interrupt can log.
    unsafe {
        let _ = log::set_logger_racy(&LOGGER);
        log::set_max_level_racy(level);
    }
}

/// Write raw text to the console outside the `log` facade.
pub fn write_fmt(args: core::fmt::Arguments<'_>) {
    with_console(|uart| {
        let _ = uart.write_fmt(args);
    });
}

/// Run `f` on the console UART with interrupts enabled; see the module
/// documentation for the borrow and drop rules.
fn with_console(f: impl FnOnce(&mut Uart0)) {
    let Some(mut uart) = cortex_m::interrupt::free(|cs| CONSOLE.borrow(cs).borrow_mut().take())
    else {
        return;
    };
    f(&mut uart);
    cortex_m::interrupt::free(|cs| {
        CONSOLE.borrow(cs).replace(Some(uart));
    });
}
