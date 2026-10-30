//! `log` backend on the kit's CH340 UART console.
//!
//! Diagnostic images only: records are formatted synchronously with
//! interrupts masked, so never log from an interrupt handler or a
//! timing-critical radio path.

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
        cortex_m::interrupt::free(|cs| {
            if let Some(uart) = CONSOLE.borrow(cs).borrow_mut().as_mut() {
                let _ = writeln!(
                    uart,
                    "{}.{:03} {} {}",
                    ms / 1000,
                    ms % 1000,
                    record.level(),
                    record.args()
                );
            }
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
    cortex_m::interrupt::free(|cs| {
        if let Some(uart) = CONSOLE.borrow(cs).borrow_mut().as_mut() {
            let _ = uart.write_fmt(args);
        }
    });
}
