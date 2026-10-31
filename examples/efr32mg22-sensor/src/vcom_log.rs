//! `log` backend on the WSTK virtual COM port (USART1, 115200 8N1).
//!
//! Blocking writes inside a critical section; never called from an ISR.
//! [`flush`] must run before EM2 so the last bytes leave the shifter.

use core::{cell::RefCell, fmt::Write};

use cortex_m::interrupt::Mutex;
use efr32mg22_devkit::Vcom;

static VCOM: Mutex<RefCell<Option<Vcom>>> = Mutex::new(RefCell::new(None));

struct VcomLogger;

static LOGGER: VcomLogger = VcomLogger;

impl log::Log for VcomLogger {
    fn enabled(&self, _metadata: &log::Metadata<'_>) -> bool {
        true
    }

    fn log(&self, record: &log::Record<'_>) {
        let ms = embassy_time::Instant::now().as_millis();
        cortex_m::interrupt::free(|cs| {
            if let Some(vcom) = VCOM.borrow(cs).borrow_mut().as_mut() {
                let uart = vcom.uart();
                let _ = write!(
                    uart,
                    "{}.{:03} {} {}\r\n",
                    ms / 1000,
                    ms % 1000,
                    level_tag(record.level()),
                    record.args()
                );
            }
        });
    }

    fn flush(&self) {
        flush();
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

/// Install the VCOM logger (once, before any `log!`).
pub fn init(vcom: Vcom) {
    cortex_m::interrupt::free(|cs| *VCOM.borrow(cs).borrow_mut() = Some(vcom));
    // SAFETY: called once from `main` before interrupts use `log`.
    unsafe { log::set_logger_racy(&LOGGER) }.ok();
    log::set_max_level(log::LevelFilter::Info);
}

/// Wait until all queued VCOM bytes have been shifted out.
pub fn flush() {
    cortex_m::interrupt::free(|cs| {
        if let Some(vcom) = VCOM.borrow(cs).borrow_mut().as_mut() {
            vcom.uart().flush();
        }
    });
}

/// Best-effort message from a fault path (panic/HardFault). Skipped if the
/// fault interrupted a VCOM write.
pub fn emergency(args: core::fmt::Arguments<'_>) {
    cortex_m::interrupt::free(|cs| {
        if let Ok(mut slot) = VCOM.borrow(cs).try_borrow_mut()
            && let Some(vcom) = slot.as_mut()
        {
            let uart = vcom.uart();
            let _ = uart.write_fmt(args);
            let _ = uart.write_str("\r\n");
            uart.flush();
        }
    });
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    emergency(format_args!("PANIC: {info}"));
    crate::platform::halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    emergency(format_args!(
        "HARDFAULT pc={:08X} lr={:08X}",
        frame.pc(),
        frame.lr()
    ));
    crate::platform::halt()
}
