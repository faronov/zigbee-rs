//! Start-up shared by the bins: trim, clocks, UART log, Embassy time.
//!
//! The only console is UART0 through the dongle's USB bridge (115200 8N1),
//! which is also the ROM-bootloader port; the log is plain text lines.

use core::cell::RefCell;
use core::fmt::Write;

use cc26x2_dongle::BoardResources;
use cc26x2_hal::chip::Identity;
use cc26x2_hal::uart::Uart0;
use cortex_m::interrupt::Mutex;

/// Resources a composition needs after clocks and time are running.
pub struct Started {
    pub identity: Identity,
    pub radio: cc26x2_dongle::BoardRadio,
    pub trng: cc26x2_hal::trng::TrngToken,
    pub crypto: cc26x2_hal::crypto::CryptoToken,
    pub flash: cc26x2_hal::flash::Flash,
}

static UART: Mutex<RefCell<Option<Uart0>>> = Mutex::new(RefCell::new(None));

struct UartLogger;

struct UartWriter<'a>(&'a mut Uart0);

impl Write for UartWriter<'_> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for chunk in s.split_inclusive('\n') {
            if let Some(line) = chunk.strip_suffix('\n') {
                self.0.write(line.as_bytes());
                self.0.write(b"\r\n");
            } else {
                self.0.write(chunk.as_bytes());
            }
        }
        Ok(())
    }
}

impl log::Log for UartLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &log::Record<'_>) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let ms = embassy_time::Instant::now().as_millis();
        cortex_m::interrupt::free(|cs| {
            if let Ok(mut slot) = UART.borrow(cs).try_borrow_mut()
                && let Some(uart) = slot.as_mut()
            {
                let _ = writeln!(
                    UartWriter(uart),
                    "[{:>7}.{:03}] {:<5} {}",
                    ms / 1000,
                    ms % 1000,
                    record.level(),
                    record.args()
                );
            }
            if LOG_RAM {
                let _ = writeln!(
                    RamWriter,
                    "[{:>7}.{:03}] {:<5} {}",
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

static LOGGER: UartLogger = UartLogger;

/// TEST-ONLY (`CC26X2_LOG_RAM=1` at build time): the log is also kept in a
/// RAM ring (`CC26X2_LOG_RAM`, `CC26X2_LOG_RAM_HEAD`) that a debug probe
/// reads, for boards whose UART is not connected to the host.
const LOG_RAM: bool = match option_env!("CC26X2_LOG_RAM") {
    Some(v) => matches!(v.as_bytes(), [b'1']),
    None => false,
};
const LOG_RAM_LEN: usize = if LOG_RAM { 8192 } else { 1 };
#[unsafe(no_mangle)]
static mut CC26X2_LOG_RAM: [u8; LOG_RAM_LEN] = [0; LOG_RAM_LEN];
#[unsafe(no_mangle)]
static mut CC26X2_LOG_RAM_HEAD: u32 = 0;

struct RamWriter;

impl Write for RamWriter {
    // `LOG_RAM_LEN` is 1 when the TEST-ONLY RAM log is off.
    #[allow(clippy::modulo_one)]
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        for &b in s.as_bytes() {
            // SAFETY: only written with interrupts masked (logger below).
            unsafe {
                let head = CC26X2_LOG_RAM_HEAD as usize;
                (*core::ptr::addr_of_mut!(CC26X2_LOG_RAM))[head % LOG_RAM_LEN] = b;
                CC26X2_LOG_RAM_HEAD = CC26X2_LOG_RAM_HEAD.wrapping_add(1);
            }
        }
        Ok(())
    }
}

/// Trim, RTC/time driver, XOSC_HF, LF clock and the UART log. Halts (with a
/// raw UART message when possible) if the silicon is not the compiled chip.
pub fn start(banner: &str) -> Started {
    let trim = cc26x2_hal::setup::trim_device();
    let BoardResources {
        host_uart,
        radio,
        rtc,
        trng,
        crypto,
        flash,
    } = BoardResources::take().unwrap_or_else(|| halt());
    let uart = host_uart
        .open(cc26x2_dongle::UART_BAUD)
        .unwrap_or_else(|_| halt());
    cortex_m::interrupt::free(|cs| *UART.borrow(cs).borrow_mut() = Some(uart));
    // SAFETY: called once before anything logs.
    unsafe { log::set_logger_racy(&LOGGER) }.ok();
    log::set_max_level(log::LevelFilter::Info);
    cc26x2_hal::time_driver::init(rtc);

    let identity = Identity::read();
    if let Err(error) = trim {
        log::error!("[CC26X2] trim/identity failed: {error:?} ({identity:?})");
        halt()
    }
    if let Err(error) = cc26x2_hal::setup::start_xosc_hf() {
        log::error!("[CC26X2] XOSC_HF start failed: {error:?}");
        halt()
    }
    // One image for every dongle: XOSC_LF from the CCFG, RCOSC_LF when the
    // crystal does not qualify in time (some ZBDongle-P V1.3 units).
    let lf = cc26x2_hal::setup::start_lf_clock(cc26x2_hal::setup::XOSC_LF_TIMEOUT_MS);
    log::info!(
        "[CC26X2] {} on {} ({}, PG{} pkg {} {} KiB, PA20 {}), LF {:?}",
        banner,
        cc26x2_dongle::BOARD_NAME,
        cc26x2_dongle::MCU_PART,
        identity.pg_rev,
        identity.package,
        identity.flash_size / 1024,
        identity.high_pa,
        lf
    );
    // The banner names the source (`RcOscFallback` when the crystal did
    // not qualify in time) and the RCOSC_LF compensation.
    if lf.is_err() {
        halt()
    }
    Started {
        identity,
        radio,
        trng,
        crypto,
        flash,
    }
}

/// Stop with interrupts masked; the last log line explains why.
pub fn halt() -> ! {
    cortex_m::interrupt::disable();
    cc26x2_hal::uart::flush_blocking();
    loop {
        cortex_m::asm::wfi();
    }
}

/// Best-effort message from a fault path straight to the UART FIFO.
fn emergency(args: core::fmt::Arguments<'_>) {
    struct Raw;
    impl Write for Raw {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            cc26x2_hal::uart::write_blocking(s.as_bytes());
            Ok(())
        }
    }
    let _ = Raw.write_fmt(args);
    let _ = Raw.write_str("\r\n");
}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo<'_>) -> ! {
    emergency(format_args!("PANIC: {info}"));
    halt()
}

#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
    emergency(format_args!(
        "HARDFAULT pc={:08X} lr={:08X}",
        frame.pc(),
        frame.lr()
    ));
    halt()
}
