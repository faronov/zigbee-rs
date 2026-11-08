//! Target-only hand-over: the BL702 [`Platform`] of the core sequence.

use bl702_hal::flash::{XipFlash, invalidate_xip_cache};
use bl702_hal::peripherals::Wdt;
use bl702_hal::watchdog::{self, Period, Watchdog};
use zrs_boot_core::{BootError, BootEvent, Bootable, Layout, Platform, image_end};

use crate::{
    BootFlash, Config, Console, Reporter, check_entry, image_error_name, reporter, write_hex,
    write_version,
};

const _: () = assert!(crate::WATCHDOG_MAX_MS == watchdog::MAX_PERIOD_MS);

/// Run one boot decision and start the application, or report the failure
/// and halt.
///
/// `flash` must be the physical XIP flash (ROM routines, physical offsets).
/// Clocks are expected in the ROM boot state, which the application assumes
/// as well; the loader changes no clock or power setting. With
/// [`Config::watchdog`] set, `wdt` is armed for the decision and disarmed
/// before the jump or the halt.
pub fn run(flash: XipFlash, config: &Config, mut wdt: Wdt, console: &mut impl Console) -> ! {
    let mut flash = BootFlash::for_config(flash, &config.boot);
    let mut shell = Bl702 {
        config,
        console,
        reporter: reporter(),
        watchdog: None,
    };
    if watchdog::take_reset_flag(&mut wdt) {
        shell.line(b"watchdog reset\r\n");
    }
    shell.watchdog = config
        .watchdog
        .and_then(|period| Period::from_ms(period.get()))
        .map(|period| Watchdog::arm(wdt, period));
    zrs_boot_core::run(&mut flash, &config.boot, &mut shell)
}

struct Bl702<'a, C> {
    config: &'a Config,
    console: &'a mut C,
    reporter: Reporter,
    watchdog: Option<Watchdog>,
}

impl<C> Bl702<'_, C> {
    fn disarm(&mut self) {
        if let Some(watchdog) = self.watchdog.take() {
            watchdog.disarm();
        }
    }
}

impl<C: Console> Bl702<'_, C> {
    fn line(&mut self, text: &[u8]) {
        self.reporter.line(&mut *self.console, text);
    }
}

impl<C: Console> Platform for Bl702<'_, C> {
    fn progress(&mut self, event: BootEvent) {
        self.reporter.event(&mut *self.console, event);
    }

    fn kick_watchdog(&mut self) {
        if let Some(watchdog) = self.watchdog.as_mut() {
            watchdog.kick();
        }
    }

    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {
        self.reporter.end_line(&mut *self.console);
        match *outcome {
            Ok(bootable) => {
                self.reporter.update(&mut *self.console, bootable.update);
                self.reporter.trial(&mut *self.console, bootable.trial);
            }
            Err(BootError::NoBootableImage { primary, update }) => {
                self.reporter.update(&mut *self.console, update);
                self.line(b"no bootable image: ");
                self.console.write(image_error_name(primary));
                self.console.write(b"\r\n");
            }
            Err(BootError::CopyCorrupt) => self.line(b"installed copy corrupt, request kept\r\n"),
            Err(BootError::Flash) => self.line(b"flash error\r\n"),
            Err(BootError::StatusFull) => self.line(b"status log full\r\n"),
            Err(BootError::Config) => self.line(b"bad configuration\r\n"),
        }
    }

    fn prepare(&mut self, layout: &Layout, bootable: &Bootable) -> Option<u32> {
        let entry = bootable.vector_table;
        match (
            check_entry(layout.primary, entry, image_end(layout, bootable)),
            self.config.xip_address(entry),
        ) {
            (Ok(()), Some(xip_entry)) => {
                self.line(b"app ");
                write_version(&mut *self.console, &bootable.image.header.version);
                self.console.write(b" @ ");
                write_hex(&mut *self.console, xip_entry);
                self.console.write(b"\r\n");
                // A rewritten primary may still be cached from the
                // validation reads through XIP (none today; the ROM
                // routines read physically) or a previous boot; drop every
                // line before fetching the new code.
                if invalidate_xip_cache().is_err() {
                    self.line(b"cache flush failed\r\n");
                    None
                } else {
                    drain_console();
                    Some(xip_entry)
                }
            }
            _ => {
                self.line(b"bad entry\r\n");
                None
            }
        }
    }

    unsafe fn start(&mut self, entry: u32) -> ! {
        // The application does not service the watchdog.
        self.disarm();
        // SAFETY: `prepare` checked that the validated image's entry is the
        // address the application is linked at and invalidated the cache;
        // no interrupt has been enabled by the loader.
        unsafe { jump(entry) }
    }

    fn fail(&mut self) -> ! {
        // Halt for good: a watchdog reset loop would rerun a failing
        // install and wear the primary slot.
        self.disarm();
        self.line(b"halted\r\n");
        halt()
    }
}

/// Give the UART FIFO time to drain before the application resets UART0
/// (32 bytes at 2 Mbaud is 160 us).
fn drain_console() {
    bl702_hal::timer::delay_us(1_000);
}

fn halt() -> ! {
    // SAFETY: masking machine interrupts has no memory-safety effect.
    unsafe { core::arch::asm!("csrci mstatus, 8") };
    loop {
        // SAFETY: `wfi` is a hint; with MIE clear it simply idles.
        unsafe { core::arch::asm!("wfi") };
    }
}

/// Branch to the application's riscv-rt `_start`.
///
/// `_start` itself disables and clears `mie`/`mip`, installs its pre-init
/// trap vector, loads `gp` and `sp` from its own link-time symbols and
/// initializes `.data`/`.bss`, so only the interrupt state is handed over:
/// global interrupts off and nothing pending from the loader.
unsafe fn jump(entry: u32) -> ! {
    // SAFETY: the caller guarantees `entry` is a validated application entry
    // in the XIP window; control never returns.
    unsafe {
        core::arch::asm!(
            "csrci mstatus, 8",
            "csrw mie, zero",
            "fence",
            "fence.i",
            "jr {entry}",
            entry = in(reg) entry,
            options(noreturn),
        )
    }
}
