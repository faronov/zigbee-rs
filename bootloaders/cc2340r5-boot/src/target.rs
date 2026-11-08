//! Target-only hand-over: the CC2340R5 [`Platform`] of the core sequence.

use cc2340r5_hal::flash::InternalFlash;
use cc2340r5_hal::power;
use cc2340r5_hal::watchdog::{self, Period, Watchdog, Wdt};
use zrs_boot_core::{BootEvent, Bootable, Layout, Platform, image_end};

use crate::{BootFlash, Config, Indicator, check_vector_table, indicate};

const _: () = assert!(crate::WATCHDOG_MAX_MS == watchdog::MAX_PERIOD_MS);

/// Fault blinks before entering SHUTDOWN.
const FAULT_BLINKS: u32 = 10;
/// 125 ms at the ROM's 48 MHz HFOSC clock.
const HALF_PERIOD_CYCLES: u32 = cc2340r5_hal::MCU_CLOCK_HZ / 8;

/// Run one boot decision and start the application, or indicate a fault and
/// enter SHUTDOWN.
///
/// Call after the product has configured its indicator pins (and, for
/// `prepare_shutdown`, its wake button): the I/O latch left by a SHUTDOWN
/// wake-up is released here. `prepare_shutdown` runs just before SHUTDOWN,
/// typically `Button::enable_shutdown_wake`. With [`Config::watchdog`] set,
/// `wdt` is started here and stays running in the application.
pub fn run(
    flash: InternalFlash,
    config: &Config,
    wdt: Wdt,
    indicator: &mut impl Indicator,
    prepare_shutdown: impl FnOnce(),
) -> ! {
    power::release_io_latch();

    let mut flash = BootFlash::for_config(flash, &config.boot);
    let mut shell = Cc2340 {
        ram: config.ram,
        indicator,
        prepare_shutdown: Some(prepare_shutdown),
        watchdog: config
            .watchdog
            .and_then(|period| Period::from_ms(period.get()))
            .map(|period| Watchdog::start(wdt, period)),
    };
    zrs_boot_core::run(&mut flash, &config.boot, &mut shell)
}

struct Cc2340<'a, I, S> {
    ram: (u32, u32),
    indicator: &'a mut I,
    prepare_shutdown: Option<S>,
    watchdog: Option<Watchdog>,
}

impl<I: Indicator, S: FnOnce()> Platform for Cc2340<'_, I, S> {
    fn progress(&mut self, event: BootEvent) {
        indicate(self.indicator, event);
    }

    fn kick_watchdog(&mut self) {
        if let Some(watchdog) = self.watchdog.as_mut() {
            watchdog.kick();
        }
    }

    fn prepare(&mut self, layout: &Layout, bootable: &Bootable) -> Option<u32> {
        let vector_table = bootable.vector_table;
        // SAFETY: the vector table lies inside a validated image in
        // memory-mapped main flash.
        let (initial_sp, reset) = unsafe {
            (
                core::ptr::read_volatile(vector_table as *const u32),
                core::ptr::read_volatile((vector_table + 4) as *const u32),
            )
        };
        check_vector_table(
            initial_sp,
            reset,
            vector_table,
            image_end(layout, bootable),
            self.ram,
        )
        .ok()
        .map(|()| vector_table)
    }

    unsafe fn start(&mut self, vector_table: u32) -> ! {
        self.indicator.off();
        // A started watchdog cannot be stopped: hand the application a full
        // period.
        self.kick_watchdog();
        // SAFETY: `prepare` returned this vector table for an image
        // validated in this boot and checked it; no interrupt has been
        // enabled by the loader.
        unsafe { jump(vector_table) }
    }

    fn fail(&mut self) -> ! {
        for blink in 0..2 * FAULT_BLINKS {
            self.kick_watchdog();
            self.indicator.fault(blink.is_multiple_of(2));
            cortex_m::asm::delay(HALF_PERIOD_CYCLES);
        }
        self.indicator.off();
        if let Some(prepare_shutdown) = self.prepare_shutdown.take() {
            prepare_shutdown();
        }
        // SHUTDOWN powers the watchdog down.
        self.kick_watchdog();
        power::shutdown()
    }
}

/// Point `VTOR` at the application and branch to its reset handler with its
/// initial `MSP`.
unsafe fn jump(vector_table: u32) -> ! {
    cortex_m::interrupt::disable();
    // SAFETY: SCB is always present; VTOR accepts any 256-byte aligned
    // address, which `HEADER_SIZE` guarantees for a sector-aligned slot.
    unsafe {
        (*cortex_m::peripheral::SCB::PTR).vtor.write(vector_table);
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        // cortex-m-rt's reset handler does not enable interrupts; restore
        // the reset state (PRIMASK clear) the application expects.
        cortex_m::interrupt::enable();
        cortex_m::asm::bootload(vector_table as *const u32)
    }
}
