//! zigbee-rs loader for EFR32 Series 2 (MG21/MG22) products.
//!
//! Composition only: the layout (slots, journal, protected regions, keys)
//! comes from the selected product's `boot/*.layout`, the boot sequence from
//! `zrs-boot-cortex-m`, flash access from the `efr32s2-hal` MSC driver.
//!
//! Hand-off state: `Peripherals::take` opens the GPIO, HFXO, SYSCFG and MSC
//! bus clocks on xG22 (gated at reset); they stay open for the application,
//! which opens the same set itself. Nothing else is configured: no GPIO,
//! oscillator or DCDC changes. No status output. With nothing bootable the
//! loader halts in WFI (recover over SWD) or, with `gecko-recovery` on a
//! chained layout, re-enters the resident Gecko Bootloader.
//!
//! Watchdog: with the layout's `watchdog-ms` > 0, WDOG0 (ULFRCO, reset
//! mode, paused in debug halt) is armed before the boot decision, kicked at
//! every engine step and stopped again before the hand-over, the halt and
//! the Gecko re-entry, so the application starts with WDOG0 disabled and
//! unlocked (its own policy). Left on are the xG22 WDOG0 bus clock and
//! `CMU_WDOG0CLKCTRL = ULFRCO`. A watchdog the loader cannot arm (locked by
//! earlier code) does not stop the boot: the decision runs without it.

#![no_std]
#![no_main]

use efr32s2_hal::peripherals::Peripherals;
use efr32s2_hal::watchdog::{Period, Watchdog};
use panic_halt as _;
use zrs_boot_cortex_m::{Config, Entry, Hooks};

#[allow(dead_code)]
mod layout {
    include!(concat!(env!("OUT_DIR"), "/boot_layout.rs"));
}

#[unsafe(link_section = ".loader_info")]
#[used]
pub static LOADER_BLOCK: [u8; 64] =
    zrs_boot_cortex_m::loader_block(layout::LOADER_VERSION, &layout::TRUSTED_KEYS);

static CONFIG: Config = Config {
    boot: layout::BOOT_CONFIG,
    header_size: layout::IMAGE_HEADER_SIZE,
    ram: (layout::RAM_START, layout::RAM_END),
    xip_base: layout::XIP_BASE,
    vtor: true,
};

/// The layout's watchdog period, `None` when `watchdog-ms` is 0.
const WATCHDOG: Option<Period> = Period::from_ms(layout::BOOT_WATCHDOG_MS);
const _: () = assert!(layout::BOOT_WATCHDOG_MS == 0 || WATCHDOG.is_some());
const _: () =
    assert!((layout::SECTOR_SIZE as usize).is_multiple_of(efr32s2_hal::flash::FLASH_PAGE_SIZE));
const _: () = assert!(layout::XIP_BASE == 0);

struct Shell {
    watchdog: Option<Watchdog>,
}

impl Shell {
    fn stop_watchdog(&mut self) {
        if let Some(watchdog) = self.watchdog.take() {
            // A synchronizer timeout leaves it running: the application
            // then resets within one period unless it owns WDOG0 itself.
            let _ = watchdog.disarm();
        }
    }
}

impl Hooks for Shell {
    fn kick_watchdog(&mut self) {
        if let Some(watchdog) = self.watchdog.as_mut() {
            watchdog.kick();
        }
    }

    fn before_start(&mut self, _entry: &Entry) {
        self.stop_watchdog();
    }

    fn fail(&mut self) -> ! {
        self.stop_watchdog();
        #[cfg(feature = "gecko-recovery")]
        gecko_bootloader();
        #[cfg(not(feature = "gecko-recovery"))]
        halt()
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let Some(peripherals) = Peripherals::take() else {
        halt()
    };
    let watchdog = WATCHDOG.and_then(|period| Watchdog::arm(peripherals.wdog, period).ok());
    zrs_boot_cortex_m::run(peripherals.flash, &CONFIG, &mut Shell { watchdog })
}

#[cfg_attr(feature = "gecko-recovery", allow(dead_code))]
fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}

/// Reset into the resident Gecko Bootloader: `BootloaderResetCause_t`
/// `{ reason: BOOTLOAD (0x0202), signature: VALID (0xF00F) }` at the start
/// of RAM, then a software reset (Gecko SDK `btl_reset_info.h`).
#[cfg(feature = "gecko-recovery")]
fn gecko_bootloader() -> ! {
    const RESET_CAUSE_BOOTLOAD: u32 = 0xF00F_0202;
    cortex_m::interrupt::disable();
    // SAFETY: the loader is about to reset; nothing reads RAM afterwards.
    unsafe { core::ptr::write_volatile(layout::RAM_START as *mut u32, RESET_CAUSE_BOOTLOAD) };
    cortex_m::asm::dsb();
    cortex_m::peripheral::SCB::sys_reset()
}
