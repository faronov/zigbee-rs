//! zigbee-rs loader for the nRF52840 sensor deployments.
//!
//! Composition only: the layout (slots, journal, protected regions, keys)
//! comes from `../boot/<deployment>.layout`, the boot sequence from
//! `zrs-boot-cortex-m`, flash access from the `embassy-nrf` NVMC driver.
//! No status output: the loader leaves GPIO, clocks and UICR untouched. With
//! nothing bootable it halts in WFI (recover over SWD, or on the UF2
//! deployments by double-tapping reset into the UF2 drive).
//!
//! Watchdog: the nRF52840 WDT cannot be stopped or reconfigured once
//! started; only a pin, power-on, brown-out or watchdog reset clears it (it
//! survives a soft reset). With the layout's `watchdog-ms` > 0 and the WDT
//! idle, the loader starts it before the boot decision with
//! `CRV = ceil(watchdog-ms * 32768 / 1000)`, reload register `RR[0]` only,
//! and `CONFIG.SLEEP = CONFIG.HALT = PAUSE` (it counts CPU-active time only;
//! it never wakes a sleeping SED). A WDT that already runs (from before a
//! soft reset) is kept as it is. Every engine step reloads every enabled
//! reload register. The application inherits the running WDT and must
//! reload it (`nrf_sensor_app::NrfWatchdogSupervisor` adopts whatever runs).
//! Halting with nothing bootable is permanent: the WDT pauses in WFI.

#![no_std]
#![no_main]

use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::peripherals::WDT;
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig, Watchdog, WatchdogHandle};
use panic_halt as _;
use zrs_boot_cortex_m::{Config, Hooks};

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

/// WDT clock (LFCLK).
const WDT_HZ: u64 = 32_768;
/// `CRV` for the layout's period, rounded up (the layout check bounds
/// `watchdog-ms` so that it fits).
const WDT_CRV: u32 = (layout::BOOT_WATCHDOG_MS as u64 * WDT_HZ).div_ceil(1000) as u32;
const _: () = assert!(layout::BOOT_WATCHDOG_MS as u64 * WDT_HZ <= u32::MAX as u64 * 1000);
const _: () = assert!((layout::SECTOR_SIZE as usize).is_multiple_of(embassy_nrf::nvmc::PAGE_SIZE));

struct Shell;

impl Hooks for Shell {
    fn kick_watchdog(&mut self) {
        reload_watchdog();
    }

    fn fail(&mut self) -> ! {
        halt()
    }
}

/// Start the WDT for the layout's period unless it already runs.
fn start_watchdog() {
    // SAFETY: the loader is the only code running; nothing else owns the WDT.
    let wdt = unsafe { WDT::steal() };
    if layout::BOOT_WATCHDOG_MS == 0 || wdt::Config::try_new(&wdt).is_some() {
        return;
    }
    let mut config = wdt::Config::default();
    config.timeout_ticks = WDT_CRV;
    config.action_during_sleep = SleepConfig::PAUSE;
    config.action_during_debug_halt = HaltConfig::PAUSE;
    // Cannot fail: the WDT is idle.
    let _ = Watchdog::try_new::<1>(wdt, config);
}

/// Reload all eight reload registers (a write to one that `RREN` leaves
/// disabled has no effect), so a WDT kept from before a soft reset is
/// served whatever handles it was started with. No-op while it is idle.
fn reload_watchdog() {
    for index in 0..8 {
        // SAFETY: a reload request only restarts the count.
        unsafe { WatchdogHandle::steal(index) }.pet();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // SAFETY: the loader is the only code running; nothing else owns the
    // NVMC. `embassy_nrf::init` is deliberately not called.
    let nvmc = Nvmc::new(unsafe { embassy_nrf::peripherals::NVMC::steal() });
    start_watchdog();
    reload_watchdog();
    zrs_boot_cortex_m::run(nvmc, &CONFIG, &mut Shell)
}

fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}
