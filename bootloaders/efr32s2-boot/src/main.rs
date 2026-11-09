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

#![no_std]
#![no_main]

use efr32s2_hal::peripherals::Peripherals;
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

const _: () = assert!(layout::BOOT_WATCHDOG_MS == 0);
const _: () =
    assert!((layout::SECTOR_SIZE as usize).is_multiple_of(efr32s2_hal::flash::FLASH_PAGE_SIZE));
const _: () = assert!(layout::XIP_BASE == 0);

struct Fail;

impl Hooks for Fail {
    fn fail(&mut self) -> ! {
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
    zrs_boot_cortex_m::run(peripherals.flash, &CONFIG, &mut Fail)
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
