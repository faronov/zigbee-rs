//! zigbee-rs loader for the nRF52840 sensor deployments.
//!
//! Composition only: the layout (slots, journal, protected regions, keys)
//! comes from `../boot/<deployment>.layout`, the boot sequence from
//! `zrs-boot-cortex-m`, flash access from the `embassy-nrf` NVMC driver.
//! No status output: the loader leaves GPIO, clocks and UICR untouched. With
//! nothing bootable it halts in WFI (recover over SWD, or on the UF2
//! deployments by double-tapping reset into the UF2 drive).

#![no_std]
#![no_main]

use embassy_nrf::nvmc::Nvmc;
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
const _: () = assert!((layout::SECTOR_SIZE as usize).is_multiple_of(embassy_nrf::nvmc::PAGE_SIZE));

struct Halt;

impl Hooks for Halt {
    fn fail(&mut self) -> ! {
        halt()
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // SAFETY: the loader is the only code running; nothing else owns the
    // NVMC. `embassy_nrf::init` is deliberately not called.
    let nvmc = Nvmc::new(unsafe { embassy_nrf::peripherals::NVMC::steal() });
    zrs_boot_cortex_m::run(nvmc, &CONFIG, &mut Halt)
}

fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}
