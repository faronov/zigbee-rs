//! zigbee-rs loader for PHY62x2 products, started by the boot ROM.
//!
//! Composition only: the layout (slots, journal, protected regions, keys)
//! comes from the selected product's `boot/*.layout`, the boot sequence from
//! `zrs-boot-cortex-m`, flash access from the `phy6222-hal` SPIF driver.
//!
//! Boot ABI (Cortex-M0, no VTOR): the boot ROM dispatches every exception
//! and interrupt through the 256-entry jump table at 0x1fff0000. An
//! application image therefore carries its jump table directly after its
//! `[SP, Reset]` pair; once the image is validated and its entry checked,
//! the loader copies those 0x400 bytes to 0x1fff0000 and starts Reset with
//! interrupts masked by the hand-over.
//!
//! Hand-off state: the XIP cache is enabled (the application enables it
//! again, idempotently); nothing else is configured. No status output. With
//! nothing bootable the loader halts in WFI (recover over SWD).
//!
//! Cold boot only: PHY62x2 system sleep wakes through the boot ROM, i.e.
//! through this loader, whose RAM initialisation would overwrite retained
//! application state. Products using system sleep cannot use this loader
//! until it has a RAM-preserving warm-wake path.

#![no_std]
#![no_main]

use panic_halt as _;
use phy6222_hal::{
    flash::{Phy62x2Flash, enable_xip_cache},
    peripherals::Peripherals,
};
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
    vtor: false,
};

/// The ROM's interrupt dispatch table.
const JUMP_TABLE: *mut u32 = 0x1fff_0000 as *mut u32;
const JUMP_TABLE_WORDS: u32 = 256;
/// The application's jump table follows its `[SP, Reset]` pair.
const APP_JUMP_TABLE_OFFSET: u32 = 8;

const _: () = assert!(layout::BOOT_WATCHDOG_MS == 0);
const _: () = assert!(layout::SECTOR_SIZE.is_multiple_of(phy6222_hal::flash::SECTOR_SIZE));
const _: () = assert!(layout::XIP_BASE == 0x1100_0000);

struct Rom;

impl Hooks for Rom {
    fn before_start(&mut self, entry: &Entry) {
        let table = entry.vector_table + APP_JUMP_TABLE_OFFSET;
        if table + JUMP_TABLE_WORDS * 4 > entry.image_end {
            // The image cannot carry a jump table: never start it with the
            // loader's table in place.
            halt()
        }
        for word in 0..JUMP_TABLE_WORDS as usize {
            // SAFETY: the source lies inside the validated, XIP-mapped image
            // (checked above); the destination is the ROM's 0x400-byte jump
            // table, which no interrupt uses (none is enabled).
            unsafe {
                let value = core::ptr::read_volatile((table as *const u32).add(word));
                core::ptr::write_volatile(JUMP_TABLE.add(word), value);
            }
        }
    }

    fn fail(&mut self) -> ! {
        halt()
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let Some(peripherals) = Peripherals::take() else {
        halt()
    };
    let Ok(flash) = Phy62x2Flash::new(peripherals.flash, layout::FLASH_END as usize) else {
        halt()
    };
    // Validation hashes the slot through XIP; the ROM hands over in cache
    // bypass.
    enable_xip_cache();
    zrs_boot_cortex_m::run(flash, &CONFIG, &mut Rom)
}

fn halt() -> ! {
    cortex_m::interrupt::disable();
    loop {
        cortex_m::asm::wfi();
    }
}
