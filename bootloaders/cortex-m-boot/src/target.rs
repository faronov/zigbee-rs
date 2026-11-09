//! Target-only hand-over.

use zrs_boot_core::embedded_storage::nor_flash::NorFlash;

use crate::{BootFlash, Config, Entry, Hooks, decide};

/// Run one boot decision and start the application, or call
/// [`Hooks::fail`].
pub fn run<F: NorFlash, H: Hooks>(flash: F, config: &Config, hooks: &mut H) -> ! {
    let mut flash = BootFlash::for_config(flash, &config.boot);
    match decide(&mut flash, config, hooks) {
        Some(entry) => {
            hooks.before_start(&entry);
            // SAFETY: `decide` returned this entry for the image validated
            // and checked in this boot; the loader enabled no interrupt.
            unsafe { jump(&entry, config.vtor) }
        }
        None => hooks.fail(),
    }
}

/// Start the application at `entry`: mask interrupts, set `VTOR` if the
/// core has one, load `MSP`, clear `PRIMASK` and branch to the reset
/// handler.
///
/// # Safety
///
/// `entry` must come from [`decide`](crate::decide) in this boot, and no
/// interrupt source the application does not expect may be enabled.
pub unsafe fn jump(entry: &Entry, vtor: bool) -> ! {
    cortex_m::interrupt::disable();
    // SAFETY: SCB is always present on cores with VTOR; the layout check
    // made the vector table VTOR-aligned. The stack pointer and reset
    // handler were range-checked.
    unsafe {
        if vtor {
            (*cortex_m::peripheral::SCB::PTR)
                .vtor
                .write(entry.vector_table);
        }
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
        // cortex-m-rt's reset handler does not enable interrupts; restore
        // the reset state (PRIMASK clear) the application expects.
        cortex_m::interrupt::enable();
        cortex_m::asm::bootstrap(entry.initial_sp as *const u32, entry.reset as *const u32)
    }
}
