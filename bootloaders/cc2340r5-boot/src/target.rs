//! Target-only boot flow: install, validate, jump, or fault and shut down.

use cc2340r5_hal::flash::InternalFlash;
use cc2340r5_hal::power;

use crate::{Config, Indicator, check_vector_table, indicate};

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
/// typically `Button::enable_shutdown_wake`.
pub fn run(
    mut flash: InternalFlash,
    config: &Config,
    indicator: &mut impl Indicator,
    prepare_shutdown: impl FnOnce(),
) -> ! {
    power::release_io_latch();

    let result = mcuboot_image::boot(&mut flash, &config.layout, &config.key, |event| {
        indicate(indicator, event)
    });
    if let Ok(bootable) = result {
        let vector_table = bootable.vector_table;
        let image_end = config.layout.primary
            + u32::from(bootable.image.header.hdr_size)
            + bootable.image.header.img_size;
        // SAFETY: the vector table lies inside a validated image in
        // memory-mapped main flash.
        let (initial_sp, reset) = unsafe {
            (
                core::ptr::read_volatile(vector_table as *const u32),
                core::ptr::read_volatile((vector_table + 4) as *const u32),
            )
        };
        if check_vector_table(initial_sp, reset, vector_table, image_end, config.ram).is_ok() {
            indicator.off();
            // SAFETY: the image was validated and its vector table checked;
            // no interrupt has been enabled by the loader.
            unsafe { jump(vector_table) }
        }
    }

    for blink in 0..2 * FAULT_BLINKS {
        indicator.fault(blink.is_multiple_of(2));
        cortex_m::asm::delay(HALF_PERIOD_CYCLES);
    }
    indicator.off();
    prepare_shutdown();
    power::shutdown()
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
