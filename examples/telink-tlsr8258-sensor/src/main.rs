//! TLSR8258 TB-04 sleepy temperature/humidity sensor.

#![no_std]
#![no_main]

mod app;
#[cfg(feature = "retention-proof")]
mod retention;
#[cfg(not(feature = "retention-proof"))]
mod suspend;

use tlsr8258_rt as _;

#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    halt()
}

#[unsafe(no_mangle)]
pub extern "C" fn irq_handler() {}

fn halt() -> ! {
    loop {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

#[cfg(not(feature = "retention-proof"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_entry() -> ! {
    arm_stack_guard();
    // A clock-bring-up failure leaves the clock tree in an unknown state:
    // never reach the MAC/radio.
    if !tlsr8258_sensor_app::init_sleep_clocks() {
        halt();
    }
    suspend::run()
}

/// Arm the 64 B SVC stack guard before any stack-deep work or flash write.
/// The LOW32K image keeps the stack and guard in non-retained upper SRAM,
/// which loses power in retention, so both of its entries re-arm it.
#[inline(never)]
fn arm_stack_guard() {
    tlsr8258_tb04_product::stack_guard::arm();
}

/// Cold entry of the LOW32K image: same clock bring-up, then every
/// long-lived object is built in retained storage.
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_cold_entry() -> ! {
    arm_stack_guard();
    if !tlsr8258_sensor_app::init_sleep_clocks() {
        halt();
    }
    retention::cold_run()
}

/// Reset-on-wake entry. Writable RAM is retained; only clocks are started
/// here, `retention_run` restores the platform before the fresh root runs.
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_retention_entry() -> ! {
    arm_stack_guard();
    if tlsr8258_hal::clocks::init().is_err() {
        retention::fail_closed_reset();
    }
    retention::retention_run()
}

/// Unreadable/unexpected wake mode: reset, never alias to a cold boot.
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_retention_fault_entry() -> ! {
    let _ = tlsr8258_hal::clocks::init();
    retention::fail_closed_reset()
}
