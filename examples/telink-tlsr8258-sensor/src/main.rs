//! TLSR8258 polling end-device sensor.

#![no_std]
#![no_main]

mod app;

use tlsr8258_rt as _;

#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    loop {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn irq_handler() {}

fn startup_failed() -> ! {
    loop {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

#[unsafe(no_mangle)]
#[cfg(not(feature = "retention-proof"))]
pub unsafe extern "C" fn _rust_entry() -> ! {
    // Fail closed: a clock-bring-up timeout means the clock tree is in an
    // unknown state, so `app::run()` must never be reached in that case.
    // This matches the panic handler above rather than introducing new
    // panic machinery, since no `.expect()`/`panic!()` is otherwise used
    // in this binary.
    if tlsr8258_hal::clocks::init().is_err() {
        startup_failed();
    }
    // Full-SRAM SUSPEND uses the independent 16 MHz system timer and the
    // calibrated RC32K wake source. Bring both up once on cold boot and fail
    // closed before any MAC/radio construction if either cannot be verified.
    if tlsr8258_hal::pm::system_timer_init().is_err() {
        startup_failed();
    }
    if tlsr8258_hal::pm::rc_32k_init_and_cal().is_err() {
        startup_failed();
    }
    app::run();
}

/// Cold entry for the explicit LOW32K image. It performs the same verified
/// clock/system-timer/RC32K initialization as the default SUSPEND image, then
/// constructs every borrowed lifecycle object directly in retained storage.
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_cold_entry() -> ! {
    if tlsr8258_hal::clocks::init().is_err()
        || tlsr8258_hal::pm::system_timer_init().is_err()
        || tlsr8258_hal::pm::rc_32k_init_and_cal().is_err()
    {
        startup_failed();
    }
    app::cold_run()
}

/// Reset-on-wake entry. `.data` and `.bss` are still retained; only clocks
/// are established here. PM time, Timer0, MAC/radio, AES, RNG, ADC/voltage
/// guard, LEDs and IRQ state are restored atomically by `retention_run`
/// before its fresh root future services the application.
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_retention_entry() -> ! {
    if tlsr8258_hal::clocks::init().is_err() {
        tlsr8258_tb04_product::sensor::fail_closed_retention_reset();
    }
    app::retention_run()
}

/// Tri-state probe failure target. It is valid before writable-section
/// initialization and can only reset/stop; it never aliases an unreadable
/// retention marker to a joined cold boot.
#[cfg(feature = "retention-proof")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_retention_fault_entry() -> ! {
    let _ = tlsr8258_hal::clocks::init();
    tlsr8258_tb04_product::sensor::fail_closed_retention_reset()
}
