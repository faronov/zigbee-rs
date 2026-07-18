//! Production TLSR8258 sleepy end-device entry point.

#![no_std]
#![no_main]

mod board;
mod executor;
mod runtime_sensor;
mod security_identity;

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

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_entry() -> ! {
    tlsr8258_hal::clocks::init();
    runtime_sensor::run();
}
