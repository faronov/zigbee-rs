//! Production TLSR8258 join/relay router entry point.
//!
//! The current router does not admit children or implement indirect queues.

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

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_entry() -> ! {
    tlsr8258_hal::clocks::init();
    app::run();
}
