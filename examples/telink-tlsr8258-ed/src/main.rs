//! Production TLSR8258 always-on End Device entry point.
//!
//! RX-on-when-idle and non-routing: the radio IRQ services every received
//! frame (with the HAL's ISR auto-ACK) between the app's bounded receive
//! slices, exactly as on the router.

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
#[unsafe(link_section = ".ram_code")]
pub extern "C" fn irq_handler() {
    tlsr8258_hal::radio::handle_irq();
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_entry() -> ! {
    // Fail closed: never start the stack on an unknown clock tree.
    if tlsr8258_hal::clocks::init().is_err() {
        loop {
            unsafe {
                core::arch::asm!("nop");
            }
        }
    }
    app::run();
}
