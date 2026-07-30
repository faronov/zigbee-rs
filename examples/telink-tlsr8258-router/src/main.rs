//! Production TLSR8258 parent-router entry point.
//!
//! Parent timing and Frame Pending behavior still require a hardware sniffer
//! gate before interoperability is claimed.

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
    tlsr8258_hal::clocks::init();
    app::run();
}
