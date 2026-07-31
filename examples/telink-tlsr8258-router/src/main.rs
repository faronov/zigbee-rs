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
    // Fail closed: a clock-bring-up timeout means the clock tree is in an
    // unknown state, so `app::run()` must never be reached in that case.
    // This matches the panic handler above rather than introducing new
    // panic machinery, since no `.expect()`/`panic!()` is otherwise used
    // in this binary.
    if tlsr8258_hal::clocks::init().is_err() {
        loop {
            unsafe {
                core::arch::asm!("nop");
            }
        }
    }
    app::run();
}
