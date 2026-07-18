//! # Production TLSR8258 Zigbee ROUTER entry point — EXPERIMENTAL.
//!
//! This is the join/relay-only counterpart of `runtime_main.rs`. It reuses
//! the exact same proven boot vector, IRQ glue, and production linker
//! layout (`memory-runtime.x`) as the sensor runtime; only the application
//! module differs (`runtime_router` instead of `runtime_sensor`).
//!
//! **Scope, read before flashing:** this firmware can join an existing
//! Zigbee network with the router capability bit and relay NWK traffic
//! (unicast forwarding, broadcast relay, route-request rebroadcast, link
//! status). It CANNOT parent child devices: no association responses, no
//! beacons, no `macAssociationPermit`/permit-joining, no indirect
//! (pending-frame) queue for sleepy children. See
//! `zigbee_mac::telink::TelinkMac::mlme_start` and README.md ("Router
//! firmware") for the full capability boundary. This mirrors the
//! `examples/nrf52840-router` precedent exactly.

#![no_std]
#![no_main]

mod runtime_router;

#[panic_handler]
fn panic_handler(_info: &core::panic::PanicInfo) -> ! {
    loop {
        unsafe {
            core::arch::asm!("nop");
        }
    }
}

core::arch::global_asm!(
    ".section .vectors, \"ax\"",
    ".balign 4",
    ".global _reset_vector",
    "_reset_vector:",
    "tj __reset",
    ".short 0x0000",
    ".word 0x00000000",
    ".byte 0x4B, 0x4E, 0x4C, 0x54",
    ".word 0x00880000 + _ramcode_size_div_16_align_256_",
    "tj __irq",
    ".short 0x0000",
    ".short 0x0000",
    ".short 0x0000",
    ".word _bin_size_",
    ".word 0x00000000",
    ".globl __irq",
    "__irq:",
    "push {{lr}}",
    "push {{r0, r1, r2, r3, r4, r5, r6, r7}}",
    ".short 0x6BD8",
    "mov r1, r8",
    "mov r2, r9",
    "mov r3, r10",
    "mov r4, r11",
    "mov r5, r12",
    "push {{r0, r1, r2, r3, r4, r5}}",
    "bl irq_handler",
    "pop {{r0, r1, r2, r3, r4, r5}}",
    "mov r8, r1",
    "mov r9, r2",
    "mov r10, r3",
    "mov r11, r4",
    "mov r12, r5",
    ".short 0x6BD0",
    "pop {{r0, r1, r2, r3, r4, r5, r6, r7}}",
    ".short 0x6900",
    "__reset:",
    "ldr r0, =0x12",
    "tmcsr r0",
    "ldr r0, =_irq_stack_top",
    "mov sp, r0",
    "ldr r0, =0x13",
    "tmcsr r0",
    "ldr r0, =_svc_stack_top",
    "mov sp, r0",
    "movs r0, #0",
    "ldr r1, =_ictag_start_",
    "str r0, [r1, #0]",
    "str r0, [r1, #4]",
    "str r0, [r1, #8]",
    "str r0, [r1, #12]",
    "str r0, [r1, #16]",
    "str r0, [r1, #20]",
    "str r0, [r1, #24]",
    "str r0, [r1, #28]",
    "str r0, [r1, #32]",
    "str r0, [r1, #36]",
    "str r0, [r1, #40]",
    "str r0, [r1, #44]",
    "str r0, [r1, #48]",
    "str r0, [r1, #52]",
    "str r0, [r1, #56]",
    "str r0, [r1, #60]",
    "str r0, [r1, #64]",
    "str r0, [r1, #68]",
    "str r0, [r1, #72]",
    "str r0, [r1, #76]",
    "str r0, [r1, #80]",
    "str r0, [r1, #84]",
    "str r0, [r1, #88]",
    "str r0, [r1, #92]",
    "str r0, [r1, #96]",
    "str r0, [r1, #100]",
    "str r0, [r1, #104]",
    "str r0, [r1, #108]",
    "str r0, [r1, #112]",
    "str r0, [r1, #116]",
    "str r0, [r1, #120]",
    "str r0, [r1, #124]",
    "adds r1, #128",
    "str r0, [r1, #0]",
    "str r0, [r1, #4]",
    "str r0, [r1, #8]",
    "str r0, [r1, #12]",
    "str r0, [r1, #16]",
    "str r0, [r1, #20]",
    "str r0, [r1, #24]",
    "str r0, [r1, #28]",
    "str r0, [r1, #32]",
    "str r0, [r1, #36]",
    "str r0, [r1, #40]",
    "str r0, [r1, #44]",
    "str r0, [r1, #48]",
    "str r0, [r1, #52]",
    "str r0, [r1, #56]",
    "str r0, [r1, #60]",
    "str r0, [r1, #64]",
    "str r0, [r1, #68]",
    "str r0, [r1, #72]",
    "str r0, [r1, #76]",
    "str r0, [r1, #80]",
    "str r0, [r1, #84]",
    "str r0, [r1, #88]",
    "str r0, [r1, #92]",
    "str r0, [r1, #96]",
    "str r0, [r1, #100]",
    "str r0, [r1, #104]",
    "str r0, [r1, #108]",
    "str r0, [r1, #112]",
    "str r0, [r1, #116]",
    "str r0, [r1, #120]",
    "str r0, [r1, #124]",
    "ldr r1, =0x80060C",
    "ldr r0, =_ramcode_size_div_256_",
    "strb r0, [r1, #0]",
    "adds r0, #1",
    "strb r0, [r1, #1]",
    "ldr r1, =0x800060",
    "ldr r0, =0xFF000000",
    "str r0, [r1, #0]",
    "movs r0, #0xFF",
    "strb r0, [r1, #4]",
    "strb r0, [r1, #5]",
    "ldr r1, =0x80000C",
    "movs r0, #0",
    "strb r0, [r1, #1]",
    "movs r0, #0xAB",
    "strb r0, [r1, #0]",
    "nop",
    "nop",
    "nop",
    "nop",
    "nop",
    "nop",
    "movs r0, #1",
    "strb r0, [r1, #1]",
    "tjl _start",
);

core::arch::global_asm!(
    ".section .vectors.startup, \"ax\"",
    ".global _start",
    ".type _start, %function",
    "_start:",
    "ldr r0, =_sdata",
    "ldr r1, =_edata",
    "cmp r0, r1",
    "bhs 2f",
    "subs r1, r0",
    "ldr r2, =_etext",
    "1:",
    "ldrb r3, [r2]",
    "strb r3, [r0]",
    "adds r2, #1",
    "adds r0, #1",
    "subs r1, #1",
    "bne 1b",
    "2:",
    "ldr r0, =_sbss",
    "ldr r1, =_ebss",
    "cmp r0, r1",
    "bhs 4f",
    "subs r1, r0",
    "movs r2, #0",
    "3:",
    "strb r2, [r0]",
    "adds r0, #1",
    "subs r1, #1",
    "bne 3b",
    "4:",
    "tjl _rust_entry",
);

#[unsafe(no_mangle)]
pub extern "C" fn irq_handler() {}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn _rust_entry() -> ! {
    tlsr8258_hal::clocks::init();
    runtime_router::run();
}

mod board;
mod security_identity;

mod executor {
    use core::future::Future;
    use core::pin::Pin;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    fn noop_waker() -> Waker {
        const VTABLE: RawWakerVTable = RawWakerVTable::new(
            |pointer| RawWaker::new(pointer, &VTABLE),
            |_| {},
            |_| {},
            |_| {},
        );
        unsafe { Waker::new(core::ptr::null(), &VTABLE) }
    }

    pub fn block_on<F: Future>(future: F) -> F::Output {
        let mut future = future;
        let mut future = unsafe { Pin::new_unchecked(&mut future) };
        let waker = noop_waker();
        let mut context = Context::from_waker(&waker);

        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            for _ in 0..100u32 {
                unsafe {
                    core::arch::asm!("nop");
                }
            }
        }
    }
}
