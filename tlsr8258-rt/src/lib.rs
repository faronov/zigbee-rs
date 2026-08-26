//! Hardware-proven TLSR8258 cold-boot and IRQ startup.
//!
//! Applications provide `irq_handler` and `_rust_entry`; linker scripts retain
//! the vector sections emitted by this crate.

#![no_std]

use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

/// Run one async application future on the single-threaded TLSR8258 runtime.
pub fn block_on<F: Future>(future: F) -> F::Output {
    const VTABLE: RawWakerVTable = RawWakerVTable::new(
        |pointer| RawWaker::new(pointer, &VTABLE),
        |_| {},
        |_| {},
        |_| {},
    );

    let mut future = future;
    let mut future = unsafe { Pin::new_unchecked(&mut future) };
    let waker = unsafe { Waker::new(core::ptr::null(), &VTABLE) };
    let mut context = Context::from_waker(&waker);

    loop {
        if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
            return output;
        }
        for _ in 0..100 {
            core::hint::spin_loop();
        }
    }
}

#[cfg(target_arch = "tc32")]
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

#[cfg(all(target_arch = "tc32", not(feature = "retention-proof")))]
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

// The retention image has a deliberately separate startup body. Detection is
// performed before either the `.data` load or `.bss` clear: cold boot takes
// the ordinary C/Rust initialization path, while a valid LOW32K wake keeps
// every writable byte intact and enters a fresh Rust root on freshly reset
// banked stacks. An unreadable or unexpected mode never aliases to cold boot.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
core::arch::global_asm!(
    ".section .vectors.startup, \"ax\"",
    ".global _start",
    ".type _start, %function",
    "_start:",
    "tjl __tlsr8258_retention_probe",
    "cmp r0, #0",
    "beq 1f",
    "cmp r0, #1",
    "beq 5f",
    "tjl _rust_retention_fault_entry",
    // Cold entry: initialize all ordinary writable sections. Explicit
    // `.retained` cells are NOLOAD and are initialized by the composition
    // root before its validity marker is committed.
    "1:",
    "ldr r0, =_sdata",
    "ldr r1, =_edata",
    "cmp r0, r1",
    "bhs 3f",
    "subs r1, r0",
    "ldr r2, =_etext",
    "2:",
    "ldrb r3, [r2]",
    "strb r3, [r0]",
    "adds r2, #1",
    "adds r0, #1",
    "subs r1, #1",
    "bne 2b",
    "3:",
    "ldr r0, =_sbss",
    "ldr r1, =_ebss",
    "cmp r0, r1",
    "bhs 4f",
    "subs r1, r0",
    "movs r2, #0",
    "6:",
    "strb r2, [r0]",
    "adds r0, #1",
    "subs r1, #1",
    "bne 6b",
    "4:",
    "tjl _rust_cold_entry",
    // Retention entry: no copy, no clear, and no jump back into the old
    // async frame. __reset has already installed fresh SVC/IRQ stack tops.
    "5:",
    "tjl _rust_retention_entry",
);

/// Tri-state early-boot probe used only by the feature-gated LOW32K image.
///
/// Return values are an ABI between the Rust probe and `_start`:
/// `0 = cold`, `1 = valid LOW32K mode`, `2 = unreadable/unexpected`.
/// The implementation uses no writable Rust state and bounds the analog-bus
/// wait, so it is valid before `.data`/`.bss` initialization.
#[cfg(all(target_arch = "tc32", feature = "retention-proof"))]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".ram_code")]
extern "C" fn __tlsr8258_retention_probe() -> u32 {
    const REG_ANALOG_ADDR: u32 = 0x0080_00b8;
    const REG_ANALOG_DATA: u32 = REG_ANALOG_ADDR + 1;
    const REG_ANALOG_TRIGGER: u32 = REG_ANALOG_ADDR + 2;
    const REG_IRQ_ENABLE: u32 = 0x0080_0643;
    const LOW32K_MODE_REGISTER: u8 = 0x7e;
    const LOW32K_MODE: u8 = 0x07;
    const POLL_LIMIT: u32 = 100_000;

    unsafe {
        let irq = core::ptr::read_volatile(REG_IRQ_ENABLE as *const u8);
        core::ptr::write_volatile(REG_IRQ_ENABLE as *mut u8, 0);
        core::ptr::write_volatile(REG_ANALOG_ADDR as *mut u8, LOW32K_MODE_REGISTER);
        core::ptr::write_volatile(REG_ANALOG_TRIGGER as *mut u8, 0x40);

        let mut count = 0;
        let mut ready = false;
        while count < POLL_LIMIT {
            if core::ptr::read_volatile(REG_ANALOG_TRIGGER as *const u8) & 1 == 0 {
                ready = true;
                break;
            }
            core::arch::asm!("nop");
            count += 1;
        }
        let mode = if ready {
            core::ptr::read_volatile(REG_ANALOG_DATA as *const u8)
        } else {
            0xff
        };
        core::ptr::write_volatile(REG_ANALOG_TRIGGER as *mut u8, 0);
        let result = if !ready {
            2
        } else if mode == 0 {
            0
        } else if mode == LOW32K_MODE {
            1
        } else {
            2
        };
        // A retention/fault entry must stay globally masked until the PM
        // completion token restores the exact recorded state. Cold startup
        // preserves the reset-time value as before.
        core::ptr::write_volatile(REG_IRQ_ENABLE as *mut u8, if result == 0 { irq } else { 0 });
        result
    }
}
