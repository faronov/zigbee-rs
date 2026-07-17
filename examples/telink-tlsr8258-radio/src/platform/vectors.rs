//! Boot vector table, banked-SP/IRQ setup, and `.data`/`.bss` startup init —
//! transcribed from `examples/telink-tlsr8258-sensor/src/main.rs`'s
//! `global_asm!` blocks (proven on hardware, tc32-45, `diag-beacon`).
//!
//! Only compiled for the real `tc32-unknown-none-elf` target (see
//! `platform`'s module docs for the `target_arch = "tc32"` gate rationale):
//! this file uses tc32-only mnemonics (`tj`, `tmcsr`, the raw
//! `.short 0x6BD8`/`0x6BD0`/`0x6900` IRQ context save/restore opcodes used by
//! the sensor lab) that a host assembler cannot encode.
//!
//! ## Deliberate simplifications vs. the sensor bring-up lab
//!
//! - No flash erase/program routines: this firmware never touches NV flash.
//! - The IRQ vector is a bare `bx lr`. This firmware runs fully polled with
//!   the CPU IRQ globally disabled (see `mac_test` and the "keep interrupts
//!   disabled" requirement) — the vector must exist for the header format,
//!   but with IRQs masked at `REG_IRQ_EN`/`REG_IRQ_MASK` it is never expected
//!   to execute. There is deliberately no IRQ-context save/restore dance
//!   here (nothing this firmware does needs it), which also means: never
//!   flip on `REG_IRQ_EN` without first giving this vector a real body.

core::arch::global_asm!(
    ".section .vectors, \"ax\"",
    ".balign 4",
    ".global _reset_vector",
    "_reset_vector:",
    // ─── Offset 0x00: Header (Telink boot ROM format) ───
    "tj __reset",
    ".short 0x0000",
    ".word 0x00000000",
    ".byte 0x4B, 0x4E, 0x4C, 0x54", // "KNLT" magic
    ".word 0x00880000 + _ramcode_size_div_16_align_256_",
    "tj __irq",
    ".short 0x0000",
    ".short 0x0000",
    ".short 0x0000",
    ".word _bin_size_",
    ".word 0x00000000",
    //
    // ─── Offset 0x20: IRQ entry — bare stub; IRQs stay globally disabled ───
    ".globl __irq",
    "__irq:",
    "bx lr",
    //
    // ─── __reset: official Telink cstartup_8258.S init sequence ───
    "__reset:",
    // 1. Banked SP init: IRQ mode first, then SVC mode (matches vendor
    //    cstartup_8258.S). Stack tops come from memory.x.
    "ldr r0, =0x12",
    "tmcsr r0",
    "ldr r0, =_irq_stack_top",
    "mov sp, r0",
    "ldr r0, =0x13",
    "tmcsr r0",
    "ldr r0, =_svc_stack_top",
    "mov sp, r0",
    // 2. Zero I-cache tags (256 bytes = 64 words), unrolled: the tc32 LLVM
    //    backend has had backward-branch codegen bugs on this target.
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
    // 3. I-cache config (0x80060C/0D): RAM-code preload size / 256, next page.
    "ldr r1, =0x80060C",
    "ldr r0, =_ramcode_size_div_256_",
    "strb r0, [r1, #0]",
    "adds r0, #1",
    "strb r0, [r1, #1]",
    // 4. System power ON (0x800060).
    "ldr r1, =0x800060",
    "ldr r0, =0xFF000000",
    "str r0, [r1, #0]",
    "movs r0, #0xFF",
    "strb r0, [r1, #4]",
    "strb r0, [r1, #5]",
    // 5. Flash deep-wakeup (0xAB via SPI at 0x80000C).
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
    // 6. Jump to Rust-callable `_start` (data/bss init below).
    "tjl _start",
);

core::arch::global_asm!(
    ".section .vectors.startup, \"ax\"",
    ".global _start",
    ".type _start, %function",
    "_start:",
    // Copy .data from flash (_etext) to RAM (_sdata..edata).
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
    // Zero .bss (_sbss..ebss). The `.diag` section is a *separate* NOLOAD
    // output section outside this range (see memory.x) and is therefore
    // never touched here — this is what lets the diagnostic record survive
    // a warm reset.
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
