/* BL702 XT-ZB1 product memory layout (application).
 *
 * The application runs from the primary MCUboot slot behind the product
 * bootloader (bootloader/), see layout/src/lib.rs:
 *   physical 0x10000 primary slot = XIP 0x2300E000; the 0x100-byte imgtool
 *   header precedes the application, so _start (riscv-rt .init) is linked
 *   at 0x2300E100. LENGTH leaves room for the signature TLVs and the slot
 *   trailer sector. The security journal (physical 0xFE000..0x100000) and
 *   the OTA secondary slot (0x87000..0xFE000) are outside this window.
 */
MEMORY
{
  FLASH : ORIGIN = 0x2300E100, LENGTH = 0x75C00
  /* .data/.bss end 64 bytes below the stack: STACK_GUARD sits immediately
     below the lowest stack address (see src/stack_guard.rs). */
  RAM1  : ORIGIN = 0x42014000, LENGTH = 72K - 64
  STACK_GUARD : ORIGIN = 0x42025FC0, LENGTH = 64
  /* Keep the initial stack below the BL702 reset-time embedded-memory window.
     The product conservatively leaves the top 16 KiB unused.
     24 KiB: the router's hardware stack paint measured 17,764 B (one-shot
     boot composition) and its static worst case is ~20.4 KiB (commissioning
     scan); tools/bl702-router-stack-gate.args enforces the budget. */
  RAM2  : ORIGIN = 0x42026000, LENGTH = 24K
}

/* SVC/IRQ stack guard: 64 bytes between the end of RAM1 and the stack. */
_stack_guard_start = ORIGIN(STACK_GUARD);
_stack_guard_end = ORIGIN(STACK_GUARD) + LENGTH(STACK_GUARD);

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM1);
REGION_ALIAS("REGION_BSS", RAM1);
REGION_ALIAS("REGION_HEAP", RAM1);
REGION_ALIAS("REGION_STACK", RAM2);

/* riscv-rt 0.16 leaves the Rust entry trampoline as an orphan section.
   Pin it in XIP flash so codegen-unit changes cannot place it in RAM. */
SECTIONS
{
  .init.rust : { KEEP(*(.init.rust)) } > REGION_TEXT
}
INSERT AFTER .text;

SECTIONS
{
  /DISCARD/ : { *(.eh_frame) *(.eh_frame_hdr) }
}

/* Build-time layout gate. The stack budget itself is enforced statically
   by tools/bl702-stack-gate.py and at run time by the guard. */
ASSERT(_stack_guard_end == ORIGIN(RAM2),
  "ERROR(bl702-xt-zb1): the stack guard must sit immediately below the stack");
ASSERT(_stack_guard_end - _stack_guard_start == 64,
  "ERROR(bl702-xt-zb1): the stack guard must be 64 bytes");
ASSERT(__ebss <= _stack_guard_start && __euninit <= _stack_guard_start &&
  __eheap <= _stack_guard_start,
  "ERROR(bl702-xt-zb1): .data/.bss/.uninit/heap extend into the stack guard");
ASSERT(__estack == ORIGIN(RAM2) && __sstack == ORIGIN(RAM2) + LENGTH(RAM2),
  "ERROR(bl702-xt-zb1): .stack must span exactly the RAM2 region");
ASSERT(LENGTH(RAM2) >= 24K,
  "ERROR(bl702-xt-zb1): the stack must be at least 24 KiB");
ASSERT(ORIGIN(RAM2) + LENGTH(RAM2) <= 0x4202C000,
  "ERROR(bl702-xt-zb1): the stack must stay below the reset-time memory window");
