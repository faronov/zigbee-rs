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
  /* Keep the initial stack below the BL702 reset-time embedded-memory window.
     The product conservatively leaves the top 16 KiB unused. */
  RAM1  : ORIGIN = 0x42014000, LENGTH = 80K
  RAM2  : ORIGIN = 0x42028400, LENGTH = 15K
}

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
