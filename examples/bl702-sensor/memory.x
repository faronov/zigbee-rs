/* BL702 memory layout for the compile-only Rust scaffold. */
MEMORY
{
  FLASH : ORIGIN = 0x23000000, LENGTH = 512K
  /* Match the vendor Zigbee linker: 80 KiB below the RF/shared reservation,
     a reserved 1 KiB at 0x42028000, then 31 KiB above it. */
  RAM1  : ORIGIN = 0x42014000, LENGTH = 80K
  RAM2  : ORIGIN = 0x42028400, LENGTH = 31K
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM1);
REGION_ALIAS("REGION_BSS", RAM1);
REGION_ALIAS("REGION_HEAP", RAM1);
REGION_ALIAS("REGION_STACK", RAM2);

/* Discard unwinding info — we use panic-halt, not panic-unwind */
SECTIONS
{
  /DISCARD/ : { *(.eh_frame) *(.eh_frame_hdr) }
}
