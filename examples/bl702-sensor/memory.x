/* BL702 memory layout for the XT-ZB1 pure-Rust radio probe. */
MEMORY
{
  FLASH : ORIGIN = 0x23000000, LENGTH = 512K
  /* Keep the initial stack below the BL702 reset-time embedded-memory window.
     The probe conservatively leaves the top 16 KiB unused. */
  RAM1  : ORIGIN = 0x42014000, LENGTH = 80K
  RAM2  : ORIGIN = 0x42028400, LENGTH = 15K
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
