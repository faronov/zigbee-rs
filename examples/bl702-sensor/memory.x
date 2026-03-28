/* BL702 memory layout for bare-metal Rust */
MEMORY
{
  FLASH : ORIGIN = 0x23000000, LENGTH = 512K
  RAM   : ORIGIN = 0x42014000, LENGTH = 112K   /* 128K SRAM - 16K cache/EM */
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM);
REGION_ALIAS("REGION_BSS", RAM);
REGION_ALIAS("REGION_HEAP", RAM);
REGION_ALIAS("REGION_STACK", RAM);

/* Discard unwinding info — we use panic-halt, not panic-unwind */
SECTIONS
{
  /DISCARD/ : { *(.eh_frame) *(.eh_frame_hdr) }
}
