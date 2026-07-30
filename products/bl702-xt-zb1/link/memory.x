/* BL702 XT-ZB1 product memory layout.
 *
 * The linked XIP payload remains capped at 512 KiB. The crash-safe Zigbee
 * security journal occupies physical flash offsets 0xFE000..0x100000, well
 * above the boot header and maximum linked payload.
 */
MEMORY
{
  FLASH : ORIGIN = 0x23000000, LENGTH = 512K
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

SECTIONS
{
  /DISCARD/ : { *(.eh_frame) *(.eh_frame_hdr) }
}
