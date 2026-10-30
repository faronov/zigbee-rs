/* BL702 XT-ZB1 bootloader memory layout, see ../layout/src/lib.rs.
 *
 * The mask ROM boots the image named by the boot header at flash 0
 * (img_start 0x2000) and maps physical 0x2000 at XIP 0x23000000, so the
 * loader's _start is the first word of the XIP window. The region ends at
 * physical 0x10000 (the primary slot); its last 32 bytes hold the
 * LoaderInfo record the application reads to select its OTA channel.
 * RAM is the application's: the loader's state is dead once it jumps.
 *
 * The boot ROM image handed to bflb-mcu-tool excludes .loader_info
 * (objcopy --remove-section): the tool rounds img_len up past the raw image
 * (0xE000 -> 0xE010), so a raw image reaching 0x10000 would make the ROM's
 * boot hash cover the primary slot's MCUboot header. tools/bl702_ota.py
 * places the record at physical 0xFFE0 in the full-flash image instead and
 * rejects a ROM-hashed image that reaches it.
 */
MEMORY
{
  FLASH       : ORIGIN = 0x23000000, LENGTH = 0xDFE0
  LOADER_INFO : ORIGIN = 0x2300DFE0, LENGTH = 32
  RAM1        : ORIGIN = 0x42014000, LENGTH = 80K
  RAM2        : ORIGIN = 0x42028400, LENGTH = 15K
}

REGION_ALIAS("REGION_TEXT", FLASH);
REGION_ALIAS("REGION_RODATA", FLASH);
REGION_ALIAS("REGION_DATA", RAM1);
REGION_ALIAS("REGION_BSS", RAM1);
REGION_ALIAS("REGION_HEAP", RAM1);
REGION_ALIAS("REGION_STACK", RAM2);

SECTIONS
{
  .init.rust : { KEEP(*(.init.rust)) } > REGION_TEXT
}
INSERT AFTER .text;

SECTIONS
{
  .loader_info : { KEEP(*(.loader_info)) } > LOADER_INFO
}
INSERT AFTER .rodata;

SECTIONS
{
  /DISCARD/ : { *(.eh_frame) *(.eh_frame_hdr) }
}
