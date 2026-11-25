/* ZRHP NCP memory map for EFR32MG21 F768 modules (Tuya ZS3L: A020F768;
 * F512 parts with the same NV block): 768 KiB flash in 8 KiB pages at 0x0,
 * 64 KiB SRAM at 0x2000_0000. Build-only: no unit has run this image.
 *
 *   0x00000000..0x00004000  Gecko bootloader (kept, never written)
 *   0x00004000..0x000A8000  application = GBL payload (656 KiB)
 *   0x000A8000..0x000C0000  NV journals, 6 x (2 x 8 KiB): identity, APS
 *                           tables, Green Power, children, Trust Center
 *                           stream, security (src/layout.rs)
 *
 *   0x20000000..0x20000004  Gecko BootloaderResetCause_t (outside the image)
 *   0x20000004..0x20010000  image RAM, stack at the top
 *
 * Keep in sync with src/layout.rs (asserted there and by
 * examples/efr32mg21-ncp/tools/verify-layout.py).
 */
MEMORY
{
    BOOTLOADER : ORIGIN = 0x00000000, LENGTH = 16K
    FLASH      : ORIGIN = 0x00004000, LENGTH = 656K
    NCP_NV     : ORIGIN = 0x000A8000, LENGTH = 96K
    RAM        : ORIGIN = 0x20000004, LENGTH = 64K - 4
}

SECTIONS
{
  .gecko_app_properties : ALIGN(4) { KEEP(*(.rodata.gecko_app_properties)) } > FLASH
  .zrhp_build_info : ALIGN(4) { KEEP(*(.rodata.zrhp_build_info)) } > FLASH
}
INSERT AFTER .rodata;

ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(FLASH),
  "EFR32MG21 NCP: the application must start right after the 16 KiB Gecko bootloader");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(NCP_NV),
  "EFR32MG21 NCP: the application must end at the NV journals");
ASSERT(ORIGIN(NCP_NV) + LENGTH(NCP_NV) == 0x000C0000,
  "EFR32MG21 NCP: the NV journals must end at the top of the 768 KiB flash");
ASSERT(SIZEOF(.gecko_app_properties) == 80,
  "EFR32MG21 NCP: the image must carry exactly one Gecko ApplicationProperties_t");
ASSERT(ORIGIN(RAM) == 0x20000004,
  "EFR32MG21 NCP: the first SRAM word is the Gecko reset-cause record, outside the image");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20010000,
  "EFR32MG21 NCP: RAM must end at the top of the 64 KiB SRAM");
ASSERT(_stack_start - __euninit >= 16K,
  "EFR32MG21 NCP: static storage must leave at least 16 KiB for the stack");
