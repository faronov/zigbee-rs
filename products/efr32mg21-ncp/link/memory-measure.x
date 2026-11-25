/* MEASUREMENT ONLY - NEVER FLASH. The F768 map with 256 KiB of "RAM", so
 * the tiny NCP links although its static RAM exceeds the part's 64 KiB and
 * `llvm-size`/`llvm-nm` can measure it (examples/efr32mg21-ncp/build.rs,
 * EFR32MG21_NCP_MEASURE=1). The 64 KiB-SRAM check is replaced by a marker
 * symbol that examples/efr32mg21-ncp/tools/verify-layout.py rejects.
 */
MEMORY
{
    BOOTLOADER : ORIGIN = 0x00000000, LENGTH = 16K
    FLASH      : ORIGIN = 0x00004000, LENGTH = 656K
    NCP_NV     : ORIGIN = 0x000A8000, LENGTH = 96K
    RAM        : ORIGIN = 0x20000004, LENGTH = 256K - 4
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
_ZRS_MEASUREMENT_ONLY_IMAGE = 1;
/* Static RAM the 64 KiB part would have to hold (data + bss + uninit). */
_zrs_static_ram = __euninit - ORIGIN(RAM);

