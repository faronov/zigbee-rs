/* ZRHP NCP memory map for EFR32MG24 F1536 boards (RF-BM-MG24B1 dev board,
 * SONOFF Dongle-M): 1536 KiB flash in 8 KiB pages at 0x0800_0000, 256 KiB
 * SRAM at 0x2000_0000. Build-only: no MG24 unit has run this image.
 *
 *   0x08000000..0x08006000  Gecko bootloader (kept, never written)
 *   0x08006000..0x08158000  application = GBL payload (1352 KiB)
 *   0x08158000..0x0815C000  identity/settings journal   2 x 8 KiB
 *   0x0815C000..0x08160000  APS tables journal          2 x 8 KiB
 *   0x08160000..0x08164000  Green Power journal         2 x 8 KiB
 *   0x08164000..0x08168000  child-table journal         2 x 8 KiB
 *   0x08168000..0x08178000  Trust Center stream journal 2 x 32 KiB
 *   0x08178000..0x08180000  security journal            2 x 16 KiB
 *
 *   0x20000000..0x20000004  Gecko BootloaderResetCause_t (outside the image)
 *   0x20000004..0x20040000  image RAM, stack at the top
 *
 * The first SRAM word is the bootloader/application reset-cause record
 * (efr32s2_hal::gecko_bootloader): kept out of .data/.bss so a cause
 * recorded before a reset (src/fatal.rs, or the bootloader's own) survives
 * start-up.
 *
 * Keep in sync with src/layout.rs (asserted there and by
 * examples/efr32mg24-ncp/tools/verify-layout.py and the router).
 */
MEMORY
{
    BOOTLOADER : ORIGIN = 0x08000000, LENGTH = 24K
    FLASH      : ORIGIN = 0x08006000, LENGTH = 1352K
    NCP_NV     : ORIGIN = 0x08158000, LENGTH = 160K
    RAM        : ORIGIN = 0x20000004, LENGTH = 256K - 4
}

SECTIONS
{
  .gecko_app_properties : ALIGN(4) { KEEP(*(.rodata.gecko_app_properties)) } > FLASH
  /* ZRHP build record (NCP only; empty in the router). */
  .zrhp_build_info : ALIGN(4) { KEEP(*(.rodata.zrhp_build_info)) } > FLASH
}
INSERT AFTER .rodata;

ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(FLASH),
  "EFR32MG24 NCP: the application must start right after the 24 KiB Gecko bootloader");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(NCP_NV),
  "EFR32MG24 NCP: the application must end at the NV journals");
ASSERT(ORIGIN(NCP_NV) + LENGTH(NCP_NV) == 0x08180000,
  "EFR32MG24 NCP: the NV journals must end at the top of the 1536 KiB flash");
ASSERT(SIZEOF(.gecko_app_properties) == 80,
  "EFR32MG24 NCP: the image must carry exactly one Gecko ApplicationProperties_t");
ASSERT(ORIGIN(RAM) == 0x20000004,
  "EFR32MG24 NCP: the first SRAM word is the Gecko reset-cause record, outside the image");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20040000,
  "EFR32MG24 NCP: RAM must end at the top of the 256 KiB SRAM");
ASSERT(_stack_start - __euninit >= 32K,
  "EFR32MG24 NCP: static storage must leave at least 32 KiB for the stack");
