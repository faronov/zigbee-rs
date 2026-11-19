/* ZRHP NCP memory map for a CC2652P7 dongle (704 KiB flash, 88 x 8 KiB
 * sectors; 144 KiB SRAM). Build-only: no CC2652P7 unit has been inspected.
 *
 *   0x00000..0x86000  application (536 KiB)
 *   0x86000..0x8A000  identity/settings journal   2 x 8 KiB
 *   0x8A000..0x8E000  APS tables journal          2 x 8 KiB
 *   0x8E000..0x92000  Green Power journal         2 x 8 KiB
 *   0x92000..0x96000  child-table journal         2 x 8 KiB
 *   0x96000..0xA6000  Trust Center stream journal 2 x 32 KiB
 *   0xA6000..0xAE000  security journal            2 x 16 KiB
 *   0xAE000..0xB0000  CCFG sector (CCFG at 0xAFFA8, rest erased)
 *
 * Keep in sync with src/layout.rs (asserted there and by
 * examples/cc26x2-router/tools/verify-layout.py --layout ncp).
 */
MEMORY
{
    FLASH    : ORIGIN = 0x00000000, LENGTH = 536K
    NCP_NV   : ORIGIN = 0x00086000, LENGTH = 160K
    CCFG_PAD : ORIGIN = 0x000AE000, LENGTH = 0x1FA8
    CCFG     : ORIGIN = 0x000AFFA8, LENGTH = 88
    RAM      : ORIGIN = 0x20000000, LENGTH = 144K
}

SECTIONS
{
  .ccfg ORIGIN(CCFG) : { KEEP(*(.ccfg)) } > CCFG
}
INSERT AFTER .rodata;

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(NCP_NV),
  "CC2652P7 NCP: application must end at the NV journals");
ASSERT(ORIGIN(NCP_NV) + LENGTH(NCP_NV) == ORIGIN(CCFG_PAD),
  "CC2652P7 NCP: NV journals must end at the CCFG sector");
ASSERT(ORIGIN(CCFG) + LENGTH(CCFG) == 704K,
  "CC2652P7 NCP: CCFG must be the last 88 bytes of the 704 KiB flash");
ASSERT(SIZEOF(.ccfg) == 88, "CC2652P7 NCP: the image must carry exactly one 88-byte CCFG");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20024000,
  "CC2652P7 NCP: RAM must end at the top of the 144 KiB SRAM");
ASSERT(_stack_start - __euninit >= 16K,
  "CC2652P7 NCP: static storage must leave at least 16 KiB for the stack");
