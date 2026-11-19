/* Product memory map for a CC2652P7 dongle (build-only: no unit inspected).
 * Flash 704 KiB (88 x 8 KiB sectors); SRAM 144 KiB.
 *
 *   0x00000..0x9E000  application
 *   0x9E000..0xA2000  router: Green Power Proxy Table journal (reserved)
 *   0xA2000..0xAA000  router: child-table and APS journals
 *   0xAA000..0xAE000  security journal (two 8 KiB sectors)
 *   0xAE000..0xB0000  CCFG sector (CCFG at 0xAFFA8, rest erased)
 *
 * Same partition order and sizes as memory-cc2652p.x, moved to the top of
 * the larger flash. Keep in sync with src/journal.rs.
 */
MEMORY
{
    FLASH          : ORIGIN = 0x00000000, LENGTH = 632K
    GP_PROXY_TABLE : ORIGIN = 0x0009E000, LENGTH = 16K
    ROUTER_TABLES  : ORIGIN = 0x000A2000, LENGTH = 32K
    PERSISTENCE    : ORIGIN = 0x000AA000, LENGTH = 16K
    CCFG_PAD       : ORIGIN = 0x000AE000, LENGTH = 0x1FA8
    CCFG           : ORIGIN = 0x000AFFA8, LENGTH = 88
    RAM            : ORIGIN = 0x20000000, LENGTH = 144K
}

SECTIONS
{
  .ccfg ORIGIN(CCFG) : { KEEP(*(.ccfg)) } > CCFG
}
INSERT AFTER .rodata;

_security_partition_start = ORIGIN(PERSISTENCE);
_security_partition_end   = ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE);

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(GP_PROXY_TABLE),
  "CC2652P7: application must end at the GP Proxy Table journal");
ASSERT(ORIGIN(GP_PROXY_TABLE) + LENGTH(GP_PROXY_TABLE) == ORIGIN(ROUTER_TABLES),
  "CC2652P7: GP Proxy Table journal must end at the router tables");
ASSERT(ORIGIN(ROUTER_TABLES) + LENGTH(ROUTER_TABLES) == ORIGIN(PERSISTENCE),
  "CC2652P7: router tables must end at the security journal");
ASSERT(ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE) == ORIGIN(CCFG_PAD),
  "CC2652P7: security journal must end at the CCFG sector");
ASSERT(ORIGIN(CCFG) + LENGTH(CCFG) == 704K,
  "CC2652P7: CCFG must be the last 88 bytes of the 704 KiB flash");
ASSERT(SIZEOF(.ccfg) == 88, "CC2652P7: the image must carry exactly one 88-byte CCFG");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20024000,
  "CC2652P7: RAM must end at the top of the 144 KiB SRAM");
ASSERT(_stack_start - __euninit >= 12K,
  "CC2652P7: static storage must leave at least 12 KiB for the stack");
