/* Product memory map for the Sonoff ZBDongle-P (CC2652P1F).
 * Flash 352 KiB (FLASH:FLASH_SIZE = 44 x 8 KiB sectors, read over the ROM
 * bootloader on the lab unit); SRAM 80 KiB (PRCM:RAMHWOPT = 3).
 *
 *   0x00000..0x4A000  application (vector table at 0, booted by the ROM)
 *   0x4A000..0x52000  router: child-table and APS journals
 *   0x52000..0x56000  security journal (two 8 KiB sectors)
 *   0x56000..0x58000  CCFG sector; the 88-byte CCFG sits at 0x57FA8 and the
 *                     rest of the sector stays erased. Never written at run
 *                     time (cc26x2-hal refuses it); only the ROM bootloader
 *                     tooling writes it, after the application.
 *
 * Unlike the CC2652P7 map there is no Green Power Proxy Table window: a
 * GP proxy build does not fit this flash, and its 16 KiB are needed by the
 * router image. Every role image links this map, so a reflash never moves
 * persistence.
 * Keep in sync with src/journal.rs and tools/verify-layout.py.
 */
MEMORY
{
    FLASH          : ORIGIN = 0x00000000, LENGTH = 296K
    ROUTER_TABLES  : ORIGIN = 0x0004A000, LENGTH = 32K
    PERSISTENCE    : ORIGIN = 0x00052000, LENGTH = 16K
    CCFG_PAD       : ORIGIN = 0x00056000, LENGTH = 0x1FA8
    CCFG           : ORIGIN = 0x00057FA8, LENGTH = 88
    RAM            : ORIGIN = 0x20000000, LENGTH = 80K
}

SECTIONS
{
  .ccfg ORIGIN(CCFG) : { KEEP(*(.ccfg)) } > CCFG
}
INSERT AFTER .rodata;

_security_partition_start = ORIGIN(PERSISTENCE);
_security_partition_end   = ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE);

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(ROUTER_TABLES),
  "CC2652P: application must end at the router tables");
ASSERT(ORIGIN(ROUTER_TABLES) + LENGTH(ROUTER_TABLES) == ORIGIN(PERSISTENCE),
  "CC2652P: router tables must end at the security journal");
ASSERT(ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE) == ORIGIN(CCFG_PAD),
  "CC2652P: security journal must end at the CCFG sector");
ASSERT(ORIGIN(CCFG) + LENGTH(CCFG) == 352K,
  "CC2652P: CCFG must be the last 88 bytes of the 352 KiB flash");
ASSERT(SIZEOF(.ccfg) == 88, "CC2652P: the image must carry exactly one 88-byte CCFG");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20014000,
  "CC2652P: RAM must end at the top of the 80 KiB SRAM");
ASSERT(_stack_start - __euninit >= 12K,
  "CC2652P: static storage must leave at least 12 KiB for the stack");
