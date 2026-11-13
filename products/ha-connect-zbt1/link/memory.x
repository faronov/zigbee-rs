/* Product-selected memory map for the Home Assistant Connect ZBT-1.
 * Part: EFR32MG21A020F1024IM32; flash: 1 MiB; RAM: 96 KiB (DEVINFO MSIZE
 * 0x00600400 on the inspected unit).
 *
 * The resident Gecko Bootloader occupies 0x0..0x4000 and must never be
 * written. 0xF0000..0xF4000 holds the router role's Green Power Proxy Table
 * journal and 0xF4000..0xFC000 its child-table and APS journals; every role
 * image links this same map, so no role's code can overlap another role's
 * persistence. Keep in sync with src/journal.rs.
 */
MEMORY
{
    BOOTLOADER  : ORIGIN = 0x00000000, LENGTH = 16K
    FLASH         : ORIGIN = 0x00004000, LENGTH = 944K
    GP_PROXY_TABLE : ORIGIN = 0x000F0000, LENGTH = 16K
    ROUTER_TABLES : ORIGIN = 0x000F4000, LENGTH = 32K
    PERSISTENCE   : ORIGIN = 0x000FC000, LENGTH = 16K
    RAM         : ORIGIN = 0x20000000, LENGTH = 96K
}

_security_partition_start = ORIGIN(PERSISTENCE);
_security_partition_end   = ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE);

ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(FLASH),
  "ZBT-1 application must begin immediately after the Gecko Bootloader");
ASSERT(ORIGIN(FLASH) == 0x00004000,
  "ZBT-1 application must start at 0x4000 (Gecko Bootloader handoff)");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(GP_PROXY_TABLE),
  "ZBT-1 application must end at the GP Proxy Table journal");
ASSERT(ORIGIN(GP_PROXY_TABLE) + LENGTH(GP_PROXY_TABLE) == ORIGIN(ROUTER_TABLES),
  "ZBT-1 GP Proxy Table journal must end at the router-table journals");
ASSERT(ORIGIN(ROUTER_TABLES) == 0x000F4000,
  "ZBT-1 router-table journals must stay at 0xF4000 (boot/zbt1.layout)");
ASSERT(ORIGIN(ROUTER_TABLES) + LENGTH(ROUTER_TABLES) == ORIGIN(PERSISTENCE),
  "ZBT-1 router-table journals must end at the security partition");
ASSERT(LENGTH(PERSISTENCE) == 16K,
  "ZBT-1 persistence must remain two 8 KiB sectors");
ASSERT(_security_partition_end == 1024K,
  "ZBT-1 persistence must occupy the top of the 1 MiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20018000,
  "ZBT-1 RAM must end at the top of the 96 KiB SRAM");
ASSERT(_stack_start - __euninit >= 16K,
  "ZBT-1 static storage must leave at least 16 KiB for the stack");
