/* Product-selected EFR32MG21 sensor map for BRD4181A/BRD4001A.
 * Part: EFR32MG21A010F1024IM32 (1 MiB flash, 64 KiB RAM). The map uses only
 * the lower 512 KiB so it also fits the F512 parts.
 *
 * 0x70000..0x74000 holds the router role's Green Power Proxy Table journal
 * and 0x74000..0x7C000 its child-table and APS journals; every role image
 * (sensor SED, End Device, Router) links this same map,
 * so no role's code can overlap another role's persistence.
 *
 * Keep in sync with src/journal.rs. The board intentionally supplies only
 * raw flash and has no memory-layout or persistence-policy ownership.
 */
MEMORY
{
    BOOTLOADER : ORIGIN = 0x00000000, LENGTH = 16K
    FLASH      : ORIGIN = 0x00004000, LENGTH = 432K
    GP_PROXY_TABLE : ORIGIN = 0x00070000, LENGTH = 16K
    ROUTER_TABLES : ORIGIN = 0x00074000, LENGTH = 32K
    PERSISTENCE : ORIGIN = 0x0007C000, LENGTH = 16K
    RAM        : ORIGIN = 0x20000000, LENGTH = 64K
}

_security_partition_start = ORIGIN(PERSISTENCE);
_security_partition_end   = ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE);

ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(FLASH),
  "EFR32MG21 application must begin immediately after the bootloader");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(GP_PROXY_TABLE),
  "EFR32MG21 application must end at the GP Proxy Table journal");
ASSERT(ORIGIN(GP_PROXY_TABLE) + LENGTH(GP_PROXY_TABLE) == ORIGIN(ROUTER_TABLES),
  "EFR32MG21 GP Proxy Table journal must end at the router-table journals");
ASSERT(ORIGIN(ROUTER_TABLES) == 0x00074000,
  "EFR32MG21 router-table journals must stay at 0x74000 (boot/devkit.layout)");
ASSERT(ORIGIN(ROUTER_TABLES) + LENGTH(ROUTER_TABLES) == ORIGIN(PERSISTENCE),
  "EFR32MG21 router-table journals must end at the security partition");
ASSERT(LENGTH(PERSISTENCE) == 16K,
  "EFR32MG21 persistence must remain two 8 KiB sectors");
ASSERT(_security_partition_end == 512K,
  "EFR32MG21 persistence must occupy the top of the 512 KiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20010000,
  "EFR32MG21 RAM must end at the top of the 64 KiB SRAM");
