/* Product-selected EFR32MG22 sensor map for BRD4183A/BRD4001A.
 * Part: EFR32MG22C224F512IM40; flash: 512 KiB (8 KiB pages); RAM: 32 KiB.
 *
 * No resident bootloader: the Secure Engine starts the application vector
 * table at 0x0. 0x74000..0x7C000 holds the router role's child-table and
 * APS journals; every role image (sensor SED, End Device, Router) links
 * this same map, so no role's code can overlap another role's persistence.
 * Keep in sync with src/journal.rs. The board supplies only raw flash and
 * owns no memory layout or persistence policy.
 */
MEMORY
{
    FLASH         : ORIGIN = 0x00000000, LENGTH = 464K
    ROUTER_TABLES : ORIGIN = 0x00074000, LENGTH = 32K
    PERSISTENCE   : ORIGIN = 0x0007C000, LENGTH = 16K
    RAM           : ORIGIN = 0x20000000, LENGTH = 32K
}

_security_partition_start = ORIGIN(PERSISTENCE);
_security_partition_end   = ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE);

ASSERT(ORIGIN(FLASH) == 0,
  "EFR32MG22 sensor has no bootloader; the image must start at 0x0");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(ROUTER_TABLES),
  "EFR32MG22 application must end at the router-table journals");
ASSERT(ORIGIN(ROUTER_TABLES) + LENGTH(ROUTER_TABLES) == ORIGIN(PERSISTENCE),
  "EFR32MG22 router-table journals must end at the security partition");
ASSERT(LENGTH(PERSISTENCE) == 16K,
  "EFR32MG22 persistence must remain two 8 KiB sectors");
ASSERT(_security_partition_end == 512K,
  "EFR32MG22 persistence must occupy the top of the 512 KiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20008000,
  "EFR32MG22 RAM must end at the top of the 32 KiB SRAM");
/* Measured stack high-water of the sensor resume path is 11.2 KiB; refuse
 * to link any role whose static RAM leaves less than 14 KiB of stack (the
 * parent-router role needs ~30 KiB of static RAM and does not fit). */
ASSERT(_stack_start - _stack_end >= 14K,
  "EFR32MG22: static RAM leaves less than 14 KiB of stack in the 32 KiB SRAM");
