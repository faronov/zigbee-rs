/* Product-selected EFR32MG22 sensor map for BRD4183A/BRD4001A.
 * Part: EFR32MG22C224F512IM40; flash: 512 KiB (8 KiB pages); RAM: 32 KiB.
 *
 * No resident bootloader: the Secure Engine starts the application vector
 * table at 0x0. Keep in sync with src/journal.rs. The board supplies only
 * raw flash and owns no memory layout or persistence policy.
 */
MEMORY
{
    FLASH       : ORIGIN = 0x00000000, LENGTH = 496K
    PERSISTENCE : ORIGIN = 0x0007C000, LENGTH = 16K
    RAM         : ORIGIN = 0x20000000, LENGTH = 32K
}

_security_partition_start = ORIGIN(PERSISTENCE);
_security_partition_end   = ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE);

ASSERT(ORIGIN(FLASH) == 0,
  "EFR32MG22 sensor has no bootloader; the image must start at 0x0");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(PERSISTENCE),
  "EFR32MG22 application must end at the protected persistence partition");
ASSERT(LENGTH(PERSISTENCE) == 16K,
  "EFR32MG22 persistence must remain two 8 KiB sectors");
ASSERT(_security_partition_end == 512K,
  "EFR32MG22 persistence must occupy the top of the 512 KiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20008000,
  "EFR32MG22 RAM must end at the top of the 32 KiB SRAM");
