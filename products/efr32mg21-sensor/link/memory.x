/* Product-selected EFR32MG21 sensor map for BRD4181A/BRD4001A.
 * Part: EFR32MG21A020F512IM32; flash: 512 KiB; RAM: 64 KiB.
 *
 * Keep in sync with src/journal.rs. The board intentionally supplies only
 * raw flash and has no memory-layout or persistence-policy ownership.
 */
MEMORY
{
    BOOTLOADER : ORIGIN = 0x00000000, LENGTH = 16K
    FLASH      : ORIGIN = 0x00004000, LENGTH = 480K
    PERSISTENCE : ORIGIN = 0x0007C000, LENGTH = 16K
    RAM        : ORIGIN = 0x20000000, LENGTH = 64K
}

_security_partition_start = ORIGIN(PERSISTENCE);
_security_partition_end   = ORIGIN(PERSISTENCE) + LENGTH(PERSISTENCE);

ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(FLASH),
  "EFR32MG21 application must begin immediately after the bootloader");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(PERSISTENCE),
  "EFR32MG21 application must end at the protected persistence partition");
ASSERT(LENGTH(PERSISTENCE) == 16K,
  "EFR32MG21 persistence must remain two 8 KiB sectors");
ASSERT(_security_partition_end == 512K,
  "EFR32MG21 persistence must occupy the top of the 512 KiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20010000,
  "EFR32MG21 RAM must end at the top of the 64 KiB SRAM");
