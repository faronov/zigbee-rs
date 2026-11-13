/* Product-selected EFR32MG22 Green Power Device map for BRD4183A/BRD4001A.
 * Part: EFR32MG22C224F512IM40; flash: 512 KiB (8 KiB pages); RAM: 32 KiB.
 *
 * No resident bootloader: the Secure Engine starts the application vector
 * table at 0x0. Each device kind has its own GPD journal (SrcID, key,
 * frame-counter reservation, channel) of two 8 KiB sectors: switch
 * 0x70000..0x74000, sensor 0x74000..0x78000. 0x78000..0x80000 is never
 * written by this product: 0x7C000 keeps the Zigbee sensor product's
 * security journal so a later reflash of the sensor resumes its network.
 * Keep in sync with src/lib.rs.
 */
MEMORY
{
    FLASH         : ORIGIN = 0x00000000, LENGTH = 448K
    GPD_JOURNAL   : ORIGIN = 0x00070000, LENGTH = 32K
    UNTOUCHED     : ORIGIN = 0x00078000, LENGTH = 32K
    RAM           : ORIGIN = 0x20000000, LENGTH = 32K
}

ASSERT(ORIGIN(FLASH) == 0,
  "EFR32MG22 GPD has no bootloader; the image must start at 0x0");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(GPD_JOURNAL),
  "EFR32MG22 GPD application must end at the GPD journal");
ASSERT(LENGTH(GPD_JOURNAL) == 32K,
  "EFR32MG22 GPD journals must remain two 8 KiB sectors per device kind");
ASSERT(ORIGIN(GPD_JOURNAL) + LENGTH(GPD_JOURNAL) == ORIGIN(UNTOUCHED),
  "EFR32MG22 GPD journal must end where the untouched range starts");
ASSERT(ORIGIN(UNTOUCHED) + LENGTH(UNTOUCHED) == 512K,
  "EFR32MG22 untouched range must reach the top of the 512 KiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20008000,
  "EFR32MG22 RAM must end at the top of the 32 KiB SRAM");
ASSERT(_stack_start - _stack_end >= 16K,
  "EFR32MG22 GPD: static RAM leaves less than 16 KiB of stack");
