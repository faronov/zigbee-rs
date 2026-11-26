/* Lab-only BRD4187 map (EFR32MG24B220F1536IM48 on a BRD4001A) for a radio
 * board without a bootloader: the image starts at the flash base
 * 0x0800_0000 and stops at the product NV journals (0x0815_8000), so it
 * never overlaps product persistence. Do not flash it over a Gecko
 * bootloader you want to keep.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 0x158000
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) <= 0x08158000,
  "BRD4187 lab image must stay below the product NV journals");
