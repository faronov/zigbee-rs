/* Lab-only Home Assistant Connect ZBT-1 map (EFR32MG21A020F1024IM32).
 * The resident Gecko Bootloader at 0x0..0x4000 is never part of the image;
 * the lab application stops 16 KiB below the top of flash so it can never
 * overlap a product persistence partition.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00004000, LENGTH = 1024K - 16K - 16K
    RAM   : ORIGIN = 0x20000000, LENGTH = 96K
}

ASSERT(ORIGIN(FLASH) >= 0x4000, "ZBT-1 lab image must not overlap the Gecko Bootloader");
