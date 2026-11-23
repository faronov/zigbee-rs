/* Lab-only RF-BM-MG24B1 map (EFR32MG24A410F1536IM48) for a module without
 * a bootloader: the image starts at the flash base 0x0800_0000 and stops at
 * the product NV journals (0x0815_8000), so it never overlaps product
 * persistence. Do not flash it over a Gecko bootloader you want to keep.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 0x158000
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) <= 0x08158000,
  "RF-BM-MG24B1 lab image must stay below the product NV journals");
