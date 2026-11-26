/* Lab-only EK2703A map (EFR32MG24B210F1536IM48 on the xG24 Explorer Kit)
 * for a kit without a bootloader (the factory demo starts at the flash
 * base): the image starts at 0x0800_0000 and stops at the product NV
 * journals (0x0815_8000), so it never overlaps product persistence. Do not
 * flash it over a Gecko bootloader you want to keep.
 */
MEMORY
{
    FLASH : ORIGIN = 0x08000000, LENGTH = 0x158000
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) <= 0x08158000,
  "EK2703A lab image must stay below the product NV journals");
