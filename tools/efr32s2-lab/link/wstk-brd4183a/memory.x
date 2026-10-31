/* Lab-only EFR32MG22C224F512IM40 (BRD4183A) map: no bootloader, image at 0.
 * The top 16 KiB are left untouched so a lab image never erases the
 * persistence partition used by the product layout.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 496K
    RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}
