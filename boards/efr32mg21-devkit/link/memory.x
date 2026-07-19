/* EFR32MG21 Memory Layout
 * Flash: 512KB (some variants: 1024KB) at 0x0000_0000
 * RAM:   64KB  at 0x2000_0000
 *
 * The EFR32MG21 maps flash at 0x00000000 and SRAM at 0x20000000.
 * First 16KB of flash is reserved for the bootloader (Gecko Bootloader).
 * Adjust ORIGIN/LENGTH based on your specific bootloader configuration.
 */
MEMORY
{
    /* Final 16 KiB at 0x7C000 is reserved for application NV. */
    FLASH : ORIGIN = 0x00004000, LENGTH = 480K
    RAM   : ORIGIN = 0x20000000, LENGTH = 64K
}
