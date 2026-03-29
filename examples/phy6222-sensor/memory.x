/* PHY6222/6252 Memory Layout
 * Flash: 512KB at 0x1100_0000
 * SRAM:  64KB  at 0x1FFF_0000
 *
 * The PHY6222 maps flash at 0x11000000 and SRAM at 0x1FFF0000.
 * First 4KB of flash reserved for bootloader/OTA header.
 * Adjust based on your specific firmware layout.
 */
MEMORY
{
    FLASH : ORIGIN = 0x11001000, LENGTH = 508K
    RAM   : ORIGIN = 0x1FFF0000, LENGTH = 64K
}
