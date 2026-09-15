/* PHY62x2 sensor product ROM boot and XIP slot layout.
 *
 * The ROM dispatches interrupts through an application table at 0x1fff0000
 * and starts applications from a two-word descriptor loaded at 0x1fff1838.
 * XIP firmware begins after the 0x100-byte PHY6 image header at 0x10100.
 *
 * The application slot is a hard 130816-byte limit. Product storage reserves
 * the final two flash sectors separately at the unchanged chip-specific
 * 0x7e000/0x7f000 (PHY6222) or 0x3e000/0x3f000 (PHY6252) addresses.
 */
MEMORY
{
    JUMP_TABLE (rw) : ORIGIN = 0x1fff0000, LENGTH = 0x400
    FLASH (rx)  : ORIGIN = 0x11010100, LENGTH = 0x1ff00
    RAM   (rwx) : ORIGIN = 0x1fff1838, LENGTH = 0x0e7c8
}
