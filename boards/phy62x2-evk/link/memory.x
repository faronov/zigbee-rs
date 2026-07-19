/* PHY62x2 EVK ROM boot layout.
 *
 * The ROM dispatches interrupts through an application table at 0x1fff0000
 * and starts applications from a two-word descriptor loaded at 0x1fff1838.
 * XIP firmware begins after the 0x100-byte PHY6 image header at 0x10100.
 *
 * Keep the XIP slot below 0x30000 so the same firmware image cannot overflow
 * a 256 KiB PHY6252. Persistent sectors are selected separately per chip.
 */
MEMORY
{
    JUMP_TABLE (rw) : ORIGIN = 0x1fff0000, LENGTH = 0x400
    FLASH (rx)  : ORIGIN = 0x11010100, LENGTH = 0x1ff00
    RAM   (rwx) : ORIGIN = 0x1fff1838, LENGTH = 0x0e7c8
}
