/* PHY62x2 sensor product ROM boot and XIP slot layout.
 *
 * The ROM dispatches interrupts through an application table at 0x1fff0000
 * and starts applications from a two-word descriptor loaded at 0x1fff1838.
 * XIP firmware begins after the 0x100-byte PHY6 image header at 0x10100.
 *
 * The PHY6 image (header at flash 0x10000) must end below 0x3e000, the
 * PHY6252 security partition; both variants share this layout, so PHY6222
 * leaves 0x3e000..0x7dfff unused. Nothing else lives in 0x10100..0x3dfff:
 * the product has no OTA bank, and the pvvx THB2 FW Boot, which loads the
 * PHY6 format, limits only SRAM segments (not the XIP application).
 *
 * phy62x2-image appends the ROM-loaded SRAM sections (0x400-byte jump table,
 * 8-byte run descriptor), padding to 16 bytes and a 4-byte CRC after the
 * last XIP byte, so the XIP slot is 0x10100..0x3e000 (0x2df00) minus a
 * 0x420-byte tail: 0x2dae0 = 187,104 bytes. Product storage reserves the
 * final two flash sectors at 0x7e000/0x7f000 (PHY6222) or 0x3e000/0x3f000
 * (PHY6252).
 */
MEMORY
{
    JUMP_TABLE (rw) : ORIGIN = 0x1fff0000, LENGTH = 0x400
    FLASH (rx)  : ORIGIN = 0x11010100, LENGTH = 0x2dae0
    RAM   (rwx) : ORIGIN = 0x1fff1838, LENGTH = 0x0e7c8
}
