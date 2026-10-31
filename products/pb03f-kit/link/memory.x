/* PB-03F-Kit (PHY6252, 512 KiB flash) ROM-boot and XIP layout.
 *
 * No second-stage bootloader. `phy62x2-image --rom-boot` emits two flash
 * regions so the factory sectors between them are never erased:
 *   <elf>.table.bin  one 4 KiB sector programmed at flash 0x2000
 *   <elf>.app.bin    SRAM load data + XIP text programmed at flash 0x5000
 * The boot ROM reads the table, copies the jump table to 0x1fff0000 and the
 * two-word [sp][reset] run descriptor to 0x1fff1838, then starts Reset.
 *
 * Flash map:
 *   0x00000-0x01fff  ROM/factory area - never written
 *   0x02000-0x02fff  boot-ROM application table (header 0x2000, entries
 *                    from 0x2100)
 *   0x03000-0x04fff  factory data (0x3000 boot/OTA record, 0x4000 BLE
 *                    public address used for the EUI-64) - never written
 *   0x05000-0x100ff  SRAM load data (jump table, run descriptor)
 *   0x10100-0x40fff  XIP application slot (200448 bytes; grown by one
 *                    4 KiB sector for the shared SoftMacCore MAC)
 *   0x41000-0x7dfff  unused; reserved for OTA staging (244 KiB, larger than
 *                    the slot so a full image always fits)
 *   0x7e000-0x7ffff  product security journal (two sectors)
 */
MEMORY
{
    JUMP_TABLE (rw) : ORIGIN = 0x1fff0000, LENGTH = 0x400
    FLASH (rx)  : ORIGIN = 0x11010100, LENGTH = 0x30f00
    RAM   (rwx) : ORIGIN = 0x1fff1838, LENGTH = 0x0e7c8
}
