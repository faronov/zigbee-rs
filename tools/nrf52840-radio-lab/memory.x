/* nRF52840 Dongle (PCA10059) with Nordic's factory Open USB DFU bootloader.
 *
 * The factory image keeps the MBR in the first page, the bootloader at
 * 0xE0000 and the MBR parameter and bootloader settings pages at the top of
 * flash. The MBR reserves the first 8 bytes of RAM. Nordic documents the
 * application window as 0x1000..0xE0000 with RAM from 0x20000008.
 */
MEMORY
{
  MBR        : ORIGIN = 0x00000000, LENGTH = 4K
  FLASH      : ORIGIN = 0x00001000, LENGTH = 0xDF000
  BOOTLOADER : ORIGIN = 0x000E0000, LENGTH = 0x1E000
  BOOT_STATE : ORIGIN = 0x000FE000, LENGTH = 8K
  RAM        : ORIGIN = 0x20000008, LENGTH = 0x3FFF8
}

ASSERT(ORIGIN(MBR) + LENGTH(MBR) == ORIGIN(FLASH),
  "PCA10059 Open DFU application must preserve the MBR page");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(BOOTLOADER),
  "PCA10059 Open DFU application must stop at the 0xE0000 bootloader");
ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(BOOT_STATE),
  "PCA10059 Open DFU bootloader must remain below the MBR params page");
ASSERT(ORIGIN(BOOT_STATE) + LENGTH(BOOT_STATE) == 0x00100000,
  "PCA10059 MBR params and bootloader settings occupy the top two pages");
ASSERT(ORIGIN(RAM) == 0x20000008,
  "PCA10059 Open DFU application RAM must start after the MBR word");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20040000,
  "PCA10059 RAM map must end at 256 KiB");
ASSERT(_stack_start - __euninit >= 16K,
  "nRF52840 radio lab static data must leave at least 16 KiB for the stack");
