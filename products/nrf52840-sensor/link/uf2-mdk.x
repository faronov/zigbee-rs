MEMORY
{
  MBR              : ORIGIN = 0x00000000, LENGTH = 4K
  FLASH            : ORIGIN = 0x00001000, LENGTH = 964K
  SECURITY_JOURNAL : ORIGIN = 0x000F2000, LENGTH = 8K
  BOOTLOADER       : ORIGIN = 0x000F4000, LENGTH = 40K
  BOOT_STATE       : ORIGIN = 0x000FE000, LENGTH = 8K
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

_application_start = ORIGIN(FLASH);
_application_end = ORIGIN(FLASH) + LENGTH(FLASH);
_security_partition_start = ORIGIN(SECURITY_JOURNAL);
_security_partition_end = ORIGIN(SECURITY_JOURNAL) + LENGTH(SECURITY_JOURNAL);
_bootloader_start = ORIGIN(BOOTLOADER);
_boot_state_start = ORIGIN(BOOT_STATE);

ASSERT(ORIGIN(MBR) + LENGTH(MBR) == ORIGIN(FLASH),
  "MDK UF2 application must preserve the MBR page");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(SECURITY_JOURNAL),
  "MDK UF2 application overlaps its security journal");
ASSERT(LENGTH(SECURITY_JOURNAL) == 8K,
  "MDK UF2 security journal must remain two pages");
ASSERT(ORIGIN(SECURITY_JOURNAL) + LENGTH(SECURITY_JOURNAL) == ORIGIN(BOOTLOADER),
  "MDK UF2 journal must stop at the 0xF4000 bootloader");
ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(BOOT_STATE),
  "MDK UF2 bootloader/config must remain below MBR params");
ASSERT(ORIGIN(BOOT_STATE) + LENGTH(BOOT_STATE) == 0x00100000,
  "MDK UF2 MBR params/settings must occupy the top two pages");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20040000,
  "MDK UF2 RAM map must cover the full 256 KiB");
