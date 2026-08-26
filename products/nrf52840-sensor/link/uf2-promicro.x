MEMORY
{
  MBR_SOFTDEVICE   : ORIGIN = 0x00000000, LENGTH = 152K
  FLASH            : ORIGIN = 0x00026000, LENGTH = 800K
  SECURITY_JOURNAL : ORIGIN = 0x000EE000, LENGTH = 8K
  BOOT_GUARD       : ORIGIN = 0x000F0000, LENGTH = 16K
  BOOTLOADER       : ORIGIN = 0x000F4000, LENGTH = 40K
  BOOT_STATE       : ORIGIN = 0x000FE000, LENGTH = 8K
  RAM              : ORIGIN = 0x20002000, LENGTH = 248K
}

_application_start = ORIGIN(FLASH);
_application_end = ORIGIN(FLASH) + LENGTH(FLASH);
_security_partition_start = ORIGIN(SECURITY_JOURNAL);
_security_partition_end = ORIGIN(SECURITY_JOURNAL) + LENGTH(SECURITY_JOURNAL);
_bootloader_start = ORIGIN(BOOTLOADER);
_boot_state_start = ORIGIN(BOOT_STATE);

ASSERT(ORIGIN(MBR_SOFTDEVICE) + LENGTH(MBR_SOFTDEVICE) == ORIGIN(FLASH),
  "ProMicro application must start after MBR/S140 at 0x26000");
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(SECURITY_JOURNAL),
  "ProMicro application overlaps its security journal");
ASSERT(LENGTH(SECURITY_JOURNAL) == 8K,
  "ProMicro security journal must remain two pages");
ASSERT(ORIGIN(SECURITY_JOURNAL) + LENGTH(SECURITY_JOURNAL) == ORIGIN(BOOT_GUARD),
  "ProMicro journal must end at the preserved 0xF0000 boundary");
ASSERT(ORIGIN(BOOT_GUARD) + LENGTH(BOOT_GUARD) == ORIGIN(BOOTLOADER),
  "ProMicro bootloader guard must end at 0xF4000");
ASSERT(ORIGIN(BOOTLOADER) + LENGTH(BOOTLOADER) == ORIGIN(BOOT_STATE),
  "ProMicro bootloader/config must remain below MBR params");
ASSERT(ORIGIN(BOOT_STATE) + LENGTH(BOOT_STATE) == 0x00100000,
  "ProMicro MBR params/settings must occupy the top two pages");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20040000,
  "ProMicro RAM map must preserve the 0x20002000 origin");
