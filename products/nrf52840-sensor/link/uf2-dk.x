MEMORY
{
  FLASH            : ORIGIN = 0x00000000, LENGTH = 1016K
  SECURITY_JOURNAL : ORIGIN = 0x000FE000, LENGTH = 8K
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

_application_start = ORIGIN(FLASH);
_application_end = ORIGIN(FLASH) + LENGTH(FLASH);
_security_partition_start = ORIGIN(SECURITY_JOURNAL);
_security_partition_end = ORIGIN(SECURITY_JOURNAL) + LENGTH(SECURITY_JOURNAL);
_bootloader_start = 0x00100000;
_boot_state_start = 0x00100000;

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(SECURITY_JOURNAL),
  "nRF52840 DK application overlaps its security journal");
ASSERT(LENGTH(SECURITY_JOURNAL) == 8K,
  "nRF52840 DK security journal must remain two pages");
ASSERT(ORIGIN(SECURITY_JOURNAL) + LENGTH(SECURITY_JOURNAL) == 0x00100000,
  "nRF52840 DK journal must occupy the top two pages");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20040000,
  "nRF52840 DK RAM map must cover the full 256 KiB");
