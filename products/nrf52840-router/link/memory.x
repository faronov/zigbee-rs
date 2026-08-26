MEMORY
{
  /* nRF52840: 1 MiB flash, 256 KiB RAM. */
  FLASH            : ORIGIN = 0x00000000, LENGTH = 1016K
  SECURITY_JOURNAL : ORIGIN = 0x000FE000, LENGTH = 8K
  RAM              : ORIGIN = 0x20000000, LENGTH = 256K
}

/* Product/layout agreement. cortex-m-rt separately rejects an application
 * that does not fit in FLASH; these assertions reject edits that move or
 * shrink the protected two-page journal itself. Keep in sync with
 * src/storage.rs. */
_security_partition_start = ORIGIN(SECURITY_JOURNAL);
_security_partition_end   = ORIGIN(SECURITY_JOURNAL) + LENGTH(SECURITY_JOURNAL);

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(SECURITY_JOURNAL),
  "nRF52840 relay application must end exactly at the security journal");
ASSERT(ORIGIN(SECURITY_JOURNAL) == 0x000FE000,
  "nRF52840 relay security journal must start at 0x000FE000");
ASSERT(LENGTH(SECURITY_JOURNAL) == 8K,
  "nRF52840 relay security journal must remain two 4 KiB pages");
ASSERT(_security_partition_end == 0x00100000,
  "nRF52840 relay security journal must occupy the top of flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20040000,
  "nRF52840 relay RAM must end at the top of the 256 KiB SRAM");
