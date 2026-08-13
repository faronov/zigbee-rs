MEMORY
{
  /* nRF52840: 1 MiB flash, 256 KiB RAM. */
  /* Product-owned: the last 8 KiB (pages 254-255) are reserved for the */
  /* crash-safe security journal (see src/storage.rs). Do not shrink this */
  /* reservation without verifying the journal's sector-rollover margin. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1016K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}

/* Post-link boundary checks (link-time only — they emit no code and leave
 * the produced image byte-identical).
 *
 * `cortex-m-rt`'s own link script already fails the build if the image does
 * not fit in FLASH; these assertions additionally fail the build if this
 * region definition itself is ever edited in a way that would let the
 * application be linked over the protected security journal, which would
 * destroy the persisted network key and frame counters.
 *
 * Keep in sync with `SECURITY_PARTITION_START` / `SECURITY_PARTITION_SIZE`
 * in src/storage.rs, which assert the same partition from the Rust side. */
_security_partition_start = 0x000FE000;
_security_partition_end   = 0x00100000;

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) <= _security_partition_start,
  "nRF52840 application flash overlaps the protected security journal partition");
ASSERT(_security_partition_end == 1024K,
  "nRF52840 security journal must occupy the top of the 1 MiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20040000,
  "nRF52840 RAM region must end at the top of the part's 256 KiB SRAM");
