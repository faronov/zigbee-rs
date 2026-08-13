MEMORY
{
  /* nRF52833 (xxAA): 512 KiB flash, 128 KiB RAM. */
  /* Product-owned: the last 8 KiB (pages 126-127, 0x0007E000..0x0007FFFF) */
  /* are reserved for the crash-safe security journal (see src/storage.rs). */
  /* Do not shrink this reservation without verifying the journal's */
  /* sector-rollover margin — the Zigbee frame counters live there and */
  /* reusing a counter after a power loss breaks network security. */
  FLASH : ORIGIN = 0x00000000, LENGTH = 504K
  RAM   : ORIGIN = 0x20000000, LENGTH = 128K
}

/* Post-link boundary checks.
 *
 * `cortex-m-rt`'s own link script already fails the build if the image does
 * not fit in FLASH; these assertions additionally fail the build if this
 * region definition itself is ever edited in a way that would let the
 * application be linked over the protected security journal, or past the
 * end of the part's real memories. Both are silent-data-loss classes of
 * mistake: the first destroys the persisted network key and frame
 * counters, the second produces an image that only faults on hardware.
 *
 * Keep in sync with `SECURITY_PARTITION_START` / `SECURITY_PARTITION_SIZE`
 * in src/storage.rs, which assert the same partition from the Rust side. */
_security_partition_start = 0x0007E000;
_security_partition_end   = 0x00080000;

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) <= _security_partition_start,
  "nRF52833 application flash overlaps the protected security journal partition");
ASSERT(_security_partition_end == 512K,
  "nRF52833 security journal must occupy the top of the 512 KiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20020000,
  "nRF52833 RAM region must end at the top of the part's 128 KiB SRAM");
