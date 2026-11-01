MEMORY
{
  /* nRF52833 (xxAA): 512 KiB flash, 128 KiB RAM. */
  /* Product-owned: the last 8 KiB (pages 126-127, 0x0007E000..0x0007FFFF) */
  /* are reserved for the crash-safe security journal (see src/storage.rs). */
  /* Do not shrink this reservation without verifying the journal's */
  /* sector-rollover margin — the Zigbee frame counters live there and */
  /* reusing a counter after a power loss breaks network security. */
  /* 0x0007A000..0x0007DFFF (pages 122-125) hold the router role's
   * child-table and APS journals (src/router.rs). Every role image (sensor
   * SED, End Device, Router) links this same map, so no role's code can
   * overlap another role's persistence. */
  FLASH         : ORIGIN = 0x00000000, LENGTH = 488K
  ROUTER_TABLES : ORIGIN = 0x0007A000, LENGTH = 16K
  RAM           : ORIGIN = 0x20000000, LENGTH = 128K
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
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(ROUTER_TABLES),
  "nRF52833 application must end at the router-table journals");
ASSERT(ORIGIN(ROUTER_TABLES) + LENGTH(ROUTER_TABLES) == _security_partition_start,
  "nRF52833 router-table journals must end at the security journal");
ASSERT(_security_partition_end == 512K,
  "nRF52833 security journal must occupy the top of the 512 KiB flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20020000,
  "nRF52833 RAM region must end at the top of the part's 128 KiB SRAM");
ASSERT(_stack_start - __euninit >= 16K,
  "nRF52833 static data must leave at least 16 KiB for the stack");
