/* CC2340R52 memory map for the LP-EM-CC2340R5 product.
 *
 * Main flash has 256 physical 2-KiB erase sectors. The final four physical
 * sectors (8 KiB total) are reserved as two logical 4-KiB sectors for the
 * crash-safe SecurityStateJournal.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 504K
    RAM   : ORIGIN = 0x20000000, LENGTH = 36K
}

_security_partition_start = 0x0007E000;
_security_partition_end   = 0x00080000;

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == _security_partition_start,
  "CC2340 application flash must end at the protected security journal");
ASSERT(_security_partition_end - _security_partition_start == 8K,
  "CC2340 security journal must reserve exactly two 4-KiB logical sectors");
ASSERT(_security_partition_end == 512K,
  "CC2340 security journal must occupy the top of the 512-KiB main flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20009000,
  "CC2340R52 RAM region must end at the top of its 36-KiB SRAM");
