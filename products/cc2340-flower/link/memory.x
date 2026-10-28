/* CC2340R52 memory map for the flower soil-sensor product.
 *
 * Main flash has 256 physical 2-KiB erase sectors. From the top:
 *   0x7E000..0x80000  two logical 4-KiB SecurityStateJournal sectors
 *   0x7D800..0x7E000  one 2-KiB sector holding the detected I2C sensor set
 *   0x00000..0x7D800  application image (flashed at 0, no bootloader yet)
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 502K
    RAM   : ORIGIN = 0x20000000, LENGTH = 36K
}

_detection_partition_start = 0x0007D800;
_security_partition_start  = 0x0007E000;
_security_partition_end    = 0x00080000;

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == _detection_partition_start,
  "flower application flash must end at the sensor-detection sector");
ASSERT(_security_partition_start - _detection_partition_start == 2K,
  "flower sensor-detection record must own exactly one 2-KiB erase sector");
ASSERT(_security_partition_end - _security_partition_start == 8K,
  "CC2340 security journal must reserve exactly two 4-KiB logical sectors");
ASSERT(_security_partition_end == 512K,
  "CC2340 security journal must occupy the top of the 512-KiB main flash");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20009000,
  "CC2340R52 RAM region must end at the top of its 36-KiB SRAM");
ASSERT(_stack_start - __euninit >= 16K,
  "CC2340 static task storage must leave at least 16 KiB for the stack");
