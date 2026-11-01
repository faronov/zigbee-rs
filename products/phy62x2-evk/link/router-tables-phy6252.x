/* PHY6252 EVK router-only layout guard (passed with -T by the example's
 * build.rs for the `phy6222-router` bin built with `phy6252` only;
 * memory.x, phy6222.x and the sensor/End Device images are unchanged).
 *
 * The 256 KiB part has no free flash outside the shared layout, so the
 * router's child-table and APS journals take 0x3A000..0x3E000 from the end
 * of the XIP slot (src/router.rs): the router's PHY6 image must end below
 * 0x3A000, i.e. 170,720 B of XIP instead of 187,104 B. Its static RAM must
 * leave 16 KiB of stack.
 *
 * The router does not fit: see examples/phy6222-sensor/src/router.rs for
 * the measured numbers. Shrinking routing/neighbour/child tables is a
 * product decision and is not done silently.
 */
_router_tables_start = 0x0003A000;

ASSERT(__xip_occupied_end + SIZEOF(.jump_table) + SIZEOF(.vector_table) + 0x18 <= 0x11000000 + _router_tables_start,
  "PHY6252 router: PHY6 image overlaps the router-table journals at 0x3A000 (router XIP slot 170720 B)");
ASSERT(_stack_start - __euninit >= 16K,
  "PHY6252 router: static RAM leaves less than 16 KiB of stack");
