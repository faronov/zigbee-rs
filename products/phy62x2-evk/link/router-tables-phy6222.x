/* PHY6222 EVK router-only layout guard (passed with -T by the example's
 * build.rs for the `phy6222-router` bin only; memory.x, phy6222.x and the
 * sensor/End Device images are unchanged).
 *
 * The router keeps its child-table and APS journals at 0x7A000..0x7E000
 * (src/router.rs), in flash the PHY6222 does not otherwise use, so the
 * shared 187,104 B XIP slot and its 0x3E000 image limit (phy6222.x) still
 * apply. Its static RAM must leave 16 KiB of stack.
 *
 * The router does not fit: see examples/phy6222-sensor/src/router.rs for
 * the measured numbers. Shrinking routing/neighbour/child tables is a
 * product decision and is not done silently.
 */
_router_tables_start = 0x0007A000;

ASSERT(__xip_occupied_end + SIZEOF(.jump_table) + SIZEOF(.vector_table) + 0x18 <= 0x11000000 + _router_tables_start,
  "PHY6222 router: PHY6 image overlaps the router-table journals at 0x7A000");
ASSERT(_stack_start - __euninit >= 16K,
  "PHY6222 router: static RAM leaves less than 16 KiB of stack");
