/* PB-03F-Kit router-only layout guard (passed with -T by the example's
 * build.rs for the `pb03f-router` bin only; memory.x, pb03f.x and the other
 * images are unchanged).
 *
 * The router keeps its child-table and APS journals at 0x7A000..0x7E000
 * (src/router.rs), the top of the reserved OTA staging area, so the
 * 200,448 B XIP slot (pb03f.x) still applies. Its static RAM must leave
 * 16 KiB of stack.
 *
 * The router does not fit: see examples/pb03f-kit/src/bin/router.rs for
 * the measured numbers. Shrinking routing/neighbour/child tables is a
 * product decision and is not done silently.
 */
_router_tables_start = 0x0007A000;

ASSERT(_stack_start - __euninit >= 16K,
  "PB03F router: static RAM leaves less than 16 KiB of stack");
