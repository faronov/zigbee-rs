/* LP-EM-CC2340R5 router-only layout guard (passed with -T by the example's
 * build.rs for the `cc2340-router` bin only; `memory.x` and the sensor/End
 * Device images are unchanged).
 *
 * The router keeps its child-table and APS journals at 0x7A000..0x7E000
 * (src/router.rs), so its application must end at 0x7A000, below the shared
 * security journal at 0x7E000. memory.x already holds every image to a
 * 16 KiB stack; the router repeats it with its own message.
 *
 * The router does not fit: see examples/cc2340-sensor/src/router.rs for the
 * measured RAM numbers. Shrinking routing/neighbour/child tables is a
 * product decision and is not done silently.
 */
_router_tables_start = 0x0007A000;

ASSERT(__sidata + SIZEOF(.data) <= _router_tables_start,
  "CC2340 router: application overlaps the router-table journals at 0x7A000 (app window 0x0..0x7A000 = 488 KiB)");
ASSERT(_stack_start - __euninit >= 16K,
  "CC2340 router: static RAM leaves less than 16 KiB of stack in the 36 KiB SRAM");
