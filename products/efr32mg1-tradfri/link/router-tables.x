/* EFR32MG1P TRADFRI router-only layout guard (passed with -T by the
 * example's build.rs for the `efr32mg1-router` bin only; `memory.x` and the
 * sensor/End Device images are unchanged).
 *
 * The router keeps its child-table and APS journals at 0x33000..0x37000
 * (src/router.rs), so its application must end at 0x33000, below the
 * shared security journal at 0x37000. Its static RAM must also leave the
 * same 16 KiB stack the production sensor is held to (check-layout.sh).
 *
 * The router does not fit: see examples/efr32mg1-sensor/src/router.rs for
 * the measured flash and RAM numbers. Shrinking routing/neighbour/child
 * tables is a product decision and is not done silently.
 */
_router_tables_start = 0x00033000;

ASSERT(__sidata + SIZEOF(.data) <= _router_tables_start,
  "EFR32MG1 router: application overlaps the router-table journals at 0x33000 (app window 0x4000..0x33000 = 188 KiB)");
ASSERT(_stack_start - _stack_end >= 16K,
  "EFR32MG1 router: static RAM leaves less than 16 KiB of stack in the 0x7C00-byte SRAM");
