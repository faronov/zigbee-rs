# Firmware Size and Role Specialization

zigbee-rs builds each product for the behavior it can actually perform.
Logical roles are Rust types, optional data types are Cargo features, and
bounded capacities are selected at compile time. This keeps a sleepy sensor
from linking parent, child-table, routing, or router-maintenance code merely
because those capabilities exist elsewhere in the stack.

## Current release measurements

The following measurements are raw release payloads produced by the repository
build scripts. Packaged boot images are listed separately because headers and
flash offsets are platform-specific.

| Platform and role | Raw payload | Packaged image | CI raw budget |
|---|---:|---:|---:|
| TLSR8258 end-device sensor | 272,148 B | — | 280 KiB |
| TLSR8258 parent router | 331,852 B | — | 336 KiB |
| BL702 end-device sensor | 165,058 B | 173,264 B | 192 KiB |
| nRF52840 end-device sensor | 202,688 B | — | 220 KiB |
| nRF52840 relay router | 192,704 B | — | — |
| nRF52833 end-device sensor | 125,544 B | — | — |
| EFR32MG1 end-device sensor | 131,776 B | — | 160 KiB |

These are not benchmark-equivalent applications. Radio implementations,
linker layouts, executor overhead, enabled peripherals, and application
profiles differ. Compare a build against its own previous measurement and CI
budget rather than ranking chips by the table alone.

The pinned `tc32-45` Telink builds provide the cleanest before/after
comparison:

| TLSR8258 image | Complete-HAL baseline | Current | Reduction |
|---|---:|---:|---:|
| End-device sensor | 323,876 B | 272,148 B | 51,728 B (16.0%) |
| Parent router | 349,792 B | 331,852 B | 17,940 B (5.1%) |

The current router is 59,704 bytes larger than the sensor because it retains
the behavior a real parent needs: route maintenance, child admission and
aging, indirect delivery, parent-side MAC commands, Update-Device handling,
and Parent Announce. The sensor instead retains the R22 End Device Timeout
client and polling lifecycle. GitHub's rounded artifact display may show both
as roughly `0.3 MB`, but the raw binaries are no longer close in size.

## What is specialized

- `EndDevice`, `RelayRouter`, and `Router` are distinct logical role types.
  Parent construction additionally requires a `ParentMacDriver`.
- Parent/router maintenance is statically dispatched. Sensor futures do not
  materialize parent-only async call graphs.
- Role-owned state is separate: `EndDeviceState` contains the R22 timeout
  client, `RelayRouter` uses zero-sized state, and `ParentState` contains the
  bounded parent queues and flags.
- Local ZCL parsing, foundation commands, cluster dispatch, reporting
  configuration, and response construction run in a synchronous,
  `MacDriver`-independent dispatcher rather than inside the generic async
  receive state machine.
- APS decryption, joined-tick tails, and receive metadata use shared
  non-generic helpers where measurement showed a real reduction.
- Small bounded descriptor sets use compact insertion ordering instead of
  linking generic slice sorting.
- `float32` and `float64` ZCL wire support are compile-time capabilities.
  Integer-only products reject disabled wire types without linking their
  conversion code.
- Constrained products can select smaller bounded runtime tables without
  changing protocol behavior.

Size work is retained only when both role images improve or a deliberate
role-specific tradeoff is justified. Async outlining that merely moves a
future constructor is not assumed to help: on TC32 it often nests another poll
state machine and increases flash.

## Correctness and regression gates

Size reductions do not weaken the active Zigbee PRO R22 / BDB 3.0.1 baseline.
The workspace tests keep unsupported-ZDP behavior, End Device Timeout,
parent/child aging, Parent Announce, persistence migration, APS replay/MIC
ordering, and ZCL Write Attributes ordering and atomicity covered.

CI also checks properties that a successful link alone cannot prove:

- per-target JSON size budgets;
- absence of generic slice sorting in constrained images;
- absence of parent/router symbols from sensors;
- presence of R22 End Device Timeout client symbols in sensors and their
  absence from routers/relays;
- BL702 absence of RV32A instructions and vendor radio symbols;
- BL702 `_start_rust` placement in XIP flash;
- TLSR8258 RAM-code, cache, BSS, DMA, and stack layout.

Oversized ZCL responses are dropped whole rather than truncated into malformed
frames. Cluster-specific responses have a compile-time proof that their
64-byte payload plus header fits the 128-byte pending-response buffer.

## Deferred hardware-gated reductions

The software AES implementation still contributes several KiB. TLSR8258 has
an AES accelerator and a Rust HAL driver, but it will not become the default
CCM* provider until real-silicon known-answer, secured join, sustained traffic,
and sleep/resume tests pass.

Compact TLSR8258 text placement, linker tail merging, and identical-code
folding are also deferred until hardware soak testing confirms startup,
interrupt, RAM-code, persistence, and OTA behavior. R23/BDB 3.1 remains a
separate optional roadmap item and must add no code or RAM to these R22
images.
