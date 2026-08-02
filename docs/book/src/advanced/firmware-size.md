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
| TLSR8258 end-device sensor (software AES) | 272,600 B | — | 280 KiB |
| TLSR8258 parent router (software AES) | 332,440 B | — | 336 KiB |
| BL702 end-device sensor (software AES) | 165,602 B | 173,808 B | 192 KiB |
| BL702 end-device sensor (hardware AES) | 161,570 B | 169,776 B | 192 KiB |
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
| End-device sensor | 323,876 B | 272,600 B | 51,276 B (15.8%) |
| Parent router | 349,792 B | 332,440 B | 17,352 B (5.0%) |

The current router is 59,840 bytes larger than the sensor because it retains
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
- BL702 hardware-AES variant rejects the RustCrypto software AES core and
  requires the SEC_ENG backend, while re-running the RV32A/vendor/XIP gates;
- TLSR8258 RAM-code, cache, BSS, DMA, and stack layout.

Oversized ZCL responses are dropped whole rather than truncated into malformed
frames. Cluster-specific responses have a compile-time proof that their
64-byte payload plus header fits the 128-byte pending-response buffer.

## Hardware-gated and opt-in reductions

The TLSR8258 hardware-AES provider is now proven on a TB-04 router: two
startup known-answer tests, CCM*, AES-MMO, Request-Key, Verify-Key/Confirm-Key,
a complete ZHA interview, more than ten minutes of secured traffic, and
reset/resume all passed under an independent channel-15 capture. The opt-in
images are 269,940 bytes for the sensor and 327,716 bytes for the router,
saving 2,660 and 4,724 bytes respectively with 8 additional bytes of RAM.

Software AES remains the release default. The hardware provider is deliberately
selected per product with `hardware-aes`, fails closed without a software
fallback, and keeps a separately named recovery image.

The BL702 SEC_ENG hardware-AES provider is the first cross-platform follow-up
to the TLSR8258 work. It is **compile/build-proven only** — it has **not** been
run on BL702 silicon. Its register contract is transcribed from the
open-source Bouffalo `bl_iot_sdk` SEC_ENG driver and cross-checked against a
FIPS-197 known-answer vector, its two startup known-answer tests fail closed,
and the release image links the SEC_ENG backend while dropping the RustCrypto
software AES core (CI rejects any `aes::soft::fixslice` symbol and requires the
`HardwareAes128`/`bl702_hal::aes` backend). In a local nightly build the opt-in
image measured 161,570 bytes versus 165,602 bytes for the same-toolchain
software build, saving 4,032 bytes and no additional RAM. On-silicon
functional and timing validation (including SEC_ENG DMA/SRAM coherency) remains
an open hardware gate.

Compact TLSR8258 text placement, linker tail merging, and identical-code
folding are also deferred until hardware soak testing confirms startup,
interrupt, RAM-code, persistence, and OTA behavior. R23/BDB 3.1 remains a
separate optional roadmap item and must add no code or RAM to these R22
images.
