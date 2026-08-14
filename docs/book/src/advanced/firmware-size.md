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
| TLSR8258 end-device sensor (hardware AES) | 271,836 B | — | 280 KiB |
| TLSR8258 parent router (hardware AES) | 328,396 B | — | 336 KiB |
| BL702 end-device sensor (hardware AES) | 178,082 B | — | 192 KiB |
| nRF52840 end-device sensor | 222,336 B | — | 220 KiB |
| nRF52840 relay router | 225,560 B | — | — |
| nRF52833 end-device sensor | 222,336 B | — | 220 KiB |
| EFR32MG1 end-device sensor (hardware AES + OTA) | 151,284 B | — | 160 KiB |

The TLSR8258 rows are `scripts/tlsr8258.sh build sensor` / `build router` on
the pinned `tc32-45` toolchain, which is exactly what CI builds; they leave
14,884 and 15,668 bytes of budget headroom respectively. The BL702, nRF and
EFR32 raw payloads were re-measured with the same release profiles CI builds.
Packaged images were not rebuilt. CI remains the authoritative source per
commit, because it enforces the budget column with
`tools/firmware-size-report.sh`.

R22 conformance work moves these numbers in both directions. Adding NWK address
and PAN identifier conflict resolution (R22 §3.6.1.9, §3.6.1.13) grew the
router image, while restricting route discovery, Route Reply and Route Record
processing to devices that can actually route (R22 §3.6.3.5.2) removed that
code from every end-device image. Both images then dropped well below their
pre-conformance size once the runtime stopped folding its maintenance,
commissioning and receive coroutines into two enormous ones (see
[coroutine outlining](#coroutine-outlining-on-tc32)):

| TLSR8258 image | Before router maintenance | With R22 conflict/Link Status work | After outlining |
|---|---:|---:|---:|
| End-device sensor | 286,288 B | 286,512 B | 271,836 B |
| Parent router | 338,276 B | 346,280 B | 328,396 B |

The middle column is why this matters: the router exceeded its 344,064-byte
budget by 2,216 bytes and the sensor had 208 bytes left. No R22 behavior was
removed to recover the budget.

### RAM and stack alongside these payloads

Flash is not the only budget. The same builds report:

| TLSR8258 image | `.bss` end | `block_on` coroutine frame | SVC stack headroom |
|---|---:|---:|---:|
| End-device sensor | `0x844C60` | 8,772 B | 7,612 B |
| Parent router | `0x847694` | 9,388 B | 6,996 B |

The SVC stack is a fixed 16 KiB (`_svc_stack_bottom = 0x0084BC00`), and the
application future is pinned inside `block_on`'s frame, so that frame is the
single largest stack consumer. Conflict resolution and the Link Status /
router-aging state added 192 bytes of static RAM to the router and 64 to the
sensor; the outlining work added 480 and 56 bytes to the coroutine frame.

These are not benchmark-equivalent applications. Radio implementations,
linker layouts, executor overhead, enabled peripherals, and application
profiles differ. Compare a build against its own previous measurement and CI
budget rather than ranking chips by the table alone.

The pinned `tc32-45` Telink builds provide the cleanest before/after
comparison:

| TLSR8258 image | Complete-HAL baseline | Current | Reduction |
|---|---:|---:|---:|
| End-device sensor | 323,876 B | 271,836 B | 52,040 B (16.1%) |
| Parent router | 349,792 B | 328,396 B | 21,396 B (6.1%) |

The current router is 56,560 bytes larger than the sensor because it retains
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
role-specific tradeoff is justified.

## Coroutine outlining on TC32

An `async fn` body compiles to a coroutine whose resume function is a separate
MIR body. `#[inline(never)]` on the `async fn` applies to the constructor that
returns the future, **not** to that resume function, so a future awaited from
exactly one place is folded into its caller's coroutine however it is
annotated. That is how `tick_with_security_store` and `process_incoming`
reached 59,796 and 26,224 bytes in the router image: every maintenance,
commissioning, transmit and receive sub-future was merged into two enormous
functions.

Three forms were measured on the pinned `tc32-45` router build over the same
set of outlined awaits:

| Form | Router flash | `block_on` frame |
|---|---:|---:|
| Inline `.await` | 345,088 B | 8,908 B |
| Generic `async fn` wrapper | 330,784 B | 22,676 B |
| `await_out_of_line!` (`Pin<&mut dyn Future>`) | 329,720 B | 9,388 B |

The shipped image adds three more outlined awaits in router maintenance,
reaching 328,396 B at the same coroutine frame size.

The generic `async fn` wrapper is the trap: the caller ends up holding the
moved-from temporary *and* the wrapper coroutine across the await, so every
outlined future's state is stored twice and the frame nearly triples — past
the 16 KiB SVC stack. Pinning the future at the call site and polling it
through `Pin<&mut dyn Future>` leaves the state exactly where an inline
`.await` would have put it and moves only the code, which is what
`await_out_of_line!` in `zigbee-runtime` does.

Reserve it for large sub-futures awaited once — periodic maintenance,
lifecycle transitions, per-frame processing. On a small leaf call the vtable
and indirect call cost more than the inlining saves. Outlining a *synchronous*
step with `#[inline(never)]` is roughly flash-neutral by comparison and is
worth doing for coroutine-size rather than flash reasons.

The other half of the reduction is deduplication: an `.await` embeds the
awaited future's state machine, so two textual copies of `start()` or
`leave()` are two copies of the whole join or leave sequence in flash. Folding
`UserAction::Toggle` onto the join/leave it resolves to, and selecting the
user-requested and automatically due secured rejoins together, removed those
duplicates.

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
- BL702 production rejects the RustCrypto software AES core and requires the
  SEC_ENG backend, while re-running the RV32A/vendor/XIP gates;
- TLSR8258 RAM-code, cache, BSS, DMA, and stack layout;
- TLSR8258 production rejects the RustCrypto software AES core and requires
  the token-owned accelerator backend.
- EFR32MG1 production rejects the RustCrypto software AES core, requires the
  CRYPTO backend, checks BUFC allocation geometry, and requires at least
  16 KiB of linked stack.

Oversized ZCL responses are dropped whole rather than truncated into malformed
frames. Cluster-specific responses have a compile-time proof that their
64-byte payload plus header fits the 128-byte pending-response buffer.

## Hardware AES release policy

The TLSR8258 hardware-AES provider is now proven on a TB-04 router: two
startup known-answer tests, CCM*, AES-MMO, Request-Key, Verify-Key/Confirm-Key,
a complete ZHA interview, more than ten minutes of secured traffic, and
reset/resume all passed under an independent channel-15 capture. The initial
hardware-AES release measured 269,960 bytes for the sensor and 327,760 bytes
for the router, saving 2,640 and 4,680 bytes respectively over the former
software builds with 8 additional bytes of RAM. The later receive-queue
redesign and the coroutine outlining above reduced the current standard images
to 271,836 and 328,396 bytes without changing that AES policy. Both production
manifests install the accelerator unconditionally and fail closed without a
software fallback.

The BL702 SEC_ENG hardware-AES provider is the first cross-platform follow-up
to the TLSR8258 work. Its two startup known-answer tests pass on XT-ZB1
silicon, followed by periodic radio operation and a complete secured ZHA
commissioning flow with Transport-Key, descriptors, binding, reporting, Trust
Center link-key exchange, and encrypted application reports. The standard
image links SEC_ENG while dropping the RustCrypto software AES core; it
measures 161,570 bytes versus 165,602 bytes for the former software build,
saving 4,032 bytes with no additional RAM. Silent reset/resume and a
cycle-derived timeout bound remain open hardware gates.

The EFR32MG1 CRYPTO provider is also production-default. Two startup KATs,
NWK/APS CCM*, Trust Center link-key derivation, full ZHA commissioning and
interview, encrypted reporting, EM2 operation, and silent reset/resume passed
on the TRÅDFRI target. The final factory-EUI image is 137,532 bytes with OTA
enabled. Hardware failures stop startup; software AES is not linked.

The reusable crypto crates retain the software provider for host tests and
platforms without a proven accelerator. It is not linked into the BL702 or
TLSR8258 or EFR32MG1 production images.

Compact TLSR8258 text placement, linker tail merging, and identical-code
folding are also deferred until hardware soak testing confirms startup,
interrupt, RAM-code, persistence, and OTA behavior. R23/BDB 3.1 remains a
separate optional roadmap item and must add no code or RAM to these R22
images.
