# R22 and BDB 3.0.1 Status

The production target is Zigbee Core R22 (`05-3474-22`) with BDB 3.0.1
(`16-02828-012`). BDB 3.1 requires R23 or newer and is not part of this
baseline.

This page reports implementation and validation evidence. It is **not** a
Zigbee certification claim. The terms are deliberately distinct:

- **implemented** — the protocol path exists and unsupported capability fails
  explicitly;
- **host-tested** — portable state-machine or wire behavior passed host tests;
- **cross-compiled/build-tested** — a target image and its named
  layout/symbol checks passed;
- **exact-image HIL/packet capture** — that exact byte image was flashed and
  observed on silicon or over the air;
- **certified** — an external Zigbee certification result.

The 2026-09-06 measurement set is build/layout-tested. Existing hardware
statements are prior path-level evidence; the repository does not record an
exact-image rerun of the freshly measured binaries.

## Role matrix

| role | software status | embedded evidence | remaining gate |
|---|---|---|---|
| sleepy End Device | steering, persisted resume/rejoin, ED timeout, polling, reporting, reset and OTA lifecycle are host-tested | all real backends currently compose as End Devices; current release images build/layout-test as listed in `BUILD.md` | exact-image security/power/interoperability acceptance per release |
| receiver-on End Device | typed non-parent composition and continuous receive path are host-tested | nRF52840 release image builds | complete HIL acceptance |
| relay router | routing without child admission is compile-time separated and host-tested | no production image is currently composed; `MockMac` can host-test it | production backend/product composition and packet capture |
| parent router | child admission, indirect delivery, timeout, reassignment, Leave cascade and durable DeviceLeft are host-tested | TLSR8258 is the only production `ParentMacDriver`; its current image builds/layout-tests | corrected-image first-attempt child join plus power-cut lifecycle captures |
| distributed router | distributed formation, provisioned-key checks and persisted PAN restart are host-tested | no production hardware image is claimed | product composition and HIL |
| coordinator / Trust Center | formation/restart, admission, install-code keys, application keys, Remove-Device and Network-Key rotation are host-tested | mock-only; no production backend advertises coordinator capability | production backend/product composition, multi-device HIL and certification |

Unsupported role capabilities are absent at compile time. A backend that
cannot provide association responses, pending transactions and indirect
delivery cannot construct a parent router or coordinator.

Only the tc32 Telink `TelinkMac` and host `MockMac` implement the sealed
`ParentMacDriver`. Telink advertises `router: true` and `coordinator: false`;
all other real backends use non-parent capability descriptors. Consequently
`examples/nrf52840-router` is an `AlwaysOnEndDeviceApp`, not a Zigbee router.

## Core R22 coverage

| area | implemented and host-tested | hardware/certification status |
|---|---|---|
| NWK security | dual Network Keys, key sequence changes, outgoing reservations, incoming replay floors, key-retirement tombstones and fail-closed restore | target flash durability and power-cut tests remain per platform |
| join and rejoin | centralized and distributed selection, secured-first persisted rejoin, centralized-only fallback and provisional-parent application gate | prior representative End Device hardware evidence exists; current exact-image and multi-router fallback captures remain open |
| routing | route discovery/repair, relay, many-to-one, source routing, broadcast transactions and Link Status | Telink relay/Link Status have hardware evidence; broader topology HIL remains open |
| maintenance | Network Report/Update, PAN conflict transition, address reassignment, Device Announce and authenticated lifecycle replay ordering | packet-level multi-router conflict tests remain open |
| parent lifecycle | child table restore, timeout, indirect queue, poll delivery, Remove-Device, Leave cascade and ACK-gated DeviceLeft | sleepy-child power-cut and first-attempt interview HIL remain open |
| APS | APS security, ACK tracking, fragmentation, groups, bindings, application keys, durable table snapshots and replacement-key replay retirement | interoperability and flash-endurance qualification remain product gates |
| ZDO | descriptors, discovery, binding, leave/rejoin and management services required by the compiled role | certification harness execution remains open |
| Trust Center | install-code CRC/AES-MMO, admission, TCLK exchange, Request/Transport/Verify/Confirm-Key, Update/Remove-Device and Network-Key rotation | no production coordinator hardware claim yet |

Crash-safe receive ordering is:

1. verify the MIC;
2. persist the side effect or durable transaction intent;
3. persist the replay floor;
4. release an ACK, response, relay, indication or other wire-visible effect.

## BDB 3.0.1 coverage

| procedure | status |
|---|---|
| initialization and attributes | implemented with role-correct capabilities and BDB 3.0.1 defaults |
| Network Steering | implemented for End Devices and Routers, including post-join TCLK authentication and Permit Joining behavior |
| Network Formation | implemented for typed Coordinator and distributed-router frontends; host-tested |
| Finding & Binding target | `finding-binding-target` provides Identify target behavior and normal Identify Query/Simple Descriptor/Groups responses; host-tested and selected by the sleepy-sensor app |
| Finding & Binding initiator | the separate `finding-binding` feature includes target support plus event-driven initiator behavior, unicast binding, and group mode; host-tested |
| application link-key installation | optional product capability; accepted only with durable APS-table storage and cross-journal commit ordering |
| install-code joining | CRC validation and AES-MMO link-key derivation are implemented and host-tested |
| Trust Center link-key exchange | Request-Key, Transport-Key, Verify-Key and Confirm-Key timing/state are host-tested |
| Basic Reset | resets writable application attributes without erasing network/security state |
| Touchlink | experimental, compiled out and not advertised by default |

`sleepy-end-device` names the role/polling capacity, not a security model.
Centralized joining with TCLK processing and distributed-security joining both
remain implemented. `compact-single-endpoint` is likewise orthogonal to BDB:
it reduces fixed product capacities to two application endpoints and four
reporting entries without removing Zigbee behavior. EFR32MG1 and TLSR8258
sensor/router products select it only after profile assertions.

## Gates before a conformance claim

- Zigbee certification test-harness execution and review of the resulting
  traces;
- multi-vendor packet captures for join, rejoin, key rotation, Remove-Device,
  child aging and PAN/address conflicts;
- production Coordinator/Trust Center hardware composition;
- repeated power-cut validation of every product's security, child and APS
  flash journals;
- final physical Flash/RAM/OTA/stack and layout checks, with hardware acceptance
  for every release image.

The host lifecycle suite additionally sweeps initial-key completion and
sleepy-child Network-Key rotation across journal commit failures; see
[host restart qualification](../advanced/security.md#host-restart-qualification).
Shared sensor OTA activation is blocked on checkpoint failures and retains
its product-selected wake deadline. Neither result closes the hardware gates
above.

Further TLSR8258 size optimization and 512 KiB OTA work are deferred. Removing
artificial regression budgets does not waive physical memory or stack limits,
introduce a TLSR OTA writer, or promote the experimental compiler into a
release toolchain. Recorded ESP32-C6 regression-budget failures remain
historical evidence, not current build blockers. Physical OTA-slot fit alone
is still not hardware acceptance or a conformance claim.

See [`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/master/BUILD.md)
for exact target commands and evidence.
