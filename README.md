# zigbee-rs

Heap-free, `no_std`, pure-Rust Zigbee PRO for embedded devices.

This worktree is the cross-platform application-model and Zigbee Core R22 /
BDB 3.0.1 hardening branch, `experiment/r22-bdb-complete`. The source
documentation is authoritative for this branch. GitHub Pages is deployed only
from `main`/`master`, so these pages will not appear at the public Pages URL
until the branch is merged and the documentation workflow deploys it.

The normative target is Zigbee Core R22 (`05-3474-22`) with BDB 3.0.1
(`16-02828-012`). Passing host/build matrices is not a certification claim;
the per-platform hardware and interoperability gates below remain explicit.

## Architecture

Dependencies and ownership flow in one direction:

```text
application/profile  device behavior, clusters, measurement mapping
        ↓
product              identity, layout, persistence, bootloader/OTA
        ↓
board                physical wiring and fitted hardware
        ↓
platform/chip HAL    clocks, GPIO, buses, timers, flash, radio
```

Example `main.rs` files are composition roots that wire these layers; they are
not another ownership layer.

The Zigbee protocol path remains shared:

```text
application profile
        |
zigbee-runtime::ZigbeeNode
        |
BDB -> ZCL/ZDO -> APS -> NWK -> MAC
        |
platform radio backend
```

- **HAL/MAC crates** own clocks, GPIO, buses, timers, flash controllers,
  radio mechanisms, and `MacDriver` implementations.
- **Board crates** own physical pins and fitted peripherals. They expose typed
  resources and do not depend on `zigbee-runtime`.
- **Product crates** own manufacturer/model identity, the selected concrete
  profile, battery and scheduling policy, protected partitions, linker layout,
  persistence, and OTA/bootloader selection.
- **Profiles** own endpoint declarations, cluster composition, reporting
  defaults, and measurement-to-ZCL conversion.
- **`zigbee-runtime`** owns common commissioning, receive/tick processing,
  reporting, persistence integration, power lifecycle, and OTA plumbing.
- **`apps/sensor-sed` and `apps/router`** expose reusable, finite
  product-facing application frontends over that runtime.
- **Example `main.rs` files** are composition roots: platform startup,
  resource construction, and the outer executor/event loop.

The detailed design is in the
[architecture chapter](docs/book/src/getting-started/architecture.md).

## Sleepy sensor application

`sensor_sed_app::SensorApp` receives one explicit ownership bundle:

```rust,ignore
let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
let mut app = SensorApp::new(
    node,
    &product::policy::SENSOR_POLICY,
    SensorSedParts {
        wake,
        status,
        environment,
        battery,
        ota,
        actions,
        supervisor,
        diagnostics,
    },
)?;
```

`SensorSedParts` is a bundle, not a platform provider: it has no resource
lookup, constructors, MAC, profile, store, or product policy. Each field is a
narrow capability with visible ownership.

`SensorPolicy` independently selects `fast_sleep_depth` and
`slow_sleep_depth` as `Active`, `Idle`, or `Retention`. The shared application
arbitrates parent polls, stack deadlines, reporting, status, user input,
watchdog service, and OTA deadlines into one bounded wait.

The lifecycle can be embedded in another scheduler:

```rust,ignore
app.initialize().await?; // one finite boot/resume transition
loop {
    app.step().await?;    // one finite wait/service iteration
}
// app.run().await is convenience sugar for the same sequence.
```

OTA events are routed to the selected `OtaLifecycle` before generic event
matching. Reset-causing activation happens only after the shared lifecycle has
checkpointed security state. `NoOta` is accepted only for a profile that
statically implements `NonOtaProfile`.

`NoStatus::PRESENT` is `false`; status-only blink deadlines and waits compile
out rather than becoming no-op runtime work.

## Router and coordinator applications

The shared router application exposes role-safe frontends:

| frontend | runtime role | child capability | permitted startup |
|---|---|---|---|
| `AlwaysOnEndDeviceApp` | `EndDevice` | none | steering/resume only |
| `RelayRouterApp` | `RelayRouter` | `NoChildren` | steering/resume only |
| `ParentRouterApp` | `Router` | `PersistentChildren` | steering/resume only |
| `DistributedRouterApp` | `Router` | `PersistentChildren` | distributed formation or persisted-PAN restart |
| `CoordinatorApp` | `Router` | `PersistentChildren` | formation or persisted-PAN restart |
| `TrustCenterCoordinatorApp` | `Router` | `PersistentChildren` plus durable TC devices | centralized formation/restart and TC transaction recovery |

```rust,ignore
let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
let mut end_device = AlwaysOnEndDeviceApp::new(
    node,
    &product::policy::ALWAYS_ON_END_DEVICE_POLICY,
    RouterParts::new(status, supervisor, diagnostics),
)?;

end_device.initialize().await?;
let events = end_device.step().await?;
```

`AlwaysOnEndDeviceApp` requires only `MacDriver`. Every frontend that
advertises `DeviceType::Router` requires both the `router` feature and
`ParentMacDriver`; a backend without parent-side association and
indirect-delivery primitives cannot construct one. Of the in-tree MACs, only
the real Telink TLSR8258 backend and host-only `MockMac` implement
`ParentMacDriver`. Telink advertises router capability but not coordinator
capability. No production backend advertises Coordinator or Trust-Center
server support, so coordinator composition remains mock-only.

The misleadingly named `examples/nrf52840-router` therefore composes
`AlwaysOnEndDeviceApp`, not a Zigbee router. `router-app` has no default
features: parent/distributed/coordinator products must explicitly enable its
router support, while End Device products compile without route, parent, or
child-table capacities.

`DistributedRouterApp` additionally requires a product-certified distributed
global link key before initialization; the public BDB test key is not a
production default.

All six frontends provide finite `initialize()` and `step()` operations plus
the infinite `run()` convenience wrapper.

## Feature boundaries

- `sleepy-end-device` is the truthful End Device polling/power feature name.
  It retains centralized joining and TCLK handling. Distributed-security
  joining is an orthogonal capability enabled by the shared sensor app by
  default; products that provision no distributed key may disable it.
- `finding-binding-target` provides only the target side: Identify mode and
  normal response handling. Full initiator Finding & Binding is the separate
  `finding-binding` feature, which includes target support.
- `application-link-key-installation` is an optional product capability. It is
  enabled only when the composition also owns durable APS-table storage;
  `NoApsTables` rejects installation rather than accepting a volatile key.
- `compact-single-endpoint` is an orthogonal product-capacity choice, not a
  Zigbee role or behavior cut. It provides two application-endpoint slots and
  four reporting entries. The EFR32MG1, PHY62x2 sensor, and TLSR8258
  sensor/router products currently select it, after compile-time
  profile-capacity assertions and tests.

## Why the configuration is static

There is no devicetree, Kconfig, heap allocator, runtime hardware discovery,
or broad “platform” god trait in the embedded application path.

- Rust board/product types encode wiring, identity, layout, and policy.
- Cargo features select genuinely alternative hardware or image variants.
- Narrow traits describe one capability at a time.
- Concrete generic types preserve ownership and let the linker remove unused
  peripherals, status behavior, OTA, parent support, and diagnostics.
- `heapless` and fixed-capacity tables bound memory use.

Public application composition uses no trait objects or allocation.
`zigbee-runtime` has one tightly scoped internal
`Pin<&mut dyn Future<...>>` outlining path used to control TC32 code size; it
does not allocate and is not a platform abstraction.

## Porting a product

Keep the shared application and protocol crates unchanged:

1. Implement or reuse the chip HAL, monotonic clock, flash controller, radio,
   and `MacDriver`.
2. Add a board crate that maps fitted pins and returns typed resources.
3. Add a product crate that selects identity, profile, policy, protected
   storage, linker layout, and optional OTA backend.
4. Implement the narrow application adapters required by `SensorSedParts` or
   `RouterParts`.
5. Compose them in a short example `main.rs`.
6. Prove startup and raw 802.15.4 first, then scan, association, Zigbee
   security, interview/reporting, reset/resume, sleep, and OTA as separate
   hardware gates.

Only board/platform adapters and product choices should change when the same
sensor or router behavior is moved to a new MCU.

## Current targets

“Build” means the pinned release image compiles and passes its layout checks;
it is not a hardware claim. The exact 2026-08-27 image measurements are
build/layout-tested only. Hardware evidence in the last column is prior
path-level evidence unless an exact-image rerun is explicitly named; this
worktree records no such rerun for the current byte-for-byte images.

| target | role/application | build | hardware validation |
|---|---|---:|---|
| nRF52840 DK | environmental sleepy End Device | yes | prior path evidence covers commissioning, reporting, AES, persistence, and reset/resume |
| nRF52833 DK | environmental sleepy End Device | yes | prior path evidence covers commissioning, reporting, AES, persistence, and reset/resume |
| nRF52840 DK | always-on End Device | yes | commissioning/resume and continuous-RX HIL remain open |
| ESP32-C6 | environmental sleepy End Device + OTA | yes | prior C6 evidence covers commissioning/reporting and OTA to 18.3%; complete activation remains open |
| ESP32-H2 | environmental sleepy End Device + OTA | yes | prior H2 evidence covers v1→v2 activation, reboot, and retained commissioned state |
| BL702 XT-ZB1 | environmental sleepy End Device | yes | prior path evidence covers radio/commissioning/interview; destructive flash persistence remains open |
| PHY6222/PHY6252 EVK | environmental sleepy End Device | yes | PHY6222 and PHY6252 exact cross-build/layout image measurements; complete hardware path remains unverified |
| CC2340R5 | environmental sleepy End Device | yes | pinned-SDK and fallback compile/link paths pass; radio HIL and entropy remain open |
| EFR32MG1P TRÅDFRI | environmental sleepy End Device | yes | prior path evidence covers commissioning through EM2; real OTA install remains open |
| EFR32MG21 BRD4181A | environmental sleepy End Device | yes | complete hardware path remains HIL-unverified |
| TLSR8258 TB-04 | environmental sleepy End Device | yes | prior evidence covers the SUSPEND primitive; repeated application/network acceptance remains open |
| TLSR8258 TB-04 | child-capable router | yes | prior evidence covers join/restart/Link Status/relay; corrected-image child acceptance remains open |

See [BUILD.md](BUILD.md) for pinned commands, measured images, partition
boundaries, and exact remaining gates.

## Quick host checks

The general toolchain is pinned to `nightly-2026-03-23`:

```bash
cargo +nightly-2026-03-23 test --workspace --locked
cargo +nightly-2026-03-23 test -p sensor-sed-app --features ota --locked
cargo +nightly-2026-03-23 test -p zigbee-runtime --features router --locked
cargo +nightly-2026-03-23 clippy --workspace --all-targets --locked -- -D warnings
cargo +nightly-2026-03-23 fmt --all -- --check
```

ESP32 and PHY6222 use `nightly-2026-08-01`. TLSR8258 uses the
`tc32-stage2-tc32-45` target toolchain; its host-side tools use Rust `1.94.1`.

## Documentation

- [Book source and navigation](docs/book/src/SUMMARY.md)
- [Build and validation matrix](BUILD.md)
- [Examples](examples/README.md)
- [API map](docs/book/src/reference/api.md)
- [R22 / BDB implementation status](docs/book/src/reference/conformance.md)
- [Architecture](docs/book/src/getting-started/architecture.md)
- [NV storage](docs/book/src/advanced/nv-storage.md)
- [Power management](docs/book/src/advanced/power.md)
- [OTA](docs/book/src/advanced/ota.md)
- [Coordinator and router roles](docs/book/src/advanced/coordinator-router.md)

## License

Workspace crates declare `MIT OR Apache-2.0`.
