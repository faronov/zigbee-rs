# zigbee-rs

Heap-free, `no_std`, pure-Rust Zigbee PRO for embedded devices.

This worktree is the final cross-platform application-model migration on
`experiment/zephyr-app-model`. The source documentation is authoritative for
this branch. GitHub Pages is deployed only from `main`/`master`, so these pages
will not appear at the public Pages URL until the branch is merged and the
documentation workflow deploys it.

## Architecture

Dependencies and ownership flow in one direction:

```text
platform/chip HAL + MacDriver
        ↓
board resources and fitted wiring
        ↓
product identity/profile/policy/storage/linker/OTA
        ↓
sensor-sed or router-app composition root
```

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
- **Product crates** own manufacturer/model identity, endpoint profile,
  battery and scheduling policy, protected partitions, linker layout,
  persistence, and OTA/bootloader selection.
- **`apps/sensor-sed` and `apps/router`** own reusable commissioning,
  receive/tick processing, reporting, persistence integration, and lifecycle
  behavior.
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
| `CoordinatorApp` | `Router` | `PersistentChildren` | formation or persisted-PAN restart |

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
indirect-delivery primitives cannot construct one. The nRF52840 example is
therefore an always-on End Device. `router-app` has no default features:
parent/relay/coordinator products must explicitly enable
`router-app/features = ["router"]`, while End Device products compile without
route, parent, or child-table capacities.

All four frontends provide finite `initialize()` and `step()` operations plus
the infinite `run()` convenience wrapper.

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
it is not a hardware claim.

| target | role/application | build | hardware validation |
|---|---|---:|---|
| nRF52840 DK | environmental SED | yes | commissioning, reporting, AES, persistence, and reset/resume proven |
| nRF52833 DK | environmental SED | yes | commissioning, reporting, AES, persistence, and reset/resume proven |
| nRF52840 DK | always-on End Device | yes | commissioning/resume and continuous-RX HIL remain open |
| ESP32-C6 | environmental SED + optional OTA | yes | OTA transfer reached 18.3%; complete activation still open |
| ESP32-H2 | environmental SED + optional OTA | yes | v1→v2 OTA activation, reboot, and commissioned-state retention proven |
| BL702 XT-ZB1 | environmental SED | yes | radio/commissioning/interview proven; destructive flash persistence validation open |
| PHY6222/PHY6252 EVK | environmental SED | yes | complete radio/join/persistence path remains hardware-unverified |
| CC2340R5 | environmental SED | yes | radio HIL and entropy backend remain open; commissioning fails closed |
| EFR32MG1P TRÅDFRI | environmental SED | yes | commissioning, interview, sensors, persistence, reset/resume, and EM2 proven; real OTA install open |
| EFR32MG21 BRD4181A | environmental SED | yes | complete hardware path remains HIL-unverified |
| TLSR8258 TB-04 | environmental SED | yes | default SUSPEND primitive proven; repeated application-level sleep/network HIL remains open |
| TLSR8258 TB-04 | child-capable router | yes | join, restart, Link Status, and relay proven; corrected-image first-attempt child acceptance remains open |

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
- [Architecture](docs/book/src/getting-started/architecture.md)
- [NV storage](docs/book/src/advanced/nv-storage.md)
- [Power management](docs/book/src/advanced/power.md)
- [OTA](docs/book/src/advanced/ota.md)
- [Coordinator and router roles](docs/book/src/advanced/coordinator-router.md)

## License

Workspace crates declare `MIT OR Apache-2.0`.
