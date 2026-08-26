# API Reference

This page maps the public crates and the application-model entry points. Use
`cargo doc --workspace --no-deps --locked` for generated item documentation.

## Application crates

### `sensor-sed-app`

Reusable environmental sleepy-end-device lifecycle.

```rust,ignore
pub struct SensorSedParts<W, St, E, B, O, A, Sv, D> {
    pub wake: W,
    pub status: St,
    pub environment: E,
    pub battery: B,
    pub ota: O,
    pub actions: A,
    pub supervisor: Sv,
    pub diagnostics: D,
}
```

Important APIs:

| API | purpose |
|---|---|
| `SensorApp::new(node, policy, parts)` | validates one manual poll owner and the statically selected status policy |
| `SensorApp::initialize()` | finite one-time boot/resume lifecycle |
| `SensorApp::step()` | one finite wait/service iteration |
| `SensorApp::run()` | `initialize()` followed by infinite steps |
| `SensorPolicy` | reporting, fast/slow polls, retries, button/status policy, and wait depths |
| `SleepDepth::{Active, Idle, Retention}` | maximum depth requested for a wait |
| `WakeController` | monotonic time, button state, and atomic MAC wait/restore |
| `StatusSink` / `NoStatus` | semantic status; absent hardware compiles out status waits |
| `EnvironmentSource` | environmental measurement source |
| `BatterySource` | product-selected battery measurement |
| `OtaLifecycle` / `NoOta` | profile-paired OTA transport and activation |
| `Supervisor` | watchdog heartbeat, maximum wait, and reset |
| `Diagnostics` | typed lifecycle event sink |

`NoOta` requires `NonOtaProfile`. OTA-advertising profiles must provide a
concrete `OtaLifecycle`.

### `router-app`

Reusable always-on End Device, router, and coordinator lifecycle.

| frontend | device role | required child/MAC capability |
|---|---|---|
| `AlwaysOnEndDeviceApp` | `EndDevice` | `MacDriver` |
| `RelayRouterApp` | `RelayRouter` | `NoChildren`, `router` + `ParentMacDriver` |
| `ParentRouterApp` | `Router` | `PersistentChildren<C>`, `router` + `ParentMacDriver` |
| `CoordinatorApp` | `Router` | `PersistentChildren<C>`, `router` + `ParentMacDriver` |

Each frontend provides:

```rust,ignore
initialize().await?; // finite startup/resume/formation
let events = step().await?; // finite receive/tick cycle
run().await;          // infinite wrapper
```

`StepEvents` contains at most one incoming event and one tick event. Router
parts contain status, supervisor, and diagnostics capabilities.

## `zigbee-runtime`

### `ZigbeeDevice`

Owns the protocol layers around a `MacDriver`.

```rust,ignore
let device = ZigbeeDevice::builder(mac)
    .power_mode(policy.power_mode())
    .automatic_polling(false)
    .manufacturer("Example")
    .model("Sensor")
    .endpoint(endpoint, profile_id, device_id, |ep| {
        profile.configure_endpoint(ep)
    })
    .build();
```

Role-selecting terminal builders:

- `build()` — end device;
- `build_relay()` — forwarding-only `RelayRouter`;
- `build_router()` — parent `Router`, bounded on `ParentMacDriver`;
- `build_coordinator()` — coordinator device type on the parent role.

Role-specific methods are not exposed to incompatible roles.

### `ZigbeeNode`

Borrows a device, security store, and application profile:

```rust,ignore
let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
```

The application frontends use it for commissioning/resume, receive/tick
processing, reporting, persistence, rejoin, factory reset, and coordinator
startup. Prefer the shared application frontend over reproducing those calls
in a platform loop. Safety-critical composition roots can use the explicit
deferred-reset initialize/step variants, make fitted outputs and application
state durable, then call `complete_pending_factory_reset_and_recommission()`;
the ordinary frontend methods still commit resets automatically.

### Profiles

`zigbee_runtime::profile` contains platform-independent endpoint archetypes,
including environmental sensors, relay plugs/lights, and `WithOta` composition.
Profiles own endpoint declarations, cluster composition, reporting defaults,
and measurement-to-ZCL conversion.

### Persistence

| API | use |
|---|---|
| `SecurityStateStore` | abstract load/store/clear of commissioned security state |
| `SecurityStateJournal<F, SECTOR_SIZE = 4 KiB>` | two-sector crash-safe keys/counter journal |
| `NvStorage` / `LogStructuredNv<F>` | generic non-security items |
| `ChildTableStore` / `ChildTableJournal<F>` | separate parent child-table persistence |

The product supplies a bounded flash partition. The board supplies only the
physical flash resource.

### OTA

| API | use |
|---|---|
| `OtaManager` | Zigbee OTA cluster protocol state |
| `OtaSession` | common server lock, retries, APS requests, and activation deferral |
| `FirmwareWriter` | product staging/verification/activation backend |
| `WithOta` | compile-time OTA client profile composition |

The sensor application routes OTA events first and checkpoints security before
calling activation.

## Protocol crates

| crate | responsibility |
|---|---|
| `zigbee-types` | addresses, channels, status values, frame primitives |
| `zigbee-crypto` | AES provider abstraction, CCM*, MMO, security helpers |
| `zigbee-mac` | `MacDriver`, parent capability boundary, software MAC and radio backends |
| `zigbee-nwk` | network state, routing, neighbour/child tables, NWK security |
| `zigbee-aps` | APS frames, security, binding, groups, fragmentation |
| `zigbee-zdo` | discovery, descriptors, leave/rejoin, parent procedures |
| `zigbee-zcl` | foundation commands, clusters, reporting, OTA cluster |
| `zigbee-bdb` | commissioning and steering/formation policy |
| `zigbee-runtime` | integration, profiles, event/tick processing, persistence and OTA plumbing |
| `zigbee` | top-level re-exports and high-level coordinator/router data structures |

Protocol crates do not depend on boards, products, vendor SDKs, logging
transports, or platform diagnostics.

## Platform, board, and product crates

- `<chip>-hal`: generic mechanisms and radio/controller support.
- `boards/<board>`: physical wiring and exclusive resources.
- `products/<product>`: identity, profile, policy, linker, persistence, OTA.
- `examples/<target-role>`: composition root.

See [Architecture](../getting-started/architecture.md) and the
[platform guides](../platform-guides/nrf.md) for concrete compositions.
