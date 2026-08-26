# Architecture

The final application model separates portable behavior from the mechanisms
and policy that vary by chip, board, and product.

## Layer ownership

```text
platform/chip HAL + MacDriver
        ↓
board resources and fitted wiring
        ↓
product identity/profile/policy/storage/linker/OTA
        ↓
sensor-sed or router-app composition root
```

| layer | owns | must not own |
|---|---|---|
| chip HAL / MAC | clocks, GPIO, buses, timers, flash controller, radio, AES mechanism, `MacDriver` | product identity, clusters, battery chemistry, partition policy |
| board | pins, buses, LEDs, buttons, sensors, physical flash devices, exclusive resources | Zigbee runtime, endpoint behavior, product storage policy |
| product | manufacturer/model, profile, reporting defaults, battery mapping, wait policy, linker map, persistence partitions, OTA/bootloader policy | generic chip mechanisms or hidden board discovery |
| application/profile | commissioning lifecycle, measurements-to-ZCL mapping, reporting, commands, role behavior | platform startup and physical pin acquisition |
| composition root | startup, concrete resource construction, executor/event loop | duplicated BDB/ZCL/ZDO/APS/NWK/MAC state machines |

The protocol path remains shared:

```text
application profile
        |
zigbee-runtime::ZigbeeNode
        |
BDB -> ZCL/ZDO -> APS -> NWK -> MAC
        |
platform radio backend
```

## Sleepy sensor frontend

`apps/sensor-sed` owns the reusable environmental sleepy-end-device lifecycle.
The composition root supplies every capability explicitly:

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

`SensorSedParts<W, St, E, B, O, A, Sv, D>` is only an ownership bundle. It
does not create peripherals, search for a platform, own the MAC/profile/store,
or hide product policy.

The fields are deliberately narrow:

- `wake`: monotonic time, button wake, and atomic MAC wait transition;
- `status`: semantic status indication;
- `environment`: environmental measurements;
- `battery`: voltage and ZCL battery mapping;
- `ota`: OTA transport and activation lifecycle;
- `actions`: product-selected short-button behavior;
- `supervisor`: watchdog heartbeat and reset;
- `diagnostics`: typed lifecycle events.

### Finite lifecycle

```rust,ignore
app.initialize().await?; // exactly one boot/resume lifecycle
loop {
    app.step().await?;    // exactly one bounded wait/service iteration
}
```

`run()` is convenience sugar for `initialize()` followed by infinite
`step()` iterations. The finite methods make the same application usable
inside Embassy, a platform-owned loop, a host test, or reset-on-wake Telink
composition without duplicating behavior.

Construction rejects a `ZigbeeDevice` with automatic polling enabled. The
application must be the single owner of sleepy-end-device parent polling.

### Fast and slow waits

`SensorPolicy` independently selects:

```rust,ignore
fast_sleep_depth: SleepDepth,
slow_sleep_depth: SleepDepth,
```

The values are:

- `Active`: keep the platform active and wait only for timer/button;
- `Idle`: quiesce the radio and use a retained light/suspend wait;
- `Retention`: enter a retention mode that requires explicit restoration.

The shared application uses the fast depth during commissioning, interview,
commands, and other short-poll windows. It uses the slow depth only during the
joined steady state. If OTA is active, the wait is forced active.

A `WakeController::wait` operation is atomic from the application's point of
view: prepare the MAC/radio, wait, restore every required clock/radio/MAC
invariant, then return.

### OTA-first routing

Every stack event is offered to `OtaLifecycle::handle_event` before generic
application matching. An OTA implementation reports that activation is
pending; it does not reset directly. The shared lifecycle first checkpoints
network keys and security counters, then invokes activation.

`NoOta` is not a permissive fallback. It implements the lifecycle only for a
profile that explicitly implements `NonOtaProfile`, so a profile advertising
the OTA client cannot accidentally be paired with no transport.

### Absent status hardware

`NoStatus::PRESENT` is `false`. Policy validation ignores meaningless blink
durations, and status-only deadlines/delays compile out. This is used by
products such as BL702 and ESP32-C6 rather than carrying fake LED work.

## Router and coordinator frontends

`apps/router` exposes four public always-on frontends:

| frontend | runtime role | child lifecycle | startup path |
|---|---|---|---|
| `AlwaysOnEndDeviceApp` | `EndDevice` | none | network steering/resume |
| `RelayRouterApp` | `RelayRouter` | `NoChildren` | network steering/resume |
| `ParentRouterApp` | `Router` | `PersistentChildren` | network steering/resume |
| `CoordinatorApp` | `Router` | `PersistentChildren` | formation or persisted-PAN restart |

```rust,ignore
let mut relay = RelayRouterApp::new(
    ZigbeeNode::new(&mut device, &mut security_store, &mut profile),
    NoChildren,
    &product::policy::ROUTER_POLICY,
    RouterParts::new(status, supervisor, diagnostics),
)?;

relay.initialize().await?;
let events = relay.step().await?;
```

`AlwaysOnEndDeviceApp` requires only `MacDriver`. Every frontend that
advertises `DeviceType::Router` (`RelayRouterApp`, `ParentRouterApp`, and
`CoordinatorApp`) requires the `router` feature and `ParentMacDriver`.
This makes unsupported parent/coordinator operations unconstructible rather
than success-shaped no-ops.

`AlwaysOnEndDeviceApp` accepts only a `ZigbeeNode<..., EndDevice>` built with
`PowerMode::AlwaysOn`; it verifies `DeviceType::EndDevice` and
`macRxOnWhenIdle`. It owns steering, rejoin, reset, bounded receive/tick, and
security checkpoints, but compiles out routing and child lifecycle.

```rust,ignore
let mut plug = AlwaysOnEndDeviceApp::new(
    ZigbeeNode::new(&mut device, &mut security_store, &mut profile),
    &product::policy::ALWAYS_ON_END_DEVICE_POLICY,
    RouterParts::new(status, supervisor, diagnostics),
)?;

plug.initialize().await?;
loop {
    let events = plug.step().await?;
    // Synchronize fitted relay hardware from profile state here.
}
```

The nRF52840 example is therefore an `AlwaysOnEndDeviceApp`. The TLSR8258
router has the parent primitives and uses `ParentRouterApp +
PersistentChildren`.

All four frontends expose finite `initialize()` and `step()` operations and
the infinite `run()` wrapper. `StepEvents` returns the bounded incoming/tick
events so a product such as a relay plug can synchronize fitted hardware after
the shared profile handles a command.

## Why there is no devicetree, Kconfig, or god trait

Rust types already encode the selected resources:

- the board constructor consumes concrete pin/peripheral tokens;
- the product type fixes identity, profile, policy, partitions, and linker
  map;
- the application parts list every capability;
- the role frontend fixes startup and parent capability;
- Cargo features select real image alternatives, not runtime discovery.

A broad `Platform` trait would hide ownership, force unrelated peripherals
into one implementation, and make mutually exclusive resources harder to
prove. Narrow traits keep LED, wake, sensor, flash, OTA, and reset choices
independent and let dead-code elimination remove unused paths.

The embedded application path has no allocator and public composition has no
trait objects. Concrete generic types monomorphize. `zigbee-runtime` does use a
single internal pinned `dyn Future` outlining mechanism for TC32 size; it is
bounded, non-allocating, and unrelated to platform configuration.

## Porting recipe

For the same product behavior on a new MCU:

1. Implement the chip's clock/timer/radio/flash primitives and `MacDriver`.
2. Add a board crate that maps fitted hardware and returns exclusive typed
   resources.
3. Add a product crate for identity, profile, battery conversion, policy,
   persistence partitions, linker layout, and OTA choice.
4. Implement the narrow `SensorSedParts` or `RouterParts` adapters.
5. Compose them in `main.rs`; do not copy the application state machine.
6. Advance through startup, raw 802.15.4, scan, association, Zigbee security,
   interview/reporting, reset/rejoin, power, and OTA gates.

Only the board/platform adapters and product selections should change. The
profile, shared application, `ZigbeeNode`, and protocol crates stay the same.
