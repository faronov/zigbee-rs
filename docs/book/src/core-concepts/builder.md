# The Device Builder

`ZigbeeDevice::builder(mac)` constructs the protocol stack and fixes its
logical role. Product profile and application lifecycle remain separate.

## Product-first construction

Products normally create a typed profile, then use it to configure the device:

```rust,ignore
let mut profile = product::profile::sensor_profile();

let mut device = ZigbeeDevice::builder(mac)
    .power_mode(product::policy::SENSOR_POLICY.power_mode())
    .automatic_polling(false)
    .manufacturer(product::MANUFACTURER)
    .model(product::MODEL)
    .date_code(product::DATE_CODE)
    .sw_build(product::SW_BUILD)
    .power_source(PowerSource::Battery)
    .channels(product::CHANNEL_MASK)
    .endpoint(
        profile.endpoint(),
        profile.profile_id(),
        profile.device_id(),
        |endpoint| profile.configure_endpoint(endpoint),
    )
    .build();
```

The profile owns endpoint declaration, clusters, reporting defaults, and
measurement-to-ZCL mapping. The builder owns protocol-layer construction and
device metadata.

## Role-selecting terminal methods

| terminal | result | MAC requirement |
|---|---|---|
| `build()` | `ZigbeeDevice<M, EndDevice>` | `MacDriver` |
| `build_relay()` | `ZigbeeDevice<M, RelayRouter>` | `router` feature + `ParentMacDriver` |
| `build_router()` | `ZigbeeDevice<M, Router>` | `router` feature + `ParentMacDriver` |
| `build_coordinator()` | coordinator device type on `Router` role | `router` feature + `ParentMacDriver` |

```rust,ignore
let sensor = ZigbeeDevice::builder(mac).build();
let parent = ZigbeeDevice::builder(parent_mac).build_router();
let coordinator = ZigbeeDevice::builder(parent_mac).build_coordinator();
```

`build_relay`, `build_router`, and `build_coordinator` do not exist without
the `router` feature. The terminal method selects the canonical `DeviceType`. A conflicting
`.device_type(...)` is rejected. `try_build*` returns a typed `BuildError`;
the convenience `build*` form panics on invalid static composition.

Parent-only APIs are unavailable to end devices and relays.

## Sleepy-end-device polling

`PowerMode::Sleepy` sets the association capability and base poll policy:

```rust,ignore
.power_mode(PowerMode::Sleepy {
    poll_interval_ms: product::policy::SENSOR_POLICY.slow_poll_ms,
    wake_duration_ms: product::policy::SENSOR_POLICY.wake_duration_ms,
})
```

The shared `SensorApp` must be the only poll owner:

```rust,ignore
.automatic_polling(false)
```

`SensorApp::new` rejects a device with automatic polling enabled. The
product's `SensorPolicy` separately selects fast/slow poll intervals and
`SleepDepth::{Active, Idle, Retention}`. The builder does not enter platform
sleep.

## Metadata

The builder populates runtime-owned Basic cluster attributes:

```rust,ignore
.manufacturer("Example")
.model("Env-Sensor")
.application_version(2)
.date_code("20260826")
.sw_build("2.0.0")
.power_source(PowerSource::Battery)
```

Identity values belong in the product crate, not a board or generic HAL.

## Channels and endpoints

```rust,ignore
.channels(ChannelMask::ALL_2_4GHZ)
.endpoint(endpoint, profile_id, device_id, |ep| {
    profile.configure_endpoint(ep)
})
```

Prefer a reusable `ApplicationProfile` over duplicating cluster lists in each
platform example. A custom profile can still configure an `EndpointBuilder`
directly with `cluster_server`, `cluster_client`, and `device_version`.

## Build into static storage

Targets that require static placement can use the corresponding `build_into`
terminal:

```rust,ignore
let device = ZigbeeDevice::builder(mac)
    // configuration
    .build_into(DEVICE.uninit());
```

Role-specific `build_relay_into`, `build_router_into`, and coordinator forms
preserve the same capability bounds.

## Connect storage, profile, and application

The builder result is not a complete product loop. Construct a `ZigbeeNode`
and select the shared application frontend:

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

app.run().await
```

For routers, pair `build_relay` with `RelayRouterApp`, `build_router` with
`ParentRouterApp`, and `build_coordinator` with `CoordinatorApp`.

See [Architecture](../getting-started/architecture.md) and
[The Event Loop](event-loop.md).
