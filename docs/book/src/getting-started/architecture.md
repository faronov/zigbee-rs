# Architecture Overview

zigbee-rs is a complete Zigbee PRO R22 protocol stack written in Rust, split
across **9 crates** that mirror the standard Zigbee layer model. Every crate is
`#![no_std]` and heap-free — suitable for the smallest microcontrollers.

## Layer Diagram

```text
┌─────────────────────────────────────┐
│  Product + application profile      │
├─────────────────────────────────────┤
│  zigbee-runtime (ZigbeeNode)        │
├──────┬──────┬───────┬──────┬───────┤
│  BDB │  ZCL │  ZDO  │  APS │       │
├──────┴──────┴───────┴──────┤       │
│         zigbee-nwk          │ types │
├─────────────────────────────┤       │
│         zigbee-mac          │       │
├─────────────────────────────┴───────┤
│      Hardware (radio)               │
└─────────────────────────────────────┘
```

The top-level **`zigbee`** crate re-exports everything and adds
coordinator/router role support. Most applications interact with the
`zigbee-runtime` layer through `ZigbeeNode`, which composes a
`ZigbeeDevice`, durable security store, and typed application profile.

Hardware composition follows a separate dependency direction:

```text
application/profile  device behavior and measurement mapping
product              identity, flash layout, bootloader/OTA selection
board                physical pins, buses, buttons, LEDs, fitted devices
platform/chip HAL    radio, clocks, sleep, reset, raw flash controller
```

A board crate must not depend on `zigbee-runtime`. Product code depends on the
board and selects the storage/OTA policy used by the application.

An application crate may be shared by several products of the same chip
family. `apps/nrf-sensor` is the reference: it owns the whole nRF sensor
lifecycle (commissioning, silent resume, bounded secure-rejoin retry, poll
windows, interview detection, Device_annce retries, reporting, button
semantics) once, and both `examples/nrf52840-sensor` and
`examples/nrf52833-sensor` run it unmodified. It is generic over the four
things that legitimately differ per product — the security store, the profile
component, the fitted environmental sensor, and the battery chemistry — and
those are bound in each firmware's composition root, so the application crate
never depends on a product or a board:

```rust,ignore
// examples/nrf52833-sensor/src/main.rs — composition root
struct Battery;                                       // product chemistry
impl nrf_sensor_app::BatteryPolicy for Battery { /* … */ }

let mut profile = nrf52833_sensor_product::profile::sensor_profile();
let mut store = nrf52833_sensor_product::storage::security_store(nvmc);
let node = ZigbeeNode::new(&mut device, &mut store, &mut profile);

let mut app: SensorApp<'_, _, _, _, Battery> =
    SensorApp::new(node, led, button, environment, saadc);
app.run().await
```

## Board Resource Ownership

Where fitted peripherals have alternative owners, board crates expose typed
resources that enforce mutual exclusion at the type level. The EFR32MG1
TRÅDFRI board is the reference example:

```rust,ignore
let board = BoardResources::take().unwrap();

// PA0: choose EITHER direct GPIO LED OR TIMER0 PWM (not both)
let led = board.pa0.into_led();        // excludes PWM
// let pwm = board.pa0.into_led_pwm(); // would fail: token consumed

// Product policy chooses EITHER direct SPI OR bootloader-managed flash.
let ota_flash = board.external_flash.into_bootloader_managed();
let profile = product::profile::sensor_profile(firmware_version, ota_flash)?;

// Remaining tokens are consumed individually
let i2c = board.sensor_i2c.into_sensor_i2c()?;
let supply = board.supply_adc.into_supply_monitor()?;
```

Each token is consumed exactly once. Unused tokens are dropped and the linker
eliminates the dead driver code. The bootloader ownership marker is retained by
the product OTA writer for its full lifetime, so the direct USART0 SPI path
cannot be selected through the typed API at the same time. Peripheral
diagnostics should exercise the same typed constructors as production.
Chip-internal radio, RTCC timing, and internal flash remain HAL/platform or
product resources rather than board tokens; their existing diagnostics stay
independently buildable.

ESP32-C6/H2 and nRF52840 use their vendor-independent HAL types directly for
chip-internal flash/radio mechanisms. Their board crates still remain
physical-only: the ESP board exposes raw whole-chip flash, while the nRF DK
board maps LED, button, and sensor-I2C pins. Product crates add partitions,
persistence, identity, and the concrete profile.

## Crate Roles

| Crate | Role |
|-------|------|
| **`zigbee-types`** | Core types shared by all layers: `IeeeAddress`, `ShortAddress`, `PanId`, `ChannelMask`, `MacAddress`. No dependencies. |
| **`zigbee-crypto`** | Shared low-stack Zigbee AES-CCM* primitives used by NWK and APS. |
| **`zigbee-mac`** | IEEE 802.15.4 MAC layer. Defines the async `MacDriver` trait and ships backends for Mock, ESP32-C6/H2, nRF52840/52833, BL702, CC2340, TLSR8258, PHY6222, EFR32MG1, and EFR32MG21. |
| **`zigbee-nwk`** | Network layer. Frame parsing, AODV + tree routing, NWK security (AES-CCM\*), the NIB (Network Information Base), and the `NwkLayer<M: MacDriver>` wrapper. |
| **`zigbee-aps`** | Application Support Sub-layer. APS frame encode/decode, binding table, group table, APS security, fragmentation, and duplicate detection. |
| **`zigbee-zdo`** | Zigbee Device Objects (endpoint 0). Handles discovery (`Active_EP_req`, `Simple_Desc_req`, `Match_Desc_req`), binding, and network management requests. |
| **`zigbee-bdb`** | Base Device Behavior. Implements BDB commissioning: network steering (end devices join), network formation (coordinators create), Finding & Binding, and Touchlink. |
| **`zigbee-zcl`** | Zigbee Cluster Library. 33 clusters, foundation commands (Read/Write/Report/Discover Attributes), attribute storage engine, and reporting engine. |
| **`zigbee-runtime`** | The integration layer your application uses. Provides `DeviceBuilder`, `ZigbeeDevice`, typed application profiles, `ZigbeeNode`, persistence algorithms, reporting, and power management. |
| **`zigbee`** | Top-level umbrella crate. Re-exports all sub-crates and adds coordinator/router role implementations. |

## Reusable Typed Profiles

`zigbee_runtime::profile` owns cluster instances, endpoint composition,
default reporting, and application-value mapping. Product crates select and
configure these reusable archetypes:

| Archetype | Implemented composition |
|-----------|-------------------------|
| `TemperatureHumidityBattery` | Temperature + humidity + power configuration |
| `TemperatureHumidityPressureBattery` | `TemperatureHumidityBattery` plus a mandatory Pressure Measurement cluster, via `TemperatureHumidityBattery::with_pressure` |
| `AirQuality` | CO₂ + temperature + humidity; optional battery |
| `Thermostat` | Thermostat local temperature/setpoint/schedule controls; optional humidity and battery |
| `OccupancyLight` | Occupancy sensing + illuminance; optional battery |
| `PlantSensor` | Soil moisture + temperature + illuminance; optional battery |
| `SmartPlug` | On/Off + basic electrical measurement; optional delivered-energy/demand metering |

Profiles do not advertise decorative clusters. For example, the air-quality
profile does not add PM2.5 until a product supplies that measurement, the
occupancy profile does not pretend to be an IAS security zone, and the smart
plug does not claim unimplemented Simple Metering commands. Illuminance input
is the ZCL-encoded measured value; the `no_std` runtime does not invent a
floating-point `log10` conversion.

Most "optional cluster" archetypes above (`AirQuality`, `Thermostat`,
`OccupancyLight`, `PlantSensor`) hold their optional cluster as an
`Option<Cluster>` field, which is appropriate when most products using that
archetype are expected to fit the optional hardware. Pressure is different:
it is the one uncommon variant of `TemperatureHumidityBattery`, fitted only
by the nRF52840 BME280 product, while EFR32 and ESP32 products never call
`with_pressure`. Because `ClusterRef` holds `&mut dyn Cluster`, an
`Option<PressureCluster>` field on `TemperatureHumidityBattery` would still
link `PressureCluster`'s `Cluster` vtable and attribute storage into *every*
firmware built from that archetype — EFR32 and ESP32 included — regardless
of whether it is ever `Some`. `with_pressure` therefore returns the distinct
`TemperatureHumidityPressureBattery` type instead, so only the one product
that actually composes it pays for it.

## Data Flow

### TX Path (Application → Radio)

When your application updates an attribute or sends a report, data flows
**down** through the stack:

```text
Application
  │  set_temperature(2350)
  ▼
ZCL         serialize attribute report frame
  │
  ▼
APS         wrap in APS Data Request, add APS header + security
  │
  ▼
NWK         add NWK header, route lookup, NWK encryption (AES-CCM*)
  │
  ▼
MAC         add MAC header, CRC, call MacDriver::mcps_data_request()
  │
  ▼
Radio       802.15.4 RF transmission
```

In code, this is what happens when the runtime's `tick()` method detects a due
attribute report:

```rust,ignore
// Inside tick() → check_and_send_cluster_reports() → send_report()
//   builds ZCL frame → APS Data Request → NWK Data Request → MAC Data Request
```

### RX Path (Radio → Application)

Incoming frames flow **up**. The application drives this by calling
`node.device_mut().receive()` and then `node.process_incoming()`:

```text
Radio       802.15.4 frame received
  │
  ▼
MAC         MacDriver::mcps_data_indication() returns raw frame
  │
  ▼
NWK         parse NWK header, verify destination, decrypt if secured
  │
  ▼
APS         parse APS header, de-duplicate, reassemble fragments
  │
  ▼
ZDO / ZCL   endpoint 0 → ZDO handles automatically
             endpoints 1-240 → ZCL dispatches to your clusters
  │
  ▼
Application  StackEvent returned to your code
```

## Async Model

zigbee-rs is designed for **single-threaded cooperative async** runtimes,
primarily [Embassy](https://embassy.dev/):

- **`no_std` throughout** — no heap allocation, no `std::thread`, no OS.
- **`async` without `Send`/`Sync`** — the `MacDriver` trait uses `async fn`
  methods with no `Send` bounds, matching Embassy's single-core executor model.
- **Periodic ticks** — your main loop calls `node.tick(elapsed_secs)` periodically.
  The profile owns the application clusters, so callers do not rebuild
  `ClusterRef` arrays before every operation.
  Between ticks the executor can run other tasks (sensor reads, display updates,
  button debouncing). The runtime never blocks indefinitely.
- **`select!` pattern** — the idiomatic event loop uses `embassy_futures::select`
  to race `device.receive()` against a timer, processing whichever fires first:

```rust,ignore
loop {
    match select(device.receive(), Timer::after(Duration::from_secs(10))).await {
        Either::First(Ok(frame)) => {
            node.process_incoming(&frame).await;
        }
        Either::First(Err(_)) => {}  // MAC error, retry
        Either::Second(_) => {
            // Timer fired — run periodic maintenance
            node.tick(10).await;
        }
    }
}
```

On host machines (mock examples), `pollster::block_on` replaces Embassy as the
executor, so the same stack code compiles for both embedded and desktop.

## Memory Model

Every buffer and collection in zigbee-rs has a **compile-time upper bound**:

- **`heapless::Vec<T, N>`** — fixed-capacity vectors for endpoint lists,
  cluster lists, pending responses, and frame buffers. No `alloc` crate needed.
- **Const generics** — limits like `MAX_ENDPOINTS` (8) and
  `MAX_CLUSTERS_PER_ENDPOINT` (16) are `const` values, so the compiler knows
  the exact memory footprint at build time.
- **Static allocation** — `ZigbeeDevice` and all its nested layers
  (`BdbLayer<M>` → `ZdoLayer` → `ApsLayer` → `NwkLayer<M>` → `M`) live on
  the stack or in a `static` cell. There is no `Box`, `Rc`, or `Arc`.
- **No `serde`** — frame serialization/deserialization uses manual bitfield
  parsing, keeping binary size small and avoiding trait-object overhead.

This means you can predict the **exact RAM usage** of a zigbee-rs device at
compile time — critical for microcontrollers with 32–64 KB of SRAM.

### Typical Memory Budget

| Component | Approximate Size |
|-----------|-----------------|
| `ZigbeeDevice` (full stack) | ~4–6 KB |
| Each ZCL cluster instance | 100–500 bytes |
| NWK routing table | ~200 bytes |
| APS binding + group tables | ~300 bytes |
| Frame buffers (TX + RX) | ~256 bytes each |

## Layer Nesting

Each layer wraps the one below it using generics, not trait objects:

```rust,ignore
ZigbeeDevice<M: MacDriver>
  └── BdbLayer<M>
        └── ZdoLayer<M>
              └── ApsLayer<M>
                    └── NwkLayer<M>
                          └── M   // your MacDriver (MockMac, Esp32Mac, ...)
```

This means the **concrete MAC type propagates** all the way up. There is zero
dynamic dispatch in the stack path — the compiler monomorphizes everything,
producing tight, inlineable code for each target platform.

## What's Next?

- **[Your First Device](./first-device.md)** — build a temperature sensor step by step
- **[The Device Builder](../core-concepts/builder.md)** — detailed builder API reference
- **[The Event Loop](../core-concepts/event-loop.md)** — how `tick()` and `process_incoming()` work
