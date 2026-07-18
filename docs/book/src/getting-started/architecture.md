# Architecture Overview

zigbee-rs is a complete Zigbee PRO R22 protocol stack written in Rust, split
across **9 crates** that mirror the standard Zigbee layer model. Every crate is
`#![no_std]` and heap-free — suitable for the smallest microcontrollers.

## Layer Diagram

```text
┌─────────────────────────────────────┐
│  Application (your code)            │
├─────────────────────────────────────┤
│  zigbee-runtime (ZigbeeDevice)      │
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
`zigbee-runtime` layer through `ZigbeeDevice`.

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
| **`zigbee-runtime`** | The integration layer your application uses. Provides `DeviceBuilder`, `ZigbeeDevice`, the event loop (`tick()` / `process_incoming()`), NV storage abstraction, power management, and pre-built device templates. |
| **`zigbee`** | Top-level umbrella crate. Re-exports all sub-crates and adds coordinator/router role implementations. |

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
`device.receive()` and then `device.process_incoming()`:

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
- **`stack_tick()` polling** — your main loop calls `device.tick(elapsed_secs, clusters)` periodically.
  Between ticks the executor can run other tasks (sensor reads, display updates,
  button debouncing). The runtime never blocks indefinitely.
- **`select!` pattern** — the idiomatic event loop uses `embassy_futures::select`
  to race `device.receive()` against a timer, processing whichever fires first:

```rust,ignore
loop {
    match select(device.receive(), Timer::after(Duration::from_secs(10))).await {
        Either::First(Ok(frame)) => {
            device.process_incoming(&frame, &mut clusters).await;
        }
        Either::First(Err(_)) => {}  // MAC error, retry
        Either::Second(_) => {
            // Timer fired — run periodic maintenance
            device.tick(10, &mut clusters).await;
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
