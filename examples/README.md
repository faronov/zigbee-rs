# Examples

Hardware examples are standalone firmware crates. Host examples are small,
finite demonstrations of the same shared application frontends.

## What an example owns

An embedded `main.rs` is a composition root:

```text
application/profile
        ↓
product identity, profile selection, policy, storage, linker, OTA
        ↓
board resources
        ↓
chip HAL / MacDriver
```

The example `main.rs` wires those four layers. It may initialize clocks,
construct exclusive peripherals, install the radio
and AES backend, open product-owned storage, build the profile/device, and run
the platform executor. It must not copy commissioning, polling, reporting,
router, persistence, or OTA state machines out of the shared application and
runtime crates.

Some platform-adapter crates still contain thin compatibility wrappers. They
delegate to `apps/sensor-sed`; they are not independent local application
implementations.

## Shared sensor composition

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

`SensorSedParts` exposes all owned capabilities. `SensorPolicy` separately
selects the fast- and slow-window sleep depths. Integrations with an outer
scheduler can call `initialize()` once and then `step()` for one finite
wait/service iteration.

Use `NoStatus` only when the product has no status indicator; this compiles
out status-only waits. Use `NoOta` only with a profile that explicitly
implements `NonOtaProfile`.

## Shared router composition

Choose the frontend that matches the MAC backend:

- `AlwaysOnEndDeviceApp`: non-routing and receiver-on-when-idle; requires
  `MacDriver`.
- `RelayRouterApp + NoChildren`: forwarding only; requires the `router`
  feature and `ParentMacDriver`.
- `ParentRouterApp + PersistentChildren`: admits children; requires
  `ParentMacDriver`.
- `CoordinatorApp + PersistentChildren`: forms or restarts a PAN; requires
  `ParentMacDriver`.

`router-app` defaults to the End Device-only build. Relay, parent-router, and
coordinator compositions explicitly enable its `router` feature.

The nRF52840 always-on End Device example is deliberately the first case. Its
MAC lacks `ParentMacDriver`, so it cannot advertise a Router descriptor.
Only the legacy Telink `TelinkRouterMac`, the host-tested
`SoftMacCore<_, Parent<..>>` and host-only `MockMac` implement
`ParentMacDriver`, and no production backend advertises coordinator
capability.

## Host examples

| example | purpose |
|---|---|
| [`mock-sleepy-sensor`](mock-sleepy-sensor/) | explicit `SensorSedParts`; finite `initialize()` plus four `step()` calls |
| [`mock-light`](mock-light/) | finite forwarding `RelayRouterApp` and ZCL light state |
| [`mock-coordinator`](mock-coordinator/) | typed `CoordinatorApp`, formation, persisted-PAN restart, and finite steps |

Run them from their directories with the pinned general toolchain:

```bash
cargo +nightly-2026-03-23 run --locked
```

## Sensor firmware

“Prior evidence” below describes earlier path-level hardware runs, not a
byte-for-byte rerun of the current image. Optional sensor drivers come from
[`zigbee-rs-drivers`](https://github.com/faronov/zigbee-rs-drivers), pinned by
git `rev`.

| example | board/chip | wait depths | status and OTA | validation |
|---|---|---|---|---|
| [`nrf52840-sensor`](nrf52840-sensor/) | PCA10056 / nRF52840 | `Idle` / `Idle` | fitted LED, `NoOta` | build/layout; prior sensor-path evidence |
| [`nrf52833-sensor`](nrf52833-sensor/) | PCA10100 / nRF52833 | `Idle` / `Idle` | fitted LED, `NoOta` | build/layout; prior sensor-path evidence |
| [`nrf52840-sensor-uf2`](nrf52840-sensor-uf2/) | ProMicro, MDK, PCA10059, or DK | `Idle` / `Idle` | board-dependent status, `NoOta` | build/layout matrix; bootloader compatibility remains board-specific |
| [`esp32c6-sensor`](esp32c6-sensor/) | ESP32-C6 DevKit | `Active` / `Active` | `NoStatus`, OTA | real `esp-radio`; prior commissioning/reporting and 18.3% OTA evidence; activation open |
| [`esp32h2-sensor`](esp32h2-sensor/) | ESP32-H2 | `Active` / `Active` | active-low LED, OTA | real `esp-radio`; prior complete v1→v2 OTA evidence |
| [`bl702-sensor`](bl702-sensor/) | XT-ZB1 / BL702 | `Idle` / `Idle` | `NoStatus`, signed OTA with own loader | HIL: join, interview, Idle long poll, leave, reset/resume, 0.1.0→0.1.1 OTA; sleep current unmeasured |
| [`phy6222-sensor`](phy6222-sensor/) | PHY6222/PHY6252 EVK | `Idle` / `Idle` | fitted status, `NoOta` | exact feature images build/layout-tested; hardware path open |
| [`pb03f-kit`](pb03f-kit/) | Ai-Thinker PB-03F-Kit / PHY6252 | `Idle` / `Idle` | fitted status, `NoOta` | HIL: ROM boot, radio, secured join, ZHA interview/reporting, reset/resume; also `pb03f-lab` and `pb03f-radio-lab` |
| [`cc2340-sensor`](cc2340-sensor/) | LP-EM-CC2340R5 | `Idle` / `Retention` | fitted status, `NoOta` | HIL on one board: join, interview, auto-ACK, standby current; entropy fails closed, commissioning not claimed |
| [`efr32mg1-sensor`](efr32mg1-sensor/) | TRÅDFRI / EFR32MG1P | `Active` / `Retention` | fitted LEDs, OTA | build/layout; prior path evidence; real OTA install open |
| [`efr32mg21-sensor`](efr32mg21-sensor/) | BRD4181A / EFR32MG21 | `Idle` / `Idle` | PB0 LED, `NoOta` | compile/layout only |
| [`telink-tlsr8258-sensor`](telink-tlsr8258-sensor/) | TB-04 / TLSR8258 | `Active` / `Idle` by default | RGB status, `NoOta` | `SoftMacCore` images build/layout-tested; prior SUSPEND primitive evidence (hand-written MAC); LOW32K HIL open |

[`telink-tlsr8258-radio`](telink-tlsr8258-radio/) is the standalone TLSR8258
radio bring-up harness (raw TX/RX, scan, association, `MacDriver` gates).

See [the build matrix](../BUILD.md) for exact pinned commands and measured
image sizes.

## Router firmware

| example | frontend | child support | commissioning after Leave | validation |
|---|---|---|---|---|
| [`nrf52840-router`](nrf52840-router/) | `AlwaysOnEndDeviceApp` | none | Button 1 short press | build/layout; HIL acceptance open |
| [`telink-tlsr8258-router`](telink-tlsr8258-router/) | `ParentRouterApp` | persistent children, bindings, groups, optional durable application-key installation | power cycle (TB-04 has no button) | build/layout; prior join/restart/relay evidence; corrected child acceptance open |

## Telink power variants

From the repository root:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
./scripts/tlsr8258.sh build router
```

The default sensor uses full-SRAM timer `SUSPEND`. The two retention images
are feature-gated LOW32K reset-on-wake proofs at 250 ms and 10 seconds.
The former independent retention HIL image was removed with
`tools/telink-tlsr8258-lab`; the LOW32K hardware gate remains open for the
current proof images and must run on them directly.

## Porting

To run an existing application on another MCU:

1. supply a real monotonic clock and `MacDriver`;
2. expose fitted hardware from a board crate;
3. define product identity, profile, policy, storage, and linker boundaries;
4. implement the narrow `SensorSedParts` or `RouterParts` adapters;
5. copy only the composition pattern, not an application's state machine.

The profile, `SensorApp`/router frontend, and protocol crates should remain
unchanged.
