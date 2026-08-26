# Quick Start

Start with a host example. It exercises the same finite application frontend
without requiring a radio board.

## Prerequisites

```bash
rustup toolchain install nightly-2026-03-23
git clone https://github.com/faronov/zigbee-rs.git
cd zigbee-rs
git switch experiment/zephyr-app-model
```

The general toolchain is pinned. Do not substitute a moving nightly when
comparing code size or embedded behavior.

## Run the sleepy-sensor application

```bash
cd examples/mock-sleepy-sensor
cargo +nightly-2026-03-23 run --locked
```

The example builds a `ZigbeeNode`, explicit `SensorSedParts`, and a
`SensorApp`. It calls `initialize()` once and `step()` four times, so the
process terminates and is suitable for tests and debuggers.

The composition is the same shape used by hardware products:

```rust,ignore
let mut app = SensorApp::new(
    node,
    &POLICY,
    SensorSedParts {
        wake: HostWake::default(),
        status: NoStatus,
        environment,
        battery,
        ota: NoOta,
        actions: ForceReportAction,
        supervisor,
        diagnostics,
    },
)?;

app.initialize().await?;
for _ in 0..4 {
    app.step().await?;
}
```

`run()` is the non-returning wrapper used by most firmware roots.

## Run typed router examples

```bash
cd ../mock-light
cargo +nightly-2026-03-23 run --locked

cd ../mock-coordinator
cargo +nightly-2026-03-23 run --locked
```

`mock-light` uses `RelayRouterApp + NoChildren`. `mock-coordinator` uses
`CoordinatorApp + PersistentChildren`, forms a PAN, persists it, and proves
that the next initialization restarts the same PAN rather than forming again.

## Run the core tests

```bash
cd ../..
cargo +nightly-2026-03-23 test --workspace --locked
cargo +nightly-2026-03-23 test -p sensor-sed-app --features ota --locked
cargo +nightly-2026-03-23 test -p zigbee-runtime --features router --locked
```

## Build hardware firmware

Hardware crates are not root-workspace members. Use their own directory and
the target-specific pinned toolchain:

```bash
cd examples/nrf52840-sensor
cargo +nightly-2026-03-23 build --release --locked
```

ESP32 and PHY6222 use `nightly-2026-08-01`. Telink uses
`tc32-stage2-tc32-45`. See the
[platform guides](../platform-guides/nrf.md) and
[`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/experiment/zephyr-app-model/BUILD.md)
before flashing.

A successful build is not a hardware-support claim. Read the target guide's
validation section and preserve its bootloader, security journal, factory
identity, and OTA regions.
