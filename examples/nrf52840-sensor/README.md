# nRF52840 Zigbee Sensor (DK / J-Link)

An async Embassy-based Zigbee 3.0 sleepy end device for the **Nordic nRF52840-DK**.
Supports optional external I2C sensors (BME280, SHT31) via feature flags, with
on-chip TEMP as the default fallback. Uses `defmt` + RTT for logging.

## Features

| Feature         | Sensor  | Clusters                    |
|-----------------|---------|------------------------------|
| *(default)*     | On-chip TEMP | Temp + fake humidity    |
| `sensor-bme280` | BME280  | Temp + humidity + pressure   |
| `sensor-sht31`  | SHT31   | Temp + humidity              |

All variants include: Basic, Power Configuration, **Identify** (LED blink),
Battery voltage (SAADC), atomic security persistence, NWK Leave handling,
default reporting, and auto-recovery on sensor failure.

## Hardware Requirements

- nRF52840-DK (PCA10056) or any nRF52840 board with a debug probe
- Button 1 (P0.11, active low) for forced reporting, immediate join, and factory reset
- Debug probe (J-Link on-board for DK, or external probe-rs-compatible)
- (Optional) BME280 or SHT31 breakout wired to I2C (see below)

## Prerequisites

- Rust stable toolchain
- `probe-rs`: `cargo install probe-rs-tools`
- Target: `thumbv7em-none-eabihf` (configured in `.cargo/config.toml`)

No vendor libraries, SoftDevice, or binary blobs are needed — the project
drives the 802.15.4 radio directly via `embassy-nrf`.

## Build

```sh
# Default (on-chip temp + fake humidity):
cargo build --release

# With BME280 (temp + humidity + pressure):
cargo build --release --features sensor-bme280

# With SHT31 (temp + humidity):
cargo build --release --features sensor-sht31
```

## Flash & Run

```sh
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-sensor
```

Or use the configured runner:

```sh
cargo run --release
```

## I2C Sensor Wiring (BME280 / SHT31)

| Sensor Pin | nRF52840 Pin | Notes |
|------------|-------------|-------|
| SDA        | P0.26       | I2C data |
| SCL        | P0.27       | I2C clock |
| VCC        | 3.3V        | |
| GND        | GND         | |
| ADDR (SHT31) | GND      | Address 0x44 (or VCC for 0x45) |

BME280 I2C address: 0x76 (SDO→GND) or 0x77 (SDO→VCC).

Both drivers are the shared, workspace-level `zigbee-bme280` / `zigbee-sht3x`
crates (`drivers/bme280`, `drivers/sht3x`) — the same register map, reset, and
oversampling as before, now reused instead of hand-duplicated. They are
**fully async** — they use embassy's TWIM (DMA-based I2C master) and yield
during transfers, so the Zigbee radio continues processing uninterrupted.
See `src/sensor.rs` for the recoverable probe/read wiring.

## What It Demonstrates

- Embassy async event loop (`SensorApp::run` in `apps/nrf-sensor`) using `select` (button press vs. poll timer) plus a bounded per-iteration MAC poll/receive window
- On-chip TEMP sensor or async external I2C sensor (BME280 / SHT31)
- Building a Zigbee device with `ZigbeeDevice` builder API
- ZCL endpoint 1 (Home Automation, device type 0x0302) with **Basic**,
  **Power Configuration**, **Identify**, **Temperature Measurement**, **Relative Humidity**,
  and optionally **Pressure Measurement** (BME280 only) server clusters
- **NWK Leave handler** — auto-erases NV and rejoins when coordinator sends Leave
- **Default reporting configuration** — temp/humidity: 60–300 s, battery: 300–3600 s
  (devices report data even before ZHA sends ConfigureReporting during interview)
- **Identify cluster** (0x0003) — LED blinks during Identify
- Automatic sensor recovery on read failure (re-init next cycle)
- Processing incoming MAC frames and generating ZCL attribute reports
- Button-driven network join/leave via the security-store lifecycle APIs
- **Atomic security journal** — CRC, generation, and commit-protected security state
  persists across power cycles in the last 8 KiB of flash (owned by the
  `nrf52840-sensor-product` product crate; see Architecture below)
- Battery voltage monitoring via SAADC (VDD internal divider)
- `log` → `defmt` bridge for stack-internal logging via RTT
- Endpoint/cluster composition, reporting defaults, and measurement mapping
  come from the shared `zigbee_runtime::profile::TemperatureHumidityBattery`
  archetype, selected by `nrf52840-sensor-product`. With `sensor-bme280`,
  the product instead selects `TemperatureHumidityPressureBattery` (built
  via `TemperatureHumidityBattery::with_pressure`), a distinct type so
  builds without BME280 never link the Pressure Measurement cluster

## Power Optimizations

The firmware applies several hardware-level power optimizations to minimize
battery drain:

| Optimization | Setting | Savings |
|-------------|---------|---------|
| DC-DC converter | `reg0` + `reg1` enabled | ~40% lower current |
| TX power | 0 dBm (down from +8 dBm) | ~50% TX current |
| HFCLK source | Internal RC (radio auto-requests XTAL) | ~250 µA idle |

`main.rs` also defines `power_down_unused_ram()`, which would power off the
unused ~192 KiB RAM bank 8 (`#[allow(dead_code)]`). It is **not currently
called** from `main()` — wiring it in requires re-verifying stack/BSS
headroom against the executor's task arena on hardware first. Treat "RAM
power-down" as an available-but-unwired primitive, not an active
optimization, until that hardware gate is closed.

**Polling scheme:**
- Fast poll: 250 ms for 120 seconds after join/activity
- Slow poll: 30 seconds in steady state
- Report interval: 60 seconds

**Reportable change thresholds** suppress unnecessary transmissions:
- Temperature: ±0.5 °C
- Humidity: ±1%
- Battery: ±2%

With these optimizations, a CR2032 (230 mAh) can last several years in a
stable environment.

## Operation

1. Power on → atomically restores saved security state (if any) and auto-resumes
2. If no saved state → automatically initiates BDB commissioning
3. Once joined → reads sensors every 60 s, reports to coordinator; state saved to flash
4. Short-press Button 1 → force an immediate sensor and battery report
5. Hold Button 1 for 3 s → durable factory reset and reboot
6. **Power cycle** → device reconnects automatically (no re-pairing needed!)

## Project Structure

```
nrf52840-sensor/
├── .cargo/config.toml   # Target, runner (probe-rs), linker path, DEFMT_LOG
├── Cargo.toml            # Features (sensor-bme280, sensor-sht31), deps
└── src/
    ├── sensor.rs         # Recoverable BME280/SHT31 wiring over the shared
    │                     # zigbee-bme280 / zigbee-sht3x drivers (feature-gated)
    └── main.rs           # Composition root: platform startup, resource
                          # construction, hardware AES + identity guard,
                          # battery-policy binding, then hands everything to
                          # nrf_sensor_app::SensorApp::run()
```

The lifecycle itself is **not** in this example. `SensorApp` (join/leave/
rejoin, polling, reporting, Identify, button, durable checkpointing) and the
host-tested poll-delay `policy` live in [`apps/nrf-sensor`](../../apps/nrf-sensor)
and are shared byte-for-byte with `examples/nrf52833-sensor`; the host tests
for `policy` are in `tests/src/nrf_sensor_policy_tests.rs`.

## Architecture

This example is a composition root only. Physical wiring, memory layout,
persistence, and Zigbee protocol behavior are owned by separate crates:

```
nrf52840-sensor (example)   startup, resource construction, policy binding
        |
nrf-sensor-app              full commissioning/event-loop lifecycle, shared
        |                   with examples/nrf52833-sensor (apps/nrf-sensor)
        |
nrf52840-sensor-product     identity, link/memory.x, security journal
        |                   partition, concrete profile (products/nrf52840-sensor)
        |
nrf52840-dk (board)         LED1/Button 1/sensor-I2C pin wiring only
                            (boards/nrf52840-dk; no zigbee-runtime dependency)
```

`products/nrf52840-sensor` owns the NVMC partition (`src/storage.rs`) and
linker layout (`link/memory.x`) that reserve the last 8 KiB of flash for the
shared, crash-safe security journal, plus the battery voltage/percentage
curve (`src/battery.rs`) and the concrete Zigbee profile (`src/profile.rs`,
built from the shared `zigbee_runtime::profile::TemperatureHumidityBattery`
archetype). `boards/nrf52840-dk` only wires LED1, Button 1, and the sensor
I2C bus pins, and has no dependency on `zigbee-runtime`.

`link/memory.x` additionally carries link-time `ASSERT`s that fail the build
if the application region is ever grown over the journal partition — the
Rust-side `const` assertions in `src/storage.rs` check the same boundary.
