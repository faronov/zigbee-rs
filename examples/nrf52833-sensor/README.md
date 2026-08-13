# nRF52833 Zigbee Sensor (production)

An async Embassy-based Zigbee 3.0 **sleepy end device** for the Nordic
nRF52833-DK (PCA10100). It runs the *same* application as
`examples/nrf52840-sensor` — both firmwares are thin composition roots over
the shared [`apps/nrf-sensor`](../../apps/nrf-sensor) crate — so the two
products cannot drift apart in commissioning, persistence, polling,
reporting, or button behavior.

## Hardware Requirements

- nRF52833-DK (PCA10100) or any nRF52833 board with a debug probe and a
  32 MHz crystal (required by the 802.15.4 radio)
- LED1 (P0.13) and Button 1 (P0.11) — wired by
  [`boards/nrf52833-dk`](../../boards/nrf52833-dk)
- Optional external I2C sensor on P0.26 (SDA) / P0.27 (SCL)
- Debug probe (on-board J-Link on the DK, or any probe-rs-compatible probe)

## Prerequisites

- Rust nightly toolchain
- `probe-rs`: `cargo install probe-rs-tools`
- Target: `thumbv7em-none-eabihf` (configured in `.cargo/config.toml`)

No vendor libraries, SoftDevice, or binary blobs are needed — the project
drives the 802.15.4 radio directly via `embassy-nrf`, and AES-128 runs on the
Nordic ECB peripheral (software AES is rejected by CI symbol gates).

## Build

```sh
cargo build --release                          # on-chip TEMP + synthetic humidity
cargo build --release --features sensor-bme280 # BME280: temp + humidity + pressure
cargo build --release --features sensor-sht31  # SHT31: temp + humidity
```

## Flash & Run

```sh
cargo run --release
# or
probe-rs run --chip nRF52833_xxAA target/thumbv7em-none-eabihf/release/nrf52833-sensor
```

## Operation

1. Power on → LED1 solid for 2 s (boot signal).
2. **Commissioning starts automatically** — no button press is required. If
   durable state exists, the device resumes silently instead of scanning.
3. While unjoined: LED1 double-blinks and a join is retried every 15 s.
4. Once joined: LED1 on, 250 ms fast polling for 120 s (or until the
   coordinator finishes `ConfigureReporting`), then 30 s slow polling.
   Sensors are sampled and reported every 60 s.
5. Security state (network key, frame counters, parent, short address) is
   checkpointed to a protected flash partition after every lifecycle event.

Button 1 is an **operator override only**:

| Action | Effect |
|--------|--------|
| Short press while joined | Read sensors and force an immediate temperature, humidity, and battery report |
| Short press while unjoined | Start a join/resume attempt immediately |
| Hold 3 s | Durable factory reset, LED confirmation blink, reboot |

The device declares its interview configuration complete only after it has
received coordinator-originated `ConfigureReporting` commands for every
reportable cluster in the active profile (3/3 for the on-chip/SHT31 build,
4/4 with BME280 pressure). RTT logs show the current count.

## What It Demonstrates

- Automatic cold-start commissioning, silent resume, and bounded
  secure-rejoin retry with a factory-reset fallback
- Crash-safe `SecurityStateJournal` in the last 8 KiB of flash, which
  survives reset *and* a firmware reflash
- Factory FICR `DEVICEID`-derived EUI-64 (never a constant)
- Nordic ECB hardware AES-128 with two startup known-answer tests
- ZHA interview, `Identify`, and NWK-secured Temperature/Humidity/Battery
  reporting from the shared `zigbee-runtime` profile archetype
- `defmt` structured logging over RTT, with the stack's own `log` records
  bridged into it

## Layer Ownership

| Concern | Crate |
|---------|-------|
| Lifecycle, polling, reporting, button, LED | `apps/nrf-sensor` |
| Identity, memory map, security partition, battery curve, profile | `products/nrf52833-sensor` |
| LED1 / Button 1 / sensor I2C pins | `boards/nrf52833-dk` |
| Radio, RNG, SAADC, TEMP, NVMC, clocks | `embassy-nrf` |

## Differences from nrf52840-sensor

Only what the silicon forces:

| | nRF52840 | nRF52833 |
|-|----------|----------|
| Flash / RAM | 1 MiB / 256 KiB | 512 KiB / 128 KiB |
| Application flash | 1016 KiB | 504 KiB |
| Security journal | `0x000FE000` | `0x0007E000` |
| Model string | `nRF52840-Sensor` | `nRF52833-Sensor` |
| DC/DC | `reg0` + `reg1` | `reg1` only (`embassy-nrf` exposes `reg0` for nRF52840 only) |
| Runner | `--chip nRF52840_xxAA` | `--chip nRF52833_xxAA` |

Everything else — endpoint, clusters, reporting defaults, timings, retry
bounds, persistence points — is literally the same code.

## Project Structure

```
nrf52833-sensor/
├── .cargo/config.toml   # Target, runner, linker script search path, DEFMT_LOG
├── Cargo.toml           # Dependencies + optional external-sensor features
└── src/
    ├── main.rs          # Composition root only (platform startup + wiring)
    └── sensor.rs        # Optional BME280/SHT31 I2C source
```

The memory layout lives in `products/nrf52833-sensor/link/memory.x`, not
here — the product owns the flash boundaries, and its link-time `ASSERT`s
fail the build if the application region would ever overlap the security
journal.
