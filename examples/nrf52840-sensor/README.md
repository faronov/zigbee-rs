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

All variants include: Basic, Power Configuration, Battery voltage (SAADC),
RAM power-down for unused banks, and auto-recovery on sensor failure.

## Hardware Requirements

- nRF52840-DK (PCA10056) or any nRF52840 board with a debug probe
- Button 1 (P0.11, active low) for join/leave control
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

Both drivers are **fully async** — they use embassy's TWIM (DMA-based I2C master)
and yield during transfers, so the Zigbee radio continues processing uninterrupted.

## What It Demonstrates

- Embassy async event loop with `select3` (radio receive, button press, timer)
- On-chip TEMP sensor or async external I2C sensor (BME280 / SHT31)
- Building a Zigbee device with `ZigbeeDevice` builder API
- ZCL endpoint 1 (Home Automation, device type 0x0302) with **Basic**,
  **Power Configuration**, **Temperature Measurement**, **Relative Humidity**,
  and optionally **Pressure Measurement** (BME280 only) server clusters
- Automatic sensor recovery on read failure (re-init next cycle)
- Processing incoming MAC frames and generating ZCL attribute reports
- Button-driven network join/leave via `UserAction::Toggle`
- RAM power-down of unused banks (~190 KB saved on nRF52840)
- **Flash NV storage** — network state persists across power cycles (last 8 KB of flash)
- Battery voltage monitoring via SAADC (VDD internal divider)
- `log` → `defmt` bridge for stack-internal logging via RTT

## Operation

1. Power on → restores saved network state from flash (if any) and auto-rejoins
2. If no saved state → press Button 1 to initiate BDB commissioning
3. Once joined → reads sensors every 30 s, reports to coordinator; state saved to flash
4. Press Button 1 → leaves the network and clears flash NV storage
5. **Power cycle** → device reconnects automatically (no re-pairing needed!)

## Project Structure

```
nrf52840-sensor/
├── .cargo/config.toml   # Target, runner (probe-rs), DEFMT_LOG level
├── Cargo.toml            # Features (sensor-bme280, sensor-sht31), deps
├── build.rs              # Linker script flags (-Tlink.x -Tdefmt.x)
├── memory.x              # Memory layout: 1016 KB Flash + 8 KB NV, 256 KB RAM
└── src/
    ├── bme280.rs         # Async BME280 I2C driver (feature: sensor-bme280)
    ├── flash_nv.rs       # Flash-backed NV storage (NVMC, last 2 pages)
    ├── sht31.rs          # Async SHT31 I2C driver (feature: sensor-sht31)
    └── main.rs           # Async entry point (#[embassy_executor::main])
```
