# ESP32-C6 Zigbee Temperature & Humidity Sensor

A `no_std` Zigbee 3.0 end device for the **ESP32-C6** that reads the on-chip
temperature sensor and reports temperature, humidity, and battery percentage.
Network state is persisted to flash — the device survives reboots without
re-pairing.

> **✅ Verified** on ESP32-C6-DevKitC-1 with Home Assistant + ZHA. Shows as
> "Zigbee-RS ESP32-C6-Sensor" with Temperature, Humidity, and Battery entities.

## Hardware Requirements

- ESP32-C6 development board (built-in IEEE 802.15.4 radio)
- USB cable for flashing and serial monitor
- BOOT button (GPIO9) used for join/leave control

## Prerequisites

- Rust nightly toolchain with `rust-src`
- `espflash` for flashing: `cargo install espflash`
- Target: `riscv32imac-unknown-none-elf` (added automatically via `.cargo/config.toml`)

No vendor libraries or binary blobs are needed — the project uses the
`esp-ieee802154` crate for native 802.15.4 radio access.

## Build

```sh
cargo build --release
```

## Flash & Monitor

```sh
espflash flash --monitor target/riscv32imac-unknown-none-elf/release/esp32c6-sensor
```

Or use the configured runner:

```sh
cargo run --release
```

## What It Demonstrates

- Initialising the ESP32-C6 IEEE 802.15.4 radio with `esp-radio`
- Building a Zigbee device with the `ZigbeeDevice` builder API
- Registering ZCL endpoint 1 (Home Automation profile, device type 0x0302)
  with **Basic**, **Power Configuration**, **Identify**, **Temperature Measurement**,
  and **Relative Humidity** clusters
- **On-chip temperature sensor** via `esp_hal::tsens::TemperatureSensor`
- **Flash NV storage** — network state saved to last 2 sectors of flash (`0x3FE000`, 8 KB)
  using `esp_storage::FlashStorage` + log-structured NV format
- **NWK Leave handler** — auto-erases NV and rejoins when coordinator sends Leave
- **Default reporting configuration** — temp/humidity: 60–300 s, battery: 300–3600 s
  (devices report data even before ZHA sends ConfigureReporting)
- **Identify cluster** (0x0003) — supports Identify, IdentifyQuery, TriggerEffect commands
- Battery percentage reporting via Power Configuration cluster
- Button-driven network join/leave via `UserAction::Toggle`

## Operation

1. Power on → restores saved network state from flash (if any) and auto-rejoins
2. If no saved state → press BOOT to initiate BDB commissioning
3. Once joined → reads on-chip temp sensor periodically, reports to coordinator
4. Press BOOT → leaves the network and clears flash NV storage
5. **Power cycle** → device reconnects automatically (no re-pairing needed!)

## Project Structure

```
esp32c6-sensor/
├── .cargo/config.toml   # Target, runner, rustflags, build-std
├── Cargo.toml            # Dependencies (esp-hal 1.0, esp-radio 0.17, esp-storage, zigbee-rs crates)
└── src/main.rs           # Application entry point (#[esp_hal::main])
```

`boards/esp32-zigbee-devkit` owns the bounded 8 KB partition and constructs
the shared `LogStructuredNv` store for both ESP32-C6 and ESP32-H2.
