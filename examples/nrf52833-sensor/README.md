# nRF52833 Zigbee Temperature Sensor

Embassy-based firmware that reads the on-chip temperature sensor and exposes
temperature and humidity via Zigbee ZCL clusters.

## Prerequisites

- Rust nightly with the `thumbv7em-none-eabihf` target:
  ```
  rustup target add thumbv7em-none-eabihf
  ```
- **probe-rs** for flashing and defmt log output:
  ```
  cargo install probe-rs-tools
  ```
- nRF52833-DK (or any board with J-Link / CMSIS-DAP debug probe)

## Build & Flash

```sh
cargo build --release
cargo run --release   # flashes via probe-rs and shows defmt logs
```

## Hardware

- **On-chip TEMP sensor** — real temperature in °C
- **Button 1 (P0.11)** — press to join/leave the Zigbee network
- **Simulated humidity** — cycles 45–55 % (connect an SHTC3 for real data)

## Memory Usage

| Resource | Available | Used    | Free   |
|----------|-----------|---------|--------|
| Flash    | 512 KB    | ~37 KB  | ~475 KB |
| RAM      | 128 KB    | ~33 KB  | ~95 KB  |
