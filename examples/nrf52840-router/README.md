# nRF52840 Zigbee Router

A `no_std` experimental Full Function Device target for the
**nRF52840-DK**. It joins an existing network with the router capability bit
and exercises always-on receive and NWK forwarding paths.

This is not yet a complete Zigbee router: the nRF MAC does not implement
`MLME-ASSOCIATE.response`, so it cannot accept child devices or buffer
indirect frames for sleeping children.

## Hardware

- **MCU:** Nordic nRF52840 — ARM Cortex-M4F, 64 MHz, 1 MB Flash, 256 KB RAM
- **Radio:** Built-in IEEE 802.15.4 (no SoftDevice needed)
- **Board:** nRF52840-DK (PCA10056)
- **LED1 (P0.13):** Solid ON = joined, blink = joining
- **LED2 (P0.14):** Blinks on frame relay
- **Button 1 (P0.11):** Short press = toggle join/leave, long press = factory reset

## Features

- Joins existing Zigbee network as a router (FFD)
- Continuous RX (`rx_on_when_idle = true`)
- Exercises unicast and broadcast forwarding paths
- RREQ rebroadcast for route discovery
- Periodic Link Status broadcasts (every 15 seconds)
- NWK Leave handler with auto-rejoin
- Button-driven join/leave with factory reset

## Prerequisites

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools
```

## Building

```bash
cd examples/nrf52840-router
cargo build --release
```

## Flashing

```bash
# Flash + live defmt log output
cargo run --release

# Or flash only
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-router
```

## How It Works

Unlike sensor examples (which are Sleepy End Devices), the router:

1. **Never sleeps** — radio is always on to relay frames
2. **Exercises forwarding** — unicast and broadcast NWK paths stay active
3. **Sends Link Status** — periodic broadcasts so neighbors know it's alive
4. **Participates in routing** — AODV route discovery and RREQ rebroadcast

Child admission remains disabled until the backend implements association
indications/responses, pending transactions, and indirect transmission.

The router uses `PowerMode::AlwaysOn` and does not implement any sleep
logic. DC-DC converters are enabled for lower power consumption while
the radio is continuously active.

## Project Structure

```
nrf52840-router/
├── .cargo/config.toml   # Target: thumbv7em-none-eabihf, probe-rs runner
├── Cargo.toml            # Dependencies
├── build.rs              # Linker script setup
├── memory.x              # Flash @ 0x00000000, RAM @ 0x20000000
└── src/
    └── main.rs           # Router entry point, event loop, frame relay
```

## Expected Serial Output (via RTT)

```
INFO  Zigbee-RS nRF52840 ROUTER starting…
INFO  Radio ready
INFO  Device ready — press Button 1 to join/leave
INFO  [btn] Joining network…
INFO  [scan] Scanning channels 11-26…
INFO  [scan] Found network: ch=15, PAN=0x1AAA
INFO  [join] Association successful, addr=0x5678
INFO  [router] Link Status broadcast
INFO  [relay] Relayed frame 0x1234 → 0x0000
```
