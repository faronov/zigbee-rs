# Introduction

**zigbee-rs** is a complete Zigbee PRO R22 protocol stack written in Rust,
targeting embedded `no_std` environments. It runs on real hardware — ESP32,
nRF52, BL702, and more — yet the same code compiles and runs on your laptop
for rapid iteration without touching a single wire.

```text
47,800+ lines of Rust · 161 source files · 9 crates · 33 ZCL clusters · 10 hardware backends
```

## Why Rust for Zigbee?

Zigbee stacks have traditionally been written in C, shipping as opaque vendor
blobs tied to a specific chipset. zigbee-rs takes a different approach:

- **Memory safety without a runtime.** Rust's ownership model eliminates
  buffer overflows, use-after-free, and data races at compile time — exactly
  the classes of bugs that plague C-based embedded stacks.
- **`#![no_std]` and zero heap allocation.** The entire stack builds without
  the standard library. Bounded collections from [`heapless`] replace
  `Vec` and `HashMap`, so every buffer size is known at compile time.
- **`async`/`await` on bare metal.** The MAC layer is an async trait with 13
  methods. Embassy, `pollster`, or any single-threaded executor can drive
  the stack — no RTOS threads, no mutexes.
- **Type-safe ZCL.** Each cluster is a Rust struct with typed attributes.
  You call `temp.set_temperature(2350)` instead of stuffing bytes into an
  anonymous attribute table. The compiler catches mismatched types before
  your firmware ever runs.
- **One stack, many chips.** The platform-specific code lives behind a single
  `MacDriver` trait. Swap an `impl` and the same application logic runs on
  ESP32-C6, nRF52840, BL702, or a host mock.

[`heapless`]: https://docs.rs/heapless

## Project Scope

zigbee-rs is structured as a Cargo workspace with **9 crates**, each
responsible for one protocol layer:

| Crate | Role |
|-------|------|
| `zigbee-types` | Core types — `IeeeAddress`, `ShortAddress`, `PanId`, `ChannelMask` |
| `zigbee-mac` | IEEE 802.15.4 MAC layer + 10 hardware backends |
| `zigbee-nwk` | Network layer — AODV + tree routing, NWK security, NIB |
| `zigbee-aps` | Application Support — binding, groups, APS security |
| `zigbee-zdo` | Zigbee Device Objects — discovery, binding, network management |
| `zigbee-bdb` | Base Device Behavior — steering, formation, commissioning |
| `zigbee-zcl` | Zigbee Cluster Library — 33 clusters, foundation frames, reporting |
| `zigbee-runtime` | Device builder, power management, NV storage, device templates |
| `zigbee` | Top-level crate — coordinator, router, re-exports |

The ZCL layer implements **33 clusters** spanning General, Measurement &
Sensing, Lighting, HVAC, Closures, Security, Smart Energy, and Touchlink.

The MAC layer provides **11 supported chip targets**:

| Backend | Target | Notes |
|---------|--------|-------|
| MockMac | Host (macOS / Linux / Windows) | Full protocol simulation, no hardware |
| ESP32-C6 | `riscv32imac-unknown-none-elf` | Native 802.15.4 via `esp-ieee802154` |
| ESP32-H2 | `riscv32imac-unknown-none-elf` | Native 802.15.4 via `esp-ieee802154` |
| nRF52840 | `thumbv7em-none-eabihf` | 802.15.4 radio peripheral |
| nRF52833 | `thumbv7em-none-eabihf` | 802.15.4 radio peripheral |
| BL702 | `riscv32imac-unknown-none-elf` | Vendor `lmac154` FFI |
| CC2340 | `thumbv6m-none-eabi` | TI SimpleLink SDK stubs |
| TLSR8258 | `tc32-unknown-none-elf` | **Pure Rust** TLSR8258 radio and MAC |
| PHY6222 | `thumbv6m-none-eabi` | **Pure Rust** — zero vendor blobs! |
| EFR32MG1 | `thumbv7em-none-eabi` | **Pure Rust** Series 1 radio |
| EFR32MG21 | `thumbv8m.main-none-eabihf` | **Pure Rust** Series 2 radio |

Telink B91 is retained only as an unsupported scaffold; no B91 radio or MAC
backend is currently implemented.

## Current Status

zigbee-rs is **functional for end devices** (sensors, lights, sleepy devices).
The mock examples exercise the full lifecycle — network scan, association,
cluster creation, and attribute reporting — and every hardware target builds
successfully in CI.

**What works today:**

- End device join flow (scan → associate → start)
- ZCL cluster creation and typed attribute read/write
- Device builder with templates for common sensor profiles
- MockMac for host-side development and testing
- ESP32-C6/H2, nRF52840/52833, and BL702 firmware that compiles and flashes
- AES-CCM* encryption (via RustCrypto, `no_std`)

**In development:**

- Full coordinator and router operation
- OTA firmware upgrade flow
- Expanded test coverage
- Key management beyond the default Trust Center link key

## What You Can Build

With zigbee-rs you can create standard Zigbee Home Automation devices that
interoperate with coordinators like Zigbee2MQTT, ZHA, and deCONZ:

- **Temperature & humidity sensors** — the `mock-sensor` example is a complete
  starting point
- **Motion and occupancy detectors** — using the IAS Zone and Occupancy
  clusters
- **Smart switches and plugs** — On/Off cluster with optional Metering
- **Dimmable lights** — On/Off + Level Control + Color Control
- **Door and window sensors** — IAS Zone with contact closure
- **Thermostats** — HVAC clusters for temperature setpoint control

## How This Book Is Organized

This book is divided into six parts:

1. **Getting Started** — Install the toolchain, run the mock examples, and
   build your first device. No hardware required.

2. **Core Concepts** — Walk through each protocol layer: the Device Builder,
   the event loop, MAC, NWK, APS, ZDO, and BDB commissioning.

3. **ZCL Clusters** — Learn the ZCL foundation, then dive into each cluster
   category: General, Measurement, Lighting, HVAC, Closures, Security,
   and Smart Energy. Includes a guide to writing custom clusters.

4. **Platform Guides** — Hardware-specific instructions for ESP32, nRF52,
   BL702, CC2340, Telink, and PHY6222. Covers wiring, flashing, and
   debugging on each chip.

5. **Advanced Topics** — Power management for sleepy end devices, NV storage,
   security, OTA updates, and coordinator/router operation.

6. **Reference** — API quick reference, PIB attributes, ZCL cluster table,
   error types, and a glossary of Zigbee terminology.

Ready to get started? Head to the [Quick Start](./getting-started/quickstart.md)
to run your first zigbee-rs example in under five minutes.
