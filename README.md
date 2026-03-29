# zigbee-rs

A complete Zigbee PRO R22 protocol stack written in Rust, targeting embedded
`no_std` environments. Built on `async` traits for seamless integration with
Embassy and other embedded async runtimes.

```text
47,800+ lines of Rust · 161 source files · 9 crates · 33 ZCL clusters · 8 hardware platforms
```

## Architecture

```text
┌──────────────────────────────────────────────────────┐
│                    zigbee (top)                       │
│           coordinator · router · re-exports           │
├──────────────────────────────────────────────────────┤
│  zigbee-runtime   │  zigbee-bdb    │  zigbee-zcl     │
│  builder, power,  │  commissioning │  33 clusters,    │
│  NV storage,      │  steering,     │  foundation,     │
│  device templates  │  formation     │  reporting       │
├───────────────────┴────────────────┴─────────────────┤
│                    zigbee-zdo                          │
│          discovery · binding · network mgmt           │
├──────────────────────────────────────────────────────┤
│                    zigbee-aps                          │
│          frames · binding · groups · security         │
├──────────────────────────────────────────────────────┤
│                    zigbee-nwk                          │
│      frames · routing (AODV+tree) · security · NIB   │
├──────────────────────────────────────────────────────┤
│                    zigbee-mac                          │
│  MacDriver trait · 8 backends (see table below)      │
├──────────────────────────────────────────────────────┤
│                   zigbee-types                         │
│     IeeeAddress · ShortAddress · PanId · Channel     │
└──────────────────────────────────────────────────────┘
```

## Quick Start

### Mock examples (no hardware needed)

```bash
# Temperature + humidity sensor simulation
cargo run -p mock-sensor

# Coordinator (network formation + device join)
cargo run -p mock-coordinator

# Dimmable light
cargo run -p mock-light

# Sleepy end device (full SED lifecycle)
cargo run -p mock-sleepy-sensor
```

### Build the entire workspace

```bash
cargo build
cargo test
```

### ESP32-C6 / ESP32-H2 firmware

```bash
cd examples/esp32c6-sensor   # or esp32h2-sensor
cargo build --release -Z build-std=core,alloc
espflash flash target/riscv32imac-unknown-none-elf/release/esp32c6-sensor
```

Or flash via the [web flasher](https://faronov.github.io/zigbee-rs/) (no tools needed, just a browser with Web Serial).

### nRF52840 firmware (with debug probe)

```bash
cd examples/nrf52840-sensor
cargo build --release
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-sensor
```

### nRF52840 firmware (nice!nano / ProMicro — UF2 drag-and-drop)

```bash
cd examples/nrf52840-sensor-uf2
cargo build --release
# Convert to UF2 (CI does this automatically):
# uf2conv.py -c -f 0xADA52840 -b 0x26000 firmware.bin -o firmware.uf2
# Double-tap RESET → copy .uf2 to the "NICENANO" USB drive
```

### BL702 firmware

```bash
cd examples/bl702-sensor
cargo build --release --features stubs  # vendor libs linked at build time
```

### CC2340 firmware

```bash
cd examples/cc2340-sensor
cargo build --release --features stubs
```

### Telink B91 / TLSR8258 firmware

```bash
cd examples/telink-b91-sensor
cargo build --release --features stubs

cd examples/telink-tlsr8258-sensor
cargo build --release --features stubs
```

### PHY6222 firmware (pure Rust — no vendor SDK!)

```bash
cd examples/phy6222-sensor
cargo build --release   # no stubs, no vendor blobs needed
```

## MAC Backends

| Backend | Radio driver | Target | Notes |
|---------|-------------|--------|-------|
| **MockMac** | ✅ Simulation | Host (macOS/Linux/Windows) | Full protocol sim, no hardware |
| **ESP32-C6** | ✅ esp-ieee802154 | `riscv32imac-unknown-none-elf` | Native 802.15.4 radio |
| **ESP32-H2** | ✅ esp-ieee802154 | `riscv32imac-unknown-none-elf` | Native 802.15.4 radio |
| **nRF52840** | ✅ nrf-radio | `thumbv7em-none-eabihf` | 802.15.4 radio peripheral |
| **nRF52833** | ✅ nrf-radio | `thumbv7em-none-eabihf` | 802.15.4 radio peripheral |
| **BL702** | ✅ lmac154 FFI | `riscv32imac-unknown-none-elf` | Vendor libs (`liblmac154.a` + `libbl702_rf.a`) included |
| **CC2340** | ⚡ ZBOSS FFI | `thumbv6m-none-eabi` | TI SimpleLink SDK stubs (50+ RTOS deps) |
| **Telink B91** | ⚡ Telink FFI | `riscv32imac-unknown-none-elf` | Telink SDK stubs |
| **Telink TLSR8258** | ⚡ Telink FFI | `riscv32-unknown-none-elf` | Telink SDK stubs (tc32 ISA) |
| **PHY6222** | 🦀 **Pure Rust** | `thumbv6m-none-eabi` | Zero vendor blobs — direct register access! |

> **Legend:** ✅ = fully functional radio driver · ⚡ = compiles with stubs, needs vendor SDK for real RF · 🦀 = pure Rust (no FFI)

All 10 firmware targets build in CI and produce downloadable artifacts.

## ZCL Clusters (33)

**General:** Basic, Power Config, Identify, Groups, Scenes, On/Off, On/Off Switch Config,
Level Control, Alarms, Time, Multistate Input, OTA Upgrade, Poll Control, Green Power,
Diagnostics

**Closures:** Door Lock, Window Covering

**HVAC:** Thermostat, Fan Control, Thermostat UI Config

**Lighting:** Color Control

**Measurement:** Illuminance, Temperature, Pressure, Flow, Humidity, Occupancy, Electrical

**Security:** IAS Zone, IAS ACE, IAS WD

**Smart Energy:** Metering

**Touchlink:** Commissioning

## Design Principles

- **`#![no_std]`** everywhere — no heap allocation, `heapless` for bounded collections
- **`async` MacDriver trait** — 13 methods, no `Send`/`Sync` requirement
- **Platform-agnostic** — same stack code runs on mock, ESP32, nRF, BL702, CC2340, Telink, PHY6222
- **Manual frame parsing** — no `serde`, bitfield encode/decode for all frame types
- **Embassy-compatible** — designed for single-threaded async executors
- **Layered crates** — each layer wraps the one below: `NwkLayer<M: MacDriver>`
- **CI-enforced** — every push builds all 10 firmware targets + clippy + fmt + tests

## Project Structure

```
zigbee-rs/
├── zigbee-types/              # Core types (addresses, channels)
├── zigbee-mac/                # MAC layer + platform backends
│   └── src/
│       ├── mock/              # Full mock for host testing
│       ├── esp/               # ESP32-C6/H2 (esp-ieee802154)
│       ├── nrf/               # nRF52840/52833 (radio peripheral)
│       ├── bl702/             # BL702 (lmac154 FFI)
│       ├── cc2340/            # CC2340 (ZBOSS FFI stubs)
│       ├── telink/            # Telink B91 + TLSR8258 (SDK stubs)
│       └── phy6222/           # PHY6222 (pure Rust radio driver!)
├── zigbee-nwk/                # Network layer (routing, security)
├── zigbee-aps/                # Application Support (binding, groups)
├── zigbee-zdo/                # Device Objects (discovery, mgmt)
├── zigbee-bdb/                # Base Device Behavior (commissioning)
├── zigbee-zcl/                # Zigbee Cluster Library (33 clusters)
├── zigbee-runtime/            # Device builder, power, NV storage
├── zigbee/                    # Top-level: coordinator, router
├── tests/                     # Integration tests
├── examples/
│   ├── mock-sensor/           # Host: temp+humidity sensor
│   ├── mock-coordinator/      # Host: coordinator
│   ├── mock-light/            # Host: dimmable light
│   ├── mock-sleepy-sensor/    # Host: SED demo
│   ├── esp32c6-sensor/        # ESP32-C6 firmware
│   ├── esp32h2-sensor/        # ESP32-H2 firmware
│   ├── nrf52840-sensor/       # nRF52840-DK (probe-rs)
│   ├── nrf52840-sensor-uf2/   # nice!nano / ProMicro (UF2 drag-drop)
│   ├── nrf52833-sensor/       # nRF52833-DK (probe-rs)
│   ├── nrf52840-bridge/       # nRF52840 coordinator bridge
│   ├── bl702-sensor/          # BL702 (vendor libs included)
│   ├── cc2340-sensor/         # TI CC2340R5 (stubs)
│   ├── telink-b91-sensor/     # Telink B91 (stubs)
│   ├── telink-tlsr8258-sensor/# Telink TLSR8258 (stubs)
│   └── phy6222-sensor/        # PHY6222 — pure Rust, no vendor SDK!
├── docs/flasher/              # ESP web flasher (GitHub Pages)
└── BUILD.md                   # Comprehensive build guide
```

## CI / Firmware Artifacts

Every push builds **10 firmware targets** plus workspace checks:

| Job | What it does |
|-----|-------------|
| Check | `cargo check --workspace` |
| Test | `cargo test --workspace` |
| Clippy | `cargo clippy --workspace` |
| Format | `cargo fmt --check` |
| Doc | `cargo doc --workspace --no-deps` |
| Build × 10 | Each platform produces a downloadable firmware artifact |
| Deploy | Web flasher published to GitHub Pages |

Download firmware artifacts from the [Actions tab](https://github.com/faronov/zigbee-rs/actions).

## Known Limitations

- **CC2340 / Telink B91 / Telink TLSR8258** backends compile with stub FFI — real RF requires linking vendor SDK libraries (blocked by complex RTOS dependencies or proprietary toolchains)
- **PHY6222** pure-Rust driver uses simplified TP calibration defaults — production firmware would need proper PLL lock sequence
- **Test coverage** is basic — the mock examples exercise more than the test crate
- **Security** — AES-CCM\* encryption works (RustCrypto `aes` + `ccm`, `no_std`) but key management is minimal
- **OTA** — cluster defined but no actual firmware upgrade flow implemented

## Documentation

See [BUILD.md](BUILD.md) for detailed instructions on building, flashing, sensor/display
integration, debugging, and peripheral wiring.

## License

GPL-2.0 (forked from zigbee-rs)
