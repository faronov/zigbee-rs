# Telink TLSR8258 Zigbee Temperature Sensor

A `no_std` Zigbee 3.0 end device firmware for the **Telink TLSR8258** (tc32 ISA),
reporting temperature and humidity via ZCL clusters 0x0402 and 0x0405.

## Hardware

- **MCU:** Telink TLSR8258 — tc32 core, 512KB Flash, 64KB SRAM
- **Radio:** Built-in IEEE 802.15.4 + BLE
- **Boards:** TB-03F, Sonoff SNZB-02, Tuya Zigbee sensors, IKEA devices
- **Button:** GPIO2 — join/leave network
- **LED:** GPIO3

## Build Modes

### 1. CI/stub mode (no TC32 toolchain needed)

```bash
cd examples/telink-tlsr8258-sensor
cargo build --release --features stubs
```

Uses `thumbv6m-none-eabi` as a stand-in target. The `stubs` feature provides
no-op FFI symbols. This verifies the Rust code compiles but does NOT produce
flashable firmware.

### 2. Real TC32 firmware (with modern-tc32 toolchain)

```bash
# Install the TC32 Rust toolchain
# See: https://github.com/modern-tc32/examples_rust

# Set paths
export TC32_TOOLCHAIN=/path/to/toolchains/tc32-stage1
export TC32_SDK_DIR=/path/to/tl_zigbee_sdk
export TC32_LLVM_BIN=$TC32_TOOLCHAIN/llvm/bin

# Build real tc32 firmware
cd examples/telink-tlsr8258-sensor
$TC32_TOOLCHAIN/bin/cargo build --release
```

This produces a real `tc32-unknown-none-elf` binary flashable to TLSR8258 hardware.

The `build.rs` automatically:
- Compiles Telink SDK C sources with `clang --target=tc32`
- Links `libdrivers_8258.a` and `libsoft-fp.a` from the SDK
- Handles startup code and linker script

## TC32 Toolchain

The TLSR8258 uses Telink's proprietary **tc32 instruction set**. The
[modern-tc32](https://github.com/modern-tc32) project provides:

- **Custom Rust compiler** with `tc32-unknown-none-elf` target
- **LLVM backend** with TC32 support (`clang --target=tc32`)
- **Prebuilt `core`/`alloc`** for the TC32 target

Setup: see [modern-tc32/examples_rust](https://github.com/modern-tc32/examples_rust)

## Vendor Library

| Library | SDK Path | Purpose |
|---------|----------|---------|
| `libdrivers_8258.a` | `platform/lib/` | Hardware drivers (RF, GPIO, timer) |
| `libsoft-fp.a` | `platform/tc32/` | Soft-float math |

```bash
git clone https://github.com/telink-semi/tl_zigbee_sdk.git
export TC32_SDK_DIR=/path/to/tl_zigbee_sdk
```

## Flashing

Use the **Telink Burning & Debug Tool (BDT)** with a USB programmer:

```bash
TelinkBDT --chip 8258 --firmware target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
```

## Features

- Zigbee 3.0 SED with Identify, Temperature, Humidity, Battery clusters
- NWK Leave handler with auto-rejoin
- Default reporting with change thresholds
- IEEE 802.15.4 radio via Telink SDK FFI
- Button-driven join/leave with factory reset
- Two build paths: CI stubs + real TC32 firmware

## Project Structure

```
telink-tlsr8258-sensor/
├── .cargo/config.toml   # Target config (thumbv6m for CI, tc32 for real)
├── Cargo.toml            # Dependencies, stubs feature flag
├── build.rs              # Dual-mode: stub linking OR TC32 SDK compilation
├── memory.x              # Flash @ 0x00000000, RAM @ 0x00840000
└── src/
    ├── main.rs           # Entry point, SED loop, sensor clusters
    └── stubs.rs          # No-op FFI stubs for CI builds
```
