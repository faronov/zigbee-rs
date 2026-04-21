# Telink TLSR8258 Zigbee Temperature Sensor

A `no_std` Zigbee 3.0 end device firmware for the **Telink TLSR8258** (tc32 ISA),
reporting temperature and humidity via ZCL clusters 0x0402 and 0x0405.

## Hardware

- **MCU:** Telink TLSR8258 — tc32 core, 512KB Flash, 64KB SRAM
- **Radio:** Built-in IEEE 802.15.4 + BLE
- **Boards:** TB-03F, Sonoff SNZB-02, Tuya Zigbee sensors, IKEA devices
- **Button:** GPIO2 — join/leave network
- **LED:** GPIO3

## Build

### Real TC32 firmware

Use the modern-tc32 prerelease toolchain from:

`https://github.com/modern-tc32/rust/releases/tag/tc32-stage2-tc32-31`

For macOS x86_64:

```bash
cd /tmp
curl -L -O https://github.com/modern-tc32/rust/releases/download/tc32-stage2-tc32-31/tc32-rust-toolchain-macos-amd64.tar.gz
tar -xf tc32-rust-toolchain-macos-amd64.tar.gz

export TC32_TOOLCHAIN=/tmp/tc32-rust-toolchain-macos-amd64
cd /path/to/zigbee-rs-fork/examples/telink-tlsr8258-sensor
$TC32_TOOLCHAIN/bin/cargo build --release
```

For macOS arm64, use the matching `tc32-rust-toolchain-macos-arm64.tar.gz`.

This produces a real `tc32-unknown-none-elf` firmware image for TLSR8258. The
example is pure Rust: no Telink SDK, no vendor libraries, no C compilation.

### Local type-check without tc32 toolchain

If you only want a fast structural check, use the repo target spec with nightly:

```bash
cd examples/telink-tlsr8258-sensor
cargo -Zbuild-std=core -Zunstable-options -Zjson-target-spec check \
  --target ../../targets/tc32-none-eabi.json
```

## TC32 Toolchain

The TLSR8258 uses Telink's proprietary **tc32 instruction set**. The
[modern-tc32](https://github.com/modern-tc32) project provides:

- **Custom Rust compiler** with `tc32-unknown-none-elf` target
- **LLVM backend** with TC32 support (`clang --target=tc32`)
- **Prebuilt `core`/`alloc`** for the TC32 target

Setup: see [modern-tc32/examples_rust](https://github.com/modern-tc32/examples_rust)

## Modes

The example has three firmware modes:

- `diag-beacon`: raw beacon request plus RX parsing, no Zigbee runtime
- `diag-assoc`: bring-up mode, uses the new polling `MacDriver` to scan, associate, and poll
- `sensor`: default mode, builds `ZigbeeDevice` on top of the same TLSR8258 MAC path

Examples:

```bash
$TC32_TOOLCHAIN/bin/cargo build --release
$TC32_TOOLCHAIN/bin/cargo build --release --no-default-features --features diag-assoc
$TC32_TOOLCHAIN/bin/cargo build --release --no-default-features --features diag-beacon
```

## Flashing

Use the **Telink Burning & Debug Tool (BDT)** with a USB programmer:

```bash
TelinkBDT --chip 8258 --firmware target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
```

## Current status

- Pure-Rust startup, linker, IRQ entry, analog/clock bring-up
- Pure-Rust TLSR8258 RF/DMA setup and channel programming
- Polling MAC path for active scan, association, poll, TX, RX indication
- `sensor` mode uses the validated pure-Rust association path as its default runtime
- SRAM/SWire markers for bring-up checkpoints
- No vendor SDK dependency in the TLSR8258 path yet

Deep sleep and a polished sleepy end-device loop are still follow-up work after
hardware validation of scan/associate/poll on the new toolchain.

## Project Structure

```
telink-tlsr8258-sensor/
├── .cargo/config.toml   # tc32 target config
├── Cargo.toml            # Dependencies
├── build.rs              # Linker script setup
├── memory.x              # Flash @ 0x00000000, RAM @ 0x00840900
└── src/
    └── main.rs           # Startup, RF bring-up, polling MAC, mode entry points
```
