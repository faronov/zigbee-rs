# Telink TLSR8258 Zigbee Sensor

A `no_std` Zigbee 3.0 end-device firmware for the **Telink TLSR8258** (tc32 ISA),
brought up on the pure-Rust Telink MAC path.

## Hardware

- **MCU:** Telink TLSR8258 — tc32 core, 512KB Flash, 64KB SRAM
- **Radio:** Built-in IEEE 802.15.4 + BLE
- **Reference board:** TB-04-Kit
- **LEDs:** PC1 red, PB5 green, PC4 blue

## Toolchain

Use the modern-tc32 prerelease toolchain:

`https://github.com/modern-tc32/rust/releases/tag/tc32-stage2-tc32-31`

For macOS x86_64:

```bash
cd /tmp
curl -L -O https://github.com/modern-tc32/rust/releases/download/tc32-stage2-tc32-31/tc32-rust-toolchain-macos-amd64.tar.gz
tar -xf tc32-rust-toolchain-macos-amd64.tar.gz
export TC32_TOOLCHAIN=/tmp/tc32-rust-toolchain-macos-amd64
```

For macOS arm64, use the matching `tc32-rust-toolchain-macos-arm64.tar.gz`.

The example also needs the local Telink flashing/debug helpers used for bring-up:

- `~/TLSRPGM/TlsrPgm.py`
- `~/zboss_opensource/tlsr_debug.py`

Both paths are overridable with environment variables.

## Modes

The example has three firmware modes:

- `sensor`: default mode, uses the validated pure-Rust MAC scan/associate/poll path plus a minimal interview responder
- `diag-assoc`: bring-up mode for scan, association, and poll
- `diag-beacon`: raw beacon request plus RX parsing, no Zigbee runtime

## Reproducible Bring-Up Flow

The helper script keeps build, flash, and SRAM dump commands in one place:

`scripts/tlsr8258.sh`

Default environment:

- `TC32_TOOLCHAIN=/tmp/tc32-rust-toolchain-macos-amd64`
- `TLSRPGM=$HOME/TLSRPGM/TlsrPgm.py`
- `TLSR_DEBUG=$HOME/zboss_opensource/tlsr_debug.py`
- `TELINK_PORT=/dev/cu.usbserial-1410`

Override any of them if your setup differs.

### 1. Build or check

```bash
cd examples/telink-tlsr8258-sensor

scripts/tlsr8258.sh check sensor
scripts/tlsr8258.sh build sensor
scripts/tlsr8258.sh build diag-assoc
scripts/tlsr8258.sh build diag-beacon
```

The build step emits:

- `target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor`
- `target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin`

The validated build path uses:

```bash
cargo rustc --release -- -C lto=no -C opt-level=1
```

The tc32 backend currently trips codegen bugs on this example with the default
high-optimization release pipeline.

### 2. Flash

```bash
cd examples/telink-tlsr8258-sensor

scripts/tlsr8258.sh flash sensor
scripts/tlsr8258.sh flash diag-assoc
scripts/tlsr8258.sh flash diag-beacon
```

Under the hood, this runs the same programmer flow used during bring-up:

```bash
python3 ~/TLSRPGM/TlsrPgm.py \
  -p /dev/cu.usbserial-1410 \
  -t 500 \
  -a 200 \
  -m we 0 \
  target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
```

### 3. Inspect debug SRAM markers

Boot-stage markers live at `0x00848400`, runtime markers at `0x00848500`.

```bash
cd examples/telink-tlsr8258-sensor

scripts/tlsr8258.sh dump-boot
scripts/tlsr8258.sh dump-mode
scripts/tlsr8258.sh dump 0x00848550 8
```

Equivalent raw command:

```bash
python3 ~/zboss_opensource/tlsr_debug.py \
  -p /dev/cu.usbserial-1410 \
  --activate read 0x00848500 20
```

## Local type-check without tc32 toolchain

If you only want a fast structural check, use the repo target spec with nightly:

```bash
cd examples/telink-tlsr8258-sensor
cargo -Zbuild-std=core -Zunstable-options -Zjson-target-spec check \
  --target ../../targets/tc32-none-eabi.json
```

This does not produce a flashable tc32 firmware image.

## Current status

- Pure-Rust startup, linker, IRQ entry, analog/clock bring-up
- Pure-Rust TLSR8258 RF/DMA setup and channel programming
- Polling MAC path for active scan, association, poll, TX, RX indication
- `sensor` mode is the default runtime and uses the validated pure-Rust association path
- minimal interview responses for `NWK/IEEE Addr`, `Node Desc`, `Power Desc`, `Active EP`, `Simple Desc`
- minimal ZCL `Read Attributes Response` for Basic, Power Configuration, Identify, and Temperature Measurement
- Flash/debug helpers are scripted for reproducible build, flash, and SRAM inspection
- No vendor SDK dependency in the TLSR8258 path

The full `ZigbeeDevice::start/tick` runtime path is still blocked by a tc32
backend codegen bug on `tc32-stage2-tc32-31`, so the current default firmware
keeps the hardware on the lighter `sensor-lite` path for join and interview.

## Project structure

```text
telink-tlsr8258-sensor/
├── Cargo.toml
├── build.rs
├── memory.x
├── scripts/
│   └── tlsr8258.sh
└── src/
    └── main.rs
```
