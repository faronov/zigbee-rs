# PHY6222 Zigbee Temperature Sensor — Pure Rust Radio!

A `no_std` Zigbee 3.0 end device firmware for the **PHY6222** (ARM Cortex-M0),
reporting temperature and humidity via ZCL clusters 0x0402 and 0x0405.

**This is the only zigbee-rs example with a 100% pure-Rust IEEE 802.15.4 radio
driver** — no vendor SDK, no binary blobs, no C FFI. All radio hardware access
is through direct register writes in Rust.

## Hardware

- **MCU:** PHY6222 — ARM Cortex-M0, 512KB Flash, 64KB SRAM
- **Radio:** Built-in 2.4 GHz IEEE 802.15.4 + BLE (pure Rust driver)
- **Boards:** Ai-Thinker PB-03F (~$1.50), Tuya THB2, TH05F, BTH01
- **Button:** GPIO15 (PROG button on PB-03F) — join/leave network
- **LEDs:** GPIO11 (red), GPIO12 (green), GPIO14 (blue) — active low on PB-03F

## Prerequisites

- Rust nightly with `thumbv6m-none-eabi` target
- Any ARM SWD debugger (J-Link, ST-Link, DAPLink, etc.)

```bash
rustup target add thumbv6m-none-eabi
```

## Vendor Library Setup

**None required!** 🎉

The PHY6222 radio driver is implemented entirely in Rust using direct register
access. No vendor SDK, no precompiled `.a` files, no environment variables
to configure.

This makes the PHY6222 the simplest example to build and the easiest to audit.

## Building

```bash
cd examples/phy6222-sensor
cargo build --release
```

That's it. No `--features stubs`, no SDK paths, no vendor blobs.

The `stubs` feature exists for CI consistency but is **not needed** — the
project builds without any external libraries.

### Binary size

The release build produces a compact firmware:

- **Flash:** ~5KB
- **RAM:** ~4KB

## Flashing

Use any ARM SWD debugger — the PHY6222 is a standard Cortex-M0:

```bash
# With probe-rs
probe-rs run --chip PHY6222 target/thumbv6m-none-eabi/release/phy6222-sensor

# With openocd
openocd -f interface/cmsis-dap.cfg -f target/phy6222.cfg \
  -c "program target/thumbv6m-none-eabi/release/phy6222-sensor verify reset exit"

# With pyOCD
pyocd flash -t phy6222 target/thumbv6m-none-eabi/release/phy6222-sensor
```

## What It Demonstrates

- **First pure-Rust IEEE 802.15.4 radio driver** in the zigbee-rs project
- Zigbee 3.0 end device on the ultra-low-cost PHY6222 (~$1.50 boards)
- Embassy async runtime on Cortex-M0
- No vendor dependencies — fully auditable, reproducible builds
- Button-driven network join/leave with edge detection
- RGB LED status indication
- ZCL Temperature Measurement + Relative Humidity clusters

## Project Structure

```
phy6222-sensor/
├── .cargo/config.toml   # Target: thumbv6m-none-eabi, build-std
├── Cargo.toml            # Dependencies (no vendor libs!)
├── build.rs              # Minimal — just linker script, no vendor linking
├── memory.x              # Flash @ 0x11001000, RAM @ 0x1FFF0000
└── src/
    ├── main.rs           # Entry point, device setup, sensor loop
    └── stubs.rs          # CI stubs (not needed for real builds)
```
