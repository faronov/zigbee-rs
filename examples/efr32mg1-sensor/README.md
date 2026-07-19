# EFR32MG1P Zigbee Sensor - Pure Rust Radio

A `no_std` Zigbee 3.0 end device firmware for the **EFR32MG1P** (ARM Cortex-M4F),
reporting temperature and humidity via ZCL clusters 0x0402 and 0x0405.

**This example uses a pure-Rust IEEE 802.15.4 radio driver** — no RAIL library,
no GSDK, no binary blobs. All radio hardware access is through direct register
writes in Rust.

Current bring-up is done in direct-boot mode with the application linked at
`0x00000000`. Bootloader integration is a later step and is intentionally kept
separate from Zigbee stack stabilization.

## Hardware

- **MCU:** EFR32MG1P (256KB flash, 32KB SRAM) — ARM Cortex-M4F @ 40 MHz
- **Radio:** Built-in 2.4 GHz IEEE 802.15.4 + BLE (pure Rust driver)
- **Boards:** IKEA TRÅDFRI modules, Thunderboard Sense (BRD4151A), BRD4100A
- **Button:** Adjust GPIO pin in `pins` module for your board
- **LED:** Adjust GPIO pin in `pins` module for your board

## Prerequisites

- Rust nightly with `thumbv7em-none-eabi` target
- Any ARM SWD debugger (J-Link, ST-Link, DAPLink, etc.)

```bash
rustup target add thumbv7em-none-eabi
```

## Vendor Library Setup

**None required!** 🎉

The EFR32MG1P radio driver is implemented entirely in Rust using direct register
access. No GSDK, no RAIL library, no precompiled `.a` files, no environment
variables to configure.

## Modes

- `sensor` - default full Zigbee sleepy end-device example
- `diag-join` - minimal Zigbee join/poll runtime without sensor reporting
- `diag-beacon` - pure radio bring-up mode for active scan / beacon RX only

## Building

```bash
cd examples/efr32mg1-sensor
cargo build --release
cargo build --release --no-default-features --features diag-join
cargo build --release --no-default-features --features diag-beacon
```

No GSDK or RAIL headers are required for the pure-Rust path.

## Flashing

Use any ARM SWD debugger — the EFR32MG1P is a standard Cortex-M4F:

```bash
# With probe-rs
probe-rs run --chip EFR32MG1P target/thumbv7em-none-eabi/release/efr32mg1-sensor

# With openocd
openocd -f interface/cmsis-dap.cfg -f target/efm32.cfg \
  -c "program target/thumbv7em-none-eabi/release/efr32mg1-sensor verify reset exit"

# With Simplicity Commander (Silicon Labs tool)
commander flash target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex
```

For the local Commander installation on this machine:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli flash \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  --device EFR32MG1P132F256
```

Recommended direct-boot cycle during bring-up:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli device masserase \
  --device EFR32MG1P132F256

/Applications/Commander-cli.app/Contents/MacOS/commander-cli flash \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  --device EFR32MG1P132F256
```

## Beacon Diagnostics

The recommended first bring-up step on real hardware is beacon RX on a single
channel before trying association or the full Zigbee runtime.

Build and flash the radio-only diagnostic image:

```bash
cd examples/efr32mg1-sensor
cargo build --release --no-default-features --features diag-beacon

/Applications/Commander-cli.app/Contents/MacOS/commander-cli flash \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  --device EFR32MG1P132F256
```

Read logs over RTT or SWO:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli rtt connect \
  --device EFR32MG1P132F256

/Applications/Commander-cli.app/Contents/MacOS/commander-cli swo read \
  --device EFR32MG1P132F256
```

The diagnostic firmware:

- initializes the pure-Rust radio path
- performs repeated active scans on channel 15
- prints beacon count and parsed PAN descriptors
- blinks the LED twice when at least one beacon is received

## Join Diagnostics

Use `diag-join` after beacon RX is confirmed. This image keeps the full
MAC/NWK/APS/BDB join path but removes the sensor reporting workload.

For the full local ZHA + sniffer + nRF baseline workflow, see
[`docs/efr32-join-debug.md`](../../docs/efr32-join-debug.md).

```bash
cd examples/efr32mg1-sensor
cargo build --release --no-default-features --features diag-join

/Applications/Commander-cli.app/Contents/MacOS/commander-cli flash \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  --device EFR32MG1P132F256
```

`diag-join`:

- requests join on boot
- retries join every 15 seconds when not attached
- polls the parent aggressively right after join
- logs join and incoming poll traffic over RTT

Useful register snapshots during parity work:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli readmem \
  --device EFR32MG1P132F256 \
  --range 0x40080000:+0x200

/Applications/Commander-cli.app/Contents/MacOS/commander-cli readmem \
  --device EFR32MG1P132F256 \
  --range 0x40083000:+0x100

/Applications/Commander-cli.app/Contents/MacOS/commander-cli readmem \
  --device EFR32MG1P132F256 \
  --range 0x40084000:+0x180
```

## What It Demonstrates

- **Pure-Rust IEEE 802.15.4 radio driver** for EFR32MG1P (no RAIL/GSDK)
- Zigbee 3.0 end device on the popular EFR32MG1P platform
- Embassy async runtime on Cortex-M4F with SysTick time driver
- Proper interrupt vector table (34 entries for all EFR32MG1P peripherals)
- No vendor dependencies — fully auditable, reproducible builds
- Button-driven network join/leave with edge detection
- LED status indication + identify blink
- ZCL Temperature Measurement + Relative Humidity + Identify clusters
- Flash NV storage — network state persists across reboots
- Default reporting with reportable change thresholds
- Radio sleep/wake for power management

## Radio Architecture

The EFR32MG1P radio consists of several interconnected blocks:

| Block | Base Address | Function |
|-------|-------------|----------|
| RAC   | 0x40084000  | Radio Controller — state machine, PA |
| FRC   | 0x40080000  | Frame Controller — CRC, format |
| MODEM | 0x40086000  | O-QPSK modulation/demodulation |
| SYNTH | 0x40083000  | PLL frequency synthesizer |
| AGC   | 0x40087000  | Automatic gain control, RSSI |
| BUFC  | 0x40082000  | TX/RX buffer controller |

All blocks are configured via memory-mapped registers — no co-processor
or mailbox protocol needed.

## Project Structure

```
efr32mg1-sensor/
├── .cargo/config.toml   # Target: thumbv7em-none-eabi, build-std
├── Cargo.toml            # Dependencies (no vendor libs!)
├── device.x              # EFR32MG1P interrupt vector names (34 IRQs)
└── src/
    ├── main.rs           # Entry point, device setup, sensor loop
    ├── time_driver.rs    # Embassy time driver (SysTick, 1ms tick)
    ├── vectors.rs        # Interrupt vector table + NVIC Interrupt enum
    └── stubs.rs          # CI stubs (not needed for real builds)
```

`efr32mg1-hal` owns the MSC flash controller. `boards/efr32mg1-tradfri` owns
the bounded application-NV partition and direct-boot linker layout.
