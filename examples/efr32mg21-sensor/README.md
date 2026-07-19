# EFR32MG21 Zigbee Sensor — Pure Rust Radio!

A `no_std` Zigbee 3.0 end device firmware for the **EFR32MG21** (ARM Cortex-M33),
reporting temperature and humidity via ZCL clusters 0x0402 and 0x0405.

**This example uses a pure-Rust IEEE 802.15.4 radio driver** — no RAIL library,
no GSDK, no binary blobs. All radio hardware access is through direct register
writes in Rust.

> **⚠️ Scaffold Implementation:** The radio register values are simplified
> approximations. The exact register sequences for 802.15.4 mode need
> verification against the EFR32xG21 Reference Manual or extraction from the
> RAIL library source.

## Hardware

- **MCU:** EFR32MG21 (512KB flash, 64KB SRAM) — ARM Cortex-M33 @ 80 MHz
- **Radio:** Built-in 2.4 GHz IEEE 802.15.4 + BLE (pure Rust driver)
- **Boards:** BRD4180A, BRD4181A, and many third-party modules
- **Button:** Adjust GPIO pin in `pins` module for your board
- **LED:** Adjust GPIO pin in `pins` module for your board

## Series 1 vs Series 2 Differences

| Feature | EFR32MG1P (Series 1) | EFR32MG21 (Series 2) |
|---------|---------------------|---------------------|
| CPU | Cortex-M4F @ 40 MHz | Cortex-M33 @ 80 MHz |
| Flash | 256 KB | 512 KB |
| RAM | 32 KB | 64 KB |
| Security | CRYPTO engine | Secure Element (SE) + TrustZone |
| Radio base | 0x40080000–0x40087FFF | 0x40090000–0x40095FFF |
| CMU base | 0x400E4000 | 0x40008000 |
| GPIO base | 0x4000A000 | 0x4003C000 |
| MSC base | 0x400E0000 | 0x40030000 |
| Flash page | 2 KB | 8 KB |

## Prerequisites

- Rust nightly with `thumbv8m.main-none-eabihf` target
- Any ARM SWD debugger (J-Link, ST-Link, DAPLink, etc.)

```bash
rustup target add thumbv8m.main-none-eabihf
```

## Vendor Library Setup

**None required!** 🎉

The EFR32MG21 radio driver is implemented entirely in Rust using direct register
access. No GSDK, no RAIL library, no precompiled `.a` files, no environment
variables to configure.

## Building

```bash
cd examples/efr32mg21-sensor
cargo build --release
```

No `--features stubs` required — the project builds without any external libraries.

## Flashing

Use any ARM SWD debugger — the EFR32MG21 is a standard Cortex-M33:

```bash
# With probe-rs
probe-rs run --chip EFR32MG21A020F512IM32 target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor

# With openocd
openocd -f interface/cmsis-dap.cfg -f target/efm32.cfg \
  -c "program target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor verify reset exit"

# With Simplicity Commander (Silicon Labs tool)
commander flash target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor.hex
```

## What It Demonstrates

- **Pure-Rust IEEE 802.15.4 radio driver** for EFR32MG21 (no RAIL/GSDK)
- Zigbee 3.0 end device on the EFR32MG21 Series 2 platform
- Embassy async runtime on Cortex-M33 with SysTick time driver
- Proper interrupt vector table (51 entries for all EFR32MG21 peripherals)
- No vendor dependencies — fully auditable, reproducible builds
- Button-driven network join/leave with edge detection
- LED status indication + identify blink
- ZCL Temperature Measurement + Relative Humidity + Identify clusters
- Flash NV storage — network state persists across reboots (8 KB pages)
- Default reporting with reportable change thresholds
- Radio sleep/wake for power management

## Radio Architecture

The EFR32MG21 radio blocks (Series 2 base addresses):

| Block | Base Address | Function |
|-------|-------------|----------|
| RAC   | 0x40093000  | Radio Controller — state machine, PA |
| FRC   | 0x40090000  | Frame Controller — CRC, format |
| MODEM | 0x40094000  | O-QPSK modulation/demodulation |
| SYNTH | 0x40092000  | PLL frequency synthesizer |
| AGC   | 0x40095000  | Automatic gain control, RSSI |
| BUFC  | 0x40091000  | TX/RX buffer controller |

All blocks are configured via memory-mapped registers — no co-processor
or mailbox protocol needed.

## Project Structure

```
efr32mg21-sensor/
├── .cargo/config.toml   # Target: thumbv8m.main-none-eabihf, build-std
├── Cargo.toml            # Dependencies (no vendor libs!)
├── device.x              # EFR32MG21 interrupt vector names (51 IRQs)
└── src/
    ├── main.rs           # Entry point, device setup, sensor loop
    ├── time_driver.rs    # Embassy time driver (SysTick @ 80 MHz, 1ms tick)
    ├── vectors.rs        # Interrupt vector table + NVIC Interrupt enum
    └── stubs.rs          # CI stubs (not needed for real builds)
```

`efr32mg21-hal` owns the MSC flash controller. `boards/efr32mg21-devkit` owns
the bounded 16 KB application-NV partition and linker layout.
