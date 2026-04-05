# EFR32 (MG1 & MG21)

The Silicon Labs EFR32 Mighty Gecko family is one of the most widely deployed
Zigbee platforms — found in IKEA TRÅDFRI modules, Sonoff ZBDongle-E, and many
commercial products. zigbee-rs supports both **Series 1 (EFR32MG1P)** and
**Series 2 (EFR32MG21)** with pure-Rust radio drivers — no GSDK, no RAIL
library, no binary blobs.

## Hardware Overview

| Spec | EFR32MG1P (Series 1) | EFR32MG21 (Series 2) |
|------|---------------------|---------------------|
| **Core** | ARM Cortex-M4F @ 40 MHz | ARM Cortex-M33 @ 80 MHz |
| **Flash** | 256 KB | 512 KB |
| **SRAM** | 32 KB | 64 KB |
| **Radio** | 2.4 GHz IEEE 802.15.4 + BLE | 2.4 GHz IEEE 802.15.4 + BLE |
| **Security** | CRYPTO engine | Secure Element (SE) + TrustZone |
| **Target** | `thumbv7em-none-eabihf` | `thumbv8m.main-none-eabihf` |
| **Flash page size** | 2 KB | 8 KB |

### Why EFR32?

- **Ubiquitous** — used in IKEA TRÅDFRI, Sonoff, and many commercial Zigbee products
- **Pure Rust** — zigbee-rs needs no GSDK, no RAIL library, no vendor blobs
- **Well-documented** — Silicon Labs reference manuals available for register-level programming
- **Excellent radio** — high sensitivity, good range, mature 802.15.4 support

### Common Boards and Modules

| Board | Series | Form Factor | Notes |
|-------|--------|-------------|-------|
| **IKEA TRÅDFRI modules** | Series 1 | PCB modules | EFR32MG1P, widely available |
| **Thunderboard Sense (BRD4151A)** | Series 1 | Dev board | EFR32MG1P + sensors |
| **BRD4100A** | Series 1 | Radio board | EFR32MG1P evaluation |
| **BRD4180A** | Series 2 | Radio board | EFR32MG21A020F1024IM32 |
| **BRD4181A** | Series 2 | Radio board | EFR32MG21A020F512IM32 |
| **Sonoff ZBDongle-E** | Series 2 | USB dongle | EFR32MG21, popular coordinator |

### Series 1 vs Series 2 Register Differences

The two series have different peripheral base addresses, requiring separate
MAC modules (`efr32/` and `efr32s2/`):

| Peripheral | Series 1 Base | Series 2 Base |
|-----------|--------------|--------------|
| Radio (RAC, FRC, MODEM, etc.) | `0x40080000–0x40087FFF` | `0x40090000–0x40095FFF` |
| CMU (Clock Management Unit) | `0x400E4000` | `0x40008000` |
| GPIO | `0x4000A000` | `0x4003C000` |
| MSC (Flash Controller) | `0x400E0000` | `0x40030000` |

## Prerequisites

### Rust Toolchain

```bash
rustup default nightly
rustup update nightly

# Series 1 (MG1P — Cortex-M4F)
rustup target add thumbv7em-none-eabihf

# Series 2 (MG21 — Cortex-M33)
rustup target add thumbv8m.main-none-eabihf

# rust-src for build-std
rustup component add rust-src
```

### No Vendor SDK Required!

Unlike the traditional Silicon Labs development flow (which requires GSDK +
RAIL library + Simplicity Studio), the zigbee-rs EFR32 backends need **no
vendor libraries, no SDK download, no environment variables**. Everything is
in Rust.

### Debug Probe

Any ARM SWD debugger works:

- **J-Link** — included with Silicon Labs dev kits
- **ST-Link** — widely available, inexpensive
- **DAPLink / CMSIS-DAP** — open-source, many options
- **probe-rs** — recommended Rust-native tool

## Building

### EFR32MG1P (Series 1)

```bash
cd examples/efr32mg1-sensor
cargo build --release
```

### EFR32MG21 (Series 2)

```bash
cd examples/efr32mg21-sensor
cargo build --release
```

No `--features stubs` required — both projects build without any external
libraries.

### CI Build

Both targets build in CI alongside the other 11 firmware targets. The CI
workflow extracts `.bin` and `.hex` artifacts from the ELF output:

```bash
OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
$OBJCOPY -O binary $ELF ${ELF}.bin
$OBJCOPY -O ihex   $ELF ${ELF}.hex
```

## Flashing

### EFR32MG1P (Series 1)

```bash
# With probe-rs
probe-rs run --chip EFR32MG1P target/thumbv7em-none-eabihf/release/efr32mg1-sensor

# With openocd
openocd -f interface/cmsis-dap.cfg -f target/efm32.cfg \
  -c "program target/thumbv7em-none-eabihf/release/efr32mg1-sensor verify reset exit"

# With Simplicity Commander (Silicon Labs tool)
commander flash target/thumbv7em-none-eabihf/release/efr32mg1-sensor.hex
```

### EFR32MG21 (Series 2)

```bash
# With probe-rs
probe-rs run --chip EFR32MG21A020F512IM32 \
  target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor

# With openocd
openocd -f interface/cmsis-dap.cfg -f target/efm32.cfg \
  -c "program target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor verify reset exit"

# With Simplicity Commander
commander flash target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor.hex
```

## Pure-Rust Radio Driver

Both EFR32 backends use direct register access for the radio — no RAIL
library, no co-processor mailbox protocol. The radio hardware consists of
several interconnected blocks:

| Block | Function |
|-------|----------|
| **RAC** | Radio Controller — state machine, PA |
| **FRC** | Frame Controller — CRC, format |
| **MODEM** | O-QPSK modulation/demodulation |
| **SYNTH** | PLL frequency synthesizer |
| **AGC** | Automatic gain control, RSSI |
| **BUFC** | TX/RX buffer controller |

All blocks are configured via memory-mapped registers. The Series 1 and
Series 2 register maps are structurally similar but use different base
addresses (see table above), which is why they live in separate MAC modules.

### MAC Backend Structure

```
zigbee-mac/src/
├── efr32/             # Series 1 (EFR32MG1P)
│   ├── mod.rs         # Efr32Mac struct, MacDriver trait impl
│   └── driver.rs      # Efr32Driver — pure-Rust register-level radio driver
└── efr32s2/           # Series 2 (EFR32MG21)
    ├── mod.rs         # Efr32S2Mac struct, MacDriver trait impl
    └── driver.rs      # Efr32S2Driver — pure-Rust register-level radio driver
```

### Feature Flags

```toml
# Series 1 (MG1P)
zigbee-mac = { features = ["efr32"] }

# Series 2 (MG21)
zigbee-mac = { features = ["efr32s2"] }
```

## Power Management

Both EFR32 platforms implement radio sleep/wake via the **CMU (Clock
Management Unit)**, which gates the radio peripheral clocks to save power
between polls:

```rust
device.mac_mut().radio_sleep();   // CMU clock gate — radio off
Timer::after(Duration::from_millis(poll_ms)).await;
device.mac_mut().radio_wake();    // CMU clock enable, re-apply channel
```

The CMU register addresses differ between Series 1 and Series 2 (see
register table above), but the sleep/wake interface is identical.

> **Note:** Full deep sleep (EM2/EM3/EM4 energy modes) is not yet
> implemented. Currently only radio clock gating is used for power
> reduction between polls.

See the [Power Management](../advanced/power.md) chapter for the full
cross-platform power framework.

## What the Examples Demonstrate

Both `efr32mg1-sensor` and `efr32mg21-sensor` implement a Zigbee 3.0
temperature & humidity end device with:

- **Pure-Rust IEEE 802.15.4 radio driver** (no RAIL/GSDK)
- Embassy async runtime (SysTick time driver)
- Proper interrupt vector table (34 IRQs for MG1P, 51 for MG21)
- Button-driven network join/leave with edge detection
- LED status indication + Identify blink
- ZCL Temperature Measurement + Relative Humidity + Identify clusters
- Flash NV storage — network state persists across reboots
- Default reporting with reportable change thresholds
- Radio sleep/wake for power management

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `probe-rs` can't connect | Wrong chip name | Use `EFR32MG1P` (S1) or `EFR32MG21A020F512IM32` (S2) |
| Flash write fails (MG21) | Wrong page size | Series 2 uses 8 KB pages (vs 2 KB for Series 1) |
| Radio not working | Register init approximations | See known limitations — init registers need verification |
| Build fails with linker errors | Wrong target | Use `thumbv7em-none-eabihf` (S1) or `thumbv8m.main-none-eabihf` (S2) |
| No serial output | No logger configured | Add a defmt or RTT logger for debug output |

### Known Limitations

- **Scaffold radio init** — the pure-Rust radio register values are simplified
  approximations. The exact register sequences for 802.15.4 mode need
  verification against the EFR32xG1/xG21 Reference Manuals or extraction from
  the RAIL library source.
- **No deep sleep** — only radio clock gating is implemented; full EM2/EM3/EM4
  energy modes are not yet supported.
- **Simulated sensors** — temperature and humidity values are placeholders.
  Replace with I²C sensor drivers for real readings.

---

## Why Pure Rust on EFR32 Matters

The traditional Silicon Labs development flow requires:
- **GSDK** (Gecko SDK) — a large multi-GB SDK download
- **RAIL library** — pre-compiled radio abstraction layer (binary blob)
- **Simplicity Studio** — Eclipse-based IDE

The zigbee-rs pure-Rust approach eliminates all of this. The radio is
configured entirely through documented memory-mapped registers, making the
firmware:

1. **Fully auditable** — every line of radio code is visible
2. **Trivially reproducible** — just `cargo build`, no SDK setup
3. **Vendor-independent** — no binary blobs, no license restrictions
4. **Small** — no unused vendor code linked in

This is the 3rd and 4th pure-Rust radio driver in zigbee-rs (after PHY6222
and TLSR8258), demonstrating that the approach scales across chip families.
