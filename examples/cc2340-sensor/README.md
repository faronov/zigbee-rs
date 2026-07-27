# CC2340R5 Zigbee Temperature Sensor

A `no_std` Zigbee end-device bring-up project for the TI CC2340R5 and
LP-EM-CC2340R5.

## Current status

The radio host driver is Rust and does not link TI RCL, ZBOSS, FreeRTOS, or a
C platform shim. The build imports TI's official IEEE PBE/MCE/RFE microcode,
PHY settings, and board PA table as data.

Raw polling TX/RX is implemented and the Cortex-M0+ firmware builds, but no
CC2340 board has yet been tested on air. The full example is not hardware-ready
because its Embassy time driver remains a compile stub and complete IOC/button
configuration is pending.

## Hardware

- MCU: CC2340R52, Cortex-M0+, 512 KB flash, 36 KB SRAM
- Radio: 2.4 GHz IEEE 802.15.4 and Bluetooth LE
- Board: LP-EM-CC2340R5
- Target: `thumbv6m-none-eabi`

## Build

```bash
rustup target add thumbv6m-none-eabi

export CC2340_SDK_DIR=/path/to/simplelink-lowpower-f3-sdk
cd examples/cc2340-sensor
cargo build --release
```

Without `CC2340_SDK_DIR`, the source still compiles, but radio initialization
returns `FirmwareUnavailable`.

## Imported SDK data

`zigbee-mac/build.rs` reads:

- `source/ti/devices/cc23x0r5/rf_patches/`
- `source/ti/devices/cc23x0r5/inc/`
- `source/ti/devices/radioconfig/.meta/config/rcl/`
- `source/ti/devices/radioconfig/.meta/config/rcl_common/`

No TI archive is linked.

## Project structure

```text
cc2340-sensor/
├── .cargo/config.toml   # thumbv6m-none-eabi
├── Cargo.toml
├── build.rs             # linker script only
├── memory.x             # CC2340R52 36 KB SRAM layout
└── src/main.rs          # device setup and simulated measurements
```

Use TI UniFlash and the board's XDS110 probe for eventual hardware bring-up.
