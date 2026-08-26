# Build and validation

This file records the commands and measurements for the current
`experiment/zephyr-app-model` worktree. A successful build proves compilation,
linking, and the checks named below. It does not prove radio timing, flash
durability, sleep current, or OTA activation on hardware.

## Pinned tools

| use | pinned version |
|---|---|
| workspace, nRF, BL702, CC2340, EFR32 | `nightly-2026-03-23` |
| ESP32-C6/H2 and PHY6222 | `nightly-2026-08-01` |
| Telink host tools | Rust `1.94.1` |
| Telink target | `tc32-stage2-tc32-45` |
| `espflash` | `4.5.0` |
| TI CC2340 SDK source | commit `68ca021502383f367d0bf2a5517fdd0dcb0ef909` |
| Microsoft UF2 | revision `90e9741f217f5a40c98ba74d663e408041037578` |
| mdBook | `0.5.4` |

Do not replace either pinned nightly with a moving `nightly`. Compiler changes
have already produced material Cortex-M and ESP image-size differences without
source changes.

## Host validation

From the repository root:

```bash
cargo +nightly-2026-03-23 check --workspace --all-targets --locked
cargo +nightly-2026-03-23 test --workspace --locked
cargo +nightly-2026-03-23 test -p sensor-sed-app --features ota --locked
cargo +nightly-2026-03-23 test -p zigbee-runtime --features router --locked
cargo +nightly-2026-03-23 test -p zigbee-zdo --features router --locked
cargo +nightly-2026-03-23 clippy --workspace --all-targets --locked -- -D warnings
cargo +nightly-2026-03-23 fmt --all -- --check
```

The root workspace deliberately excludes most hardware images. Build those
from their own manifest or directory.

## Current release measurements

These are flashable payloads rebuilt from the current dirty worktree with the
pinned toolchains. Regression budgets and protected-region checks remain in
`.github/workflows/ci.yml`.

| image | measured bytes | artifact |
|---|---:|---|
| nRF52840 sensor, default | 223,344 | raw `.bin` |
| nRF52840 sensor, BME280 | 230,560 | raw `.bin` |
| nRF52840 sensor, SHT31 | 227,040 | raw `.bin` |
| nRF52833 sensor, default | 223,400 | raw `.bin` |
| nRF52833 sensor, BME280 | 230,976 | raw `.bin` |
| nRF52833 sensor, SHT31 | 227,128 | raw `.bin` |
| nRF52840 always-on End Device | 214,864 | raw `.bin` |
| nRF52840 UF2 board variants | 221,736–223,456 | linked image before UF2 container |
| ESP32-C6 sensor | 354,512 | application image |
| ESP32-H2 sensor | 339,568 | application image |
| BL702 sensor | 182,786 | raw linked image |
| BL702 sensor | 190,992 | packaged boot image |
| CC2340 sensor, pinned SDK | 212,688 | raw `.bin` |
| PHY6222 sensor | 129,556 | packaged `.phy6.bin` |
| EFR32MG1 sensor | 156,612 | raw `.bin` |
| EFR32MG21 sensor | 201,192 | raw `.bin` |
| TLSR8258 sensor, default SUSPEND | 279,652 | raw `.bin` |
| TLSR8258 sensor, LOW32K 250 ms proof | 284,436 | raw `.bin` |
| TLSR8258 sensor, LOW32K 10 s proof | 284,440 | raw `.bin` |
| TLSR8258 parent router | 343,660 | raw `.bin` |

The ESP merged flash images are 420,048 B (C6) and 405,104 B (H2), but the
application-image values above are the useful firmware growth measurements.

### Static RAM snapshot

Current `llvm-size` `.data + .bss` values, excluding stack and fragmented
linker reservations:

| image | static bytes |
|---|---:|
| nRF52840 sensor, default | 37,464 |
| nRF52833 sensor, default | 37,464 |
| nRF52840 always-on End Device | 4,232 |
| ESP32-C6 sensor | 52,268 |
| ESP32-H2 sensor | 51,848 |
| BL702 sensor | 31,984 |
| PHY6222 sensor | 4,288 |
| EFR32MG1 sensor | 14,912 |
| EFR32MG21 sensor | 17,136 |
| TLSR8258 sensor | 13,468 |
| TLSR8258 router | 24,280 |

EFR32MG1's application still has exactly `0x7C00` bytes of usable SRAM. Do not
replace that linker region with a nominal 32 KiB total. Static-section numbers
are not stack-headroom proof; use each target's linker/layout checker.

## nRF52840 and nRF52833

```bash
cd examples/nrf52840-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 build --release --locked --features sensor-bme280
cargo +nightly-2026-03-23 build --release --locked --features sensor-sht31

cd ../nrf52833-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 build --release --locked --features sensor-bme280
cargo +nightly-2026-03-23 build --release --locked --features sensor-sht31

cd ../nrf52840-router
cargo +nightly-2026-03-23 build --release --locked
```

The sensor products use `Idle` for both fast and slow waits and pair their
non-OTA profiles with `NoOta`. The nRF52840 always-on image is
`AlwaysOnEndDeviceApp`, so its non-parent MAC never advertises
`DeviceType::Router`.

Sensor commissioning, reporting, hardware AES, persistence, and reset/resume
are hardware-proven on nRF52840 and nRF52833. The new always-on End Device
composition still needs its complete HIL acceptance run.

### UF2 variants

```bash
cd examples/nrf52840-sensor-uf2
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-promicro
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-mdk
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-nrf-dongle
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-nrf-dk
```

Convert ELF → Intel HEX → UF2 with the pinned Microsoft UF2 revision. The
board feature selects the product linker map; never guess the application
base from a flat binary.

## ESP32-C6 and ESP32-H2

Use the fixed ESP/PHY nightly:

```bash
cd examples/esp32c6-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc

cd ../esp32h2-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc
```

Install and use `espflash 4.5.0`. The configured runner writes the
product-owned OTA partition table, places the wired image in `ota_0`, and
clears `otadata`:

```bash
cargo +nightly-2026-08-01 run --release --locked -Z build-std=core,alloc
```

Both products use `Active` for fast and slow waits. C6 selects `NoStatus`; H2
has an active-low status LED. Both use a concrete OTA transport with
`WithOta`, and OTA events enter the OTA lifecycle before generic
application handling.

- H2: full v1→v2 download, activation, reboot, and retained commissioned state
  are hardware-proven.
- C6: transfer through 18.3% is hardware-proven; complete activation remains
  open.

## BL702 XT-ZB1

```bash
cd examples/bl702-sensor
python3 -m pip install bflb-mcu-tool==1.10.0 pyserial
./build-image.sh
```

The script builds the production hardware-AES image, creates the raw binary,
and packages the BL702 boot header. `BL702_DIAGNOSTIC_LOG=1 ./build-image.sh`
retains the larger UART diagnostic trace.

The product owns `0x000FE000..0x00100000` as the two-sector security journal.
The shared sensor uses `NoStatus`, `NoOta`, `NoUserAction`, and
`Active`/`Active` waits. Temperature and humidity are synthetic; battery uses
the GPADC path. Radio commissioning and ZHA interview are hardware-proven.
Destructive sector erase/program plus reset/resume persistence remains open.

## CC2340R5

CI builds two forms:

1. a fallback compile with `CC2340_SDK_DIR` unset, which must fail radio
   initialization with `FirmwareUnavailable`;
2. the release image against the pinned TI SDK commit.

```bash
git clone https://github.com/TexasInstruments/simplelink-lowpower-f3-sdk.git
git -C simplelink-lowpower-f3-sdk checkout \
  68ca021502383f367d0bf2a5517fdd0dcb0ef909

cd examples/cc2340-sensor
CC2340_SDK_DIR=/absolute/path/to/simplelink-lowpower-f3-sdk \
  cargo +nightly-2026-03-23 build --release --locked --target-dir target/sdk
```

The board exposes real SysTick time, IOC button/LED setup, typed physical
resources, reset, and flash. The firmware composition maps those raw resources
to RTT diagnostics and the selected lifecycle action/status behavior. The
product reserves `0x0007E000..0x00080000` for security state and selects
`Active`/`Active` waits, synthetic/fixed measurements, and software AES.
Commissioning is not claimed: radio HIL is pending and the entropy backend
deliberately fails closed.

## PHY6222 / PHY6252

```bash
cd examples/phy6222-sensor
cargo +nightly-2026-08-01 build --release --locked

# Layout-only PHY6252 selection; still unverified on hardware.
cargo +nightly-2026-08-01 build --release --locked \
  --no-default-features --features phy6252
```

Package the default PHY6222 image exactly as CI does:

```bash
ELF=target/thumbv6m-none-eabi/release/phy6222-sensor
OBJCOPY=$(find "$(rustc +nightly-2026-08-01 --print sysroot)" \
  -name llvm-objcopy -print -quit)
"$OBJCOPY" -O ihex "$ELF" "$ELF.hex"
sh check-layout.sh "$ELF"
cargo +nightly-2026-08-01 run --quiet --locked \
  --manifest-path ../../Cargo.toml -p phy62x2-image -- \
  "$ELF.hex" "$ELF.phy6.bin"
```

The products reserve:

- PHY6222: `0x0007E000..0x00080000`
- PHY6252: `0x0003E000..0x00040000`

Fast and slow waits use `Idle` radio sleep. `Retention` is rejected. No AON
current value is claimed; the entire ROM boot/radio/join/journal path remains
hardware-unverified.

## EFR32MG1 and EFR32MG21

```bash
cd examples/efr32mg1-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 clippy --release --locked \
  --target thumbv7em-none-eabi -- -D warnings
python3 tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor

cd ../efr32mg21-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 clippy --release --locked \
  --target thumbv8m.main-none-eabihf -- -D warnings
python3 tools/verify-layout.py \
  target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor
```

EFR32MG1 uses `Active` fast waits and `Retention` slow waits. Commissioning,
hardware AES, SHT3x and battery reporting, Identify, persistence,
reset/resume, RTCC wake, and EM2 are hardware-proven. A real OTA image
install/reboot remains open.

EFR32MG21 targets BRD4181A on BRD4001A:

- PB0: active-high LED
- PD2: active-low button
- HFXO: 38.4 MHz, CTUNE 133
- bootloader: `0x00000000..0x00004000`
- application: `0x00004000..0x0007C000`
- persistence: `0x0007C000..0x00080000`

Its `Idle` wait is radio-gated WFE with a 1 kHz SysTick, not EM2. The complete
MG21 hardware path remains HIL-unverified.

## Telink TLSR8258

Install the repository's `tc32-stage2-tc32-45` target toolchain under
`.toolchains/tc32-stage2-tc32-45`, then run:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
./scripts/tlsr8258.sh build router
```

The default sensor uses:

- fast wait: `Active`;
- slow wait: `Idle`;
- atomic full-SRAM timer `SUSPEND`.

`sensor-retention` is a feature-gated reset-on-wake LOW32K image with a
250 ms policy. `sensor-retention-10s` uses the same restoration path with a
10-second steady-state slow poll. Neither changes the default production
selection.

The independent retention lab is:

```bash
./scripts/tlsr8258.sh build diag-retention
./scripts/tlsr8258.sh flash diag-retention
```

Its completion marker is `0x5254600D`; `0xDEADxxxx` indicates failure. A
compile-time symbol/layout pass is not the HIL completion marker.

The router is `ParentRouterApp + PersistentChildren`. Its product partitions
are `0x72000..0x74000` for children, `0x74000..0x76000` for security,
`0x76000..0x77000` for factory EUI, and `0x77000..0x78000` for factory
configuration/calibration.

## Documentation

```bash
cargo install mdbook --version 0.5.4 --locked
mdbook build docs/book
```

Pages deployment is intentionally gated to pushes on `main`/`master`.
Building this experiment branch validates the book locally but does not
publish it.
