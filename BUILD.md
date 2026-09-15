# Build and validation

This file records build commands and dated image measurements for `master`.
A successful build proves compilation, linking, and the checks named below.
It does not prove radio timing, flash
durability, sleep current, or OTA activation on hardware.

Validation terms in this file are strict:

- **protocol implementation** means the code path exists and unsupported
  capabilities fail explicitly;
- **host-tested** means portable state-machine or wire behavior passed host
  tests;
- **cross-compiled/build-tested** means the pinned target image and named
  layout/symbol checks passed;
- **exact-image HIL/packet capture** means that exact byte image was flashed
  and observed on hardware or over the air;
- **certified** means an external Zigbee certification result.

The project targets Zigbee Core R22 (`05-3474-22`) and BDB 3.0.1
(`16-02828-012`). BDB 3.1 is R23-or-newer future guidance. Nothing in this
file is a certification claim.

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

## Size policy and recorded measurements

Artificial regression budgets are no longer enforced. Size reports still
measure final artifacts, but exceeding a former budget does not fail the
build. Physical Flash/RAM limits, protected partitions, OTA-slot bounds,
stack reserves, and linker/layout checks remain mandatory.

Baseline snapshot: **2026-09-06**, with TLSR8258 parent-router measurements
refreshed **2026-09-07** and EFR32MG1/ESP32-C6/H2 refreshed **2026-09-08**.
The parent router and ESP32-C6 failed the regression budgets enforced at those
snapshots; EFR32MG1 and ESP32-H2 passed their build/layout gates. Those former
budgets are now historical comparisons, not live blockers. PHY62x2 occupied-XIP
measurements were refreshed **2026-09-15**: PHY6222 fits, while PHY6252 still
fails the physical limit. These are not exact-image hardware reruns. Prior
hardware evidence is called out separately below. Margins against former
budgets are not physical headroom.

| image | measured bytes | physical limit / former regression budget | margin vs reference | measured artifact |
|---|---:|---:|---:|---|
| PHY6222 sensor | 130,752 | 130,816 hard XIP | 64 | occupied XIP span |
| PHY6252 feature-selected sensor | 130,912 | 130,816 hard XIP | -96 | failed-link occupied XIP span |
| BL702 sensor | 189,442 | 192,512 regression | 3,070 | raw linked `.bin` |
| nRF52840 sensor, default | 224,472 | 225,280 regression | 808 | raw `.bin` |
| nRF52840 sensor, BME280 | 231,792 | 245,760 regression | 13,968 | raw `.bin` |
| nRF52840 sensor, SHT31 | 228,216 | 241,664 regression | 13,448 | raw `.bin` |
| nRF52840 always-on End Device | 210,072 | 253,952 regression | 43,880 | raw `.bin` |
| nRF52840 UF2 ProMicro | 222,968 | 237,568 regression | 14,600 | linked image before UF2 |
| nRF52840 UF2 MDK | 222,848 | 237,568 regression | 14,720 | linked image before UF2 |
| nRF52840 UF2 PCA10059 | 224,536 | 237,568 regression | 13,032 | linked image before UF2 |
| nRF52840 UF2 DK | 224,552 | 237,568 regression | 13,016 | linked image before UF2 |
| nRF52833 sensor, default | 224,464 | 225,280 regression | 816 | raw `.bin` |
| nRF52833 sensor, BME280 | 231,784 | 245,760 regression | 13,976 | raw `.bin` |
| nRF52833 sensor, SHT31 | 228,208 | 241,664 regression | 13,456 | raw `.bin` |
| EFR32MG1 sensor | 163,236 | 167,936 regression | 4,700 | raw `.bin` |
| EFR32MG21 sensor | 202,820 | 212,992 regression | 10,172 | raw `.bin` |
| CC2340R5 sensor, pinned SDK | 223,536 | 225,280 regression | 1,744 | raw `.bin` |
| ESP32-C6 sensor | 369,248 | 368,640 regression | -608 | application image; above former budget |
| ESP32-H2 sensor | 354,096 | 356,352 regression | 2,256 | application image |
| TLSR8258 sensor, default SUSPEND | 290,616 | 294,912 regression | 4,296 | raw `.bin` |
| TLSR8258 sensor, LOW32K 250 ms | 295,548 | 299,008 regression | 3,460 | raw `.bin` |
| TLSR8258 sensor, LOW32K 10 s | 295,552 | 299,008 regression | 3,456 | raw `.bin` |
| TLSR8258 parent router | 433,756 | 430,080 regression | -3,676 | raw `.bin`; above former budget |

Additional exact packaging and physical limits:

- PHY6252's feature-selected failed link is 96 B over the mandatory 130,816 B
  XIP slot. No current executable/package is qualified; its hardware path
  remains unverified.
- BL702's packaged boot image is 197,648 B. The packager/device physical slot
  is 1,044,480 B; the product still independently protects its
  `0x000FE000..0x00100000` security journal and linked XIP limit.
- EFR32MG1's prior Zigbee OTA container is 162,538 B and assumes the resident
  Gecko bootloader; it has not been regenerated for the current image.
- EFR32MG21 has no OTA packaging path.
- CC2340R5's physical application slot is 516,096 B.
- ESP32-C6/H2 merged flash images are 434,784 B and 419,632 B respectively.
  The prior 368,098 B C6 and 352,994 B H2 Zigbee OTA containers
  have not been regenerated for the current applications. Each OTA slot is
  2,031,616 B.
- TLSR8258's physical application boundary is 458,752 B (`0x70000`).

### Static RAM snapshot

Exact RAM/layout measurements recorded with this snapshot:

| image | `.data` | `.bss` | static total | available/linked stack |
|---|---:|---:|---:|---:|
| PHY6222 / PHY6252 | 652 | 4,288 | 4,940 | 54,384 |
| EFR32MG1 | 260 | 14,720 | 14,980 | 16,760 |
| EFR32MG21 | 308 | 18,280 | 18,588 | 46,944 |
| CC2340R5 | 16 | 4,756 | 4,772 | — |
| ESP32-C6 | 3,008 | 50,344 | 53,352 | — |
| ESP32-H2 | 2,652 | 50,272 | 52,924 | — |
| TLSR8258 LOW32K fresh-root SVC stack | — | — | — | 8,448 |

ESP32-H2 initialized data includes 84 B in `.data.wifi`.
EFR32MG1's application still has exactly `0x7C00` bytes of usable SRAM. Its
available stack is 376 B above the 16 KiB gate. The TLSR8258 retained
fresh-root SVC stack is 256 B above its 8 KiB gate. Static-section numbers are
not runtime high-water proof; use each target's linker/layout checker and
hardware watermark where available.

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

Earlier nRF52840/nRF52833 sensor images produced hardware evidence for
commissioning, reporting, hardware AES, persistence, and reset/resume. The
exact 2026-09-06 images above have build/layout evidence only. The always-on
End Device composition still needs its complete HIL acceptance run.

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

These are real pure-Rust `esp-radio` IEEE 802.15.4 backends, not scaffolds.
Earlier hardware runs provide narrow path evidence:

- H2: a full v1→v2 download, activation, reboot, and retained commissioned
  state were demonstrated.
- C6: commissioning/reporting and transfer through 18.3% were demonstrated;
  complete activation remains open.

Those runs are not recorded as reruns of the exact 369,248 B and 354,096 B
images in the recorded measurement table. That C6 image fits its physical OTA
slot but failed its then-enforced regression budget. A controlled build before
the shared OTA deadline fix was already 369,184 B; the fix added 64 B without
changing the budget or selected compiler at the time. That artificial budget
has since been removed; physical OTA and other validation checks remain.

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
the GPADC path. Earlier images produced radio commissioning and ZHA interview
evidence. The exact 189,442 B raw / 197,648 B boot-image pair is build/package
tested only. Destructive sector erase/program plus reset/resume persistence
remains open.

## CC2340R5

CI builds two forms:

1. a fallback compile with `CC2340_SDK_DIR` unset, which must fail radio
   initialization with `FirmwareUnavailable`;
2. the release image against the pinned TI SDK commit.

The fallback command used by CI is:

```bash
cd examples/cc2340-sensor
env -u CC2340_SDK_DIR \
  cargo +nightly-2026-03-23 build --release --locked \
  --target-dir target/fallback
```

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
deliberately fails closed. The recorded pre-static-task pinned-SDK image is
223,536 B; its 4,772 B static-RAM figure is not a current task-capacity check.
The physical application slot remains 516,096 B.

## PHY6222 / PHY6252

```bash
cd examples/phy6222-sensor
cargo +nightly-2026-08-01 build --release --locked

# This PHY6252 feature build currently fails the physical XIP limit.
cargo +nightly-2026-08-01 build --release --locked \
  --no-default-features --features phy6252
```

The default PHY6222 occupied XIP span is 130,752 B against the hard 130,816 B
gate, leaving 64 B. The exact PHY6252 feature build has a failed-link span of
130,912 B, exceeding the same physical gate by 96 B; it produces no validated
executable/package. These limits remain mandatory after removal of artificial
regression budgets. Neither measurement is hardware proof.

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

EFR32MG1 uses `Active` fast waits and `Retention` slow waits. Its exact image
is 163,236 B; `.data` is 260 B, `.bss` is 14,720 B, and 16,760 B remains for
the linked stack (376 B above the 16 KiB gate). The prior 162,538 B Zigbee OTA
container has not been regenerated; it assumes the resident Gecko bootloader.
Earlier images produced
hardware evidence for commissioning, hardware AES, SHT3x and battery
reporting, Identify, persistence, reset/resume, RTCC wake, and EM2. A real OTA
install/reboot remains open, and the current exact image has no recorded HIL
rerun.

EFR32MG21 targets BRD4181A on BRD4001A:

- PB0: active-high LED
- PD2: active-low button
- HFXO: 38.4 MHz, CTUNE 133
- bootloader: `0x00000000..0x00004000`
- application: `0x00004000..0x0007C000`
- persistence: `0x0007C000..0x00080000`

Its `Idle` wait is radio-gated WFE with a 1 kHz SysTick, not EM2. The complete
MG21 hardware path remains HIL-unverified. The current raw image is 202,820 B;
`.data` is 308 B, `.bss` is 18,280 B, and the linked available stack is
46,944 B. There is no MG21 OTA packaging path.

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
are `0x70000..0x72000` for APS bindings/groups/application keys,
`0x72000..0x74000` for children, `0x74000..0x76000` for security,
`0x76000..0x77000` for factory EUI, and `0x77000..0x78000` for factory
configuration/calibration.

The recorded router snapshot measured 433,756 B, exceeding its then-enforced
430,080 B (`0x69000`) regression budget by 3,676 B. That historical gate was
introduced above the earlier 427,776 B R22 image, replacing the pre-R22
356,352 B budget. Artificial regression budgets no longer block builds.
The mandatory physical `0x70000` boundary and all journals are unchanged:
that snapshot has 24,996 B of physical headroom. The 28,672 B difference
between the former budget and physical boundary is not a reserved partition.

The LOW32K images retain an 8,448 B fresh-root SVC stack, 256 B above the
8 KiB gate.

## Documentation

```bash
cargo install mdbook --version 0.5.4 --locked
mdbook build docs/book
```

Pages deployment is intentionally gated to pushes on `main`/`master`.
Documentation can deploy after its core/code checks succeed even when a
required firmware job fails, but prebuilt binaries and one-click installation
require successful firmware jobs. Removing artificial size budgets does not
bypass physical layout, image, or other required checks.
