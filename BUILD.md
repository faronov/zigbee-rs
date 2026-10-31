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
| Telink target | `tc32-1.98.1-20261003-31a272` (LLVM 23.1.2) |
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

Recorded image sizes, physical limits, and RAM/stack snapshots are kept in a
single place: the book's
[Firmware Size](docs/book/src/advanced/firmware-size.md) page. CI firmware
jobs also upload `*.size.json` measurements with their artifacts (PHY62x2 is
measured by its layout check instead); those describe the exact commit that
was built, whereas the book's tables are dated snapshots.

Both PHY62x2 variants fit their mandatory 187,104 B XIP slot (see
[PHY6222 / PHY6252](#phy6222--phy6252)).

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
python3 -m pip install --require-hashes --no-deps \
  -r tools/bl702-image-tool/build-requirements.txt
python3 -m pip install --require-hashes --no-deps --no-build-isolation \
  -r tools/bl702-image-tool/requirements.txt
cd examples/bl702-sensor
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

# PHY6252 feature build; same physical XIP gate.
cargo +nightly-2026-08-01 build --release --locked \
  --no-default-features --features phy6252
```

The default PHY6222 occupied XIP span is currently 132,576 B against the hard
187,104 B gate and the PHY6252 feature build 132,544 B. The slot ends below
the PHY6252 security partition at 0x3E000 with room for the image tail that
`phy62x2-image` appends. Release builds from
`examples/phy6222-sensor` use the `immediate-abort` panic strategy (configured
in its `.cargo/config.toml`): panics trap into the halting `HardFault` handler.
These limits remain mandatory after removal of artificial regression budgets.
Neither measurement is hardware proof.

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

### PB-03F-Kit (PHY6252)

```bash
cd examples/pb03f-kit
cargo +nightly-2026-08-01 build --release --locked   # pb03f-zigbee, pb03f-lab, pb03f-radio-lab

ELF=target/thumbv6m-none-eabi/release/pb03f-zigbee
OBJCOPY=$(find "$(rustc +nightly-2026-08-01 --print sysroot)" \
  -name llvm-objcopy -print -quit)
"$OBJCOPY" -O ihex "$ELF" "$ELF.hex"
XIP_SLOT_BYTES=200448 sh ../phy6222-sensor/check-layout.sh "$ELF"
cargo +nightly-2026-08-01 run --quiet --locked \
  --manifest-path ../../Cargo.toml -p phy62x2-image -- --rom-boot \
  "$ELF.hex" "$ELF.table.bin" "$ELF.app.bin"
```

Program `$ELF.app.bin` at flash `0x5000`, then `$ELF.table.bin` at `0x2000`,
through the boot ROM's SPIF routines; `0x3000..0x4FFF` (factory record and BLE
address) is never written. See the
[PHY6222 platform guide](docs/book/src/platform-guides/phy6222.md#pb-03f-kit-phy6252).

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

Install the [pinned TC32 release](https://github.com/modern-tc32/rust/releases/tag/tc32-1.98.1-20261003-31a272)
under `.toolchains/tc32-1.98.1-20261003-31a272`, stripping the archive's
single top-level directory. Select the archive matching the build host:

| host | archive | SHA-256 |
|---|---|---|
| Linux x86-64 (Actions) | `tc32-rust-toolchain-linux-amd64.tar.gz` | `d72e68cfb7490583911323c358fd9dcc760baaa7ebdf1145cdc7208bae6b931b` |
| macOS x86-64 | `tc32-rust-toolchain-macos-amd64.tar.gz` | `6c8fb85675a97a79eb51d52f69005620516d49a32e5202c9fb527ec6fa2138cf` |
| macOS ARM64 | `tc32-rust-toolchain-macos-arm64.tar.gz` | `5f4eca476474d91c20b5dcc9b6779fbb7a6b9bdf77696c788a5886e16da9653d` |

Verify the archive checksum before extraction (`sha256sum` on Linux,
`shasum -a 256` on macOS). The packaged compiler identifies itself as
`1.98.1-dev`, LLVM `23.1.2`, with `commit-hash: unknown`; the pinned release
asset checksum, not that missing compiler metadata, establishes provenance.
`TC32_TOOLCHAIN` can override the installation path for all production,
lab, and radio helpers. Host tests/Clippy remain on Rust `1.94.1`.

Then run:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
./scripts/tlsr8258.sh build router
```

Production, lab, and radio builds explicitly enable LLVM tail merging with the
pinned compiler. The October 4 macOS rebuild measured 195,688 B for the default
sensor, 198,764/198,768 B for the two LOW32K variants, and 288,508 B for the
router. See the
[tail-merge comparison](docs/book/src/advanced/firmware-size.md#tlsr8258-tail-merging-2026-10-04)
and the separate compiler-only baseline on that page.
The earlier measurements below remain historical; the new images are not
hardware-qualified.

The default sensor uses:

- fast wait: `Active`;
- slow wait: `Idle`;
- atomic full-SRAM timer `SUSPEND`.

`sensor-retention` is a feature-gated reset-on-wake LOW32K image with a
250 ms policy. `sensor-retention-10s` uses the same restoration path with a
10-second steady-state slow poll. Neither changes the default production
selection.

The former independent retention lab (`diag-retention`, marker `0x5254600D`)
was removed with `tools/telink-tlsr8258-lab`. LOW32K hardware proof must run
on the `sensor-retention` images; a compile-time symbol/layout pass is not
that proof.

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
