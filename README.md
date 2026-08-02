# zigbee-rs

A complete Zigbee PRO R22 protocol stack written in Rust, targeting embedded
`no_std` environments. Built on `async` traits for seamless integration with
Embassy and other embedded async runtimes.

> **Specification roadmap:** Zigbee PRO R22 with BDB 3.0.1 remains the active
> production target. R23/BDB 3.1 is a deferred, optional roadmap item rather
> than a current release goal. R23 support must remain compile-time isolated
> so it adds no code or RAM to R22 products.

> **Role-specialized firmware:** sensor builds compile out parent/router
> maintenance and parent state rather than carrying dormant code. With the
> pinned `tc32-45` toolchain, the current software-AES TLSR8258 release images
> are 272,600 bytes for the end-device sensor and 332,440 bytes for the parent
> router.
> See the [firmware-size report](docs/book/src/advanced/firmware-size.md).

```text
63,000+ lines of Rust · 199 source files · 30 crates · 45 ZCL clusters · 12 hardware platforms · 300+ tests · 5 pure-Rust radios · crash-safe NV storage across nRF, ESP32, BL702, and TLSR8258
```

## Architecture

```text
┌──────────────────────────────────────────────────────┐
│                    zigbee (top)                       │
│           coordinator · router · re-exports           │
├──────────────────────────────────────────────────────┤
│  zigbee-runtime   │  zigbee-bdb    │  zigbee-zcl     │
│  builder, power,  │  commissioning │  45 clusters,    │
│  NV storage,      │  steering,     │  foundation,     │
│  device templates  │  formation     │  reporting       │
├───────────────────┴────────────────┴─────────────────┤
│                    zigbee-zdo                          │
│          discovery · binding · network mgmt           │
├──────────────────────────────────────────────────────┤
│                    zigbee-aps                          │
│          frames · binding · groups · security         │
├──────────────────────────────────────────────────────┤
│                    zigbee-nwk                          │
│      frames · routing (AODV+tree) · security · NIB   │
├──────────────────────────────────────────────────────┤
│                    zigbee-mac                          │
│  MacDriver trait · 12 backends (see table below)     │
├──────────────────────────────────────────────────────┤
│           zigbee-crypto · zigbee-types                │
│ low-stack AES-CCM* · addresses · PANs · channels     │
└──────────────────────────────────────────────────────┘
```

## Quick Start

### Mock examples (no hardware needed)

```bash
# Temperature + humidity sensor simulation
cargo run -p mock-sensor

# Coordinator (network formation + device join)
cargo run -p mock-coordinator

# Dimmable light
cargo run -p mock-light

# Sleepy end device (full SED lifecycle)
cargo run -p mock-sleepy-sensor
```

### Build the entire workspace

```bash
cargo build
cargo test
```

### ESP32-C6 / ESP32-H2 firmware

```bash
cd examples/esp32c6-sensor   # or esp32h2-sensor
cargo build --release -Z build-std=core,alloc
espflash flash target/riscv32imac-unknown-none-elf/release/esp32c6-sensor
```

Or flash via the [web flasher](https://faronov.github.io/zigbee-rs/) (no tools needed, just a browser with Web Serial).

### nRF52840 firmware (with debug probe)

```bash
cd examples/nrf52840-sensor
# Default (on-chip temp):
cargo build --release
# With BME280 sensor (temp + humidity + pressure):
cargo build --release --features sensor-bme280
# With SHT31 sensor (temp + humidity):
cargo build --release --features sensor-sht31
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-sensor
```

### nRF52840 router (with debug probe)

```bash
cd examples/nrf52840-router
cargo build --release
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-router
```

> **Router mode** — joins as an always-on FFD, relays frames, and sends
> periodic Link Status broadcasts. Child association is not implemented yet.

### Telink TLSR8258 parent router — EXPERIMENTAL

```bash
./scripts/tlsr8258.sh build router
./scripts/tlsr8258.sh flash router
```

> **Router mode (Telink)** — joins as an FFD, relays routed and broadcast NWK
> traffic, sends router maintenance frames, responds to beacon requests, and
> admits polling sleepy children while permit joining is open. Association
> Response delivery, Trust Center Update-Device/Tunnel handling, indirect
> queues, child authorization, and rejoin are implemented. ZHA join,
> TCLK exchange, interview, Identify, routed-frame relay, and silent
> reset/resume are hardware-proven. The reset capture contained no scan,
> association, or Device_annce traffic. Parent timing and Frame Pending still
> require the child-device sniffer gate documented in
> `examples/telink-tlsr8258-router/README.md`.

> **Flash NV storage** — network state is saved to internal flash (last 8 KB) and automatically
> restored on power-up. No re-pairing after power cycles!

### nRF52840 firmware (nice!nano / ProMicro — UF2 drag-and-drop)

```bash
cd examples/nrf52840-sensor-uf2
cargo build --release
# Convert to UF2 (CI does this automatically):
# uf2conv.py -c -f 0xADA52840 -b 0x26000 firmware.bin -o firmware.uf2
# Double-tap RESET → copy .uf2 to the "NICENANO" USB drive
```

### BL702 firmware

```bash
cd examples/bl702-sensor
python3 -m pip install bflb-mcu-tool==1.10.0 pyserial
./build-image.sh
```

This builds a pure-Rust XT-ZB1 sleepy Zigbee sensor with live
ACAL/KCAL/ROSCAL/RCCAL, polling RX/TX, CCA, and energy detection. No Bouffalo
archive is linked. Direct RF operation, joining, ZHA interview, Trust Center
link-key exchange, and attribute reporting are hardware-tested. Temperature
and humidity remain synthetic. Battery reporting now samples the nominal
internal VBAT/2 GPADC path, with an explicit synthetic fallback if conversion
fails. The reusable BL702 HAL also provides I2C, SPI, GPIO, PWM, UART, timer,
eFuse, and XIP flash, but the new sensor/flash paths still need silicon
validation and application integration. An opt-in `hardware-aes` build
(`BL702_HARDWARE_AES=1 ./build-image.sh`) routes Zigbee CCM*/AES-MMO through
the SEC_ENG AES-128 accelerator and drops the software AES core; it is
compile/build-proven only and not yet silicon-validated, so software AES
remains the default recovery image.

### CC2340 firmware

```bash
cd examples/cc2340-sensor

# Compile-only fallback without radio assets:
cargo build --release

# Embed TI radio microcode/configuration:
CC2340_SDK_DIR=/path/to/simplelink_lowpower_f3_sdk cargo build --release
```

The CC2340 host driver is Rust and does not link RCL or ZBOSS. A build without
`CC2340_SDK_DIR` compiles, but radio initialization returns
`FirmwareUnavailable`.

### Telink TLSR8258 firmware (pure Rust — no vendor SDK!)

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build router
```

> The TLSR8258 radio driver uses pure-Rust register access. For real tc32
> firmware, use the [modern-tc32](https://github.com/modern-tc32) toolchain.
> The HAL also exposes typed single-owner GPIO, I2C, SPI, UART, ADC, PWM,
> flash, timer/watchdog, IRQ/reset/clock, wake/capture, RNG, and AES APIs.
> Hardware bring-up diagnostics live separately in
> `tools/telink-tlsr8258-lab`.

### PHY6222 firmware (pure Rust — no vendor SDK!)

```bash
cd examples/phy6222-sensor
cargo build --release   # no stubs, no vendor blobs needed!
```

### EFR32MG1 firmware (pure Rust — no vendor SDK!)

```bash
cd examples/efr32mg1-sensor
cargo build --release   # no stubs, no GSDK, no RAIL library needed!
tools/verify-layout.py target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

> The TRÅDFRI image is linked at `0x4000` for the resident Gecko bootloader.
> It is an unconditional 30-second RTCC/EM2 sleepy end device. Hardware
> diagnostics are separate binaries under `tools/efr32mg1-lab`. Verify every
> ELF before creating a flash image.

### EFR32MG21 firmware (pure Rust — no vendor SDK!)

```bash
cd examples/efr32mg21-sensor
cargo build --release   # no stubs, no GSDK, no RAIL library needed!
```

> Series 2 Cortex-M33 — independent `efr32s2` MAC module. Works with
> Sonoff ZBDongle-E and BRD4180A/BRD4181A dev kits.

### Vendor Libraries

Some experimental backends depend on vendor radio libraries. A successful
stub build proves only that the Rust side compiles.

#### BL702 — direct Rust radio + retained legacy FFI

The direct backend reconstructs the BL702 digital PHY, analog calibration,
channel synthesizer, M154 FIFO, CCA, RX, and TX register paths in Rust. The
XT-ZB1 sensor builds for `riscv32imc-unknown-none-elf` and links no vendor
code. The RV32A instructions emitted by `riscv32imac` trap on the tested
BL702.

The old `Bl702Mac` FFI backend remains side-by-side for comparison with
Bouffalo's `liblmac154.a` and `libbl702_rf.a`. Those archives use the
`ilp32f` ABI and are not used by the Rust sensor.

#### CC2340 — TI SimpleLink Low Power F3 SDK

```bash
# Download TI SimpleLink SDK from https://www.ti.com/tool/SIMPLELINK-LOWPOWER-F3-SDK
export CC2340_SDK_DIR=/path/to/simplelink_lowpower_f3_sdk
cd examples/cc2340-sensor && cargo build --release
```

The build script does not link TI host libraries. It imports TI's official
IEEE PBE/MCE/RFE microcode, PHY register configuration, and LP-EM-CC2340R5 PA
table as build-time data. Rust code owns LRFD setup, FIFO access, and the
radio operation state machine.

#### Telink TLSR8258 — Pure Rust (no vendor library needed)

```bash
./scripts/tlsr8258.sh build sensor
```

The TLSR8258 radio driver uses pure-Rust register access — no `libdrivers_8258.a` required.

> **PHY6222**, **TLSR8258**, **EFR32MG1**, **EFR32MG21**, and **nRF52840/52833** and **ESP32-C6/H2** do **not** need any vendor libraries.

## Peripheral HALs

| Platform | Reusable peripherals | Validation boundary |
|----------|----------------------|---------------------|
| **BL702** | GPIO, I2C0, SPI0, GPADC/VBAT, PWM, UART0/1, timer, eFuse, XIP flash | UART, timer, eFuse, and radio paths hardware-proven; new GPIO/I2C/SPI/ADC/PWM/flash paths host-tested and RV32IMC-compiled, not yet silicon-tested |
| **TLSR8258** | GPIO/edge capture, I2C, SPI, UART, ADC, PWM/IR, flash, timers/watchdog, IRQ/reset/clock, suspend/wake, RNG, AES-128 | Radio, GPIO, flash, timer-wake suspend, and deployed persistence paths have hardware evidence. UART, I2C/SPI, advanced PWM/capture, pad/comparator wake, RNG entropy quality, and AES remain silicon-validation gates |
| **EFR32MG1** | GPIO, I2C, SPI, ADC, PWM, flash, RTCC, bootloader storage | Board peripheral paths hardware-proven except the limitations called out below |

BL702 and TLSR8258 stateful bus/PWM constructors consume unique peripheral
tokens. TLSR8258 I2C and SPI intentionally consume the same serial-controller
token and non-`Clone` route pins because those functions overlap in hardware.
Their board crates preserve exclusive ownership even when an application
leaves a peripheral unused.

TLSR8258 factory data is geometry-specific for 512 KiB, 1 MiB, 2 MiB, and
4 MiB flashes. Non-512-KiB products must verify the JEDEC capacity before
reading the factory EUI-64 or ADC calibration. Zbit program/erase operations
fail closed unless the owned ADC/PC5 voltage guard measures a safe, stable
supply immediately before each physical operation. The RNG uses an
AES-128 CTR_DRBG seeded from SHA-256-conditioned VBAT/GND ADC observations;
the DRBG has a NIST known-answer test, but the physical entropy source still
requires SP 800-90B characterization.

The TLSR8258 AES block has a bounded, token-owned ECB driver and an opt-in,
silicon-proven `zigbee-crypto` backend serving NWK/APS CCM* and AES-MMO.
Software AES remains the recovery default. The Telink MAC still reports
`hardware_security = false` because the Rust stack performs security itself;
the accelerator is a block-cipher provider, not autonomous MAC offload.

## MAC Backends

| Backend | Radio driver | Target | Notes |
|---------|-------------|--------|-------|
| **MockMac** | ✅ Simulation | Host (macOS/Linux/Windows) | Full protocol sim, no hardware |
| **ESP32-C6** | ✅ esp-ieee802154 | `riscv32imac-unknown-none-elf` | Native 802.15.4 radio |
| **ESP32-H2** | ✅ esp-ieee802154 | `riscv32imac-unknown-none-elf` | Native 802.15.4 radio |
| **nRF52840** | ✅ nrf-radio | `thumbv7em-none-eabihf` | 802.15.4 radio peripheral |
| **nRF52833** | ✅ nrf-radio | `thumbv7em-none-eabihf` | 802.15.4 radio peripheral |
| **BL702** | 🦀 Pure-Rust direct registers | `riscv32imc-unknown-none-elf` | XT-ZB1 RF, join, ZHA interview, security, and reporting hardware-tested |
| **CC2340** | 🦀 Rust host + TI microcode | `thumbv6m-none-eabi` | Raw polling TX/RX implemented; hardware validation, CCA, IRQs, filtering, and auto-ACK pending |
| **Telink TLSR8258** | 🦀 **Pure Rust** | `tc32-unknown-none-elf` | Real tc32 builds in the dedicated [modern-tc32](https://github.com/modern-tc32) CI workflow |
| **PHY6222** | 🦀 **Pure Rust** | `thumbv6m-none-eabi` | Zero vendor blobs — direct register access! |
| **EFR32MG1** | 🦀 **Pure Rust** | `thumbv7em-none-eabi` | Series 1, Cortex-M4F — hardware-proven direct register access |
| **EFR32MG21** | 🦀 **Pure Rust** | `thumbv8m.main-none-eabihf` | Series 2, Cortex-M33 — independent `efr32s2` module |

> **Legend:** ✅ = functional radio driver · ⚡ = compile-only scaffold · 🦀 = Rust host driver. BL702 is hardware-tested on XT-ZB1; CC2340 still embeds official TI radio microcode as data.

All 12 supported firmware targets build in CI and produce downloadable artifacts.

## ZCL Clusters (45)

**General:** Basic, Power Config, Device Temp Config, Identify, Groups, Scenes, On/Off,
On/Off Switch Config, Level Control, Alarms, Time, Analog Input, Analog Output, Analog
Value, Binary Input, Binary Output, Binary Value, Multistate Input, OTA Upgrade, Poll
Control, Green Power, Diagnostics

**Closures:** Door Lock, Window Covering

**HVAC:** Thermostat, Fan Control, Thermostat UI Config

**Lighting:** Color Control, Ballast Config

**Measurement:** Illuminance, Illuminance Level, Temperature, Pressure, Flow, Humidity,
Occupancy, Electrical, Carbon Dioxide, PM2.5, Soil Moisture

**Security:** IAS Zone, IAS ACE, IAS WD

**Smart Energy:** Metering

**Touchlink:** Commissioning

## Design Principles

- **`#![no_std]`** everywhere — no heap allocation, `heapless` for bounded collections
- **`async` MacDriver trait** — 13 methods, no `Send`/`Sync` requirement
- **Platform-agnostic** — same stack code runs on mock, ESP32, nRF, BL702, CC2340, Telink, PHY6222, EFR32
- **Power-aware** — two-phase polling (fast/slow), DC-DC, TX power reduction, radio sleep, CPU suspend, system sleep, flash deep power-down, GPIO preparation, reportable change thresholds
- **Five pure-Rust radios** — BL702, PHY6222, TLSR8258, EFR32MG1, and EFR32MG21 need zero vendor blobs
- **Router support** — full relay, RREQ rebroadcast, Link Status, indirect queue, source routing
- **Manual frame parsing** — no `serde`, bitfield encode/decode for all frame types
- **Embassy-compatible** — designed for single-threaded async executors
- **Layered crates** — each layer wraps the one below: `NwkLayer<M: MacDriver>`
- **CI-enforced** — every push builds all 12 supported firmware targets + clippy + fmt + tests

## Project Structure

```
zigbee-rs/
├── zigbee-types/              # Core types (addresses, channels)
├── zigbee-crypto/             # Shared low-stack AES-CCM* primitives
├── zigbee-mac/                # MAC layer + platform backends
│   └── src/
│       ├── mock/              # Full mock for host testing
│       ├── esp/               # ESP32-C6/H2 (esp-ieee802154)
│       ├── nrf/               # nRF52840/52833 (radio peripheral)
│       ├── bl702/             # BL702 direct Rust radio + legacy FFI
│       ├── cc2340/            # CC2340 Rust host driver + TI radio microcode
│       ├── telink/            # TLSR8258 pure-Rust backend
│       ├── phy6222/           # PHY6222 (pure Rust radio driver!)
│       ├── efr32/             # EFR32MG1 (pure Rust radio driver!)
│       └── efr32s2/           # EFR32MG21 Series 2 (pure Rust radio driver!)
├── zigbee-nwk/                # Network layer (routing, security, router relay)
├── zigbee-aps/                # Application Support (binding, groups)
├── zigbee-zdo/                # Device Objects (discovery, mgmt)
├── zigbee-bdb/                # Base Device Behavior (commissioning)
├── zigbee-zcl/                # Zigbee Cluster Library (45 clusters)
├── zigbee-runtime/            # Device builder, power, NV storage
├── zigbee/                    # Top-level: coordinator, router
├── bl702-hal/                 # BL702 GPIO, buses, ADC, PWM, flash, clocks
├── tlsr8258-hal/              # TLSR8258 radio and reusable peripheral HAL
├── boards/
│   ├── bl702-xt-zb1/          # XT-ZB1 wiring and ownership tokens
│   └── tlsr8258-tb04/         # TB-04 wiring, LEDs, flash and pin ownership
├── products/
│   ├── bl702-xt-zb1/          # XT-ZB1 storage and linker policy
│   ├── efr32mg1-tradfri/      # TRADFRI sensor profile, storage, linker policy
│   ├── esp32-zigbee-devkit/   # ESP32 product profiles and partitions
│   ├── nrf52840-sensor/       # nRF52840 profile, storage, linker policy
│   └── tlsr8258-tb04/         # TB-04 storage and linker policy
├── drivers/                   # Transport-independent sensor drivers (blocking + async)
│   ├── sht3x/                 # Sensirion SHT3x temperature + humidity
│   ├── sht4x/                 # Sensirion SHT4x temperature + humidity
│   ├── scd4x/                 # Sensirion SCD4x CO2 + temperature + humidity
│   ├── bme280/                # Bosch BME280/BMP280 temperature + humidity + pressure
│   ├── bme680/                # Bosch BME680 temperature + humidity + pressure
│   └── bh1750/                # ROHM BH1750 ambient light
├── tests/                     # Integration tests
├── examples/
│   ├── mock-sensor/           # Host: temp+humidity sensor
│   ├── mock-coordinator/      # Host: coordinator
│   ├── mock-light/            # Host: dimmable light
│   ├── mock-sleepy-sensor/    # Host: SED demo
│   ├── esp32c6-sensor/        # ESP32-C6 firmware (NV flash storage, on-chip temp sensor, Identify)
│   ├── esp32h2-sensor/        # ESP32-H2 firmware
│   ├── nrf52840-sensor/       # nRF52840-DK (probe-rs) + BME280/SHT31 + flash NV
│   ├── nrf52840-sensor-uf2/   # nice!nano / ProMicro (UF2 drag-drop, simple demo)
│   ├── nrf52833-sensor/       # nRF52833-DK (probe-rs)
│   ├── nrf52840-router/       # nRF52840 Zigbee router (relay, permit join, Link Status)
│   ├── bl702-sensor/          # XT-ZB1 pure-Rust Zigbee sensor
│   ├── cc2340-sensor/         # TI CC2340R5 (SDK-backed compile, hardware pending)
│   ├── telink-tlsr8258-sensor/# Telink TLSR8258 polling end-device sensor
│   ├── telink-tlsr8258-router/# Telink TLSR8258 parent router
│   ├── phy6222-sensor/        # PHY6222 — pure Rust, no vendor SDK!
│   ├── efr32mg1-sensor/       # EFR32MG1P — pure Rust, Series 1 Cortex-M4F!
│   └── efr32mg21-sensor/      # EFR32MG21 — pure Rust, Series 2 Cortex-M33!
├── docs/
│   ├── book/                  # mdBook source → GitHub Pages
│   └── flasher/               # ESP web flasher (GitHub Pages)
├── tools/
│   └── telink-tlsr8258-lab/  # Telink hardware diagnostics and legacy regression firmware
└── BUILD.md                   # Comprehensive build guide
```

## CI / Firmware Artifacts

Every push builds **12 supported firmware targets** plus workspace checks:

| Job | What it does |
|-----|-------------|
| Check | `cargo check --workspace` |
| Test | `cargo test --workspace` |
| Clippy | `cargo clippy --workspace` |
| Format | `cargo fmt --check` |
| Doc | `cargo doc --workspace --no-deps` |
| Build × 12 | Each supported platform produces a downloadable firmware artifact |
| Deploy | Book + web flasher published to GitHub Pages |

Download firmware artifacts from the [Actions tab](https://github.com/faronov/zigbee-rs/actions).

## Verified Hardware

The following hardware has been tested end-to-end with **Home Assistant + ZHA**:

| Board | Coordinator | Status | Notes |
|-------|-------------|--------|-------|
| **nRF52840-DK** (PCA10056) | ZHA (via zigpy) | ✅ Prior baseline verified | Flash NV, Identify LED blink, BME280/SHT31 optional; current product/profile refactor awaits hardware revalidation |
| **ESP32-C6-DevKitC-1** | ZHA (via zigpy) | ✅ Prior baseline verified | Temperature, Humidity, Battery and flash NV proven; current product/profile refactor and real OTA upgrade await hardware validation |
| **ESP32-H2-DevKitM-1** | ZHA (via zigpy) | ✅ Current refactor verified | Legacy NV migrated atomically to the security journal; reset/resume, reporting, TSENS, interview, and Identify verified on hardware. OTA absent. |
| **BL702 XT-ZB1** | Home Assistant ZHA / Ember | ✅ End-device path verified | Pure-Rust RF calibration, join, TCLK exchange, interview, Identify, and reporting; temperature/humidity currently synthetic |
| **TLSR8258** | Home Assistant ZHA / Ember | ✅ End-device and router paths verified | Join, TCLK exchange, interview, reporting, routed-frame relay, silent reset resume, and crash-safe counter persistence. Parent/child traffic has been exercised, but clean first-attempt sleepy-child commissioning with the corrected child image remains the release gate |
| **EFR32MG1P TRÅDFRI** | Home Assistant ZHA / Ember | ✅ End-device path verified | Pure-Rust radio, SHT3x reporting, crash-safe journal rollover, reset resume, and secure rejoin |

All sensor examples include **Identify cluster** (0x0003), **NWK Leave handling** (auto-erase NV + rejoin), and **default reporting configuration** (so devices report data even before the coordinator sends ConfigureReporting).

## Known Limitations

- **CC2340** has Rust LRFD bring-up and polling raw TX/RX, but no target has
  been connected yet. CCA, IRQ-driven completion, hardware address filtering,
  auto-ACK, temperature compensation, and a real Embassy time driver remain.
- **BL702** now has product-owned crash-safe network/security persistence in
  the final 8 KiB of the XT-ZB1's 1 MiB flash. The integration compiles and is
  covered by host tests, but erase/program and reset-resume still require
  hardware validation. Retention sleep is not implemented. The new
  I2C/SPI/ADC/PWM paths also still require hardware validation.
- **Telink TLSR8258** production examples are split by role. The sensor is a
  polling end device, not yet a retention SED. Timer-only suspend is
  hardware-proven in the HAL, while pad/comparator wake paths and production
  runtime integration remain unvalidated. The router's parent path is
  software-complete and has been exercised on hardware, but release still
  requires a clean first-attempt sleepy-child join and complete ZHA interview
  with the corrected child image under an independent sniffer. Child and route
  tables remain RAM-only across reboot. Real tc32 firmware requires the
  [modern-tc32](https://github.com/modern-tc32) toolchain.
- **PHY6222** pure-Rust driver uses simplified TP calibration defaults — production firmware would need proper PLL lock sequence; temp/humidity sensors are simulated (battery ADC is real); comprehensive power management is implemented (two-tier sleep with AON system sleep ~3 µA, radio sleep/wake, flash deep power-down, GPIO leak prevention)
- **EFR32MG1** is hardware-proven for an always-on end device, but EM2 sleep, wake-time radio restoration, battery ADC, and long-duration stability remain.
- **EFR32MG21** still uses an unverified pure-Rust radio initialization path and needs independent hardware validation.
- **Test coverage** is basic — the mock examples exercise more than the test crate
- **Security** — AES-CCM\* encryption works (RustCrypto `aes` + `ccm`, `no_std`) but key management is minimal
- **OTA** — the transport/session flow is shared in `zigbee-runtime`; EFR32MG1 has a Gecko Bootloader writer and ESP32-C6 has a checked dual-slot (`ota_0`/`ota_1` + `otadata`) writer. Real version-upgrade/reboot validation remains pending. ESP32-H2, nRF52840, and BL702 have no writer.

## Documentation

- **[The zigbee-rs Book](https://faronov.github.io/zigbee-rs/)** — online guide: architecture, platform setup, ZCL clusters, power management, OTA
- **[BUILD.md](BUILD.md)** — detailed instructions for building, flashing, sensor/display integration, debugging, and peripheral wiring
- **[API docs](https://docs.rs/zigbee-rs)** — generated from `cargo doc --workspace`

## License

GPL-2.0 (forked from zigbee-rs)
