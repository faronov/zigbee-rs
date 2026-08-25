# nRF52840 / nRF52833

Nordic's nRF52840 and nRF52833 are ARM Cortex-M4F SoCs with a built-in
IEEE 802.15.4 radio. The zigbee-rs nRF backend uses Embassy's radio driver
for interrupt-driven, DMA-based TX/RX — **no SoftDevice required**.

> **Hardware baseline before the app-layer extraction:** The
> product/profile/`ZigbeeNode`
> sensor was tested end-to-end on an nRF52840-DK revision 2 with
> **Home Assistant + ZHA**, including fresh commissioning, unique-TCLK
> authentication, End Device Timeout, interview, reporting, Identify, durable
> security state, and silent reset/resume. The **nRF52833-DK (PCA10100)** now
> runs the same lifecycle and has been verified
> on hardware for join, interview, secured reporting, durable persistence and
> silent resume — see [nRF52833 hardware acceptance](#nrf52833-hardware-acceptance).
>
> **Hardware-AES parity status:** Nordic ECB AES-128 is hardware-proven on
> both nRF52840 and nRF52833, and is mandatory in all four nRF builds.

## Hardware Overview

| | nRF52840 | nRF52833 |
|---|----------|----------|
| **Core** | ARM Cortex-M4F, 64 MHz | ARM Cortex-M4F, 64 MHz |
| **Flash** | 1024 KB | 512 KB |
| **RAM** | 256 KB | 128 KB |
| **Radio** | BLE 5.3 + 802.15.4 + NFC | BLE 5.3 + 802.15.4 + NFC |
| **Target** | `thumbv7em-none-eabihf` | `thumbv7em-none-eabihf` |

### Hardware Radio Features

- Auto-CRC generation and checking
- Hardware address filtering (PAN ID + short address)
- Auto-ACK for frames with ACK request bit set
- Energy Detection (ED) via EDREQ task
- RSSI measurement per packet
- DMA-driven TX/RX buffers
- Factory-programmed IEEE address in FICR registers

### Common Development Boards

- **nRF52840-DK (PCA10056)** — J-Link debugger, 4 buttons, 4 LEDs
- **nRF52840 USB Dongle (PCA10059)** — USB bootloader, compact form
- **nice!nano v2** — Pro Micro form factor, UF2 bootloader
- **Seeed XIAO nRF52840** — compact, USB-C
- **Makerdiary nRF52840 MDK USB Dongle** — UF2 bootloader
- **nRF52833-DK (PCA10100)** — J-Link debugger, 4 buttons, 4 LEDs

### No SoftDevice Needed

Unlike BLE-only projects, zigbee-rs accesses the 802.15.4 radio peripheral
directly through Embassy's `embassy-nrf` radio driver. There is no dependency
on Nordic's SoftDevice. This gives full control over the radio and avoids the
SoftDevice's RAM/Flash overhead.

> **UF2 variant note:** If your board has a SoftDevice-based UF2 bootloader
> (e.g., nice!nano with Adafruit bootloader), the `nrf52840-sensor-uf2`
> example disables the SoftDevice at startup via an SVC call. See the
> [UF2 section](#uf2-drag-and-drop-flash) below.

## Fail-closed hardware AES-128

The `nrf52840` and `nrf52833` MAC features always select the Nordic ECB
backend (`hardware-aes-nrf`). `NrfEcbToken::take()` provides one process-wide
owner because Embassy 0.3 does not expose ECB in its peripheral singleton.
`NrfMac::install_aes_engine()` consumes that token, runs two back-to-back
known-answer tests with different keys, and only then makes the engine
available to NWK/APS CCM* and AES-MMO.

Each transaction uses the 48-byte, word-aligned EasyDMA structure in Data RAM
(key, cleartext, ciphertext), clears both sticky events, programs
`ECBDATAPTR`, starts ECB, and waits with a finite bound. `ERRORECB`, timeout,
or abort-cleanup failure is returned as a hard error; output is not published
and software AES is never retried. Production examples halt before constructing
the Zigbee device if token acquisition or either startup KAT fails. CI also
rejects an nRF ELF containing `aes::soft::fixslice`/`SoftwareAes128`.

Nordic documents the common nRF52840/nRF52833 ECB layout, Data-RAM-only
EasyDMA requirement, and the fact that ECB shares the AES core with CCM/AAR:

- [nRF52840 ECB product specification](https://docs.nordicsemi.com/r/bundle/ps_nrf52840/page/ecb.html)
- [nRF52833 ECB product specification](https://docs.nordicsemi.com/r/bundle/ps_nrf52833/page/ecb.html)

Physical acceptance completed on an nRF52840-DK revision 2:

- both startup KATs passed on the ECB peripheral;
- a factory-new device joined ZHA, installed the network key, completed the
  unique Trust Center link-key Transport-Key/Verify-Key/Confirm-Key exchange,
  negotiated End Device Timeout, and completed the full interview;
- secured temperature, humidity, and battery reports were accepted;
- a hardware reset restored the same PAN, parent, and short address without
  scanning, association, or Device Announce;
- a secured Identify command and its response succeeded after that silent
  resume.

The accepted nRF52840 sensor binary is 200,840 bytes and contains no
`SoftwareAes128` or `aes::soft::fixslice` symbols.

The hardware run also exposed and fixed a missing
`MacAssociatedPanCoord` PIB implementation in the Nordic MAC backend.

### nRF52833 hardware acceptance

The nRF52833-DK (PCA10100, J-Link serial `001050692138`) was accepted with
the same firmware application, on the same ZHA network:

- both ECB startup KATs passed (`Nordic ECB hardware AES KAT passed`);
- the EUI-64 came from FICR `DEVICEID` (`15:46:65:AD:B8:B3:99:97`), not a
  constant, and ZHA registered the little-endian counterpart
  `97:99:b3:b8:ad:65:46:15`;
- the device commissioned **automatically at boot** — no button press — and
  joined PAN `0xDFE9` on channel 15 as short address `0xF86B` through parent
  `0x1F0F`;
- ZHA completed the interview and read the product identity
  (`Zigbee-RS` / `nRF52833-Sensor`), endpoint 1 = `0x0302`
  Temperature Sensor with input clusters `0x0000, 0x0001, 0x0003, 0x0402,
  0x0405`, node descriptor `logical_type = 2` (End Device);
- the device independently observed coordinator-originated
  `ConfigureReporting` for Power Configuration, Temperature, and Humidity,
  then logged `Interview configuration complete: 3/3 clusters`;
- NWK-secured Temperature (`0x0402`) and Humidity (`0x0405`) reports were
  MAC-ACKed and accepted; ZHA created Identify, Battery, Temperature, and
  Humidity entities;
- a short Button 1 press forced a fresh sensor/battery sample and immediate
  reports for all three configured clusters; Home Assistant updated to
  23.0 °C / 52.3 % / 100 %;
- a ZDO `NWK_ADDR_REQ` from a neighbour was answered (`ZDO OK`);
- after a **reflash and reset**, the device resumed silently from the
  crash-safe journal at `0x0007_E000` — same PAN, channel, parent, and short
  address, with no scan, association, or Device Announce. `probe-rs` rewrites
  only the 504 KiB application region, so this also demonstrates that the
  protected partition survives a firmware update.

Not yet exercised on nRF52833 silicon: the 3 s long-press durable factory
reset, battery-percentage reporting over its full reporting interval, and
long-duration low-power current measurement.

### Shared sleepy-sensor application

`apps/sensor-sed` holds the platform-independent lifecycle.
`apps/nrf-sensor` supplies the Nordic GPIO/time/reset, radio-sleep, SAADC,
on-chip-temperature, and `defmt` adapters. The nRF52840 root composes those
capabilities directly; nRF52833 temporarily retains a source-compatible
adapter wrapper over the same lifecycle:

| Concern | Owner |
|---------|-------|
| Commissioning, resume, retry, polling, interview, reporting, button semantics | `apps/sensor-sed` |
| Nordic GPIO/time/reset, radio sleep, SAADC, on-chip TEMP, diagnostics | `apps/nrf-sensor` |
| Identity strings, flash layout, security partition, battery curve, profile | `products/nrf5283x-sensor` |
| LED1/Button 1/I2C pins | `boards/nrf5283x-dk` |
| Radio, RNG, SAADC, TEMP, NVMC, clocks | `embassy-nrf` |

The only intentional runtime difference between the two products is the
DC/DC configuration: `embassy-nrf` exposes the VDDH→VDD first stage (`reg0`)
only for nRF52840, so the nRF52833 composition root enables the second stage
(`reg1`) alone. On a PCA10100 running in normal-voltage mode (VDDH tied to
VDD) REG0 is bypassed anyway, so this is a HAL/API difference rather than a
behavioral one.

## Prerequisites

### Rust Toolchain

```bash
rustup default nightly
rustup update nightly

# Add the ARM Cortex-M4F target
rustup target add thumbv7em-none-eabihf
```

### Debug Probe (for DK boards)

```bash
# probe-rs handles flashing + defmt log viewing
cargo install probe-rs-tools
```

Supported probes:
- On-board J-Link (nRF52840-DK, nRF52833-DK)
- Any CMSIS-DAP probe
- Segger J-Link (external)

### For UF2 boards (no probe needed)

```bash
pip install intelhex   # for uf2conv.py
```

## Building

### nRF52840-DK (probe-rs)

```bash
cd examples/nrf52840-sensor
cargo build --release
```

### nRF52833-DK (probe-rs)

```bash
cd examples/nrf52833-sensor
cargo build --release
```

### nRF52840 UF2 (nice!nano / ProMicro / MDK Dongle)

```bash
cd examples/nrf52840-sensor-uf2
cargo build --release                                    # ProMicro (default)
cargo build --release --no-default-features --features board-mdk         # MDK Dongle
cargo build --release --no-default-features --features board-nrf-dongle  # PCA10059
cargo build --release --no-default-features --features board-nrf-dk      # DK (J-Link)
```

### nRF52840 Router

```bash
cd examples/nrf52840-router
cargo build --release
```

### nRF52840 Router

```bash
cd examples/nrf52840-router
cargo build --release
```

### What `.cargo/config.toml` Sets

```toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip nRF52840_xxAA"

[env]
DEFMT_LOG = "info"
```

### CI Build Commands

From `.github/workflows/ci.yml`:

```bash
# nRF52840 sensor
cd examples/nrf52840-sensor
cargo build --release

# nRF52840 router
cd examples/nrf52840-router
cargo build --release

# nRF52833 sensor
cd examples/nrf52833-sensor
cargo build --release

# UF2 variant (includes .uf2 conversion)
cd examples/nrf52840-sensor-uf2
cargo build --release

# Firmware artifact extraction
OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
$OBJCOPY -O binary $ELF ${ELF}.bin
$OBJCOPY -O ihex   $ELF ${ELF}.hex

# UF2 conversion (CI uses uf2conv.py from Microsoft's UF2 repo)
python uf2conv.py -c -f 0xADA52840 ${ELF}.hex -o ${ELF}.uf2
```

### Memory Layout

The `memory.x` linker script defines the memory regions:

**nRF52840 sensor product** (no bootloader; last 8 KiB protected for Zigbee
security persistence):
```
FLASH : ORIGIN = 0x00000000, LENGTH = 1016K
RAM   : ORIGIN = 0x20000000, LENGTH = 256K
```

`products/nrf52840-sensor/link/memory.x` owns this layout. The protected
`0x000FE000..0x000FFFFF` region contains the two-sector crash-safe
`SecurityStateJournal`; the board crate owns no flash addresses or persistence
policy.

**nRF52833 sensor product** (no bootloader; last 8 KiB protected for Zigbee
security persistence, exactly mirroring the nRF52840 product):
```
FLASH : ORIGIN = 0x00000000, LENGTH = 504K
RAM   : ORIGIN = 0x20000000, LENGTH = 128K
```

`products/nrf52833-sensor/link/memory.x` owns this layout. The protected
`0x0007E000..0x0007FFFF` region contains the two-sector crash-safe
`SecurityStateJournal`. Both nRF `memory.x` files carry link-time `ASSERT`s
that fail the build if the application region is ever grown over the journal
partition, or if the region stops matching the part's real memories; the
matching `const` assertions in each product's `storage.rs` check the same
partition from the Rust side.

**nRF52840 UF2 (with SoftDevice S140 bootloader)**:
```
FLASH : ORIGIN = 0x00026000, LENGTH = 808K    ← app starts after SoftDevice
RAM   : ORIGIN = 0x20002000, LENGTH = 248K
```

The UF2 example's `build.rs` selects the memory layout based on the board feature.

## Flashing

### probe-rs (DK boards)

```bash
cd examples/nrf52840-sensor

# Flash + live defmt log output
cargo run --release

# Or flash only
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/nrf52840-sensor
```

> **Tip:** Plug in the DK before running `cargo run`. probe-rs auto-detects
> the probe. Check with `probe-rs list` if detection fails.

### UF2 Drag-and-Drop Flash

For boards with UF2 bootloaders (nice!nano, ProMicro, MDK Dongle):

1. **Build the firmware:**
   ```bash
   cd examples/nrf52840-sensor-uf2
   cargo build --release
   ```

2. **Convert to UF2:**
   ```bash
   # Extract binary
   OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
   $OBJCOPY -O ihex target/thumbv7em-none-eabihf/release/nrf52840-sensor-uf2 fw.hex

   # Convert to UF2 (download uf2conv.py from Microsoft's UF2 repo)
   python uf2conv.py -c -f 0xADA52840 fw.hex -o fw.uf2
   ```

3. **Enter bootloader mode:** Double-tap the RESET button on the board.
   A USB mass storage device appears (e.g., `NICENANO`).

4. **Copy the `.uf2` file** to the USB drive. The board flashes automatically
   and reboots into your firmware.

### J-Link Commander (alternative)

```bash
nrfjprog --program target/thumbv7em-none-eabihf/release/nrf52840-sensor.hex --chiperase --verify
nrfjprog --reset
```

## MAC Backend Notes

The nRF MAC backend lives in `zigbee-mac/src/nrf/mod.rs` (single file — no
separate driver module needed since Embassy provides the radio abstraction).

### Feature Flags

| Feature | Chip | Cargo.toml dependency |
|---------|------|----------------------|
| `nrf52840` | nRF52840 | `zigbee-mac = { features = ["nrf52840"] }` |
| `nrf52833` | nRF52833 | `zigbee-mac = { features = ["nrf52833"] }` |

### Key Dependencies

```toml
embassy-nrf = { version = "0.3", features = ["nrf52840", "time-driver-rtc1", "gpiote"] }
embassy-executor = { version = "0.7", features = ["arch-cortex-m", "executor-thread"] }
```

### How It Works

1. **`NrfMac<T: Instance>`** wraps Embassy's `Radio<T>` and implements `MacDriver`
2. Radio TX/RX is fully interrupt-driven with DMA — no polling needed
3. Hardware auto-ACK is enabled for frames with the ACK request bit
4. Hardware address filtering is configured through the radio peripheral
5. The factory-programmed IEEE address is read from FICR registers
6. Embassy's `time-driver-rtc1` provides async timers via RTC1

### Embassy Integration

The nRF examples use Embassy's cooperative async executor:

```rust
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TEMP => embassy_nrf::temp::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mac = zigbee_mac::nrf::NrfMac::new(radio);
    // ...
}
```

The `select3` combinator handles concurrent events:

```rust
match select3(
    device.receive(),                                    // Radio RX
    button.wait_for_falling_edge(),                      // Button press
    Timer::after(Duration::from_secs(REPORT_INTERVAL)),  // Periodic report
).await {
    Either3::First(event)  => { /* handle stack event */ }
    Either3::Second(_)     => { /* handle button press */ }
    Either3::Third(_)      => { /* read sensor, update clusters */ }
}
```

## Power Optimization

### Sensor (End Device)

The nRF52840 sensor example includes several hardware-level power optimizations
that bring the average current draw down to ~5 µA. See the
[Power Management](../advanced/power.md) chapter for full details.

### DC-DC Converter

The nRF52840's internal DC-DC converter replaces the default LDO regulators,
reducing current draw by ~40%. Both `reg0` (main supply) and `reg1` (radio
supply) are enabled at startup:

```rust
config.dcdc = embassy_nrf::config::DcdcConfig {
    reg0: true,
    reg0_voltage: None,
    reg1: true,
};
```

### TX Power

TX power is set to 0 dBm (down from the default +8 dBm), which cuts TX
current roughly in half while maintaining adequate range for home environments:

```rust
mac.set_tx_power(0); // 0 dBm — saves ~50% TX current vs +8 dBm
```

### HFCLK Source

The high-frequency clock source is set to the internal RC oscillator. The
radio peripheral automatically requests the external crystal when it needs
high accuracy (during TX/RX), saving ~250 µA during idle periods:

```rust
config.hfclk_source = embassy_nrf::config::HfclkSource::Internal;
```

### Poll and Report Intervals

The sensor uses a two-phase polling scheme:

| Phase | Poll Interval | Duration | Current |
|-------|--------------|----------|---------|
| Fast poll | 250 ms | 120 s after join/activity | Higher (responsive) |
| Slow poll | 30 s | Steady state | Very low (~5 µA avg) |

Reports are sent every 60 seconds, but only when sensor values change by more
than the configured thresholds (±0.5 °C temperature, ±1% humidity, ±2%
battery). This suppresses unnecessary transmissions in stable environments.

### RAM Power-Down

The example contains a `power_down_unused_ram()` primitive, but it is not
currently called. Enabling it requires hardware validation of the linked BSS,
Embassy task arena, and stack headroom first; do not count RAM-bank power-down
as an active optimization in the current firmware.

### Radio Sleep

Between polls, the radio is disabled via `TASKS_DISABLE` register write,
saving ~4-8 mA of radio RX/idle current. The `radio_wake()` method re-applies
the channel setting and re-enables the radio before the next TX/RX operation.

### Router (Always-On)

The nRF52840 router uses `PowerMode::AlwaysOn` — the radio is always on since
routers must relay frames continuously. DC-DC converters are still enabled for
lower power, but no sleep logic is applied. Typical current draw with DC-DC
enabled is ~5-7 mA (radio RX idle).

---

## Example Walkthrough

### nrf52840-sensor

The flagship example: an Embassy-based Zigbee 3.0 end device that reads the
on-chip temperature sensor and reports simulated humidity. Includes:

- **Crash-safe security journal** — product-owned last 8 KiB partition with
  CRC/commit protection and outgoing-frame-counter reservation
- **NWK Leave handler** — auto-erases NV and rejoins when coordinator sends Leave
- **Default reporting** — configures report intervals at boot (temp/hum: 60–300 s, battery: 300–3600 s)
- **Identify cluster** (0x0003) — LED blinks during Identify
- **Battery monitoring** via SAADC (VDD internal divider)
- **Optional external sensors** — shared `zigbee-bme280` (temp + humidity +
  pressure) or `zigbee-sht3x` (temp + humidity) drivers
- **Typed product profile + `ZigbeeNode`** — endpoint composition,
  reporting, persistence lifecycle, tick, and receive dispatch are shared
  rather than rebuilt in the example

#### Architecture: composition root + shared application state machine

Like the EFR32MG1 and ESP32-H2 sensors, this firmware separates platform
startup from the lifecycle — but unlike them, the lifecycle is **not** part
of the example. It lives in `apps/sensor-sed`; Nordic-only capability
implementations live in `apps/nrf-sensor`:

| File | Owns |
|------|------|
| `examples/nrf5283x-sensor/src/main.rs` | Embassy/Nordic platform startup (clocks, DC-DC, boot signal), board/sensor resource construction, hardware AES install + startup KAT, the crash-safe security journal, the concrete product profile, the battery-policy binding, and the identity guard. Builds `device`/`security_store`/`profile` as plain locals and hands borrows of them to `SensorApp`. |
| `examples/nrf5283x-sensor/src/sensor.rs` | Optional external BME280/SHT31 I2C source (board-typed, therefore per-example). |
| `apps/sensor-sed/src/app.rs` | Generic `SensorApp`, the full commissioning and event-loop lifecycle: bounded MAC receive/poll windows, fast/slow polling, interview window, Device_annce retries, button handling, and durable checkpointing. |
| `apps/sensor-sed/src/{capabilities,parts,ota,battery,environment,diagnostics}.rs` | Atomic MAC quiesce/wait/readiness, semantic status, explicit resource ownership, OTA/profile pairing, fallible measurements, profile updates, supervision, and diagnostics. |
| `apps/sensor-sed/src/policy.rs` | Product-selected `u32` timing policy plus host-tested deadline arbitration. |
| `apps/nrf-sensor/src/{platform,battery,environment,diagnostics}.rs` | `embassy-nrf` and product-policy adapters; no Zigbee lifecycle state machine. |

Interview detection is **not** application state. The app reacts to
`StackEvent::ReportingConfigured` — emitted by `zigbee-runtime` only for a
non-empty Configure Reporting command made entirely of Send-direction records
whose every record succeeded — and reads the `n/n` progress straight from
`ZigbeeNode::remote_reporting_cluster_count()` /
`remote_reporting_is_complete()`. Those node APIs filter the runtime's generic
record through the active profile's exact expected cluster IDs, so an
unrelated cluster cannot substitute for a missing sensor cluster. A rejected,
receive-only, or mixed-direction command arrives as the generic
`CommandReceived` and is logged as non-progress rather than counted.

`main.rs` stays a thin composition root; `SensorApp::run()` is the only place
that drives the network, for every nRF product.

The public application type is `SensorApp<'a, M, S, P, R>`: MAC, security
store, complete application profile, and one explicit resource bundle.
`SensorSedParts<W, St, E, B, O, A, Sv, D>` retains concrete ownership of the
wake controller, status, sensors, OTA lifecycle, user action, supervisor, and
diagnostics capabilities. Nothing is hidden behind dynamic dispatch; all
parts are monomorphized.

Unlike the EFR32MG1/ESP32-H2 sensors, `SensorApp` is lifetime-generic
and **not** built via `StaticCell`/`build_into`. Those two products use the
`embassy-executor` crate's *default* 4 KiB task arena (they set no
`task-arena-size-*` feature), so `ZigbeeDevice`/security-store/profile must
be pulled out into `'static` storage just to fit the future that big at
all. This firmware explicitly requests a much larger arena
(`task-arena-size-32768`, see `Cargo.toml`) sized for the whole
single-future firmware, and that arena is a **fixed-size static
reservation regardless of what's stored inside it** — so adding
`StaticCell`s on top would only add a second, unnecessary reservation
rather than shrink anything. Measured effect: building with plain locals
(borrowed into `SensorApp<'a>` for the remainder of the never-returning
`main()`) keeps `.bss` byte-for-byte identical to the pre-refactor
single-file firmware; an earlier `StaticCell`-based version of this same
refactor measured ~11.6 KiB of *additional* `.bss` for zero benefit, which
is why this file does not use that pattern. See
`examples/nrf52840-sensor/src/main.rs` for the full comment.

#### Event handling

`SensorApp::handle_control_event` matches every [`StackEvent`] variant
explicitly — no wildcard arm, so adding a new variant to `zigbee-runtime`
fails to compile here until it is deliberately handled (OTA variants, which
this product does not use, still get an explicit logged arm rather than
being silently dropped). In particular:

- `BasicResetToFactoryDefaults` is treated as the Basic cluster Reset to Factory
  Defaults operation: writable Basic attributes are reset, while network
  state, security counters, bindings, and groups are preserved as required
  by BDB. It is not conflated with a local installer factory-new action or
  an NWK Leave.
- `RejoinRequested` attempts a secure rejoin immediately.
- A `CommissioningComplete { success: false }` while a secure rejoin is
  pending is retried, bounded by a small failure counter, before falling
  back to a full factory reset and fresh join — a stale/unreachable parent
  can no longer wedge the device indefinitely.

`SensorApp` also honors [`TickResult::RunAgain`]: when the runtime asks to
be ticked again sooner than the current fast/slow poll window (for example
mid-Trust-Center-link-key exchange), the next poll/sleep wait is shortened
accordingly instead of the request being discarded. Runtime elapsed seconds
come from a separate monotonic `last_tick` clock rather than the cumulative
age of the last sensor report, so extra `RunAgain` wakeups cannot advance
reporting, Identify, NWK, or End Device Timeout timers faster than wall clock.
A failed restored `rejoin_pending` attempt during boot is counted as the first
bounded secure-rejoin failure rather than being lost before periodic retries
begin.

**Initialization (`main.rs`):**

```rust
let p = embassy_nrf::init(config);

// Board-owned physical wiring.
let mut led = nrf52840_dk::led(p.P0_13);
let button = nrf52840_dk::button(p.P0_11);

// IEEE 802.15.4 MAC driver (interrupt-driven, DMA-based) + hardware AES.
let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
let rng = rng::Rng::new(p.RNG, Irqs);
let mut mac = zigbee_mac::nrf::NrfMac::new(radio, rng);
mac.install_aes_engine(aes).expect("Nordic ECB AES startup KAT");

// Product-owned concrete profile and device — plain locals (see above).
let mut profile = nrf52840_sensor_product::profile::sensor_profile();
let mut device = ZigbeeDevice::builder(mac)
    .power_mode(nrf52840_sensor_product::policy::SENSOR_POLICY.power_mode())
    .automatic_polling(false)
    // identity and endpoint come from nrf52840-sensor-product
    .build();

// Product-owned persistence — same journal partition as before.
let nvmc = embassy_nrf::nvmc::Nvmc::new(p.NVMC);
let mut security_store = nrf52840_sensor_product::storage::security_store(nvmc);

let node = ZigbeeNode::new(&mut device, &mut security_store, &mut profile);
let mut app = SensorApp::new(
    node,
    &nrf52840_sensor_product::policy::SENSOR_POLICY,
    sensor_sed_app::SensorSedParts {
        wake: nrf_sensor_app::NrfWakeController::new(button),
        status: nrf_sensor_app::NrfStatus::new(led),
        environment,
        battery: nrf_sensor_app::NrfBattery::<Battery>::new(saadc),
        ota: sensor_sed_app::NoOta,
        actions: nrf52840_sensor_product::policy::USER_ACTIONS,
        supervisor: nrf_sensor_app::NrfSupervisor,
        diagnostics: nrf_sensor_app::NrfDiagnostics,
    },
)
.expect("manual SensorApp polling requires automatic_polling(false)");
app.run().await
```

**Real temperature reading (`apps/nrf-sensor/src/environment.rs`):**

```rust
// Read actual die temperature (°C with 0.25° resolution)
let raw_temp = self.temp.read().await;
let temperature_centi_celsius = (raw_temp.to_bits() * 100 / 4) as i16;
```

### nrf52840-sensor-uf2

The UF2 variant supports multiple boards via cargo features:

| Feature | Board | LED | Flash Origin |
|---------|-------|-----|-------------|
| `board-promicro` | ProMicro / nice!nano | P0.15 (HIGH) | 0x26000 |
| `board-mdk` | Makerdiary MDK Dongle | P0.22 (LOW) | 0x1000 |
| `board-nrf-dongle` | Nordic PCA10059 | P0.06 (LOW) | 0x1000 |
| `board-nrf-dk` | Nordic DK (PCA10056) | P0.13 (LOW) | 0x0000 |

This variant auto-joins on boot (no button press needed) and includes a
`log` → `defmt` bridge so internal stack log messages appear in RTT output.

### nrf52840-router

A Zigbee 3.0 router that extends network range. Key differences from the
sensor examples:

- **Device type:** Router (FFD) instead of End Device
- **Power mode:** `AlwaysOn` — radio is never turned off
- **Frame relay:** Relays unicast, broadcast, and indirect frames
- **Child management:** Accepts end device joins, buffers frames for sleepy children
- **Link Status:** Sends periodic broadcasts (every 15 seconds)
- **RREQ rebroadcast:** Participates in AODV route discovery
- **LEDs:** LED1 = joined status, LED2 = blink on frame relay

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `probe-rs` can't find device | Probe not connected | Check USB; run `probe-rs list` |
| `probe-rs` permission denied | Missing udev rules (Linux) | See [probe-rs setup](https://probe.rs/docs/getting-started/probe-setup/) |
| `292` / RAM overflow | Too many features enabled | Check Embassy feature flags, reduce arena size |
| defmt output garbled | Version mismatch | Ensure `defmt`, `defmt-rtt`, `panic-probe` versions match |
| UF2 board not appearing | Not in bootloader | Double-tap RESET quickly; look for USB drive |
| Device doesn't join | Coordinator not permitting | Enable permit-join on coordinator |
| No temperature reading | TEMP interrupt not bound | Ensure `bind_interrupts!` includes TEMP handler |

### Adjusting Log Level

```bash
# Via environment variable
DEFMT_LOG=trace cargo run --release

# Or set in .cargo/config.toml
[env]
DEFMT_LOG = "debug"
```

### Expected Serial Output (via RTT)

```
INFO  Zigbee-RS nRF52840 sensor starting…
INFO  Nordic ECB hardware AES KAT passed
INFO  Radio ready (TX 0 dBm)
INFO  Security journal ready
INFO  Joined/resumed network: addr=0x1234 ch=15 pan=0x1AAA
INFO  Default reporting configured
INFO  Remote ConfigureReporting: cluster=0x0402 3/3 clusters
INFO  Interview configuration complete: 3/3 clusters
INFO  Button → force report (interview configuration 3/3)
INFO  T=23.75°C H=52.30%
INFO  Battery: 3000mV (100%)
```
