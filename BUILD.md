# BUILD.md — zigbee-rs Build, Test, Flash & Integration Guide

A practical guide for building, testing, flashing, and extending the
**zigbee-rs** Zigbee PRO R22 stack — a `#![no_std]` Rust implementation
targeting embedded devices and host-based simulation.

---

## 1. Prerequisites

### Rust Toolchain

zigbee-rs requires **Rust nightly** (edition 2024, async fn in traits,
`no_std` GAT features):

```bash
rustup default nightly
rustup update nightly
```

### Platform-Specific Setup

| Target Platform | Rust Target | Extra Tools |
|-----------------|-------------|-------------|
| **Mock (host)** | Your default host target | None |
| **nRF52840** | `thumbv7em-none-eabihf` | `probe-rs` |
| **ESP32-C6** | `riscv32imac-unknown-none-elf` | `espflash` |
| **ESP32-H2** | `riscv32imac-unknown-none-elf` | `espflash` |
| **ESP32-C5** | `riscv32imac-unknown-none-elf` | `espflash` |
| **BL702** | `riscv32imc-unknown-none-elf` | `bflb-mcu-tool` |

#### nRF52840

```bash
rustup target add thumbv7em-none-eabihf
cargo install probe-rs-tools          # flash + defmt RTT viewer
```

#### ESP32-C6 / H2 / C5

```bash
rustup target add riscv32imac-unknown-none-elf
cargo install espflash                # flash + serial monitor
```

#### Mock (Host)

No additional setup — uses the regular host target for development and testing.

#### BL702 (Bouffalo Lab)

```bash
rustup target add riscv32imc-unknown-none-elf
rustup component add llvm-tools-preview
python3 -m pip install bflb-mcu-tool==1.10.0 pyserial
```

The XT-ZB1 sensor uses the direct-register Rust radio and links no Bouffalo
archive. RF calibration, joining, ZHA interview, Trust Center link-key
exchange, and attribute reporting are hardware-tested:

```bash
cd examples/bl702-sensor
./build-image.sh
```

The script asks the official tool to retain the 32 MHz XTAL clock, then emits
`bl702-sensor.bin` and the bootable `bl702-sensor.flash.bin`. The application
currently reports synthetic sensor values and does not persist network state.

---

## 2. Project Structure

The workspace contains **9 stack crates** layered bottom-up, plus examples
and tests:

```
zigbee-rs-fork/
├── Cargo.toml              # workspace root (resolver = "2", edition 2024)
│
├── zigbee-types/           # Core types: addresses, channels, PAN IDs
├── zigbee-mac/             # IEEE 802.15.4 MAC trait + platform backends
│   └── src/
│       ├── mock/           #   MockMac — host testing (feature "mock")
│       ├── esp/            #   ESP32-C6/H2/C5 radio (feature "esp32c6")
│       ├── nrf/            #   nRF52840 radio (feature "nrf52840")
│       ├── stm32wb/        #   STM32WB55 coprocessor (feature "stm32wb55")
│       ├── efr32/          #   EFR32MG24 RAIL (feature "efr32mg24")
│       └── cc26xx/         #   TI CC2652 (feature "cc2652")
├── zigbee-nwk/             # NWK layer: formation, join, routing, security
├── zigbee-aps/             # APS layer: binding, groups, key management
├── zigbee-zdo/             # ZDO/ZDP: device object & device profile
├── zigbee-bdb/             # BDB: Base Device Behavior commissioning (v3.0.1)
├── zigbee-zcl/             # ZCL: clusters, attributes, commands, reporting
├── zigbee-runtime/         # Device runtime: event loop, builder, NV storage
├── zigbee/                 # Façade crate — re-exports the full stack
│
├── examples/
│   └── nrf52840-sensor/    # Embassy-based sensor (BME280/SHT31 I2C, on-chip TEMP)
│
└── tests/
    └── src/
        ├── mac_tests.rs          # MockMac scan, associate, data, PIB
        ├── nwk_tests.rs          # NWK frames, neighbor/routing tables, security
        ├── types_tests.rs        # Address, PAN, channel primitives
        ├── runtime_tests.rs      # DeviceBuilder, templates, NV, power mgmt
        └── integration_tests.rs  # Trust center, coordinator, router flows
```

### MAC Backend Feature Matrix

| Feature Flag | Backend | Status |
|-------------|---------|--------|
| `mock` | Host simulation | ✅ Stable — development & CI |
| `nrf52840` | Nordic nRF52840 via Embassy | ✅ Stable |
| `esp32c6` | ESP32-C6 radio via `esp-radio` | ✅ Stable (also H2, C5) |
| `stm32wb55` | STM32WB55 IPCC coprocessor | 🔧 In progress |
| `efr32mg24` | Silicon Labs RAIL | 🔧 Placeholder |
| `cc2652` | TI CC26x2 | 🔧 Placeholder |

---

## 3. Building

### Mock (Host) — Development & Testing

The mock backend uses the `mock` feature on `zigbee-mac` and runs on the
host machine with no hardware. Useful for stack development, unit tests,
and integration testing.

```bash
# Build workspace crates with mock backend
cargo build -p zigbee-mac --features mock
cargo build -p zigbee --features mock

# Run tests (see §4 for details)
cargo test --workspace
```

To create a mock-based example binary, add a crate under `examples/` (or
as a workspace member) that depends on `zigbee-mac` with `features = ["mock"]`.

### nRF52840

The nRF52840 example is a standalone Cargo project (not a workspace member)
that references the stack crates by path:

```bash
cd examples/nrf52840-sensor

# Build (target is set in .cargo/config.toml → thumbv7em-none-eabihf)
cargo build --release

# Flash + live defmt log output via probe-rs
cargo run --release
```

The `.cargo/config.toml` configures:

```toml
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip nRF52840_xxAA"

[env]
DEFMT_LOG = "info"
```

> **Tip:** Plug in the nRF52840-DK (or any J-Link/CMSIS-DAP probe) before
> running `cargo run`. probe-rs auto-detects the probe.

### ESP32-C6 / ESP32-H2 / ESP32-C5

To create an ESP32-based example, set up a crate like:

```bash
mkdir -p examples/esp32c6-sensor/src examples/esp32c6-sensor/.cargo
```

**`Cargo.toml`** — depend on `zigbee-mac` with the `esp32c6` feature:

```toml
[package]
name = "esp32c6-sensor"
version = "0.1.0"
edition = "2021"

[dependencies]
zigbee-mac = { path = "../../zigbee-mac", features = ["esp32c6"] }
zigbee-zcl = { path = "../../zigbee-zcl" }
zigbee-runtime = { path = "../../zigbee-runtime" }
esp-hal = { version = "1.0.0", features = ["rt", "unstable", "esp32c6"] }
esp-println = { version = "0.14", features = ["esp32c6"] }
```

**`.cargo/config.toml`**:

```toml
[build]
target = "riscv32imac-unknown-none-elf"

[target.riscv32imac-unknown-none-elf]
runner = "espflash flash --monitor"
```

Then build and flash:

```bash
cd examples/esp32c6-sensor
cargo build --release
cargo run --release      # espflash: flash + serial monitor
```

**Switching chips:** Replace all `esp32c6` occurrences with `esp32h2` or
`esp32c5` in both `Cargo.toml` feature flags and `.cargo/config.toml`.
The MAC driver code is shared across all three chips — only the HAL
feature gate changes.

---

## 4. Testing

### Unit Tests (Host)

The workspace crates include `#[cfg(test)]` modules. Run everything:

```bash
cargo test --workspace
```

Run a specific crate's tests:

```bash
cargo test -p zigbee-types
cargo test -p zigbee-mac --features mock
cargo test -p zigbee-nwk
cargo test -p zigbee-aps
cargo test -p zigbee-zcl
```

### Integration Tests with MockMac

The `tests/src/` directory contains integration tests that exercise the
full stack using the `MockMac` backend. MockMac lets you:

1. **Pre-configure responses** — set up expected scan results, association
   responses, and data frames before the test runs.
2. **Exercise the full stack** — call the stack's join, data, and ZCL
   operations as you would on real hardware.
3. **Verify TX history** — inspect every frame the stack transmitted,
   assert on frame types, destinations, and payloads.
4. **Check cluster attribute values** — verify that ZCL attributes were
   updated correctly after processing incoming commands.

Example test flow:

```rust
#[tokio::test]
async fn test_device_joins_network() {
    // 1. Create a MockMac and pre-load scan/association responses
    let mut mac = MockMac::new();
    mac.expect_scan(ScanResult { channel: 15, pan_id: 0x1AAA, .. });
    mac.expect_associate(AssociateResponse { addr: 0x1234, status: Success });

    // 2. Build a device using the mock backend
    let device = DeviceBuilder::new()
        .mac(mac)
        .device_type(EndDevice)
        .build();

    // 3. Run commissioning
    device.join().await.unwrap();

    // 4. Verify the stack transmitted the expected frames
    let history = device.mac().tx_history();
    assert!(history.iter().any(|f| f.is_beacon_request()));
    assert!(history.iter().any(|f| f.is_association_request()));
}
```

Test files and what they cover:

| File | Coverage |
|------|----------|
| `mac_tests.rs` | MockMac scans, association, data TX/RX, reset, PIB get/set |
| `nwk_tests.rs` | NWK frame serialization, neighbor/routing tables, security headers, key management, replay protection |
| `types_tests.rs` | Addresses, PAN IDs, channels, channel masks |
| `runtime_tests.rs` | DeviceBuilder, device templates, NV storage, power management |
| `integration_tests.rs` | Trust center, coordinator formation, router join flows |

### On Real Hardware

To test on physical hardware with a Zigbee network:

1. **Set up a coordinator** — use any Zigbee coordinator:
   - ZZH/Sonoff stick + [Zigbee2MQTT](https://www.zigbee2mqtt.io/)
   - deCONZ with ConBee II
   - ZHA in Home Assistant

2. **Enable permit joining** on the coordinator (usually via the
   coordinator's web UI or CLI).

3. **Flash the device:**
   ```bash
   # nRF52840
   cd examples/nrf52840-sensor && cargo run --release

   # ESP32-C6
   cd examples/esp32c6-sensor && cargo run --release
   ```

4. **Monitor serial output** — watch for:
   - `Scanning channels...`
   - `Found network on channel XX, PAN 0xYYYY`
   - `Association successful, short addr = 0xZZZZ`
   - `Joined network`

5. **Verify on the coordinator** — the device should appear in:
   - Z2M: Devices tab → new device with interview data
   - deCONZ: Phoscon → Devices list
   - ZHA: Devices page → Zigbee info

6. **Read attributes** — use the coordinator's frontend to read ZCL
   cluster values (temperature, humidity, etc.) from the device.

---

## 5. Peripheral Integration

### I2C Sensors (embedded-hal Compatible)

All sensor drivers use the **`embedded-hal` 1.0** traits. The same driver
code works on any platform (nRF52840, ESP32, STM32, etc.) — only the I2C
bus initialization differs.

| Sensor | Crate | Measures | I2C Addr | ZCL Clusters |
|--------|-------|----------|----------|--------------|
| SHT31 | `zigbee-sht3x` (`drivers/sht3x`) | Temp + Humidity | 0x44 / 0x45 | 0x0402 + 0x0405 |
| SHT40 | `sht4x` | Temp + Humidity | 0x44 | 0x0402 + 0x0405 |
| BME280 | `zigbee-bme280` (`drivers/bme280`) | Temp + Humidity + Pressure | 0x76 / 0x77 | 0x0402 + 0x0405 + 0x0403 |
| BMP280 | `bmp280-rs` | Temp + Pressure | 0x76 / 0x77 | 0x0402 + 0x0403 |
| HDC1080 | `hdc1080` | Temp + Humidity | 0x40 | 0x0402 + 0x0405 |
| SHTC3 | `shtcx` | Temp + Humidity | 0x70 | 0x0402 + 0x0405 |
| LPS22HB | `lps22hb` | Pressure | 0x5C / 0x5D | 0x0403 |
| TSL2561 | `tsl256x` | Illuminance | 0x29 / 0x39 / 0x49 | 0x0400 |
| VEML7700 | `veml7700` | Illuminance | 0x10 | 0x0400 |
| MAX44009 | `max44009` | Illuminance | 0x4A / 0x4B | 0x0400 |
| SCD40 | `scd4x` | CO₂ + Temp + Humidity | 0x62 | Custom + 0x0402 + 0x0405 |

> The nRF52840-sensor example wires the shared, workspace-level
> `zigbee-bme280` / `zigbee-sht3x` async drivers (`drivers/bme280`,
> `drivers/sht3x`) over the board's TWISPI0 bus — enable via
> `--features sensor-bme280` or `--features sensor-sht31`. See
> `examples/nrf52840-sensor/src/sensor.rs` for the recoverable probe/read
> wiring and `examples/nrf52840-sensor/README.md` for pin wiring and build
> instructions. Both drivers are fully async (embassy TWIM DMA) and don't
> block the radio.

### Pattern: Reading Sensor → Updating ZCL Cluster

```rust
// Shared async BME280 driver (drivers/bme280, `zigbee_bme280::asynch::Bme280`)
if let Ok(measurement) = bme280.measure_forced(&mut delay).await {
    // Temperature: ZCL uses units of 0.01 °C (i16) — driver returns centidegrees
    environment.update_environment(TemperatureHumidityMeasurement {
        temperature_centi_celsius: measurement.temperature_centi_celsius as i16,
        humidity_centi_percent: measurement.humidity_centi_percent.unwrap_or(0),
    });
    // Pressure: driver returns Pascal; the ZCL Pressure Measurement cluster
    // uses 0.1 hPa units (`PressureCluster::set_pressure`'s doc comment).
    environment.update_pressure((measurement.pressure_pa / 10) as i16);
}

// Shared async SHT3x driver (drivers/sht3x, `zigbee_sht3x::asynch::Sht3x`)
if let Ok(measurement) = sht3x.measure(&mut delay).await {
    environment.update_environment(TemperatureHumidityMeasurement {
        temperature_centi_celsius: measurement.temperature_centi_celsius,
        humidity_centi_percent: measurement.humidity_centi_percent,
    });
}

// `environment` is the shared `zigbee_runtime::profile::TemperatureHumidityBattery`
// archetype selected by the product crate; the stack handles reporting
// changes to the coordinator automatically based on the configured min/max
// reporting interval and reportable change.
```

### SPI Displays

| Display | Crate | Type | Typical Use |
|---------|-------|------|-------------|
| SSD1306 | `ssd1306` | 128×64 OLED (I2C/SPI) | Network status, sensor values |
| SH1106 | `sh1106` | 128×64 OLED | Same as SSD1306, different controller |
| ST7789 | `st7789` | Color TFT (SPI) | Rich UI with color |
| UC8151 / SSD1680 | `epd-waveshare` or custom | E-ink (SPI) | Ultra-low-power display |
| IL3820 (2.9″) | `epd-waveshare` | E-ink (SPI) | Battery sensor devices |

### Display Integration Pattern (SSD1306 OLED)

```rust
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

// Initialize display over I2C
let interface = I2CDisplayInterface::new(i2c);
let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
    .into_buffered_graphics_mode();
display.init().unwrap();

// Draw sensor data + network status
let style = MonoTextStyleBuilder::new()
    .font(&FONT_6X10)
    .text_color(BinaryColor::On)
    .build();

display.clear(BinaryColor::Off).unwrap();
Text::new("T: 23.5°C", Point::new(0, 10), style).draw(&mut display).unwrap();
Text::new("H: 65.0%",  Point::new(0, 24), style).draw(&mut display).unwrap();
Text::new("Net: Joined", Point::new(0, 38), style).draw(&mut display).unwrap();
display.flush().unwrap();
```

### E-Paper for Ultra-Low-Power Devices

E-paper is ideal for Zigbee sleepy end devices because:

- **Display persists with zero power** — content stays visible after
  the MCU enters deep sleep.
- **Only draws power during refresh** — ~26 mA for ~2 seconds.
- **Perfect for battery sensors** that update every 30–60 seconds.

```rust
// E-paper update cycle (SPI-based controller)
epd.wake_up()?;                    // Exit sleep, reinitialize
epd.update_frame(framebuffer)?;    // Transfer new image
epd.display_frame()?;              // Trigger e-ink refresh (~2s)
epd.sleep()?;                      // Back to zero power

// Total energy per refresh: ~26 mA × 2 s = ~52 mAs
// At one refresh per minute, e-paper adds < 1 mA average.
```

---

## 6. Creating a New Device Type

### Step-by-Step

1. **Choose ZCL clusters** for your device type (see table below).
2. **Create a new example crate** under `examples/`:
   ```bash
   mkdir -p examples/my-device/src examples/my-device/.cargo
   ```
3. **Set up `Cargo.toml`** — depend on the stack crates and your sensor/actuator drivers.
4. **Initialize platform peripherals** — I2C, SPI, GPIO as needed.
5. **Build ZCL clusters** with appropriate attribute ranges.
6. **Create the device** with `DeviceBuilder` or a template.
7. **Main loop:**
   ```
   loop {
       read sensors → update ZCL attributes → stack tick → sleep
   }
   ```

### Common Device Types

| Device | ZCL Clusters | HA Device ID |
|--------|-------------|--------------|
| Temperature sensor | Basic + TemperatureMeasurement + PowerConfig | 0x0302 |
| Weather station | Basic + Temp + RelativeHumidity + PressureMeasurement | 0x0302 |
| Motion sensor | Basic + OccupancySensing + IlluminanceMeasurement | 0x0107 |
| Door/window sensor | Basic + IASZone | 0x0402 |
| Smart plug | Basic + OnOff + ElectricalMeasurement + Metering | 0x0051 |
| Light bulb | Basic + OnOff + LevelControl + ColorControl | 0x0102 |
| Thermostat | Basic + Thermostat + ThermostatUI + FanControl | 0x0301 |
| Door lock | Basic + DoorLock | 0x000A |
| Roller shade | Basic + WindowCovering | 0x0202 |

### Skeleton: Custom Temperature Sensor

```rust
#![no_std]
#![no_main]

// Platform-specific imports (nRF52840 example)
use embassy_executor::Spawner;
use embassy_nrf::{self as hal, twim};
use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use zigbee_mac::NrfMac;
use zigbee_runtime::DeviceBuilder;

// Embassy interrupt binding for I2C
bind_interrupts!(struct Irqs { TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>; });

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = hal::init(Default::default());

    // 1. Set up I2C for the sensor
    let i2c_config = twim::Config::default();
    let mut i2c = twim::Twim::new(p.TWISPI0, Irqs, p.P0_26, p.P0_27, i2c_config);

    // 2. Initialize sensor driver (async, no blocking)
    bme280::init(&mut i2c, 0x76).await;

    // 3. Initialize the 802.15.4 radio
    let mac = NrfMac::new(p.RADIO);

    // 4. Build the Zigbee device with ZCL clusters
    let device = DeviceBuilder::new()
        .mac(mac)
        .device_type(EndDevice)
        .add_cluster(TemperatureMeasurement::new(-40_00, 85_00))  // -40°C to 85°C
        .add_cluster(RelativeHumidity::new(0, 100_00))            // 0–100%
        .add_cluster(PressureMeasurement::new(300, 1100))         // 300–1100 hPa
        .build();

    // 5. Join the network
    device.join().await.unwrap();
    info!("Joined network");

    // 6. Main loop: read → update → tick → sleep
    loop {
        let m = sensor.measure().unwrap();

        device.cluster::<TemperatureMeasurement>()
            .set_temperature((m.temperature * 100.0) as i16);
        device.cluster::<RelativeHumidity>()
            .set_humidity((m.humidity * 100.0) as u16);
        device.cluster::<PressureMeasurement>()
            .set_pressure((m.pressure * 10.0) as u16);

        device.tick().await;
        embassy_time::Timer::after_secs(30).await;
    }
}
```

---

## 7. Debugging

### Serial / defmt Output

| Platform | Logging | Viewer |
|----------|---------|--------|
| **nRF52840** | `defmt` + RTT | `probe-rs run` (auto-decodes defmt) |
| **ESP32-C6/H2/C5** | `esp-println` macros | `espflash flash --monitor` or any serial terminal |
| **Mock (host)** | `log` + `env_logger` | stdout |

Adjust log level:

```bash
# nRF52840 — set in .cargo/config.toml or env
DEFMT_LOG=trace cargo run --release

# ESP32 — serial output, control at compile time via features
# Mock — use RUST_LOG
RUST_LOG=debug cargo run -p mock-sensor
```

### Zigbee Sniffer

Capture over-the-air 802.15.4 frames for protocol debugging:

| Sniffer Hardware | Firmware | Software |
|------------------|----------|----------|
| nRF52840 USB Dongle | [nRF Sniffer for 802.15.4](https://www.nordicsemi.com/Products/Development-tools/nrf-sniffer-for-802154) | Wireshark + nRF plugin |
| TI CC2531 USB | Z-Stack sniffer firmware | Wireshark |
| Ubisys IEEE 802.15.4 | Built-in | Wireshark |

**Wireshark tips:**

1. Set the capture channel to match your network (default scan: channels 11–26).
2. Filter by PAN ID to isolate your network: `wpan.dst_pan == 0x1AAA`.
3. Add the Zigbee network key to decrypt NWK/APS layers:
   Edit → Preferences → Protocols → ZigBee → Security Keys.

### Common Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Device doesn't join | Permit joining not enabled on coordinator | Enable permit join (usually a 60–254 s window) |
| Device doesn't join | Wrong channel or PAN ID | Check coordinator config; stack scans channels 11–26 by default |
| No attribute reports | Reporting not configured | Set min/max reporting interval and reportable change on the cluster |
| Frame counter errors | NV storage mismatch after reflash | Clear NV storage on the device; re-pair with coordinator |
| CRC / receive errors | RF interference or distance | Move closer, reduce TX power, check antenna connection |
| `probe-rs` can't find device | Probe not connected or driver issue | Check USB, install libusb; try `probe-rs list` |
| `espflash` timeout | Chip not in download mode | Hold BOOT button, press RESET, release BOOT; or check USB-UART bridge |

---

## 8. CI / Reproducible Builds

For CI environments, pin the nightly toolchain with a `rust-toolchain.toml`
at the workspace root:

```toml
[toolchain]
channel = "nightly"
components = ["rust-src", "clippy", "rustfmt"]
targets = ["thumbv7em-none-eabihf", "riscv32imac-unknown-none-elf", "riscv32imc-unknown-none-elf"]
```

Typical CI steps:

```bash
# Lint
cargo clippy --workspace -- -D warnings

# Format check
cargo fmt --all -- --check

# Build all workspace crates (host)
cargo build --workspace

# Test all workspace crates
cargo test --workspace

# Build embedded examples (cross-compile only, no flash)
cd examples/nrf52840-sensor && cargo build --release
```

---

## License

zigbee-rs is dual-licensed under **MIT OR Apache-2.0**.
See the repository root for license files.
