# ESP32-H2 Zigbee Temperature & Humidity Sensor

A `no_std` Zigbee 3.0 end device for the **ESP32-H2** that reports the on-chip
die temperature and simulated humidity. Uses the **esp-hal 1.0**
`#[esp_hal::main]` entry point with `block_on()` for the async runtime.

## Hardware Requirements

- ESP32-H2 development board (built-in IEEE 802.15.4 + BLE 5 radio)
- USB cable for flashing and serial monitor
- BOOT button (GPIO9) used for join/leave control

## Prerequisites

- Rust nightly toolchain with `rust-src`
- `espflash` 4.5.0 or newer for ESP32-H2 revision 1.2:
  `cargo install espflash`
- Target: `riscv32imac-unknown-none-elf` (added automatically via `.cargo/config.toml`)

No vendor libraries or binary blobs are needed — the project uses the
`esp-ieee802154` crate for native 802.15.4 radio access.

## Build

```sh
cargo build --release
```

## Flash & Monitor

```sh
espflash flash --monitor target/riscv32imac-unknown-none-elf/release/esp32h2-sensor
```

Or use the configured runner:

```sh
cargo run --release
```

## What It Demonstrates

- Initialising the ESP32-H2 IEEE 802.15.4 radio with `esp-radio`
- A typed product profile (`esp32_zigbee_devkit_product::profile::SensorProfile`)
  and `zigbee_runtime::node::ZigbeeNode`, instead of building the device and its
  cluster list by hand
- **esp-hal 1.0 pattern** — `#[esp_hal::main]` entry point with `block_on()` async runtime
  (replaces the removed `embassy_executor` / `riscv_rt` approach)
- Registering ZCL endpoint 1 (Home Automation profile, device type 0x0302)
  with **Basic**, **Power Configuration**, **Identify**, **Temperature Measurement**,
  and **Relative Humidity** clusters
- **NWK Leave handler** — auto-rejoins when coordinator sends Leave
- **Default reporting configuration** — temp/humidity: 60–300 s, battery: 300–3600 s
  (from the shared `TemperatureHumidityBattery` profile archetype)
- **Identify cluster** (0x0003) — LED blinks during Identify
- Button toggles join/leave directly through `ZigbeeNode`
  (`start_or_resume`/`secure_rejoin`/`factory_reset`)
- Periodic on-chip temperature and simulated humidity updates
- **Crash-safe security-state journal** through the product crate (see below)

## Hardware Validation

The current product/profile/`ZigbeeNode` build was validated on an ESP32-H2
SuperMini revision 1.2 with 4 MiB flash on 2026-07-26:

- native USB Serial/JTAG boot and flashing
- one-time migration of the existing flat `LogStructuredNv` records into the
  crash-safe `SecurityStateJournal`, while retaining the original PAN, short
  address, parent, key, and IEEE address
- the legacy sector remained intact while committed `ZBSS`/`CMIT` records were
  written to the other sector
- reset and secure resume without reopening pairing
- monotonically increasing secured NWK counters after migration
- Device Announce, ZHA interview completion, reporting, and Identify responses
- live Home Assistant values: on-chip temperature, simulated humidity, and
  battery percentage

Fresh factory-reset commissioning and long-duration sleepy operation remain
separate hardware gates. This H2 product intentionally has no OTA backend.

The ESP-IDF 5.5 bootloader on this board requires an application descriptor.
The example provides it through `esp-bootloader-esp-idf` 0.4, whose linker
section is compatible with the `esp-hal` 1.0 linker layout.

`esp-hal` 1.0 does not expose its generic TSENS driver for the H2, so this
example contains a small H2 register-level driver that powers the sensor only
for each measurement.

## Operation

1. Power on → device starts idle
2. Press BOOT → joins the nearest open Zigbee network
3. Once joined → reports simulated sensor values periodically
4. Press BOOT again → leaves the network

## Project Structure

```
esp32h2-sensor/
├── .cargo/config.toml   # Target, runner, rustflags, build-std
├── Cargo.toml            # Dependencies (esp-hal 1.0, esp-radio 0.17, product crate, zigbee-rs crates)
└── src/
    ├── main.rs           # Composition root: platform startup + resource construction (#[esp_hal::main])
    ├── app.rs            # SensorApp: ZigbeeNode-driven event loop, button, poll windows, LED
    └── chip_temperature.rs  # H2 on-chip TSENS register driver
```

Architecture: `boards/esp32-zigbee-devkit` exposes only the physical chip
flash (`esp32_zigbee_devkit::flash::RawFlash`, wrapping `esp_storage`), with
no dependency on `zigbee-runtime`. `products/esp32-zigbee-devkit` (the
`esp32-zigbee-devkit-product` crate) owns the bounded 8 KB NV window and
constructs the crash-safe `SecurityStore`
(`zigbee_runtime::security_journal::SecurityStateJournal`) plus the typed
`SensorProfile`, shared with ESP32-C6.

## OTA

The product crate's OTA firmware writer
(`esp32_zigbee_devkit_product::ota::EspFirmwareWriter`) is only compiled for
the `esp32c6` feature — it is not part of this build at all. It is **not
wired into this example**: the OTA client, the packaging tool, and the
two-slot partition table have only been brought up for the ESP32-C6 (see
[`examples/esp32c6-sensor`](../esp32c6-sensor/README.md)), and nothing here
has yet been verified through a real H2 OTA transfer. Adding OTA to this
build would mean giving the H2 build its own two-slot partition table and
firmware writer, gated on an `esp32h2` feature in the product crate the same
way the C6 backend is gated today.
