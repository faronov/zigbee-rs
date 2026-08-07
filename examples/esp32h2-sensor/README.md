# ESP32-H2 Zigbee Temperature & Humidity Sensor

A `no_std` Zigbee 3.0 end device for the **ESP32-H2** that reports the on-chip
die temperature and simulated humidity. Network state is persisted to flash,
and the checked dual-slot layout supports Zigbee OTA upgrades. Uses the
**esp-hal 1.0** `#[esp_hal::main]` entry point with `block_on()` for the async
runtime.

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
espflash flash \
  --partition-table ../../products/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv \
  --target-app-partition ota_0 \
  --erase-parts otadata \
  --monitor target/riscv32imac-unknown-none-elf/release/esp32h2-sensor
```

Or use the configured runner:

```sh
cargo run --release
```

Both commands install the checked-in OTA partition table, flash the application
to `ota_0`, and erase `otadata` so a previously selected `ota_1` cannot hide
the wired image. Back up the Zigbee NV pages before the first migration; their
addresses are preserved by the new table.

If the firmware starts with a missing or incompatible table, it disables OTA
and continues as a normal Zigbee sensor without advertising cluster `0x0019`.

## What It Demonstrates

- Initialising the ESP32-H2 IEEE 802.15.4 radio with `esp-radio`
- **Hardware-only AES-128** for Zigbee NWK/APS security, with two startup
  known-answer tests and no software fallback
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
- **OTA Upgrade client** (0x0019) — stages images into the inactive
  `ota_0`/`ota_1` slot and activates them through `otadata`
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
- a complete v1-to-v2 OTA upgrade through ZHA on 2026-08-05: the image was
  staged in `ota_1`, its appended SHA-256 was verified, v2 booted at
  `0x200000`, and IEEE address, PAN `0xDFE9`, short address `0x9A2A`, parent
  `0xCED9`, and network credentials survived the activation reset
- Home Assistant reported installed firmware version `0x00000002` after the
  upgrade

Fresh factory-reset commissioning and long-duration sleepy operation remain
separate hardware gates. The verified OTA transfer took about 91 minutes over
a weak parent link; the protocol and activation path completed correctly, but
that run is not a throughput target.

The ESP-IDF 5.5 bootloader on this board requires an application descriptor.
The example provides it through `esp-bootloader-esp-idf` 0.4, whose linker
section is compatible with the `esp-hal` 1.0 linker layout.

`esp-hal` 1.0 does not expose its generic TSENS driver for the H2, so this
example contains a small H2 register-level driver that powers the sensor only
for each measurement.

## Operation

1. Power on → restores saved state or starts BDB commissioning automatically
2. Once joined → reports sensor values and polls its parent periodically
3. Short BOOT press → leaves; the normal unjoined retry later commissions again
4. Long BOOT press → factory reset and reboot

## Project Structure

```
esp32h2-sensor/
├── .cargo/config.toml    # Target, OTA-aware runner, rustflags, build-std
├── build.rs              # ESP32_OTA_VERSION -> FIRMWARE_VERSION
├── Cargo.toml            # esp-hal, esp-radio, product and zigbee-rs crates
├── tools/create-ota.py   # Thin wrapper over ../../tools/create-esp32-ota.py
└── src/
    ├── main.rs           # Composition root: platform startup + resource construction (#[esp_hal::main])
    ├── app.rs            # SensorApp: ZigbeeNode-driven event loop, button, poll windows, LED
    ├── chip_temperature.rs  # H2 on-chip TSENS register driver
    └── time_driver.rs     # embassy-time driver
```

Architecture: `boards/esp32-zigbee-devkit` exposes only the physical chip
flash (`esp32_zigbee_devkit::flash::RawFlash`, wrapping `esp_storage`), with
no dependency on `zigbee-runtime`. `products/esp32-zigbee-devkit` (the
`esp32-zigbee-devkit-product` crate) owns the bounded 8 KB NV window and
constructs the crash-safe `SecurityStore`
(`zigbee_runtime::security_journal::SecurityStateJournal`) plus the typed
`SensorProfile`, shared with ESP32-C6.

## OTA

The firmware hosts an OTA Upgrade client on endpoint 1 and stages images into
whichever of `ota_0`/`ota_1` is not running through
`esp32_zigbee_devkit_product::ota::EspFirmwareWriter`. Sectors are erased
lazily, the staged ESP application image is verified against its appended
SHA-256, and activation writes a redundant `otadata` entry only after the
application checkpoints Zigbee security state.

Build an image:

```sh
tools/create-ota.py 2                 # -> target/ota/esp32h2-sensor-v2.ota
```

`ESP32_OTA_VERSION` accepts decimal or `0x`-prefixed values (default `1`).
The packager sets it while building, so the version reported by the firmware
cannot differ from the Zigbee OTA container. The same value is exposed through
`Basic::ApplicationVersion` and `Basic::SWBuildID`.

The H2 image uses manufacturer `0x1234`, image type `0x0002`, and hardware
version 1. The shared packager also regenerates a `zigpy_local` `index.json`
with SHA3-256 checksums for ZHA. The canonical chip configuration lives in
`../../tools/create-esp32-ota.py`; this example's script only selects H2.
