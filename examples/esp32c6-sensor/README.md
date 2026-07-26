# ESP32-C6 Zigbee Temperature & Humidity Sensor

A `no_std` Zigbee 3.0 end device for the **ESP32-C6** that reads the on-chip
temperature sensor and reports temperature, humidity, and battery percentage.
Network state is persisted to flash — the device survives reboots without
re-pairing.

> **✅ Verified** on ESP32-C6-DevKitC-1 with Home Assistant + ZHA (pre-refactor
> build: flat NV persistence, direct `ZigbeeDevice` calls). Shows as
> "Zigbee-RS ESP32-C6-Sensor" with Temperature, Humidity, and Battery entities.
>
> **Hardware gate open:** the current code builds on a typed product profile,
> `zigbee_runtime::node::ZigbeeNode`, and a crash-safe `SecurityStateJournal`
> in place of the flat NV item store (see
> [NV storage](../../docs/book/src/advanced/nv-storage.md)). This is
> build- and host-test-verified only; commissioning, reporting, OTA, and
> persistence have not yet been re-run against this exact code on hardware.

## Hardware Requirements

- ESP32-C6 development board (built-in IEEE 802.15.4 radio)
- USB cable for flashing and serial monitor
- BOOT button (GPIO9) used for join/leave control

## Prerequisites

- Rust nightly toolchain with `rust-src`
- `espflash` for flashing: `cargo install espflash`
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
  --monitor target/riscv32imac-unknown-none-elf/release/esp32c6-sensor
```

Or use the configured runner:

```sh
cargo run --release
```

Both commands install the checked-in OTA partition table, flash the application
to `ota_0`, and erase `otadata` so the wired image boots even after a previous
OTA selected `ota_1`. Back up the Zigbee NV pages before the first migration;
their addresses are preserved by the new table.

If the firmware is started with a missing or incompatible table, it logs that
OTA is disabled and continues as a normal Zigbee sensor without advertising
the OTA client cluster.

## What It Demonstrates

- Initialising the ESP32-C6 IEEE 802.15.4 radio with `esp-radio`
- A typed product profile (`esp32_zigbee_devkit_product::profile::SensorProfile`)
  and `zigbee_runtime::node::ZigbeeNode`, instead of building the device and its
  cluster list by hand
- Registering ZCL endpoint 1 (Home Automation profile, device type 0x0302)
  with **Basic**, **Power Configuration**, **Identify**, **Temperature Measurement**,
  and **Relative Humidity** clusters
- **On-chip temperature sensor** via `esp_hal::tsens::TemperatureSensor`
- **Crash-safe security-state journal** — network/security state saved to the
  last 2 sectors of flash (`0x3FE000`, 8 KB) through
  `esp32_zigbee_devkit_product::storage::SecurityStore`
  (`zigbee_runtime::security_journal::SecurityStateJournal`), which reserves a
  bounded block of NWK frame-counter values on every commit so a power loss
  can never replay one
- **OTA Upgrade client** (0x0019) — stages images into the inactive `ota_0`/`ota_1`
  slot and activates them through `otadata`, composed into the profile only
  when the checked partition table matches (`zigbee_runtime::profile::OptionalOta`)
- **NWK Leave handler** — auto-erases NV and rejoins when coordinator sends Leave
- **Default reporting configuration** — temp/humidity: 60–300 s, battery: 300–3600 s
  (devices report data even before ZHA sends ConfigureReporting)
- **Identify cluster** (0x0003) — supports Identify, IdentifyQuery, TriggerEffect commands
- Battery percentage reporting via Power Configuration cluster
- Button toggles join/leave directly through `ZigbeeNode` (`start_or_resume`/`secure_rejoin`/`factory_reset`)

## Operation

1. Power on → restores saved network state from flash (if any) and auto-rejoins
2. If no saved state → press BOOT to initiate BDB commissioning
3. Once joined → reads on-chip temp sensor periodically, reports to coordinator
4. Press BOOT → leaves the network and clears flash NV storage
5. **Power cycle** → device reconnects automatically (no re-pairing needed!)

## OTA Updates

> **Not yet run on hardware.** The device-side code and the packaging tool are
> implemented and covered by host tests; moving an already-joined device onto
> the two-slot partition table is a separate, deliberate step (see the
> [ESP32 platform guide](../../docs/book/src/platform-guides/esp32.md)).

The firmware hosts an **OTA Upgrade client** on endpoint 1 and stages images
into whichever of `ota_0`/`ota_1` is not running, using
`esp32_zigbee_devkit_product::ota::EspFirmwareWriter`. Sectors are erased lazily during
the download, the staged slot is verified against the SHA-256 that `espflash`
appends to every application image, and activation writes a single `otadata`
entry and reboots — after the application has checkpointed its network state.

Build an image:

```sh
tools/create-ota.py 2                 # -> target/ota/esp32c6-sensor-v2.ota
```

`ESP32_OTA_VERSION` (default `1`) is the OTA file version the firmware reports;
the tool sets it while building so the container and the firmware can never
disagree. The same number is exposed as `Basic::ApplicationVersion` (low byte)
and `Basic::SWBuildID`.

The tool also regenerates `target/ota/index.json` in `zigpy_local` format with
sha3-256 checksums, ready to hand to ZHA:

```yaml
zigpy_config:
  ota:
    providers:
      - type: zigpy_local
        index_file: /config/zigbee-rs-ota/index.json
```

The device advertises manufacturer `0x1234`, image type `0x0001` and hardware
version 1 — the constants at the top of `src/main.rs` and in
`tools/create-ota.py` must stay in sync.

## Project Structure

```
esp32c6-sensor/
├── .cargo/config.toml    # Target, runner, rustflags, build-std
├── build.rs              # ESP32_OTA_VERSION -> FIRMWARE_VERSION
├── Cargo.toml            # Dependencies (esp-hal 1.0, esp-radio 0.17, product crate, zigbee-rs crates)
├── tools/create-ota.py   # Build -> ESP image -> Zigbee OTA container -> zigpy index
└── src/
    ├── main.rs           # Composition root: platform startup + resource construction (#[esp_hal::main])
    ├── app.rs            # SensorApp: ZigbeeNode-driven event loop, button, poll windows
    ├── ota_client.rs     # Thin logging/policy wrapper over runtime OtaSession
    └── time_driver.rs    # embassy-time driver
```

Architecture: `boards/esp32-zigbee-devkit` exposes only the physical chip
flash (`esp32_zigbee_devkit::flash::RawFlash`, wrapping `esp_storage`), with
no dependency on `zigbee-runtime`. `products/esp32-zigbee-devkit` (the
`esp32-zigbee-devkit-product` crate) owns the partition layout (the bounded
8 KB NV window at the tail of `zbnv`, unchanged), the `otadata` codec, the OTA
firmware writer, and the typed `SensorProfile` — shared by ESP32-C6 and
ESP32-H2, with the two-slot partition table and OTA writer gated to the
`esp32c6` feature.
