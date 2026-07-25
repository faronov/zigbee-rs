# ESP32-C6 / ESP32-H2

Espressif's ESP32-C6 and ESP32-H2 are RISC-V SoCs with native IEEE 802.15.4
radio support, making them a great fit for zigbee-rs. Both chips share the
same MAC driver code — only the HAL feature flag differs.

> **✅ Hardware Verified:** The ESP32-C6 has been tested end-to-end on an
> **ESP32-C6-DevKitC-1** board with **Home Assistant + ZHA**. It appears as
> "Zigbee-RS ESP32-C6-Sensor" with Temperature, Humidity, and Battery entities.
> Network state is persisted to flash — the device survives reboots without
> re-pairing.

## Hardware Overview

| | ESP32-C6 | ESP32-H2 |
|---|----------|----------|
| **Core** | RISC-V (single, 160 MHz) | RISC-V (single, 96 MHz) |
| **Flash** | 4 MB (external SPI) | 4 MB (external SPI) |
| **SRAM** | 512 KB | 320 KB |
| **Radio** | WiFi 6 + BLE 5 + 802.15.4 | BLE 5 + 802.15.4 |
| **Target** | `riscv32imac-unknown-none-elf` | `riscv32imac-unknown-none-elf` |

Both chips have a built-in IEEE 802.15.4 radio driven by the `esp-radio`
crate's `ieee802154` module. The radio supports hardware CRC, configurable
TX power, RSSI/LQI measurement, and software address filtering.

### Common Development Boards

- **ESP32-C6-DevKitC-1** — USB-C, BOOT button on GPIO9
- **ESP32-H2-DevKitM-1** — USB-C, BOOT button on GPIO9
- **Seeed XIAO ESP32-C6** — compact, castellated pads
- **Ai-Thinker ESP-C6-12F** — module with PCB antenna

## Prerequisites

### Rust Toolchain

```bash
# Install nightly (required for no_std async + build-std)
rustup default nightly
rustup update nightly

# Add the RISC-V target
rustup target add riscv32imac-unknown-none-elf

# Ensure rust-src is available (needed for -Z build-std)
rustup component add rust-src
```

### Flash Tool

```bash
cargo install espflash
```

`espflash` handles flashing and serial monitoring in one command. Alternatively,
use the [web flasher](https://faronov.github.io/zigbee-rs/) — no tools needed,
just a browser with Web Serial API support (Chrome/Edge).

## Building

### ESP32-C6

```bash
cd examples/esp32c6-sensor
cargo build --release -Z build-std=core,alloc
```

### ESP32-H2

```bash
cd examples/esp32h2-sensor
cargo build --release -Z build-std=core,alloc
```

> **Note:** The `-Z build-std=core,alloc` flag is configured in each example's
> `.cargo/config.toml` under `[unstable]`, so a plain `cargo build --release`
> also works from within the example directory.

### What `.cargo/config.toml` Sets

```toml
[build]
target = "riscv32imac-unknown-none-elf"

[target.riscv32imac-unknown-none-elf]
runner = "espflash flash --partition-table ../../boards/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv --target-app-partition ota_0 --erase-parts otadata --monitor"
rustflags = ["-C", "link-arg=-Tlinkall.x"]

[unstable]
build-std = ["core", "alloc"]

[env]
ESP_LOG = "info"
```

This is the ESP32-C6 OTA-capable runner. It always reinstalls the checked-in
partition table, writes the wired image to `ota_0`, and erases `otadata` so a
previously selected `ota_1` cannot hide the newly flashed firmware. The H2
example, which does not yet instantiate an OTA client, keeps the plain
`espflash flash --monitor` runner.

The `linkall.x` linker script is provided by `esp-hal` and sets up the ESP32
memory layout, interrupt vectors, and boot sequence.

### CI Build Command

From `.github/workflows/ci.yml`:

```bash
# Exact command used in CI (ubuntu-latest, nightly toolchain)
cd examples/esp32c6-sensor
cargo build --release -Z build-std=core,alloc

# Firmware artifact extraction
OBJCOPY=$(find $(rustc --print sysroot) -name llvm-objcopy | head -1)
$OBJCOPY -O binary target/riscv32imac-unknown-none-elf/release/esp32c6-sensor \
         target/riscv32imac-unknown-none-elf/release/esp32c6-sensor.bin
```

### Release Profile

Both examples use an optimized release profile:

```toml
[profile.release]
opt-level = "s"    # Optimize for size
lto = true         # Link-Time Optimization
```

## Flashing

### espflash (recommended)

```bash
cd examples/esp32c6-sensor

# Flash and open serial monitor
espflash flash \
  --partition-table ../../boards/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv \
  --target-app-partition ota_0 \
  --erase-parts otadata \
  --monitor target/riscv32imac-unknown-none-elf/release/esp32c6-sensor

# Or use cargo run (runner configured in .cargo/config.toml)
cargo run --release
```

### Web Flasher (no tools needed)

Visit [https://faronov.github.io/zigbee-rs/](https://faronov.github.io/zigbee-rs/)
in Chrome or Edge:

1. Select your chip (ESP32-C6 or ESP32-H2)
2. Click **Connect** and choose the serial port
3. Click **Flash** — firmware is downloaded from the latest CI build

The web flasher uses the [ESP Web Tools](https://esphome.github.io/esp-web-tools/)
library and the Web Serial API. The firmware `.bin` artifacts are published to
GitHub Pages on every push to `main`.

### espflash Troubleshooting

If `espflash` times out:

1. Hold the **BOOT** button
2. Press and release **RESET** (while holding BOOT)
3. Release **BOOT**
4. Retry the flash command

## MAC Backend Notes

The ESP32 MAC backend lives in `zigbee-mac/src/esp/`:

```
zigbee-mac/src/esp/
├── mod.rs      # EspMac struct, MacDriver trait impl, PIB management
└── driver.rs   # Ieee802154Driver — low-level radio wrapper
```

### Feature Flags

| Feature | Chip | Cargo.toml dependency |
|---------|------|----------------------|
| `esp32c6` | ESP32-C6 | `zigbee-mac = { features = ["esp32c6"] }` |
| `esp32h2` | ESP32-H2 | `zigbee-mac = { features = ["esp32h2"] }` |

### Key Dependencies

```toml
esp-hal = { version = "1.0.0", features = ["esp32c6", "unstable"] }
esp-radio = { version = "0.17.0", features = ["esp32c6", "ieee802154", "unstable"] }
```

### How It Works

1. **`EspMac`** wraps `Ieee802154Driver` and implements the `MacDriver` trait
2. **`Ieee802154Driver`** wraps `esp_radio::ieee802154::Ieee802154` for
   synchronous TX and polling-based RX
3. The EUI-64 address is read from the chip's eFuse factory MAC
4. Scanning uses real beacon parsing — the radio enters RX mode and collects
   beacon frames across channels 11–26
5. CSMA-CA is implemented in software with configurable backoff parameters

### Switching Chips

To switch between ESP32-C6 and ESP32-H2, replace all feature flags:

```diff
- zigbee-mac = { path = "../../zigbee-mac", features = ["esp32c6"] }
+ zigbee-mac = { path = "../../zigbee-mac", features = ["esp32h2"] }

- esp-hal = { version = "1.0.0", features = ["esp32c6", "unstable"] }
+ esp-hal = { version = "1.0.0", features = ["esp32h2", "unstable"] }

- esp-radio = { version = "0.17.0", features = ["esp32c6", "ieee802154", "unstable"] }
+ esp-radio = { version = "0.17.0", features = ["esp32h2", "ieee802154", "unstable"] }
```

The MAC driver code is shared — only the HAL feature gate changes.

## Example Walkthrough

The `esp32c6-sensor` example implements a Zigbee 3.0 temperature & humidity
end device with:

- **On-chip temperature sensor** (via `esp_hal::tsens::TemperatureSensor`)
- **Flash NV storage** — network state persists across power cycles (no re-pairing)
- **NWK Leave handler** — auto-erases NV and rejoins when coordinator sends Leave
- **Default reporting** — configures report intervals at boot so data flows before ZHA interview
- **Identify cluster** (0x0003) — supports Identify, IdentifyQuery, TriggerEffect
- **Battery percentage** reporting via Power Configuration cluster
- Join/leave button (BOOT / GPIO9)

### Initialization

```rust
#[esp_hal::main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // BOOT button (GPIO9, active low with pull-up)
    let button = Input::new(
        peripherals.GPIO9,
        InputConfig::default().with_pull(Pull::Up),
    );

    // IEEE 802.15.4 MAC driver
    let ieee802154 = esp_radio::ieee802154::Ieee802154::new(peripherals.IEEE802154);
    let config = esp_radio::ieee802154::Config::default();
    let mac = zigbee_mac::esp::EspMac::new(ieee802154, config);
```

### Device Setup

```rust
    use zigbee_zcl::clusters::basic::PowerSource;
    use zigbee_zcl::{ClusterId, DeviceId};

    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Zigbee-RS")
        .model("ESP32-C6-Sensor")
        .sw_build("0.1.0")
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, DeviceId::TEMPERATURE_SENSOR, |ep| {
            ep.cluster_server(ClusterId::BASIC)
                .cluster_server(ClusterId::POWER_CONFIG)
                .cluster_server(ClusterId::IDENTIFY)
                .cluster_server(ClusterId::TEMPERATURE)
                .cluster_server(ClusterId::HUMIDITY)
        })
        .build();
```

### Main Loop

The main loop handles button presses (join/leave), updates simulated sensor
values every 30 seconds, and ticks the Zigbee stack.

### Adding a Real Sensor

To add an external SHTC3 I²C sensor (SDA→GPIO6, SCL→GPIO7):

```rust
use esp_hal::i2c::master::I2c;

let i2c = I2c::new(peripherals.I2C0, /* config */)
    .with_sda(peripherals.GPIO6)
    .with_scl(peripherals.GPIO7);

// Use any embedded-hal 1.0 compatible sensor driver
```

## Flash NV Storage

Both sensor examples persist Zigbee network state to the last two 4 KB sectors
of the external flash (addresses `0x3FE000`-`0x3FFFFF`, 8 KB total).
`boards/esp32-zigbee-devkit` owns this partition and wraps the official
`esp_storage::FlashStorage` implementation of the standard
`embedded-storage` NOR traits.

The bounded board flash is wrapped in `LogStructuredNv<ApplicationFlash>`, a
log-structured format that appends writes and only erases during compaction.
The example never sees physical flash addresses or raw controller calls.

On boot, the device checks for saved network state and automatically rejoins
the previous network. If the coordinator sends a NWK Leave command, the device
erases NV storage and starts fresh commissioning.

## Partition Table

OTA needs two application slots, so the devkits are flashed with an explicit
ESP-IDF partition table, checked in at
`boards/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv`:

| Partition | Type / SubType | Offset | Size |
|-----------|----------------|--------|------|
| `otadata` | `data` / `ota` | `0x9000` | `0x2000` |
| `ota_0` | `app` / `ota_0` | `0x10000` | `0x1F0000` |
| `ota_1` | `app` / `ota_1` | `0x200000` | `0x1F0000` |
| `zbnv` | `data` / `undefined` | `0x3F0000` | `0x10000` |

Notes:

* `ota_0` starts at `0x10000`, which is where a plain `espflash flash` already
  puts the application — a device flashed before this table existed is already
  running from `ota_0`.
* The Zigbee NV pages stay at `0x3FE000`-`0x3FFFFF`, now the tail of `zbnv`.
  Adding the table does not move a single byte of joined-network state.
* `0xB000`-`0x10000` is intentionally unmapped (ESP-IDF would put `nvs` and
  `phy_init` there; this firmware uses neither).
* `esp32-zigbee-devkit::layout` mirrors these addresses, asserts the invariants
  at compile time and parses the CSV in a unit test, so the constants and the
  table cannot drift apart.
* The OTA writer validates the four on-device partition entries before it
  exposes the OTA client. A plain single-app table disables OTA while leaving
  the sensor operational, rather than accepting a download that the bootloader
  can never activate or halting the device during boot.
* Do not use a plain `espflash flash` command after migration: without
  `--partition-table`, espflash reinstalls its default single-app table.

Build the 3072-byte binary that goes at `0x8000` with:

```bash
boards/esp32-zigbee-devkit/partitions/build-partition-table.sh
```

The script uses `espflash partition-table --to-binary`, checks the size and
converts the result back to CSV to confirm the round trip.

## OTA Updates

> **Status:** implemented in software and covered by host tests; **not yet
> exercised on hardware**. Migrating a device to the two-slot layout rewrites
> its partition table, which is a deliberate, separate step.

### How it works

`esp32-zigbee-devkit::ota::EspFirmwareWriter` implements the runtime's
`FirmwareWriter` trait:

1. **Slot selection** — `otadata` is read to find the running slot; the *other*
   slot is the staging target. Erased `otadata` means the bootloader fell back
   to `ota_0`, so staging goes to `ota_1`.
2. **Lazy erase** — `erase_slot()` only resets bookkeeping. Erasing 1.9 MiB in
   one go would hold `esp-storage`'s critical section for many seconds and the
   radio would miss every parent poll, so each 4 KiB sector is erased just
   before the first byte lands in it.
3. **Word alignment** — Zigbee blocks are 48 bytes and the last one is ragged,
   but ESP flash programs 4-byte words. Sub-word tails are buffered in RAM and
   the final partial word is padded with `0xFF`, past the end of the image, so
   no image byte is ever altered.
4. **Verification** — the staged slot is re-read: image magic `0xE9`, the chip
   ID (`0x000D` for C6, `0x0010` for H2), the `hash_appended` flag and the
   trailing SHA-256 all have to match.
5. **Activation** — one 32-byte `otadata` entry is written into the sector that
   does *not* hold the active entry, then the chip resets. Any power failure
   before that write leaves the old entry, and therefore the old firmware, in
   charge. `abort()` never touches `otadata`.

The application defers activation until after it has checkpointed its Zigbee
state to NV, so the reset into the new image cannot lose the network keys.

### Building an OTA image

The payload of the Zigbee container must be an **ESP application image**
(`espflash save-image` output) — not an ELF and not a merged flash image:

```bash
cd examples/esp32c6-sensor
tools/create-ota.py 2            # writes target/ota/
```

The tool builds the firmware with `ESP32_OTA_VERSION=2`, runs
`espflash save-image`, wraps the result in an OTA container
(manufacturer `0x1234`, image type `0x0001`, hardware version 1), parses the
container back and compares it byte for byte with the image, and regenerates a
`zigpy_local` index with the sha3-256 checksums ZHA verifies:

```json
{
  "firmwares": [
    {
      "path": "esp32c6-sensor-v2.ota",
      "file_version": 2,
      "file_size": 279554,
      "image_type": 1,
      "manufacturer_id": 4660,
      "checksum": "sha3-256:…"
    }
  ]
}
```

Point ZHA at it with a `zigpy_local` OTA provider:

```yaml
zigpy_config:
  ota:
    providers:
      - type: zigpy_local
        index_file: /config/zigbee-rs-ota/index.json
```

### Migrating a device that is already joined

These steps **write flash** and are not performed by `cargo build`:

1. Back up the NV pages: `espflash read-flash 0x3FE000 0x2000 nv-backup.bin`.
2. Flash the OTA-capable firmware together with the required table:

   ```bash
   cd examples/esp32c6-sensor
   espflash flash \
     --partition-table ../../boards/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv \
     --target-app-partition ota_0 \
     --erase-parts otadata \
     --monitor target/riscv32imac-unknown-none-elf/release/esp32c6-sensor
   ```

   The configured `cargo run --release` runner executes the same command.
3. Confirm the device rejoins with its existing keys before publishing an
   image; only then run an upgrade from ZHA.

## ESP32-C6-DevKitC-1 LED Note

The ESP32-C6-DevKitC-1 has a **WS2812 addressable RGB LED** (on GPIO8), not
a simple GPIO LED. The Identify cluster blink feature in the ESP32-C6 example
does not drive this LED. If you want LED feedback during Identify, you would
need to add a WS2812 driver (e.g., `smart-leds` + `esp-hal-smartled`). The
ESP32-H2 example does implement LED blinking during Identify.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `espflash` can't find device | Not in download mode | Hold BOOT → press RESET → release BOOT |
| `espflash` timeout | USB-UART bridge issue | Try a different USB cable/port |
| Build error: `rust-src` not found | Missing component | `rustup component add rust-src` |
| Linker error: `linkall.x` not found | `esp-hal` version mismatch | Check `esp-hal` version matches `esp-radio` |
| Serial output garbled | Wrong baud rate | Default is 115200 — check monitor settings |
| Device doesn't join network | Coordinator not in permit-join mode | Enable permit joining on your coordinator |
| No beacon found | Wrong channel | Ensure coordinator and device scan the same channels |

### Serial Monitor

```bash
# Standalone monitor (without flashing)
espflash monitor

# Or any serial terminal at 115200 baud
screen /dev/ttyUSB0 115200
```

Expected output:

```
[init] ESP32-C6 Zigbee sensor starting
[init] Radio ready
[init] NV: restored network state from flash
[init] Default reporting configured (temp: 60-300s, hum: 60-300s, battery: 300-3600s)
[init] Device ready — press BOOT button to join/leave
[btn] Joining network…
[scan] Found network on channel 15, PAN 0x1AAA
[join] Association successful, short addr = 0x1234
[sensor] T=22.50°C  H=50.00%  Battery=100%
[nv] State saved to flash
```
