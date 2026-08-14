# ESP32-C6 / ESP32-H2

Espressif's ESP32-C6 and ESP32-H2 are RISC-V SoCs with native IEEE 802.15.4
radio support, making them a great fit for zigbee-rs. Both chips share the
same MAC driver code — only the HAL feature flag differs.

> **✅ Hardware Verified:** The ESP32-C6 has been tested end-to-end on an
> **ESP32-C6-DevKitC-1** board with **Home Assistant + ZHA**. The ESP32-H2
> revision 1.2 has additionally passed the production hardware-AES and
> persisted-resume gate plus a complete v1-to-v2 OTA activation: two startup
> KATs, secured traffic, preserved network identity, live sensor entities, and
> a silent restart without re-pairing or `Device_annce`.

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
TX power, RSSI/LQI measurement, and software address filtering. Their shared
AES accelerator supplies all production NWK/APS CCM* and AES-MMO operations;
the hardware RNG supplies runtime entropy while the RF subsystem is active.

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
runner = "espflash flash --partition-table ../../products/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv --target-app-partition ota_0 --erase-parts otadata --monitor"
rustflags = ["-C", "link-arg=-Tlinkall.x"]

[unstable]
build-std = ["core", "alloc"]

[env]
ESP_LOG = "info"
```

Both examples use this OTA-capable runner. It always reinstalls the checked-in
partition table, writes the wired image to `ota_0`, and erases `otadata` so a
previously selected `ota_1` cannot hide the newly flashed firmware.

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
  --partition-table ../../products/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv \
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

The ESP32 MAC backend lives in `zigbee-mac/src/esp/`, with its reusable
bounded AES register protocol beside it:

```
zigbee-mac/src/
├── esp_aes.rs  # bounded AES protocol, KATs, timeout/dead-engine tests
└── esp/
    ├── mod.rs  # EspMac, AES ownership, MacDriver and platform services
    └── driver.rs
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
6. `EspMac` exclusively owns `peripherals.AES`, runs two startup known-answer
   tests, and exposes only the bounded hardware cipher to the Zigbee stack
7. `PlatformServices::fill_random` reads the ESP hardware RNG; it never
   substitutes predictable pseudo-random bytes

### Hardware AES policy

ESP32 production builds are hardware-only. `esp-hal` retains the AES token and
clock guard, while zigbee-rs drives the shared H2/C6 register protocol directly
because `esp-hal` 1.0.0's convenience method waits without a bound. Every block
must observe:

1. idle before programming;
2. a bounded return to idle after the trigger;
3. completion before the result is published.

The BUSY state is transient and may complete before software can sample it,
particularly on ESP32-C6, so it is not a valid start handshake. Each observable
wait has a finite iteration limit, and the two startup KATs reject a gated,
trigger-ignoring, corrupt, or non-reusable accelerator. A stuck busy operation
returns an error without modifying the caller's output. There is no RustCrypto
fallback. CI requires the `EspHardwareAes128` backend and rejects
`aes::soft::fixslice` / `SoftwareAes128` symbols in both production ELFs.

The ESP32-H2 revision 1.2 acceptance run on 2026-08-05 passed both KAT vectors
on silicon and then resumed its existing secured network. An ESP32-C6 revision
0.0 run later exposed two assumptions that were not portable: software cannot
reliably sample the short BUSY interval, and Espressif's low-level AES reset
sequence explicitly clears the Digital Signature reset because AES can
otherwise remain held in reset. `EspAesEngine` now accepts an already-complete
IDLE state and performs that coupled-reset release through esp-hal's
chip-specific PAC accessor (`PCR.DS_CONF` is at 0xe0 on C6 and 0xdc on H2)
before running the KATs. The corrected path passed both KAT vectors on an
ESP32-C6 revision 0.0, and both release images pass the build and linked-symbol
gates.

The independent channel-15 capture
`esp32h2-hardware-aes-event-parity-resume-20260805.pcap` contains 10,921
packets over 351.785955 seconds (SHA-256
`aba93a40d71c39cb46dee5d2ba0423de53f2a506c1a42b3d8c3c7504b265ef67`).
Across two reset/resume cycles it contains no Beacon Request, Association
Request, or Device Announce from the H2. Direct secured frames advance from
NWK counter 21,335 to 22,364 across a fresh reservation, all with non-zero
MICs; decoded temperature, humidity, and battery reports reached the
coordinator. The final ESP application images are 285,792 bytes for H2
(SHA-256
`035b4e1dd4eaf4297d61745e516591826a435ecb50aa721f8a99249d491056b2`)
and 326,752 bytes for C6 (SHA-256
`1a9806a630ee032ac94966a3500fd62aaac205315fff91d12419591987d4ec9d`).

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

The `esp32c6-sensor` and `esp32h2-sensor` examples implement a Zigbee 3.0
temperature & humidity end device around a typed product profile
(`esp32_zigbee_devkit_product::profile::SensorProfile`, from
`products/esp32-zigbee-devkit`) and `zigbee_runtime::node::ZigbeeNode`, with:

- **On-chip temperature sensor** (via `esp_hal::tsens::TemperatureSensor` on
  C6; a small register-level TSENS driver on H2)
- **Crash-safe security-state journal** — network/security state persists
  across power cycles with a bounded frame-counter reservation (no re-pairing,
  no counter reuse after a crash)
- **Hardware-only AES** — all NWK/APS CCM* and Trust Center key derivation use
  the token-owned accelerator after two startup KATs
- **Unified lifecycle events** — incoming frames and periodic `tick()` results
  share one Joined/Commissioning/Rejoin/Leave/FactoryReset policy
- **Bounded parent recovery** — repeated poll `NoAck` outcomes request a secure
  rejoin; four consecutive failures discard stale credentials and commission
  cleanly
- **Silent persisted resume** — restored devices keep their identity and
  counters without repeating `Device_annce`
- **NWK Leave handler** — auto-erases state and rejoins when the coordinator
  sends Leave
- **Interview-aware reporting** — completion is read from the runtime's
  *remote* reporting record (`ZigbeeNode::remote_reporting_is_complete()`),
  which counts only Configure Reporting commands the stack accepted in full;
  after a bounded timeout the shared default reporting policy is installed
  instead and logged with the remote count (`remote configured n/m clusters`)
  so a fallback is never reported as a completed coordinator interview
- **Identify cluster** (0x0003) — supports Identify, IdentifyQuery, TriggerEffect
- **Battery percentage** reporting via Power Configuration cluster
- Join/leave button (BOOT / GPIO9)
- **OTA Upgrade client**, composed into the profile only when the checked
  partition table matches

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
    let mut mac = zigbee_mac::esp::EspMac::new(ieee802154, config);
    mac.install_aes_engine(peripherals.AES)
        .expect("ESP hardware AES KAT");
```

### Device Setup

The composition root builds the product's profile, then hands the endpoint
declaration straight through to the builder instead of listing clusters by
hand:

```rust
    use esp32_zigbee_devkit_product as product;
    use zigbee_runtime::profile::ApplicationProfile;
    use zigbee_zcl::clusters::basic::PowerSource;

    let security = product::storage::security_store();
    let profile = product::profile::sensor_profile(FIRMWARE_VERSION, esp_hal::system::software_reset);

    let device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer(product::MANUFACTURER)
        .model(product::MODEL)
        .sw_build(FIRMWARE_VERSION_STR)
        .power_source(PowerSource::Battery)
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(
            profile.endpoint(),
            profile.profile_id(),
            profile.device_id(),
            |ep| profile.configure_endpoint(ep),
        )
        .build();

    let node = zigbee_runtime::node::ZigbeeNode::new(device, security, profile);
```

(`device`/`security`/`profile` are given `'static` storage through
`static_cell::StaticCell` so `ZigbeeNode`'s borrow of all three stays valid
across the diverging `block_on` call, the same pattern the EFR32MG1 product
uses.)

### Main Loop

`SensorApp` (`src/app.rs` in each example) owns the event loop: button
presses (join/leave/factory-reset), the fast/slow poll window, periodic
sensor sampling, Device_annce retries, and driving the shared product-layer
`OtaTransport` over whatever backend the profile composed in. Initial and
retried commissioning goes through
`ZigbeeNode::start_or_resume`/`secure_rejoin`/`factory_reset`, which
`tick`/`process_incoming` alone do not perform.

Both event sources feed the same control dispatcher: a
`StackEvent::RejoinRequested` returned by a periodic tick is handled exactly
like one returned while processing an incoming frame. Announce retry timing
uses saturating time differences and refreshes the clock after asynchronous
rejoin, preventing the stale-`Instant` underflow class fixed on EFR32MG1.

The example's `OtaTransport` is only platform logging/poll-window policy. The
server lock, APS retry, cleanup, and activation-pending state are shared in
`zigbee_runtime::ota_transport::OtaSession`.

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

Both sensor examples persist Zigbee network state to the last two 4 KB
sectors of the physical flash chip (addresses `0x3FE000`-`0x3FFFFF`, 8 KB
total) — the same addresses used before any partition table existed, so
introducing one on the C6 build does not move already-joined network state.

`boards/esp32-zigbee-devkit` (the board crate) exposes only
`esp32_zigbee_devkit::flash::RawFlash`, whole-chip access wrapping the
official `esp_storage::FlashStorage` implementation of the standard
`embedded-storage` NOR traits — no partition, NV, or OTA policy, and no
dependency on `zigbee-runtime`.

`products/esp32-zigbee-devkit` (the `esp32-zigbee-devkit-product` crate)
bounds that raw flash to the reserved 8 KiB window and constructs
`zigbee_runtime::security_journal::SecurityStateJournal`, a crash-safe
two-sector journal: every commissioning/rejoin commit reserves a bounded block
of NWK frame-counter values up front, so a power loss between reservation and
use can never replay a previously used counter. This replaces the flat
`LogStructuredNv`-based item store the product used before the product/board
split (`ZigbeeDevice::save_state`/`restore_state`, documented as "not
suitable for production secured restore").

### One-time legacy migration

Because the journal reuses the *same* two sectors the legacy `LogStructuredNv`
item store occupied, a device already joined under the old firmware boots
facing legacy records where the journal expects its own. `open_security_store`
(in `products/esp32-zigbee-devkit`, called from each example's composition
root) runs a one-time, crash-safe migration before the store is handed to the
node:

* It prefers any already-committed journal, so reboots after the first
  migration — and after an interrupted one — are idempotent.
* Otherwise it reads the legacy NWK identity, network key and *live* frame
  counter, and commits a **commissioned** journal record: the device keeps its
  PAN, short address, network key and key sequence and resumes on its existing
  network instead of being treated as factory-new. Both counter floors (the
  NWK/global range and the range a unique TCLK would later use) sit strictly
  above the legacy counter — plus the legacy safety margin and a fresh
  reservation block — so no counter can be reused across the format switch.
* The migrated record is committed into the *scratch* (erased) sector first, so
  the authoritative legacy page is never erased before the new record is
  durable — a power loss leaves either the intact legacy record (retried next
  boot) or a valid journal (preferred), never a silent factory-new wipe.
* A flash read fault or an unparseable-but-present legacy region is surfaced as
  an error (the boot halts) rather than being mistaken for a factory-new
  device.

The legacy format never persisted a unique Trust Center Link Key. The legacy
runtime could negotiate one, but after reboot its restore path only recovered
the well-known *default global* TC link key, whose outgoing counter space is
the NWK frame counter (`Apsde::next_default_tc_link_key_frame_counter`). Every
unique-TCLK APS frame was also carried by a NWK-secured frame, so the migrated
NWK counter floor conservatively bounds either counter space. The migrated
record represents the recoverable post-reboot state explicitly with
`PersistentSecurityState::legacy_default_tclk`: commissioned, `tclk_present`
false, no APS key installed and no Trust Center address or key invented. The
runtime accepts that state on restore and keeps using the default global key
from the durable NWK counter space; as soon as the Trust Center delivers a
unique key, the reservation continues above the migrated floor and the marker
is cleared.

If a legacy record cannot describe a commissioned network — no persisted IEEE
address (NWK security nonces need it), or values the new format rejects — the
migration reports `MigratedCounters` instead: the counter floors are still
carried over and the device re-pairs once. It never fabricates the missing
identity, and never silently reuses counters.

`nwkUpdateId` is migrated with its validity. A legacy region that really holds
the `NwkUpdateId` item keeps that value as authoritative; one that does not
migrates as *unknown*, which journal record version 4 encodes explicitly (flags
bit 7) without changing the 98-byte state or the slot geometry. The network is
still kept (`MigratedNetwork`): an unknown update state simply means rejoin
parent selection rejects nothing as stale until the update state is learned
from the network, whereas migrating the absent item as a known `0` would make
every beacon advertising `0x81..=0xFF` look stale and could strand the device
off its own network.

The migration was exercised on an ESP32-H2 revision 1.2 on 2026-07-26. The
pre-flash `0x3FE000..0x3FFFFF` dump contained the legacy `0xA55A` item records.
After flashing without erasing NV, the product wrote committed `ZBSS`/`CMIT`
journal records into the other sector, restored PAN `0xDFE9`, short address
`0x44CB`, channel 15 and parent `0x1F0F`, and resumed with secured NWK counters
above the legacy value. The 2026-08-05 hardware-AES/event-policy image then
reported `JournalPresent`, restored the same identity without Beacon Request,
Association Request, or `Device_annce`, resumed secured polls/reports, and
updated the Home Assistant temperature, humidity, and battery entities. The
original legacy sector remained intact.

On boot, `ZigbeeNode::start_or_resume` checks for saved network state and, if
present, performs a secure silent rejoin instead of full commissioning. If the
coordinator sends a NWK Leave command, the device erases the security-state
journal and starts fresh commissioning.

## Partition Table

OTA needs two application slots, so the devkits are flashed with an explicit
ESP-IDF partition table, checked in at
`products/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv`:

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
* `esp32_zigbee_devkit_product::layout` mirrors these addresses, asserts the invariants
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
products/esp32-zigbee-devkit/partitions/build-partition-table.sh
```

The script uses `espflash partition-table --to-binary`, checks the size and
converts the result back to CSV to confirm the round trip.

## OTA Updates

> **Status:** implemented for both chips. ESP32-H2 completed a full v1-to-v2
> ZHA download, SHA verification, `ota_1` activation, reboot, and preserved
> network resume. ESP32-C6 has exercised the same path through 18.3% of a
> transfer; complete C6 activation remains pending. Migrating a device to the
> two-slot layout rewrites its partition table as a deliberate, separate step.

### How it works

`esp32_zigbee_devkit_product::ota::EspFirmwareWriter` implements the runtime's
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
cd examples/esp32c6-sensor       # or examples/esp32h2-sensor
tools/create-ota.py 2             # writes target/ota/
```

The tool builds the firmware with `ESP32_OTA_VERSION=2`, runs
`espflash save-image`, wraps the result in an OTA container
(manufacturer `0x1234`, image type `0x0001` for C6 or `0x0002` for H2,
hardware version 1), parses the container back and compares it byte for byte
with the image, and regenerates a `zigpy_local` index with the sha3-256
checksums ZHA verifies:

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
     --partition-table ../../products/esp32-zigbee-devkit/partitions/esp32-4mb-ota.csv \
     --target-app-partition ota_0 \
     --erase-parts otadata \
     --monitor target/riscv32imac-unknown-none-elf/release/esp32c6-sensor
   ```

   The configured `cargo run --release` runner executes the same command.
   Use the corresponding H2 example directory and `esp32h2-sensor` binary for
   an ESP32-H2 board.
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
| Startup stops at hardware AES | KAT, clock, or accelerator handshake failed | Do not bypass it; inspect the AES clock/reset and register state |
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
