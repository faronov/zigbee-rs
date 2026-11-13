# ESP32-C6 Zigbee sensor

`no_std` environmental sleepy-end-device profile on an ESP32-C6 DevKit.
The current product intentionally uses `Active` for both fast and slow waits;
it makes no low-power sleep claim.

The DevKitC-1 LED is a WS2812 on GPIO8. The sensor does not drive it; the
router bin always shows status on it (`status-led` is part of the
`router` feature); the ED takes `--features ed,status-led`; see the ESP32
platform guide.

The `router` feature also enables the Green Power Proxy Basic (default-on
for every router; see the Green Power chapter); `router-core` builds the
router without it.

## Architecture

```text
environmental OTA profile
        ↓
products/esp32-zigbee-devkit
        ↓
boards/esp32-zigbee-devkit
        ↓
esp-hal + esp-radio + EspRadioPhy + SoftMacCore
```

This uses the real pure-Rust `esp-radio` IEEE 802.15.4 backend, not a
scaffold. `main.rs` is the composition root.

The root supplies:

- active timer/button wake;
- `NoStatus` (no fitted status indicator in this product);
- on-chip temperature and synthetic humidity;
- fixed battery;
- product `OtaTransport`;
- toggle-join action, supervisor, and diagnostics.

`NoStatus` removes status-only waits. OTA is represented by `WithOta`, so
cluster `0x0019` is always advertised. Startup fails explicitly if the
partition table is incompatible. OTA events are routed before generic sensor
events, and activation occurs only after the shared application checkpoints
security state.

## Flash layout

```text
0x009000..0x00B000  otadata
0x010000..0x200000  ota_0
0x200000..0x3F0000  ota_1
0x3F0000..0x400000  zbnv
0x3FE000..0x400000  SecurityStateJournal
```

The board exposes raw flash; the product checks and owns this layout.

## Build and flash

Use the fixed CI toolchain and `espflash 4.5.0`:

```bash
cd examples/esp32c6-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc
cargo +nightly-2026-08-01 run --release --locked -Z build-std=core,alloc
```

The configured runner installs the partition table, writes `ota_0`, and clears
`otadata`. Back up commissioned state before the first layout migration.

Recorded application image: **368,032 B** against the former **368,640 B**
regression budget, which is no longer enforced. The merged flash image is
**433,568 B**, the Zigbee OTA v2 container is
**368,098 B**, the OTA slot is **2,031,616 B**, and static `.data + .bss` is
**53,268 B**.

Physical partition, OTA-slot, RAM, and image/layout checks remain mandatory.

## OTA image

```bash
tools/create-ota.py 2
```

The product stages into the inactive slot, erases lazily, verifies chip ID and
the appended SHA-256, then writes `otadata` only after the security checkpoint.

## Validation

The recorded application and package artifacts are
build/layout/package-tested. Earlier ESP32-C6 images demonstrated:

- hardware AES KAT and secured commissioning/reporting;
- migration to the crash-safe security journal;
- OTA Image Notify/query/block transfer and writes into `ota_1`;
- transfer progress through 18.3% before intentional cancellation.

On 2026-11-13 a signed TEST-ONLY build (`light-sleep,signed-ota`, built with
`tools/create-esp32-ota.py --sign-key`) completed a ZHA upgrade from 0.2.0 to
0.2.1 on one DevKit:

- a 419,640 B container, downloaded in about 43 min, with two parent-loss
  rejoins during the download that kept the address and staged data;
- `slot 1 verified`, `otadata` sector 1 seq 2, reboot into `ota_1`;
- pending verify, then `running image confirmed` after the resume, with no
  rejoin;
- the same NWK address `0xFA41` and forward-only frame counters.

Details and observations are in
[the bootloader results](../../docs/book/src/advanced/bootloader.md#esp32-c6-signed-zha-ota-2026-11-13).
The signed test build is about +13.0 KB flash and +352 B `.bss` against the
unsigned light-sleep image. The default unsigned image, sleep current,
power-loss recovery and rollback have no C6 hardware run.
