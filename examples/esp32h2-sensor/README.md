# ESP32-H2 Zigbee sensor

`no_std` environmental sleepy-end-device profile for ESP32-H2. The product
uses the shared `SensorApp`, an active-low status LED, product-owned security
storage/OTA, and `Active` for both fast and slow waits.

## Composition

`main.rs` owns ESP startup, radio/AES resources, BOOT input, LED, on-chip
temperature adapter, flash, and `block_on`. It constructs explicit
`SensorSedParts` and hands commissioning, parent polling, reporting,
persistence, and OTA-first routing to `apps/sensor-sed`.

The parts include:

- `ActiveWake`;
- active-low semantic status;
- H2 temperature + synthetic humidity;
- fixed battery;
- product `OtaTransport`;
- toggle-join action, reset supervisor, and diagnostics.

## Product flash layout

```text
0x009000..0x00B000  otadata
0x010000..0x200000  ota_0
0x200000..0x3F0000  ota_1
0x3F0000..0x400000  zbnv
0x3FE000..0x400000  SecurityStateJournal
```

`WithOta` always advertises the OTA cluster. Startup fails explicitly if the
product cannot verify this table. The writer cannot enter `zbnv`.

## Build and flash

```bash
cd examples/esp32h2-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc
cargo +nightly-2026-08-01 run --release --locked -Z build-std=core,alloc
```

Use `espflash 4.5.0`. The configured runner writes `ota_0` and clears
`otadata`. Current application image: **339,568 B**.

## OTA

```bash
tools/create-ota.py 2
```

The product writer stages the inactive slot, verifies the ESP image and
appended SHA-256, and changes boot selection only after `SensorApp`
checkpoints Zigbee keys/counters.

## Hardware validation

Hardware-proven on an ESP32-H2 revision 1.2 with 4 MiB flash:

- migration from legacy NV to `SecurityStateJournal`;
- secure reset/resume, interview, reporting, and Identify;
- complete v1→v2 ZHA OTA transfer;
- staging in `ota_1`, verification, activation, and reboot;
- retained IEEE address, PAN, parent, keys, and monotonic counters.

Fresh factory-reset commissioning and long-duration power behavior remain
separate gates. This product currently makes no low-power sleep claim.
