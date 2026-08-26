# ESP32-C6 / ESP32-H2

The ESP products share a board flash resource, product profile/storage/OTA
implementation, and the platform-independent sleepy-sensor lifecycle.

## Layering

```text
esp-hal + esp-radio + EspMac
        ↓
boards/esp32-zigbee-devkit (physical flash resource)
        ↓
products/esp32-zigbee-devkit (profile, policy, partitions, OTA)
        ↓
ESP32-C6 or ESP32-H2 composition root
```

The example root owns chip startup, radio resources, button/LED/temperature
adapters, and `block_on`. `SensorApp` owns commissioning, parent polling,
reporting, persistence checkpoints, and OTA-first event routing.

## Application parts and policy

Both images provide a concrete
`esp32_zigbee_devkit_product::ota_transport::OtaTransport` paired with an
`WithOta` profile.

| product | status | fast wait | slow wait |
|---|---|---|---|
| ESP32-C6 | `NoStatus` | `Active` | `Active` |
| ESP32-H2 | active-low fitted LED | `Active` | `Active` |

Neither image claims low-power sleep. `Active`/`Active` is an explicit product
choice, not a missing default.

OTA cluster events reach `OtaTransport` before generic application handling.
When the image is ready, `SensorApp` checkpoints Zigbee security state before
the product writer changes `otadata` and resets.

## Product-owned flash layout

The checked 4 MiB table is:

```text
0x000000..0x008000  second-stage bootloader
0x008000..0x008C00  partition table
0x009000..0x00B000  redundant otadata sectors
0x010000..0x200000  ota_0
0x200000..0x3F0000  ota_1
0x3F0000..0x400000  zbnv
0x3FE000..0x400000  security journal within zbnv
```

The board crate exposes raw physical flash. The product checks the table,
bounds writes, selects the inactive slot, owns `otadata`, and constructs the
security journal.

If the on-device table is missing or incompatible, startup fails explicitly.
These OTA-capable images always advertise the OTA client cluster and do not
pretend an unsafe staging path exists.

The writer:

- erases staging sectors lazily;
- pads the final 4-byte write with `0xFF`;
- verifies image magic, chip ID, and appended SHA-256;
- writes one redundant `otadata` entry for activation.

## Build and flash

Use the fixed `nightly-2026-08-01` and `espflash 4.5.0`:

```bash
cd examples/esp32c6-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc

cd ../esp32h2-sensor
cargo +nightly-2026-08-01 build --release --locked -Z build-std=core,alloc
```

The configured runner installs the product partition table, writes `ota_0`,
and clears `otadata`:

```bash
cargo +nightly-2026-08-01 run --release --locked -Z build-std=core,alloc
```

Back up commissioned state before the first partition-table migration.

Measured application images:

| image | bytes | static `.data + .bss` |
|---|---:|---:|
| ESP32-C6 | 354,512 | 52,268 |
| ESP32-H2 | 339,568 | 51,848 |

The corresponding merged flash images are 420,048 B and 405,104 B.

## OTA packaging

From the selected example:

```bash
tools/create-ota.py 2
```

The tool builds the application with `ESP32_OTA_VERSION=2`, creates the Zigbee
OTA container, and updates a `zigpy_local` index. C6 and H2 use distinct image
types, so a server cannot offer one chip's image to the other.

## Hardware validation

### ESP32-H2

Hardware-proven on a 4 MiB ESP32-H2 revision 1.2 path:

- migration to the security journal;
- secure reset/resume and reporting;
- complete v1→v2 ZHA OTA transfer;
- verification, activation, reboot into `ota_1`;
- retained IEEE address, PAN, parent, network credentials, and counters.

Fresh factory-reset commissioning and long-duration power behavior remain
separate gates.

### ESP32-C6

Commissioning/reporting and the OTA transfer path are hardware-tested. A
v1→v2 image reached 18.3% before intentional cancellation. Complete C6
verification/activation/reboot remains open.
