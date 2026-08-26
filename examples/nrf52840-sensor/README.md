# nRF52840 Zigbee sensor

`no_std` environmental sleepy end device for the nRF52840-DK (PCA10056).

## Architecture

```text
embassy-nrf + NrfMac
        ↓
boards/nrf52840-dk
        ↓
products/nrf52840-sensor
        ↓
this composition root
        ↓
sensor_sed_app::SensorApp
```

`main.rs` initializes clocks, Nordic radio/RNG/AES, board resources, product
profile, and the product-owned security journal. Commissioning, parent
polling, reporting, Identify, reset/rejoin, and persistence checkpoints live
in `apps/sensor-sed`.

The root supplies explicit `SensorSedParts`:

- `NrfWakeController`;
- semantic LED status;
- selected environment source;
- SAADC battery adapter;
- `NoOta`;
- force-report user action;
- Nordic supervisor and diagnostics.

The product policy uses `Idle` for both fast and slow waits. This is a retained
System-ON/radio-off wait, not System OFF. No battery-life estimate is claimed
from the build alone.

## Sensor variants

| feature | source | clusters |
|---|---|---|
| default | on-chip TEMP + synthetic humidity | temperature, humidity |
| `sensor-bme280` | BME280 on P0.26/P0.27 | temperature, humidity, pressure |
| `sensor-sht31` | SHT31 on P0.26/P0.27 | temperature, humidity |

All variants include Basic, Power Configuration, Identify, crash-safe security
persistence, and the product's reporting defaults.

## Board controls

- LED1 P0.13, active low: semantic status.
- Button 1 P0.11, active low:
  - short joined press: force a measurement/report;
  - short unjoined press: immediate join attempt;
  - three-second hold: durable factory reset and reboot.

## Security and storage

- FICR-derived EUI-64 with persisted identity guard.
- Nordic ECB hardware AES; two startup KATs, no software fallback.
- Security journal: `0x000FE000..0x00100000`.
- Automatic polling is disabled; `SensorApp` is the single parent-poll owner.
- The non-OTA profile is explicitly paired with `NoOta`.

## Build and flash

Use the CI-pinned compiler:

```bash
cd examples/nrf52840-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 build --release --locked --features sensor-bme280
cargo +nightly-2026-03-23 build --release --locked --features sensor-sht31

probe-rs run --chip nRF52840_xxAA \
  target/thumbv7em-none-eabihf/release/nrf52840-sensor
```

Measured raw images:

| variant | bytes |
|---|---:|
| default | 223,344 |
| BME280 | 230,560 |
| SHT31 | 227,040 |

## Validation

Hardware-proven on nRF52840:

- commissioning and secure resume;
- Nordic hardware AES;
- ZHA interview, Identify, and reporting;
- security journal persistence and reset/resume;
- on-chip and external-sensor application variants.

Current power policy and code compile successfully, but this README does not
claim a measured sleep current or battery lifetime.
