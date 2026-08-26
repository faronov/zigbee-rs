# nRF52833 Zigbee sensor

`no_std` environmental sleepy end device for the nRF52833-DK (PCA10100).
It uses the same platform-independent `apps/sensor-sed` lifecycle and product
profile archetypes as the nRF52840 sensor.

The root constructs `sensor_sed_app::SensorApp` directly from its Nordic
capability adapters and `SensorSedParts`; it does not own a separate
commissioning, polling, or reporting state machine.

## Layer ownership

| concern | owner |
|---|---|
| radio, RNG, ECB, SAADC, TEMP, NVMC | `embassy-nrf` / `zigbee-mac` |
| LED1 P0.13, Button 1 P0.11, I²C P0.26/P0.27 | `boards/nrf52833-dk` |
| identity, profile, battery curve, wait policy, linker/storage | `products/nrf52833-sensor` |
| Zigbee sensor lifecycle | `apps/sensor-sed` |
| startup and resource construction | this example |

The product uses `Idle` for both fast and slow waits and explicitly selects
`NoOta`.

## Sensor variants

```bash
cd examples/nrf52833-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 build --release --locked --features sensor-bme280
cargo +nightly-2026-03-23 build --release --locked --features sensor-sht31
```

Flash with:

```bash
probe-rs run --chip nRF52833_xxAA \
  target/thumbv7em-none-eabihf/release/nrf52833-sensor
```

Measured images:

| variant | bytes |
|---|---:|
| default | 223,400 |
| BME280 | 230,976 |
| SHT31 | 227,128 |

## Storage and security

- Application: `0x00000000..0x0007E000`.
- Security journal: `0x0007E000..0x00080000`.
- FICR-derived EUI-64 and identity mismatch guard.
- Nordic ECB hardware AES with two startup KATs.
- Automatic polling disabled; the shared app owns parent polls.

## Operation

- cold boot commissions automatically when no state exists;
- valid state resumes without reopening pairing;
- short press forces a report or immediate join attempt;
- three-second hold performs a durable factory reset;
- default/BME280/SHT31 profiles report only their statically selected clusters.

## Validation

Commissioning, ZHA interview/reporting, hardware AES, security persistence,
and reset/resume are hardware-proven. A release build is not a measured
current or battery-life claim.
