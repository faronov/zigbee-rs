# Nordic nRF52840 / nRF52833

Nordic sensor products use Embassy's chip HAL, the shared `NrfMac`, board
resource crates, product-owned profiles/policy/storage, and
`sensor_sed_app::SensorApp`.

## Layering

```text
embassy-nrf + zigbee_mac::nrf::NrfMac
        ↓
nrf52840-dk / nrf52833-dk / UF2 board crate
        ↓
nrf52840-sensor or nrf52833-sensor product
        ↓
short sensor composition root
```

`apps/nrf-sensor` contains Nordic capability adapters such as
`NrfWakeController`, `NrfStatus`, `NrfBattery`, diagnostics, and supervisor.
It delegates lifecycle behavior to `apps/sensor-sed`; every Nordic sensor
composition root constructs that lifecycle directly.

## Sensor composition

Nordic sensor roots construct explicit parts:

```rust,ignore
SensorSedParts {
    wake: NrfWakeController::new(button),
    status: NrfStatus::new(led),
    environment,
    battery: NrfBattery::<Battery>::new(saadc),
    ota: NoOta,
    actions: ForceReportAction,
    supervisor: NrfSupervisor,
    diagnostics: NrfDiagnostics,
}
```

Both product policies select:

```text
fast wait: Idle
slow wait: Idle
```

The wake adapter quiesces the radio and uses a retained System-ON wait. This is
not System OFF and no battery-life estimate is inferred from the build.
Profiles are explicitly non-OTA and pair with `NoOta`.

The device builder disables automatic polling so `SensorApp` is the only
parent-poll owner.

## Boards

### Development kits

| board | MCU | status | action | sensor I²C |
|---|---|---|---|---|
| PCA10056 | nRF52840 | LED1 P0.13, active low | Button 1 P0.11 | P0.26/P0.27 |
| PCA10100 | nRF52833 | LED1 P0.13, active low | Button 1 P0.11 | P0.26/P0.27 |

The default profile uses the on-chip temperature source plus synthetic
humidity. `sensor-bme280` and `sensor-sht31` select external sensor adapters
and matching product profile composition.

### UF2 boards

`examples/nrf52840-sensor-uf2` supports:

- nice!nano-compatible ProMicro;
- Makerdiary MDK USB Dongle;
- Nordic PCA10059;
- Nordic PCA10056 DK.

Each feature selects a board crate and a product linker layout. Preserve the
resident bootloader/SoftDevice regions and always package ELF → Intel HEX →
UF2; do not infer an address from a flat binary.

## Persistence and identity

| product | application end | security journal |
|---|---:|---:|
| nRF52840 DK sensor/router | `0x000FE000` | `0x000FE000..0x00100000` |
| nRF52833 sensor | `0x0007E000` | `0x0007E000..0x00080000` |

The product constructs `SecurityStateJournal` over Embassy NVMC. FICR-derived
EUI-64 is checked before resume; a changed identity clears incompatible
persisted membership rather than using keys/counters under another address.

Nordic ECB hardware AES is token-owned. Both startup known-answer tests must
pass; production images do not silently fall back to software AES.

## Always-on End Device

`examples/nrf52840-router` uses:

```rust,ignore
ZigbeeDevice::builder(mac)
    .device_type(DeviceType::EndDevice)
    .power_mode(PowerMode::AlwaysOn)
    .build();
AlwaysOnEndDeviceApp::new(node, policy, parts)?;
```

`NrfMac` does not implement `ParentMacDriver`, so the image cannot construct
`ParentRouterApp`, admit a child, maintain pending transactions, provide
indirect delivery, or advertise Router/Link Status behavior. It is therefore
a receiver-on, always-on End Device.

## Build

All Nordic images use `nightly-2026-03-23`:

```bash
cd examples/nrf52840-sensor
cargo +nightly-2026-03-23 build --release --locked
cargo +nightly-2026-03-23 build --release --locked --features sensor-bme280
cargo +nightly-2026-03-23 build --release --locked --features sensor-sht31

cd ../nrf52833-sensor
cargo +nightly-2026-03-23 build --release --locked

cd ../nrf52840-router
cargo +nightly-2026-03-23 build --release --locked
```

Measured raw images:

| image | bytes |
|---|---:|
| nRF52840 default / BME280 / SHT31 | 223,344 / 230,560 / 227,040 |
| nRF52833 default / BME280 / SHT31 | 223,400 / 230,976 / 227,128 |
| nRF52840 always-on End Device | 214,864 |
| UF2 board variants before container | 221,736–223,456 |

CI also checks the product partition symbols, hardware-AES symbols, and
role-specific symbol removal.

## Validation

The nRF52840 and nRF52833 sensor paths are hardware-proven for commissioning,
interview/reporting, hardware AES, security persistence, and reset/resume.

The nRF52840 always-on End Device compiles and passes role/layout checks. Its
complete HIL acceptance—commissioning/resume, continuous receive, reset, and
proof that it does not advertise router behavior—remains open.
