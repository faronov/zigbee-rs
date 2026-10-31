# Nordic nRF52840 / nRF52833

Nordic sensor products use Embassy's chip HAL, the shared software MAC
(`SoftMacCore<NrfRadioPhy>`, alias `zigbee_mac::nrf::NrfSoftMac`), board
resource crates, product-owned profiles/policy/storage, and
`sensor_sed_app::SensorApp`.

## Layering

```text
environmental profile / always-on End Device behavior
        ↓
nrf52840-sensor / nrf52833-sensor / nrf52840-router product
        ↓
nrf52840-dk / nrf52833-dk / UF2 board crate
        ↓
zigbee_mac::nrf::NrfSoftMac = SoftMacCore<NrfRadioPhy>
        ↓
embassy-nrf RADIO / RNG / ECB
```

The short example root constructs and connects these layers.

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
    actions: nrf52840_sensor_product::policy::USER_ACTIONS,
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
and matching product profile composition. The `zigbee-bme280`, `zigbee-sht3x`
and `zigbee-sensor` crates come from
[`zigbee-rs-drivers`](https://github.com/faronov/zigbee-rs-drivers) as git
dependencies pinned by `rev`.

### UF2 boards

`examples/nrf52840-sensor-uf2` supports, one Cargo feature each:

- `board-promicro` (default): nice!nano-compatible ProMicro;
- `board-mdk`: Makerdiary MDK USB Dongle;
- `board-nrf-dongle`: Nordic PCA10059;
- `board-nrf-dk`: Nordic PCA10056 DK.

Each feature selects a board crate and a product linker layout. Preserve the
resident bootloader/SoftDevice regions and always package ELF → Intel HEX →
UF2; do not infer an address from a flat binary.

## Persistence and identity

| product | application end | security journal |
|---|---:|---:|
| nRF52840 DK sensor/always-on End Device | `0x000FE000` | `0x000FE000..0x00100000` |
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

The image uses the end-device `NrfSoftMac`, which does not implement
`ParentMacDriver` (the `Parent` role of `SoftMacCore` exists but has not been
validated on nRF hardware), so the image cannot construct
`ParentRouterApp`, admit a child, maintain pending transactions, provide
indirect delivery, or advertise Router/Link Status behavior. It is therefore
a receiver-on, always-on End Device.

Button 1 is product-mapped in `nrf52840_router_product::button`: a short
press calls `request_commissioning()` (the only automatic-search trigger,
besides a power cycle, after a Leave without rejoin), and a three-second hold
commits the journal-aware factory reset.

This composition uses Embassy's pinned-nightly static task allocation. The
compiler emits storage for the actual main future; it no longer tries to
allocate that future from the insufficient default 4 KiB arena. The linker
also requires at least 16 KiB between static storage and the initial stack
pointer. This is a linked reserve, not a measured stack high-water mark.

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

cd ../nrf52840-sensor-uf2
cargo +nightly-2026-03-23 build --release --locked   # board-promicro
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-mdk
```

Measured release images on 2026-09-06, before the move to `SoftMacCore`:

| image | bytes | former regression budget |
|---|---:|---:|
| nRF52840 default / BME280 / SHT31 | 224,472 / 231,792 / 228,216 | 225,280 / 245,760 / 241,664 |
| nRF52833 default / BME280 / SHT31 | 224,464 / 231,784 / 228,208 | 225,280 / 245,760 / 241,664 |
| nRF52840 always-on End Device | 210,072 | 253,952 |
| UF2 ProMicro / MDK / PCA10059 / DK | 222,968 / 222,848 / 224,536 / 224,552 | 237,568 each |

The `SoftMacCore` port reduced sensor flash by 352–724 B and router flash by
636 B; see the MAC unification section of the
[firmware-size chapter](../advanced/firmware-size.md). The former budgets are
historical comparisons, not enforced thresholds.
CI still measures image bytes and checks physical memory limits, product
partition symbols, hardware-AES symbols, and role-specific symbol removal.

## Validation

The exact images above are build/layout-tested. Earlier nRF52840 and nRF52833
sensor images, running the since-deleted `NrfMac`, produced hardware evidence
for commissioning, interview/reporting, hardware AES, security persistence, and
reset/resume. The current `SoftMacCore<NrfRadioPhy>` images (commit
`786e209`) are build- and host-tested only; no nRF hardware has run them yet,
so ACK timing, polling, and orphan scan on this path remain a hardware gate.

The nRF52840 always-on End Device compiles and passes role/layout checks. Its
complete HIL acceptance—commissioning/resume, continuous receive, reset, and
proof that it does not advertise router behavior—remains open.
