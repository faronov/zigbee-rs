# nRF52840 Zigbee Sensor — UF2 Deployments

This is a thin nRF52840 composition of the shared `sensor-sed-app`.
Commissioning, manual parent polling, reporting, rejoin, and durable factory
reset are owned by that application archetype. The product crate owns the
Zigbee profile, policy, and security journal; physical board crates own pins
and signal polarity.

The root constructs explicit `SensorSedParts`, uses `Idle` for both fast and
slow waits, disables automatic polling, and pairs the non-OTA profile with
`NoOta`.

## Board contracts

Select exactly one feature:

| Feature | Physical board | Status LED | User action | Application | Security journal |
|---|---|---|---|---:|---:|
| `board-promicro` | nice!nano-compatible ProMicro | P0.15, active high | none | `0x26000..0xEE000` | `0xEE000..0xF0000` |
| `board-mdk` | Makerdiary MDK USB Dongle | P0.22 green, active low | none | `0x01000..0xF2000` | `0xF2000..0xF4000` |
| `board-nrf-dongle` | Nordic PCA10059 | P0.06, active low | SW1/P1.06 | `0x01000..0xF2000` | `0xF2000..0xF4000` |
| `board-nrf-dk` | Nordic PCA10056 DK | P0.13, active low | Button 1/P0.11 | `0x00000..0xFE000` | `0xFE000..0x100000` |

The first three maps require the Adafruit nRF52 bootloader contract:

- bootloader/config: `0xF4000..0xFE000`
- MBR parameters/settings: `0xFE000..0x100000`
- ProMicro additionally preserves S140 below `0x26000` and
  `0xF0000..0xF4000` as a guard.

Do not use the MDK or PCA10059 feature with an unknown factory bootloader.
Verify its installed bootloader/UICR boundaries first. The DK map deliberately
has no resident bootloader and is intended for probe/on-board-debugger
flashing.

## Build

The CI-pinned compiler is `nightly-2026-03-23`:

```sh
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-promicro
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-mdk
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-nrf-dongle
cargo +nightly-2026-03-23 build --release --locked \
  --no-default-features --features board-nrf-dk
```

`board-promicro` is the default when no feature flags are supplied.
The four current linked images measure **221,736–223,456 B** before UF2
container overhead.

## Package from Intel HEX

Always convert **ELF → Intel HEX → UF2**. Intel HEX retains the linked
addresses and excludes ELF metadata-only load segments. Do not convert a
sparse ELF to BIN and then guess or prepend a base address.

```sh
ELF=target/thumbv7em-none-eabihf/release/nrf52840-sensor-uf2
arm-none-eabi-objcopy -O ihex "$ELF" firmware.hex
python3 /path/to/microsoft-uf2/utils/uf2conv.py \
  -c -f 0xADA52840 firmware.hex -o firmware.uf2
```

For a compatible UF2 installation, enter its bootloader and copy
`firmware.uf2` to the mass-storage volume. UF2 updates must preserve the
selected two-page security journal; confirm that behavior on the installed
bootloader before relying on persistence.

## Runtime behavior

- External 32 MHz crystal and Nordic DC/DC configuration.
- Nordic ECB hardware AES with startup dual known-answer tests; networking
  halts on failure.
- FICR-derived IEEE identity guard clears stale persisted network state if the
  chip identity changes.
- Exactly one parent-poll owner:
  `automatic_polling(false)` plus `sensor-sed-app` manual polling.
- Crash-safe two-sector `SecurityStateJournal`; `NoOta` is explicit.
- ProMicro disables S140 before Embassy claims peripherals and halts if that
  operation fails.
- PCA10059/DK short press forces a report; a three-second hold performs a
  durable factory reset. ProMicro/MDK use timer-only wake with no button
  action.
- The polarity-aware LED reports semantic joining, reporting, identifying,
  reset, and fault states.
