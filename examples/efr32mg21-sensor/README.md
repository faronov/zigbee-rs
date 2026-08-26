# EFR32MG21 sensor (BRD4181A + BRD4001A)

Compile-tested `no_std` composition of the shared environmental
`SensorApp`.

## Exact target

| item | value |
|---|---|
| MCU | EFR32MG21A020F512IM32 |
| HFXO | 38.4 MHz, CTUNE 133 |
| LED0 | PB0, active high |
| BTN0 | PD2, active low |

The WSTK provides the button bias. These are the current BRD4181A pins.

## Application

- `DeviceProfile<TemperatureHumidityBattery>`;
- synthetic temperature/humidity;
- fixed 3000 mV battery;
- `NoOta`;
- short press toggles join/leave;
- three-second hold requests durable factory reset;
- `Idle` for both fast and slow waits.

`Idle` gates the radio and uses WFE with the real 1 kHz SysTick. It is not EM2.
`Retention` is unsupported.

## Ownership

```text
efr32mg21-hal + Efr32s2Mac
        ↓
boards/efr32mg21-devkit
        ↓
products/efr32mg21-sensor
        ↓
this root + sensor_sed_app::SensorApp
```

The product owns identity, profile, policy, bounded storage, and linker map.
The board owns PB0/PD2, clocks, and physical flash.

## Layout

```text
0x00000000..0x00004000  bootloader
0x00004000..0x0007C000  application
0x0007C000..0x00080000  security persistence
0x20000000..0x20010000  SRAM
```

## Build

```bash
cd examples/efr32mg21-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor
```

Current raw image: **203,180 B**.

## Validation

Compile, clippy, linker, image, and persistence-layout checks pass. The
complete startup/radio/factory-EUI/join/reporting/flash/button/idle path remains
HIL-unverified.
