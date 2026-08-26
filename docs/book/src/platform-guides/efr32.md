# Silicon Labs EFR32

The repository supports two distinct products:

- EFR32MG1P TRÅDFRI environmental sleepy end device;
- EFR32MG21 BRD4181A development sensor.

Both use the shared `SensorApp`; their boards, products, clocks, storage, and
power behavior are intentionally separate.

## EFR32MG1P TRÅDFRI

### Layering and fitted hardware

```text
efr32mg1-hal + Efr32Mac
        ↓
boards/efr32mg1-tradfri
        ↓
products/efr32mg1-tradfri
        ↓
examples/efr32mg1-sensor
```

The typed board resources keep fitted peripherals independently testable:

| resource | wiring |
|---|---|
| direct or TIMER0 PWM LED | PA0, active high |
| user button | PB13, active low |
| SHT3x I²C | I2C0, PC10 SDA / PC11 SCL, 10 kHz |
| supply measurement | ADC0 AVDD |
| direct external flash | USART0: PD13 CLK, PD14 MISO, PD15 MOSI, PB11 CS |
| wake timer | RTCC/LFRCO |

Direct USART0 access and Gecko Bootloader storage access consume alternative
owners of the same external-flash path. They cannot coexist through the typed
board API.

The product owns identity, profile, battery chemistry, sensor mapping, policy,
linker regions, persistence, and Gecko Bootloader OTA selection.

### Power and lifecycle

The product policy selects:

```text
fast wait: Active
slow wait: Retention
```

Fast commissioning/interview/OTA windows remain active. Joined steady-state
polls use RTCC/LFRCO wake and EM2 after the platform has quiesced the radio and
applied the Series-1 DCDC safety gate. PB13 remains an external wake source.

The shared app routes OTA first, checkpoints security, and only then calls the
product activation backend.

### Memory

The product linker layout preserves bootloader, application, generic NV, and
security regions. The usable SRAM region is exactly `0x7C00` bytes. It is not
the nominal rounded 32 KiB total.

Current release measurement:

| value | bytes |
|---|---:|
| raw image | 156,612 |
| static `.data + .bss` | 14,912 |

### Validation

Hardware-proven:

- commissioning and security;
- CRYPTO hardware AES;
- ZHA interview/reporting;
- SHT3x and supply/battery measurement;
- Identify and button behavior;
- crash-safe persistence and reset/resume;
- RTCC wake and EM2.

Still open: a real Zigbee OTA download, bootloader install, reboot into the new
version, and retained commissioned state.

## EFR32MG21 BRD4181A

### Exact target

| item | value |
|---|---|
| radio board | BRD4181A |
| main board | BRD4001A |
| MCU | EFR32MG21A020F512IM32 |
| HFXO | 38.4 MHz |
| HFXO CTUNE | 133 |
| LED0 | PB0, active high |
| BTN0 | PD2, active low |

These are the fitted BRD4181A pins. Other Series-2 kit pin maps do not apply.
BRD4001A provides BTN0's external bias, so the board adapter uses no internal
pull and routes PD2 through EXTI line 2.

### Composition and power

```text
efr32mg21-hal + Efr32s2Mac
        ↓
boards/efr32mg21-devkit
        ↓
products/efr32mg21-sensor
        ↓
examples/efr32mg21-sensor
```

The product uses a non-OTA environmental profile, `NoOta`, synthetic
temperature/humidity, a fixed 3000 mV battery value, and `Idle` for both fast
and slow waits.

`Idle` is radio-gated WFE driven by the real 1 kHz SysTick. It is not EM2 and
no deep-sleep current is claimed.

### Memory layout

```text
0x00000000..0x00004000  bootloader
0x00004000..0x0007C000  application
0x0007C000..0x00080000  persistence
```

The 16 KiB persistence window is two 8 KiB security-journal sectors.

Current release measurement:

| value | bytes |
|---|---:|
| raw image | 203,180 |
| static `.data + .bss` | 17,136 |

### Build

Both products use `nightly-2026-03-23`:

```bash
cd examples/efr32mg1-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor

cd ../efr32mg21-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor
```

MG21 currently passes compile, clippy, image, linker, and persistence-layout
checks. Its complete startup/radio/join/flash/power path remains
HIL-unverified.
