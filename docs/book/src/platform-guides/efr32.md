# Silicon Labs EFR32

The repository supports two distinct products:

- EFR32MG1P TRÅDFRI environmental sleepy end device;
- EFR32MG21 BRD4181A development sensor.

Both use the shared `SensorApp`; their boards, products, clocks, storage, and
power behavior are intentionally separate.

## EFR32MG1P TRÅDFRI

### Layering and fitted hardware

```text
environmental OTA profile
        ↓
products/efr32mg1-tradfri
        ↓
boards/efr32mg1-tradfri
        ↓
efr32mg1-hal + Efr32Mac
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
It selects the orthogonal `compact-single-endpoint` capacity only after
profile assertions prove that two application-endpoint slots and four
reporting entries are sufficient; no Zigbee behavior is removed.

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

Recorded measurement (2026-09-08):

| value | bytes |
|---|---:|
| raw image | 163,236 |
| former regression budget (not enforced) | 167,936 |
| `.data` | 260 |
| `.bss` | 14,720 |
| static total | 14,980 |
| available linked stack | 16,760 |

The former image-size budget is no longer enforced; physical Flash/RAM,
protected-partition, OTA, and stack checks remain mandatory.
The stack has 376 B above the 16 KiB gate. The previous 162,538 B Zigbee OTA
container has not been regenerated for this image. OTA packaging assumes
the resident Gecko bootloader.

### Validation

The exact raw image above is build/layout-tested, not OTA-qualified. Earlier
TRÅDFRI images produced hardware evidence for:

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
environmental non-OTA profile
        ↓
products/efr32mg21-sensor
        ↓
boards/efr32mg21-devkit
        ↓
efr32mg21-hal + Efr32s2Mac
```

The product uses a non-OTA environmental profile, `NoOta`, synthetic
temperature/humidity, a fixed 3000 mV battery value, and `Idle` for both fast
and slow waits.

`Idle` is radio-gated WFE driven by the real 1 kHz SysTick. It is not EM2 and
no deep-sleep current is claimed.

The Embassy timebase is **1 MHz**, distinct from the 1 kHz interrupt cadence:
each SysTick advances it by 1,000 ticks, with microsecond interpolation between
interrupts. Clock reads account for a pending reload and retry if it occurs
during sampling. They retain a nondecreasing timestamp if multiple exceptions
coalesce while interrupts are masked; SysTick cannot reconstruct the lost
masked time. Alarms remain quantized to the 1 ms interrupt cadence. Accurate
long masked intervals would require a separate free-running time source.

### Memory layout

```text
0x00000000..0x00004000  bootloader
0x00004000..0x0007C000  application
0x0007C000..0x00080000  persistence
```

The 16 KiB persistence window is two 8 KiB security-journal sectors.

Recorded release measurement:

| value | bytes |
|---|---:|
| raw image | 202,820 |
| former regression budget (not enforced) | 212,992 |
| `.data` | 308 |
| `.bss` | 18,280 |
| static total | 18,588 |
| available linked stack | 46,944 |

Physical application, persistence, RAM, and stack checks remain mandatory.
There is no EFR32MG21 OTA packaging path.

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
