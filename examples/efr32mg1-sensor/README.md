# EFR32MG1P TRÅDFRI environmental SED

Pure-Rust production composition for the connected
EFR32MG1P132F256IM32.

## Architecture

```text
efr32mg1-hal + Efr32Mac
        ↓
boards/efr32mg1-tradfri
        ↓
products/efr32mg1-tradfri
        ↓
this root + sensor_sed_app::SensorApp
```

The shared app owns commissioning, fast/slow polling, reporting, Identify,
security checkpoints, and OTA-first event routing. The root owns startup and
constructs explicit EFR32 wake/status/sensor/battery/OTA/supervisor adapters.

The product policy uses:

```text
fast wait: Active
slow wait: Retention
```

Steady-state waits use RTCC/LFRCO and EM2; commissioning/interview/OTA windows
remain active.

## Fitted hardware

| function | wiring |
|---|---|
| LED / TIMER0 PWM | PA0, active high |
| button | PB13, active low |
| SHT3x I²C | PC10 SDA, PC11 SCL |
| battery | ADC0 AVDD |
| external flash | USART0 PD13/PD14/PD15, PB11 CS |

Direct SPI flash and Gecko Bootloader storage access consume alternative typed
owners of the same physical path.

## Layout

```text
0x00000000..0x00004000  resident Gecko bootloader
0x00004000..0x00037000  application
0x00037000..0x00039000  security journal
0x00039000..0x0003A000  Rust application NV
0x0003A000..0x00040000  preserved native NVM3
0x20000000..0x20007C00  usable SRAM (exactly 0x7C00 bytes)
```

## Build and verify

```bash
cd examples/efr32mg1-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

Current raw image: **156,612 B**. Never use a mass erase on this layout.

## OTA

```bash
tools/create-ota.sh 2
```

The product writer stages a GBL through Gecko Bootloader access. `SensorApp`
checkpoints security before activation. A real Zigbee download/install/reboot
has not yet completed on hardware.

## Validation

Hardware-proven:

- commissioning, hardware AES, and ZHA interview;
- SHT3x and battery reporting;
- Identify/button behavior;
- persistence and reset/resume;
- RTCC wake and EM2.

Remaining gate: real OTA version upgrade, reboot into the new image, and
retained commissioned state.
