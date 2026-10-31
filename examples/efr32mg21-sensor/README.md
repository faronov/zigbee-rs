# EFR32MG21 sensor (BRD4181A + BRD4001A)

`no_std` composition of the shared environmental `SensorApp` on the
pure-Rust Series 2 radio driver (`efr32s2-hal`, feature `efr32mg21`) and the
shared software MAC (`Efr32s2RadioPhy` + `SoftMacCore`). It is the same
composition as the hardware-proven `examples/efr32mg22-sensor`, with MG21
pins, IRQ numbers and linker layout.

## Status

**Builds; not runnable on hardware yet.** The xG21 radio PHY/analog tables
(`efr32s2-hal/src/radio/xg21.rs`) are empty, so `Efr32s2RadioPhy::new`
returns `PhyError::Unsupported` and the firmware logs the error over VCOM and
halts. It deliberately does not program xG22 values into an xG21 radio.
Everything before radio init (clocks/HFXO, VCOM, RTCC time driver, EM2,
persistence) is the code that is hardware-proven on xG22, but it has not run
on an xG21 die.

## Exact target

| item | value |
|---|---|
| MCU | EFR32MG21A020F512IM32 |
| HFXO | 38.4 MHz, CTUNE 133 (BRD4181A default) |
| LED0 | PB0, active high |
| BTN0 | PD2, active low |
| VCOM | USART0 TX PA5, enable PD4, 115200 8N1 |

PD2 cannot wake the chip from EM2 (Series 2 keeps EM2 asynchronous GPIO
interrupts only on ports A and B), so a press is seen at the next RTCC wake.

## Application

- `DeviceProfile<TemperatureHumidityBattery>`;
- die temperature (EMU_TEMP), synthetic humidity, fixed 3000 mV battery;
- `NoOta`;
- short press toggles join/leave, three-second hold requests a durable
  factory reset;
- sleepy end device: RTCC (LFRCO, 32.768 kHz) Embassy time driver, EM2
  between polls (`em2-debug` keeps the debug domain powered).

## Ownership

```text
environmental non-OTA profile
        ↓
products/efr32mg21-sensor      identity, profile, persistence, linker map
        ↓
boards/efr32mg21-devkit        LED0/BTN0/VCOM pins, radio/RTCC tokens
        ↓
efr32s2-hal (efr32mg21) + zigbee-mac efr32s2 RadioPhy
```

## Layout

```text
0x00000000..0x00004000  Gecko bootloader
0x00004000..0x0007C000  application (61-IRQ xG21 vector table at 0x4000)
0x0007C000..0x00080000  security persistence (2 x 8 KiB)
0x20000000..0x20010000  SRAM
```

## Build

```bash
cd examples/efr32mg21-sensor
cargo +nightly-2026-03-23 build --release --locked
python3 tools/verify-layout.py \
  target/thumbv8m.main-none-eabihf/release/efr32mg21-sensor
```

Features: `em2-debug` (keep SWD usable in EM2), `stack-paint` (paint free
stack with `0xCCCCCCCC` for high-water measurement).

Recorded image (`tools/mac-footprint.py report`): flash 229,084 B, text
187,048 B, `.bss` 14,600 B, Embassy task pool 2,744 B, static stack bound
20,652 B; linked stack 50,424 B.

## Validation

Compile, clippy, linker and persistence-layout checks pass. Hardware gates
open: xG21 radio tables (needs an xG21 reference capture), then the full
join/report/EM2 path on BRD4181A or a Home Assistant Connect ZBT-1.
