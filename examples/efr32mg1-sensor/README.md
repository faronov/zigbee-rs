# EFR32MG1P TRÅDFRI Zigbee/SHT3x Sensor

Pure-Rust firmware for the connected `EFR32MG1P132F256IM32`. The default
profile remains the existing Zigbee sleepy-end-device example; `diag-sht` is
the minimal phase-1/2 hardware path and does not initialize radio or NV.

## Fixed board assumptions

| Function | Configuration |
|---|---|
| HFXO/HCLK | 38.4 MHz, CTUNE 360 |
| LED | PA0, active high |
| Button | PB13, active low, pull-up/filter |
| I2C0 SDA/SCL | PC10/PC11, LOC15, open drain |
| I2C speed | 10 kHz with weak internal pull-ups for bring-up margin |
| SHT3x | Probe `0x44`, then `0x45` only |

The controller supports external pull-ups, but this board definition
intentionally uses the known native-project internal-pull-up configuration.

## Bootloader-safe memory map

```text
0x00000000..0x00003FFF  resident Gecko bootloader (not emitted)
0x00004000..0x00039FFF  Rust application
0x0003A000..0x0003FFFF  existing native NVM3 (preserved)
0x20000000..0x20007BFF  usable SRAM (0x7C00 bytes)
```

The custom board linker script places the vector table at `0x4000` and writes
the `APP_PROPERTIES` address at vector word 13 (`0x4034`). `cortex-m-rt`'s
`set-vtor` startup writes `SCB->VTOR = 0x4000` before Rust initialization.

## Profiles

- `sensor` (default): existing full Zigbee SED; its application measurements
  remain simulated for now.
- `diag-sht`: HFXO, SysTick/RTT, PA0 LED, I2C0, and real SHT3x only; no NV/radio.
- `diag-beacon`: radio TX/active-scan diagnostic.
- `diag-join`: Zigbee join/poll diagnostic.

Exactly one profile must be enabled.

## Build and verify

Run Cargo from this directory so `.cargo/config.toml` supplies the Cortex-M
target, build-std, and bootloader linker script:

```bash
cd examples/efr32mg1-sensor

cargo build --release
cargo build --release --no-default-features --features diag-sht
cargo build --release --no-default-features --features diag-beacon
cargo build --release --no-default-features --features diag-join

tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

The ELF artifact is:

```text
target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

Create a range-limited Intel HEX only after layout verification:

```bash
rust-objcopy -O ihex \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex
```

The verifier rejects a first file-backed load below `0x4000`, a load entering
`0x3A000`, an invalid SP, a missing application-properties pointer, or a Reset
handler without the early VTOR write.

## `diag-sht` RTT markers

The diagnostic performs:

1. bounded 38.4 MHz HFXO startup;
2. I2C0 initialization and bus recovery if SDA/SCL are not idle;
3. soft reset (`30 A2`) and 2 ms wait;
4. status read (`F3 2D`) with CRC validation;
5. high-repeatability single shot (`24 00`), 20 ms wait, six-byte read;
6. independent temperature and humidity CRC checks.

Expected markers include:

```text
[EFR32][diag-sht] CLOCK_READY ...
[EFR32][diag-sht] I2C_READY ...
[EFR32][diag-sht] SHT_FOUND address=0x44 status=... crc=ok
[EFR32][diag-sht] MEAS_OK ... temp_centi_c=... humidity_centi_percent=... crc=ok
```

Probe, status, CRC, transfer, and clock failures have distinct RTT markers.

## Hardware-proven `diag-sht`

The isolated diagnostic is proven on the connected
`EFR32MG1P132F256IM32`:

- resident bootloader and `0x3A000..0x3FFFF` native NVM3 remained unchanged;
- HFXO/HCLK started at 38.4 MHz with CTUNE 360;
- SHT3x was detected at `0x44`, status CRC passed;
- a 140-sample 10 kHz stress run completed with zero I2C or CRC errors after
  enabling the Series-1 GPIO input filter;
- observed values were approximately 25.2 °C and 66.6 %RH.

Only `diag-sht` is currently authorized for hardware use. The default Zigbee
profile still initializes Rust NV inside the legacy NVM3 range and must not be
flashed until the persistence migration is explicit.

Safe identification/read-only checks remain:

```bash
commander adapter probe
commander device info --device EFR32MG1P132F256IM32
commander readmem --device EFR32MG1P132F256IM32 --range 0x00000000:+0x40
commander readmem --device EFR32MG1P132F256IM32 --range 0x00004000:+0x40
```

Build and verify `diag-sht`, then perform a page-limited flash with no
mass-erase/recover option:

```bash
cd examples/efr32mg1-sensor
cargo build --release --no-default-features --features diag-sht
tools/verify-layout.py target/thumbv7em-none-eabi/release/efr32mg1-sensor
rust-objcopy -O ihex \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex
commander flash \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex \
  --device EFR32MG1P132F256IM32
```

Never use `device masserase` for this layout. Re-read vectors at `0x0` and
`0x4000`, then attach RTT without requesting an erase.

## Architecture

- `efr32mg1-hal`: raw CMU/MSC wait-state, GPIO, and I2C0 controller code.
- `boards/efr32mg1-tradfri`: HFXO/pin/LOC/I2C speed and application flash map.
- `drivers/sht3x`: generic blocking embedded-hal 1.0 SHT3x protocol.
- this example: profile sequencing, Embassy timers, RTT, and LED patterns.

DCDC and EM2 are deliberately deferred. Phase 1/2 runs in normal active mode;
power management begins only after sensor and radio behavior are proven.
