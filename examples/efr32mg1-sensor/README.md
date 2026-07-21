# EFR32MG1P TRÅDFRI Zigbee SED

Pure-Rust production firmware for the connected
`EFR32MG1P132F256IM32`. This package has one unconditional binary and no
profile-selection modes. The optional `ota` feature adds the OTA Upgrade
client without changing the default production image.

The firmware is a Zigbee temperature/humidity sleepy end device with:

- factory EUI and crash-safe security persistence;
- strict fast-poll interview gate until Power Configuration, Temperature,
  and Humidity reporting are configured;
- secure restore/rejoin and Device Announce retries;
- real SHT3x temperature/humidity and ADC0 battery measurements;
- Identify handling and ZCL reporting;
- 250 ms fast polling followed by 30-second RTCC/EM2 parent polling;
- PB13 interrupt wake with 80 ms debounce;
- short-press immediate sensor/battery sample;
- three-second hold for crash-safe factory reset.

## Build and verify

Run Cargo from this directory so `.cargo/config.toml` supplies the Cortex-M
target, `build-std`, and bootloader-safe linker script:

```bash
cargo build --release
tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

Create a range-limited HEX only after layout verification:

```bash
rust-objcopy -O ihex \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex
```

No command here flashes hardware. Never use `device masserase` for this
layout.

## OTA image

The OTA client stages a Gecko Bootloader GBL in the 256 KiB external flash,
keeps the SED in fast-poll mode during transfer, verifies the GBL with the
resident bootloader, checkpoints Zigbee security state, and then resets into
the bootloader for installation.

Build a versioned GBL and Zigbee OTA container with Simplicity Commander:

```bash
tools/create-ota.sh 2
```

This produces `target/ota/efr32mg1-sensor-v2.{s37,gbl,ota}`. The numeric
version is compiled into both `APP_PROPERTIES` and the OTA client attributes;
it must be higher than the version currently installed on the device.

## Fixed board configuration

| Function | Configuration |
|---|---|
| HFXO/HCLK | 38.4 MHz, CTUNE 360 |
| LED | PA0, active high |
| Button | PB13, active low, falling-edge interrupt |
| I2C0 SDA/SCL | PC10/PC11, LOC15 |
| I2C speed | 10 kHz with internal pull-ups |
| SHT3x | Probe `0x44`, then `0x45` |
| Battery | ADC0 AVDD against calibrated internal 5 V reference |
| External flash SPI | USART0 mode 0, PD15/PD14/PD13 LOC23/21/19, PB11 CS |
| LED PWM | TIMER0 CC0 LOC0 on PA0, 1 kHz |

## Bootloader-safe layout

```text
0x00000000..0x00003FFF  resident Gecko bootloader
0x00004000..0x00036FFF  production application
0x00037000..0x00038FFF  security journal
0x00039000..0x00039FFF  Rust application NV
0x0003A000..0x0003FFFF  existing native NVM3
0x20000000..0x20007BFF  usable SRAM
```

The custom linker script places vectors at `0x4000`, places
`APP_PROPERTIES` at vector word 13, and enables the early VTOR write through
`cortex-m-rt`'s `set-vtor` feature.

Hardware diagnostics live in
[`../../tools/efr32mg1-lab`](../../tools/efr32mg1-lab), including read-only
external-flash JEDEC probing and LED PWM waveform tests.
