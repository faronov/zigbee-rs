# BL702 Pure-Rust Zigbee Sensor

This firmware runs a Zigbee temperature, humidity, and battery end device on
an XT-ZB1 without linking `liblmac154.a`, `libbl702_rf.a`, or another Bouffalo
radio archive.

The direct-register Rust path has been validated on real hardware:

- cold-start ACAL, KCAL, ROSCAL, and RCCAL;
- channel selection, CCA/energy detect, RX, and TX;
- network steering and association on channel 15;
- sleepy-device indirect polling and Transport-Key reception;
- ZHA Node/Simple descriptor interview;
- Configure Reporting for Power Configuration, Temperature, and Humidity;
- unique Trust Center link-key exchange;
- live entities and attribute reports in Home Assistant.

The application currently publishes synthetic values (22.50 C, 50% RH, and
3.0 V). BL702 I2C and ADC drivers are not implemented yet.

## Build

```bash
cd examples/bl702-sensor
rustup component add rust-src llvm-tools-preview
python3 -m pip install bflb-mcu-tool==1.10.0 pyserial
./build-image.sh
```

The local Cargo configuration builds for `riscv32imc-unknown-none-elf`. BL702
must not use `riscv32imac`: RV32A atomic instructions trap on the tested
XT-ZB1.

The script creates:

- `target/riscv32imc-unknown-none-elf/release/bl702-sensor`
- `target/riscv32imc-unknown-none-elf/release/bl702-sensor.bin`
- `target/riscv32imc-unknown-none-elf/release/bl702-sensor.flash.bin`

The raw payload is about 194 KiB. The generated flash image contains the
official 176-byte BL702 boot header, payload hash and CRC, a 32 MHz XTAL clock
configuration, and the application at flash offset `0x2000`.

## Flash and monitor

Connect the CH340 USB-serial port, hold BOOT/GPIO28, and run:

```bash
./flash.sh
```

The script auto-detects VID:PID `1A86:7523`, resets the board through RTS,
programs and verifies the raw image, and requires the tool to print
`[All Successful]`.

Release BOOT, open permit-join on the coordinator, then run:

```bash
./monitor.sh
```

`monitor.sh` resets through RTS and opens UART0 on GPIO14/GPIO15 at 2 Mbaud.
The firmware scans channel 15 and retries joining every 15 seconds.

Expected commissioning milestones include:

```text
[BDB:Steering] Joining PAN ... ch 15
[NWK] Joined PAN ... as ...
[BDB:Steering] NWK key received from TC!
stack joined: short=..., PAN=..., channel=15
[BDB:Steering] Commissioning security complete
commissioning complete: success=true
```

## Current peripheral support

There is no general BL702 HAL crate yet. The example currently owns only:

- UART0 TX and GPIO14/GPIO15 pinmux;
- a free-running 1 MHz timer used by Embassy and radio deadlines;
- factory chip-ID reads from the boot-ROM-loaded eFuse shadow.

I2C, SPI, ADC, general GPIO ownership, flash/NV storage, hardware RNG, and
retention sleep are not exposed through a Rust HAL. A real BME280/SHT3x and
battery build therefore needs a `bl702-hal` crate with `embedded-hal` I2C and
digital traits, an ADC API, and `embedded-storage` flash support.

## Known limitations

- Network and security state are not persisted yet; a reset requires a fresh
  permit-join commissioning.
- The sensor values are synthetic until I2C and ADC support is added.
- M154 operation and the Embassy executor are polling-only; the firmware does
  not enter low-power retention sleep.
- Hardware address filtering and hardware auto-ACK are disabled.
- RF output power and spectral behavior still need lab measurement before a
  production or regulatory-compliance build.
