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

The application currently publishes synthetic temperature and humidity
values. Battery reporting uses the internal VBAT/2 GPADC path with factory
eFuse gain trim and falls back explicitly if initialization or conversion is
implausible.

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

The production hardware-AES raw payload is 161,570 bytes; the packaged flash
image is 169,776 bytes. Every build installs the token-owned SEC_ENG AES-128
engine, runs two startup known-answer tests, and fails closed rather than
falling back to software. Production release builds compile `log` records out
while retaining the direct UART boot markers. The generated flash image
contains the official 176-byte BL702 boot header, payload hash and CRC, a
32 MHz XTAL clock configuration, and the application at flash offset `0x2000`.
The build and flash scripts run `bflb-mcu-tool` from an isolated copy because
version 1.10.0 keeps mutable clock settings inside its installed package. They
also reject a boot header unless it explicitly selects the 32 MHz XTAL.

For a release-optimized hardware diagnostic image with the full Zigbee UART
trace, build with:

```bash
BL702_DIAGNOSTIC_LOG=1 ./build-image.sh
```

The diagnostic build is intentionally larger because it retains the full
Zigbee trace.

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
The firmware scans channel 15 and retries joining every 15 seconds. Build the
diagnostic image above before flashing if the full commissioning trace is
needed; the production image emits only compact boot markers.

Expected commissioning milestones include:

```text
[BDB:Steering] Joining PAN ... ch 15
[NWK] Joined PAN ... as ...
[BDB:Steering] NWK key received from TC!
stack joined: short=..., PAN=..., channel=15
[BDB:Steering] Commissioning security complete
commissioning complete: success=true
```

## Current peripheral and storage support

`bl702-hal` provides typed GPIO, I2C0, SPI0, GPADC/VBAT, PWM, UART, timer,
eFuse, power-state, and XIP-flash APIs. The board crate owns the physical
resources, while `products/bl702-xt-zb1` owns the protected final-8-KiB flash
partition and constructs the two-sector `SecurityStateJournal`.

The sensor uses durable start/resume, tick, incoming-frame, rejoin, leave, and
factory-reset paths. GPIO, buses, ADC, PWM, and destructive flash operations
are host-tested and RV32IMC-compiled; only UART, timer, eFuse, and radio paths
are currently hardware-proven.

## Known limitations

- Flash-backed network and security persistence is integrated, but sector
  erase/program and reset-resume still need cautious XT-ZB1 hardware
  validation.
- Temperature and humidity remain synthetic until a physical sensor is wired
  through the available I2C API.
- The GPADC battery path still needs hardware validation.
- M154 operation and the Embassy executor are polling-only; the firmware does
  not enter low-power retention sleep.
- Hardware address filtering and hardware auto-ACK are disabled.
- RF output power and spectral behavior still need lab measurement before a
  production or regulatory-compliance build.
