# BL702 pure-Rust Zigbee sensor

Environmental sleepy-end-device profile for the XT-ZB1 without Bouffalo radio
archives.

## Composition

```text
bl702-hal + Bl702RadioPhy + SoftMacCore
        ↓
boards/bl702-xt-zb1
        ↓
products/bl702-xt-zb1
        ↓
this root + sensor_sed_app::SensorApp
```

The root constructs explicit `SensorSedParts`:

- active-only wake;
- `NoStatus`;
- synthetic temperature/humidity;
- GPADC supply battery;
- `NoOta`;
- no user action;
- BL702 supervisor and diagnostics.

The product selects `Active` for both fast and slow waits. No PDS/HBN sleep
path is claimed.

## Radio and Zigbee validation

Hardware-proven:

- cold ACAL/KCAL/ROSCAL/RCCAL;
- channel selection, CCA/ED, RX, and TX;
- steering and association on channel 15;
- indirect polling and Transport-Key reception;
- ZHA interview, Configure Reporting, TCLK exchange, and live reports.

The production path installs token-owned SEC_ENG AES, runs two startup KATs,
and fails closed rather than using software AES.

## Storage

The board exposes the physical XIP flash resource. The product reserves and
constructs:

```text
0x00000000..0x000FE000  application
0x000FE000..0x00100000  SecurityStateJournal
```

Persistence is integrated. Destructive erase/program and reset/resume still
need controlled XT-ZB1 hardware validation.

## Build

```bash
cd examples/bl702-sensor
python3 -m pip install bflb-mcu-tool==1.10.0 pyserial
./build-image.sh
```

Diagnostic logging:

```bash
BL702_DIAGNOSTIC_LOG=1 ./build-image.sh
```

Current production images:

| artifact | bytes |
|---|---:|
| raw binary | 182,786 |
| packaged boot image | 190,992 |

The package script verifies BL702 boot magic and explicit 32 MHz XTAL clock
fields.

## Flash and monitor

Connect the CH340 port, hold BOOT/GPIO28, then:

```bash
./flash.sh
./monitor.sh
```

The flash script programs and verifies the packaged image. Do not infer
persistence validation from a successful program operation.

## Remaining gates

- security-journal erase/program, power interruption, and reset recovery;
- GPADC battery accuracy;
- low-power modes;
- RF output/spectral qualification;
- hardware address filtering and auto-ACK.
