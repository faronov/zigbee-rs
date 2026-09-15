# CC2340R5 Zigbee sensor

Compile-tested `no_std` environmental sensor composition for
LP-EM-CC2340R5 / CC2340R52.

## Current architecture

```text
environmental Sleepy End Device profile
        ↓
products/cc2340-sensor
        ↓
boards/lp-em-cc2340r5
        ↓
CC2340 register/radio mechanisms + MacDriver
```

This example is the composition root.

The board supplies real SysTick time, IOC/GPIO configuration, typed raw
LED/button resources, reset, identity, and flash:

| function | DIO |
|---|---:|
| LED1 | 7 |
| LED2 | 6 |
| Button 1 | 13 |
| Button 2 | 14 |

The product owns identity/profile, synthetic temperature/humidity, fixed
battery, `Active`/`Active` policy, the linker map, and the security partition.
This composition root maps the raw board buttons and LEDs to the selected wake
and status behavior, and installs RTT diagnostics and reset supervision.

Compile-stub time and “IOC pending” descriptions are obsolete.

## Security and commissioning gate

The image currently uses software AES and reserves:

```text
0x00000000..0x0007E000  application
0x0007E000..0x00080000  SecurityStateJournal
```

Commissioning is deliberately not claimed. The radio path has no completed HIL
run, and the production entropy backend is not qualified. Entropy fails closed
instead of returning predictable key material.

## Build

Use the exact SDK source pinned by CI:

```bash
git clone https://github.com/TexasInstruments/simplelink-lowpower-f3-sdk.git
git -C simplelink-lowpower-f3-sdk checkout \
  68ca021502383f367d0bf2a5517fdd0dcb0ef909

cd examples/cc2340-sensor
CC2340_SDK_DIR=/absolute/path/to/simplelink-lowpower-f3-sdk \
  cargo +nightly-2026-03-23 build --release --locked --target-dir target/sdk
```

Recorded pre-static-task pinned-SDK image: **223,536 B** against the former
**225,280 B** regression budget, which is no longer enforced. That snapshot's
**4,772 B** static-RAM figure is not a current task-capacity check.
The physical application slot remains **516,096 B**; memory, partition, and
stack checks remain mandatory.

With `CC2340_SDK_DIR` unset, the fallback build compiles but radio
initialization returns `FirmwareUnavailable`. It is not a flashable radio
product.

```bash
env -u CC2340_SDK_DIR \
  cargo +nightly-2026-03-23 build --release --locked \
  --target-dir target/fallback
```

## Hardware gates

- startup and clock proof on the target board;
- raw 802.15.4 TX/RX/FCS;
- scan, association, and ACK timing;
- entropy source;
- secured commissioning/interview/reporting;
- flash journal and reset recovery;
- power behavior.
