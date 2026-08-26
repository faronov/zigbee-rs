# PHY62x2 sleepy end-device example

This `no_std` example is the application shell for the pure-Rust PHY62x2 MAC.
It exposes Basic, Identify, Power Configuration, Temperature Measurement, and
Relative Humidity clusters and polls its parent as a Zigbee sleepy end device.

## Status

The firmware is **compile-tested, not hardware-proven**. Do not treat it as a
ready-to-flash production image yet.

Implemented:

- PHY62x2 ROM-compatible SRAM/XIP linker layout;
- hard 130816-byte XIP application-slot gate;
- PHY6 segmented image generator;
- crash-safe `SecurityStateJournal` persistence;
- factory-identity guard that rejects the shared fallback address;
- RAM-resident flash program/erase operations;
- bounded SPIF timeouts;
- shared `SensorApp` commissioning, reporting, and parent-poll lifecycle;
- active/idle waits with radio sleep between joined polls;
- explicit rejection of unproven retention sleep;
- fallible exclusive ADC and `embedded-hal` 1.0 I2C resources;
- shared synthetic temperature/humidity test values.

Still requires hardware proof:

- ROM boot and interrupt dispatch;
- radio scan, association, ACK timing, and secured ZHA commissioning;
- flash journal writes and reset recovery;
- AON system sleep, retention wake, and current consumption.

AON system sleep is deliberately not used by this example. The runtime remains
in RAM and uses Embassy timers plus radio sleep between polls.

## Chip selection

The default build selects the 512 KiB PHY6222 layout and uses the pinned
`nightly-2026-08-01`:

```bash
cargo +nightly-2026-08-01 build --release --locked
```

The 256 KiB PHY6252 journal addresses can be selected explicitly:

```bash
cargo +nightly-2026-08-01 build --release --locked \
  --no-default-features --features phy6252
```

PHY6252 remains unverified; selecting the feature only prevents the known
out-of-range 512 KiB NV addresses.

## ROM image layout

The PHY62x2 ROM reserves the beginning of SRAM for jump/configuration tables.
The firmware uses:

```text
ROM IRQ jump table: 0x1fff0000..0x1fff03ff
ROM-owned SRAM:     0x1fff0400..0x1fff1837
Run descriptor:     0x1fff1838 (initial SP + Reset)
XIP application:    0x11010100..0x1102ffff
PHY6 image header:  flash offset 0x10000, size 0x100
```

The ELF is not itself a flashable PHY62x2 application. Generate an Intel HEX
file, validate the layout, and package the ROM-loader image:

```bash
ELF=examples/phy6222-sensor/target/thumbv6m-none-eabi/release/phy6222-sensor
OBJCOPY=$(find "$(rustc +nightly-2026-08-01 --print sysroot)" \
  -name llvm-objcopy -print -quit)

"$OBJCOPY" -O ihex "$ELF" "$ELF.hex"
sh examples/phy6222-sensor/check-layout.sh "$ELF"
cargo +nightly-2026-08-01 run --quiet --locked \
  --manifest-path ../../Cargo.toml -p phy62x2-image -- \
  "$ELF.hex" "$ELF.phy6.bin"
```

Write `phy6222-sensor.phy6.bin` at flash offset `0x10000` with a PHY62x2-aware
UART/SWD tool. Raw `llvm-objcopy -O binary` output loses the SRAM segment table
and is not a valid replacement.

## Persistence

Security state uses the shared atomic two-sector journal:

| Feature | PHY6222 | PHY6252 |
|---|---:|---:|
| Journal sector A | `0x7e000` | `0x3e000` |
| Journal sector B | `0x7f000` | `0x3f000` |

The complete flash program/erase/cache-bypass path executes from SRAM with
interrupts disabled. The board's whole-flash resource pauses SysTick and
accounts for the interval with the continuously running AON RTC, so journal
updates do not make Embassy time run slow. Network keys and outgoing counter
reservations are saved only when security state changes, not on every poll
cycle.

`products/phy62x2-evk` owns the protected partition addresses, journal policy,
identity, battery chemistry, lifecycle policy, profile, and ROM-aware linker
layout. `boards/phy62x2-evk` owns only fitted wiring, whole-device flash, and
platform timing. `phy6222-hal` provides exclusive raw peripheral mechanisms.

The current packaged PHY6222 image is **129,556 B**. No AON sleep current or
battery-life value is claimed.
