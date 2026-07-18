# PHY62x2 sleepy end-device example

This `no_std` example is the application shell for the pure-Rust PHY62x2 MAC.
It exposes Basic, Identify, Power Configuration, Temperature Measurement, and
Relative Humidity clusters and polls its parent as a Zigbee sleepy end device.

## Status

The firmware is **compile-tested, not hardware-proven**. Do not treat it as a
ready-to-flash production image yet.

Implemented:

- PHY62x2 ROM-compatible SRAM/XIP linker layout;
- PHY6 segmented image generator;
- crash-safe `SecurityStateJournal` persistence;
- RAM-resident flash program/erase operations;
- bounded SPIF timeouts;
- parent polling and light sleep between polls;
- shared synthetic temperature/humidity test values.

Still requires hardware proof:

- ROM boot and interrupt dispatch;
- radio scan, association, ACK timing, and secured ZHA commissioning;
- flash journal writes and reset recovery;
- AON system sleep, retention wake, and current consumption.

AON system sleep is deliberately not used by this example. The runtime remains
in RAM and uses Embassy timers plus radio sleep between polls.

## Chip selection

The default build selects the 512 KiB PHY6222 layout:

```bash
cargo build --release
```

The 256 KiB PHY6252 journal addresses can be selected explicitly:

```bash
cargo build --release --no-default-features --features phy6252
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
OBJCOPY=$(find "$(rustc --print sysroot)" -name llvm-objcopy | head -1)

"$OBJCOPY" -O ihex "$ELF" "$ELF.hex"
sh examples/phy6222-sensor/check-layout.sh "$ELF"
cargo run --quiet -p phy62x2-image -- "$ELF.hex" "$ELF.phy6.bin"
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
interrupts disabled. The example pauses SysTick and accounts for the interval
with the continuously running AON RTC, so journal updates do not make Embassy
time run slow. Network keys and outgoing counter reservations are saved only
when security state changes, not on every poll cycle.
