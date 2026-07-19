# Telink TLSR8258

The supported Telink platform is a pure-Rust TLSR8258 implementation. Telink
B91 remains an unsupported scaffold because it has no radio or MAC backend.

## Hardware and SRAM layout

| Property | TLSR8258 |
|---|---|
| Core | Telink TC32 |
| Flash | 512 KiB |
| SRAM | 64 KiB at `0x840000..0x850000` |
| Rust target | `tc32-unknown-none-elf` |
| Toolchain | [modern-tc32](https://github.com/modern-tc32/rust) |

RAM-resident code occupies the bottom of SRAM. The hardware instruction cache
then requires 0x100 bytes of tags and 0x800 bytes of cache data. Writable
`.data` and `.bss` therefore start at:

```text
0x840900 + align256(ram_code_size)
```

Both production linker scripts and the post-link checker enforce this
reservation. The SVC stack occupies `0x84BC00..0x84FC00`; the IRQ stack ends
at `0x850000`.

## Repository structure

```text
examples/telink-tlsr8258-sensor/  polling end-device sensor
examples/telink-tlsr8258-router/  always-on join/relay router
tools/telink-tlsr8258-lab/        bring-up and regression firmware
tlsr8258-hal/                     clocks, timers, flash, radio, GPIO, ADC, PM
tlsr8258-rt/                      reset, IRQ context, RAM initialization
zigbee-mac/src/telink/            reusable TLSR8258 MacDriver
```

The application examples contain only board configuration, flash-journal
adaptation, and role-specific Zigbee logic. The old direct-MMIO radio, local
MAC, SRAM markers, and diagnostic modes are retained only in the hardware lab.

## Toolchain

Install the current supported release under `.toolchains`:

```bash
TAG=tc32-stage2-tc32-45
case "$(uname -s)-$(uname -m)" in
  Darwin-arm64)  ASSET=tc32-rust-toolchain-macos-arm64.tar.gz ;;
  Darwin-x86_64) ASSET=tc32-rust-toolchain-macos-amd64.tar.gz ;;
  Linux-x86_64)  ASSET=tc32-rust-toolchain-linux-amd64.tar.gz ;;
  *) echo "Unsupported host"; exit 1 ;;
esac

DEST=".toolchains/${TAG}"
mkdir -p "$DEST"
curl -fL \
  "https://github.com/modern-tc32/rust/releases/download/${TAG}/${ASSET}" \
  -o /tmp/tc32-toolchain.tar.gz
tar -xzf /tmp/tc32-toolchain.tar.gz --strip-components=1 -C "$DEST"
"$DEST/bin/rustc" --version
```

An external extraction can be selected with `TC32_TOOLCHAIN=/path/to/toolchain`.

## Production examples

Build from the repository root:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build router
```

The generated images are:

```text
examples/telink-tlsr8258-sensor/target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
examples/telink-tlsr8258-router/target/tc32-unknown-none-elf/release/telink-tlsr8258-router.bin
```

`tools/tlsr8258-firmware.sh` builds with tc32-45, emits the binary, and checks
the cache reservation, RAM code, BSS/stack separation, production image size,
and absence of the legacy lab MAC.

### Sensor

The sensor is a polling Zigbee end device:

- Basic, Power Configuration, Identify, Temperature, and Humidity clusters;
- deterministic test variation for temperature and humidity;
- crash-safe two-sector security journal;
- secured rejoin and parent polling;
- `rx_on_when_idle = false`.

`PowerMode::Sleepy` currently selects the polling end-device behavior. It does
not put the TC32 CPU into retention sleep. A separate SED example will be
added only after the production runtime layout and full Zigbee state survive
repeated LOW32K retention wakeups.

### Router

The router joins as an FFD, enters continuous receive, relays NWK traffic, and
sends router maintenance frames. Hardware has proven join, interview,
Identify, and reset/resume.

It is not yet a parent router. Child association responses, beacon
transmission, permit joining, and indirect queues are not implemented.

## Hardware lab

The 7,000-line bring-up firmware is deliberately not an example. Run its
diagnostics through the root wrapper:

```bash
./scripts/tlsr8258.sh build diag-beacon
./scripts/tlsr8258.sh build diag-assoc
./scripts/tlsr8258.sh build diag-smoke
./scripts/tlsr8258.sh build diag-pm
./scripts/tlsr8258.sh build lab-sensor
```

The lab preserves the hardware evidence for raw RF, MAC timing, startup,
flash, and retention PM without obscuring the production applications.

## Current capability boundary

The TLSR8258 backend provides active/passive/energy scan, association, data
request polling, unicast TX/RX, CSMA-CA, ACK retries, software ACK generation,
mandatory timing, and crash-safe security persistence. Home Assistant ZHA has
verified commissioning, TCLK exchange, interview, reporting, reset resume,
secured rejoin, and router join/relay setup.

Full coordinator support is not advertised. The B91 scaffold is excluded from
the firmware matrix until a separate B91 radio and MAC implementation exists.
