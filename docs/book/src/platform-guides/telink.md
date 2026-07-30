# Telink TLSR8258

The supported Telink platform is a pure-Rust TLSR8258 implementation.

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
examples/telink-tlsr8258-router/  always-on parent router
tools/telink-tlsr8258-lab/        bring-up and regression firmware
tlsr8258-hal/                     clocks, timers, flash, radio, GPIO, ADC,
                                  I2C, SPI, PWM, PM, ownership tokens
tlsr8258-rt/                      reset, IRQ context, RAM initialization
boards/tlsr8258-tb04/             fitted LEDs, flash token, typed resources
products/tlsr8258-tb04/           protected flash partition and linker policy
zigbee-mac/src/telink/            reusable TLSR8258 MacDriver
```

The board crate exposes only physical resources. The product crate owns the
bounded security partition, journal construction, and linker layout. The
application examples contain role-specific Zigbee logic. The old direct-MMIO
radio, local MAC, SRAM markers, and diagnostic modes are retained only in the
hardware lab.

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
TCLK exchange, Identify, routed-frame relay, and silent reset/resume. The
accepted reset capture resumed on the previous short address without a Beacon
Request, Association Request, or Device_annce.

The bounded parent path now handles beacon requests, finite or indefinite
permit joining, child association, and indirect delivery to polling sleepy
children. Child-table entries are RAM-only and children must re-associate
after router reboot; commissioned network and security state remain
persistent.

The security path waits for the over-the-air Association Response ACK before
notifying the Trust Center, forwards tunneled Transport-Key commands without
rewriting the embedded APS frame, and keeps a child provisional until it
proves possession of the network key. Secured and centralized unsecured
rejoins are supported; distributed unsecured rejoins are rejected.

Child-parent operation is not yet hardware-validated. A sniffer gate must
verify ACK turnaround and Frame Pending timing, deferred Association Response
after an extended-address poll, and indirect data after a short-address child
poll.

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

## Peripheral HAL status

The reusable HAL exposes GPIO, ADC, flash, timers, power management, I2C, SPI,
and six-channel PWM. `embedded-hal` I2C, SPI bus, digital, and PWM traits are
implemented where applicable.

`tlsr8258_hal::peripherals::Peripherals::take()` creates one shared
`SerialController` token, the PWM token, and a non-`Clone` token for every
GPIO pad. I2C and SPI consume the same serial token because TLSR8258 implements
them with overlapping control and route registers; safe code therefore cannot
keep both controllers live. On TC32 the single-take guard masks interrupts
rather than emitting unsupported atomic read-modify-write instructions.

`boards/tlsr8258-tb04::resources::BoardResources` owns those controller tokens,
the fitted RGB/status LED pins, the supported non-LED I2C/SPI route pins, and
the fitted flash token. `products/tlsr8258-tb04` consumes that token and bounds
the security journal to `0x74000..0x76000`; its linker script prevents firmware
overlap. Choosing direct GPIO LEDs consumes the same
lighting token that would otherwise create PWM0/PWM5/PWM2 outputs. The raw
`Pin::steal` escape hatch is unsafe; normal code obtains unique pins only from
`Peripherals`.

The I2C implementation validates the four documented pin groups, supports
bounded bus recovery and repeated-start transactions, and does not invent an
arbitration-loss flag that TLSR8258 does not expose. SPI supports the two
documented groups and MSB-first operation. All PWM channels share one
validated clock/divider.

Host tests and TC32 production builds cover the new APIs. Existing GPIO, ADC,
flash, radio, and retention-PM paths have hardware evidence; the new
I2C/SPI/PWM paths have not yet been tested on silicon. TB-04 does not expose a
fixed I2C or SPI convenience constructor because fitted bus wiring is not
documented. Its `SerialResources` instead exposes the valid route pins and the
single controller token for an application-selected bus.

## Current capability boundary

The TLSR8258 backend provides active/passive/energy scan, association, data
request polling, unicast TX/RX, CSMA-CA, ACK retries, software ACK generation,
mandatory timing, and crash-safe security persistence. Home Assistant ZHA has
verified commissioning, TCLK exchange, interview, reporting, reset resume,
secured rejoin, and router join/relay setup.

Router restart, maintenance traffic, and NWK relay are hardware-verified.
Child-parent support is software-complete but remains behind the sniffer gate
described above. SWire RAM/flash inspection is restart-intrusive on the tested
programmer and must be performed only after stopping an acceptance capture.
Full coordinator support is not advertised.
