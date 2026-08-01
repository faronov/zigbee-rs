# Telink TLSR8258

The supported Telink platform is a pure-Rust TLSR8258 implementation.

## Hardware and SRAM layout

| Property | TLSR8258 |
|---|---|
| Core | Telink TC32 |
| HAL flash geometries | 512 KiB, 1 MiB, 2 MiB, 4 MiB |
| TB-04 fitted flash | 512 KiB |
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
                                  I2C, SPI, UART, PWM, PM, IRQ/reset,
                                  capture, RNG, AES, ownership tokens
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

### Firmware footprint

With the pinned `tc32-45` toolchain, the current production payloads are:

| Image | Raw payload | Complete-HAL baseline | Reduction | CI budget |
|---|---:|---:|---:|---:|
| End-device sensor | 272,148 B | 323,876 B | 51,728 B (16.0%) | 280 KiB |
| Parent router | 331,852 B | 349,792 B | 17,940 B (5.1%) | 336 KiB |

The router is 59,704 bytes larger because it retains route maintenance, child
admission and aging, indirect delivery, parent-side MAC commands,
Update-Device handling, and Parent Announce. The sensor compiles those paths
out and retains only its leaf behavior, including the R22 End Device Timeout
client. See [Firmware Size and Role Specialization](../advanced/firmware-size.md)
for the cross-platform measurements and CI gates.

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

Child-parent operation has been exercised once on hardware, including child
admission and Trust Center link-key exchange. That run used an older BL702
child image, and a later corrected capture exposed parent-delivery blockers
that are now fixed in source. The release gate therefore remains a clean
first-attempt join with the corrected child image, followed by a complete ZHA
interview under an independent channel-15 sniffer.

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

The chip HAL is direct-register Rust. Vendor objects were used only as
disassembly evidence where open headers did not contain a function body; no
Telink library is linked.

| Surface | Implemented support | Validation boundary |
|---|---|---|
| System control | Fail-closed clock bring-up, typed clock/reset gates, canonical IRQ masks/W1C acknowledgement, Timer0/Timer1, Timer2 watchdog, immediate software reset | Clock/radio/timer behavior has hardware evidence; the consolidated facades are host-tested and TC32-built |
| GPIO and capture | Unique PA0-PE7 tokens, GPIO/mux/pulls/drive, three GPIO IRQ comparators, fixed-capacity rising-edge capture queue | Existing GPIO paths have hardware evidence; the generalized IRQ/capture APIs are compile-tested |
| I2C/SPI | Four I2C groups with repeated starts and bounded recovery; two MSB-first SPI groups | Host-tested and TC32-built, not silicon-tested |
| UART | All documented TX/RX routes, fixed 8 data bits, parity/stop bits, RTS/CTS, bounded flush, nonblocking byte I/O, trigger/error IRQ status | Disassembly-derived and cross-checked, including the PB1/PB7 4800-8N1 smart-plug profile; not silicon-tested |
| ADC | Exclusive MISC-channel owner, geometry-aware factory calibration, GPIO voltage sampling, serialized sharing with RNG/flash | Register path is evidenced; physical accuracy depends on fitted calibration and board wiring |
| Flash | Raw NOR read/program/erase, bounded partitions, geometry verification, factory EUI/UID fallback, Zbit voltage guard | Deployed TB-04 persistence is hardware-proven; new geometry/guard behavior is compile-tested |
| PWM | Six channels, normal/count/IR modes, CPU-fed IR FIFO, shadow cycle/duty, typed IRQ status | Basic register path is compile-tested; advanced modes are not silicon-tested |
| Power management | Suspend/deep-sleep/retention entry, timer/pad/comparator wake-source arming, RC32K calibration, typed wake status | Timer-only suspend is hardware-proven. Pad/comparator wake and comparator front-end behavior remain unvalidated |
| RNG | AES-128 CTR_DRBG with SHA-256-conditioned VBAT/GND ADC samples, stuck-source rejection, full ADC state restore | NIST DRBG vector passes; physical min-entropy is uncharacterized and still requires SP 800-90B work |
| AES | Token-owned, bounded AES-128 ECB encrypt/decrypt plus optional `zigbee-crypto` forward-cipher backend | Vendor-protocol/KAT tested, never exercised on silicon; default timeout is unmeasured |

`tlsr8258_hal::peripherals::Peripherals::take()` returns one
`SerialController`, plus independent PWM, UART, ADC, AES, and non-`Clone` GPIO
tokens. I2C and SPI consume the same serial token because their control and
route registers overlap. Radio and RNG use separate IRQ-safe singleton
handles. Shared register read-modify-writes are serialized by the central
nested-safe IRQ critical section.

`boards/tlsr8258-tb04::resources::BoardResources` retains the serial, UART,
ADC, AES, lighting, PC5, and fitted-flash ownership. The production sensor
and router install the owned ADC/PC5 flash-voltage guard before constructing
the security journal. `products/tlsr8258-tb04` bounds that journal to
`0x74000..0x76000`; the linker script prevents firmware overlap.

### Flash geometry and identity

Telink factory locations move with the fitted flash:

| Geometry | Factory EUI-64 | Factory config | ADC calibration |
|---|---:|---:|---:|
| 512 KiB | `0x76000` | `0x77000` | `0x770C0` |
| 1 MiB | `0xFF000` | `0xFE000` | `0xFE0C0` |
| 2 MiB | `0x1FF000` | `0x1FE000` | `0x1FE0C0` |
| 4 MiB | `0x3FF000` | `0x3FE000` | `0x3FE0C0` |

Non-512-KiB products must use geometry-aware constructors. They verify the
JEDEC capacity before reading a factory sector, preventing ordinary
application bytes at another geometry's address from becoming a plausible
device identity. Existing TB-04 sensor/router EUI offsets remain unchanged
for deployed persistence and ZHA compatibility; new products should use the
unchanged factory/UID-derived EUI.

Zbit `ZB25WD40B`/`ZB25WD80B` parts require a real ADC check before every
physical page program or sector erase. The HAL drives PC5 high, samples it,
and fails closed below 2200 mV, at 500 mV or more fluctuation, or whenever
the reading is unavailable. A constant-voltage callback cannot be installed
through the public API.

### Explicitly unsupported

UART DMA, generic DMA-channel ownership, PWM DMA, complementary PWM outputs,
USB, audio/PGA, QDEC, EMI/test features, and SWire debug control are not
modeled. The AES accelerator is also not yet the default Zigbee CCM* path:
NWK/APS currently call software free functions without carrying a persistent
cipher object, so `MacCapabilities.hardware_security` remains `false`.

Host tests and TC32 builds cover the complete API. TB-04 exposes no fixed
I2C/SPI convenience constructor because fitted bus wiring is undocumented;
applications choose from the validated generic route groups.

## Current capability boundary

The TLSR8258 backend provides active/passive/energy scan, association, data
request polling, unicast TX/RX, CSMA-CA, ACK retries, software ACK generation,
mandatory timing, and crash-safe security persistence. Home Assistant ZHA has
verified commissioning, TCLK exchange, interview, reporting, reset resume,
secured rejoin, and router join/relay setup.

Router restart, maintenance traffic, and NWK relay are hardware-verified.
Child-parent support is software-complete and has partial hardware evidence,
but remains behind the clean corrected-image sniffer gate described above.
SWire RAM/flash inspection is restart-intrusive on the tested programmer and
must be performed only after stopping an acceptance capture. Full coordinator
support is not advertised.
