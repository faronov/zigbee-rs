# Telink TLSR8258 Zigbee sensor

This package is the current end-to-end TLSR8258 hardware gate for join,
Trust Center exchange, ZHA interview, reporting, persistence, and reset
resume.

It is pure Rust, but it is not yet the clean production target: `src/main.rs`
still contains a large local diagnostic radio module and local
`Tlsr8258Mac`. The reusable backend lives in `tlsr8258-hal` plus
`zigbee_mac::telink::TelinkMac` and is exercised by
`examples/telink-tlsr8258-radio`.

## Hardware

- MCU: Telink TLSR8258, TC32 core
- Flash: 512 KiB
- SRAM: 64 KiB at `0x840000..0x850000`
- Reference board: TB-04-Kit
- LEDs: PC1 red, PB5 green, PC4 blue

Writable data starts after aligned RAM code plus the 0x900-byte I-cache
reservation. The linker script and build helper verify cache, DMA, BSS,
stack, NV, and image boundaries after every link.

## Toolchain

Use the modern-tc32 stage2 release selected by the helper script:

```text
tc32-stage2-tc32-45
```

Install it under the repository default path:

```bash
TAG=tc32-stage2-tc32-45
case "$(uname -s)-$(uname -m)" in
  Darwin-arm64)  ASSET=tc32-rust-toolchain-macos-arm64.tar.gz ;;
  Darwin-x86_64) ASSET=tc32-rust-toolchain-macos-amd64.tar.gz ;;
  Linux-x86_64)  ASSET=tc32-rust-toolchain-linux-amd64.tar.gz ;;
  *) echo "Unsupported host"; exit 1 ;;
esac

DEST=".toolchains/${TAG}"
mkdir -p "${DEST}"
curl -fL \
  "https://github.com/modern-tc32/rust/releases/download/${TAG}/${ASSET}" \
  -o /tmp/tc32-toolchain.tar.gz
tar -xzf /tmp/tc32-toolchain.tar.gz --strip-components=1 -C "${DEST}"
"${DEST}/bin/rustc" --version
```

Alternatively set `TC32_TOOLCHAIN` to another extracted stage2 toolchain.

## Firmware modes

| Mode | Purpose |
|------|---------|
| `runtime-sensor` | Default full NWK/APS/BDB/ZDO/ZCL runtime, security persistence, and reporting |
| `sensor` | Legacy lighter interview path retained for comparison |
| `diag-assoc` | Scan, association, polling, and unicast diagnostics |
| `diag-beacon` | Raw Beacon Request and beacon parsing |
| `diag-smoke` | Minimal startup/radio smoke gate |

## Build

Always use the helper so the custom cargo, binary conversion, and layout
checks stay consistent:

```bash
cd examples/telink-tlsr8258-sensor

./scripts/tlsr8258.sh check runtime-sensor
./scripts/tlsr8258.sh build runtime-sensor
```

Other modes are selected explicitly:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build diag-assoc
./scripts/tlsr8258.sh build diag-beacon
```

The helper invokes the tc32 compiler with the hardware-proven codegen
settings:

```text
-C lto=no -C opt-level=1
```

The large-image failure previously attributed to compiler code generation was
reduced to delayed TX-done to RX activation. The radio now enters RX
synchronously at TX completion, matching the official Telink ACK-turnaround
sequence.

Outputs:

```text
target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor
target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
```

The current runtime image is 305,760 bytes (`0x4AA60`), so the checker warns
that it exceeds the 256 KiB production/OTA slot while still enforcing the
factory-data boundary.

## Flash and inspect

The default helper paths are:

- `TLSRPGM=$HOME/TLSRPGM/TlsrPgm.py`
- `TLSR_DEBUG=$HOME/zboss_opensource/tlsr_debug.py`
- `TELINK_PORT=/dev/cu.usbserial-1410`

All are overridable:

```bash
./scripts/tlsr8258.sh flash runtime-sensor
./scripts/tlsr8258.sh dump-boot
./scripts/tlsr8258.sh dump-mode
./scripts/tlsr8258.sh dump 0x00848550 8
```

## Proven behavior

The runtime gate has been exercised with Home Assistant ZHA and an Ember
coordinator:

- active scan, association, and indirect polling;
- Network-Key transport and secured Device Announce;
- Request-Key, unique TCLK transport, Verify-Key, and Confirm-Key;
- Node Descriptor, Active Endpoints, Simple Descriptor, Match, and Bind;
- Basic, Power Configuration, Identify, Temperature, and Humidity clusters;
- crash-safe two-sector security journal;
- reset resume with monotonic reserved global and TCLK counters.

## Remaining work

- Replace the local radio/MAC copy with the reusable
  `zigbee_mac::telink::TelinkMac` path.
- Remove the remaining application-owned SRAM markers and HA probe logic.
- Add production factory-reset UI and hardware validation.
- Add retention/deep-sleep radio reconfiguration and the Zbit flash voltage
  guard.
- Reduce the image below the 256 KiB OTA slot.
