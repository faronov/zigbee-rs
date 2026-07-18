# Telink TLSR8258

The supported Telink backend is the TLSR8258 pure-Rust end-device path. The
repository does not currently implement a B91 radio or MAC backend.

## Hardware

| Property | TLSR8258 |
|----------|----------|
| Core | Telink TC32 |
| Flash | 512 KiB |
| SRAM | 64 KiB |
| Radio | IEEE 802.15.4 + BLE |
| Rust target | `tc32-unknown-none-elf` |
| Toolchain | [modern-tc32](https://github.com/modern-tc32/rust) |

The 64 KiB SRAM region starts at `0x840000`. RAM-resident code is followed by
the hardware I-cache reservation: 0x100 bytes of tags and 0x800 bytes of cache
data. Writable `.data` and `.bss` must begin after:

```text
0x840900 + align256(ram_code_size)
```

The application linker script and post-link checker enforce this layout.

## Current implementation paths

The repository currently has two TLSR8258 paths:

1. The reusable backend used by the standalone radio/conformance harness:

```text
zigbee-runtime / BDB / ZDO / APS / NWK
                    |
             MacDriver
                    |
       zigbee-mac::telink::TelinkMac
                    |
              tlsr8258-hal
                    |
     Timer0 / flash / RF DMA / RF IRQ / MMIO
```

2. The full join/interview/persistence runtime gate in
   `examples/telink-tlsr8258-sensor`. It predates the HAL extraction and still
   defines a local diagnostic-heavy `Tlsr8258Mac` and local radio module.

`tlsr8258-hal` and the reusable `zigbee_mac::telink::TelinkMac` own the proven
platform and radio primitives for the first path:

- clock and Timer0 access;
- factory IEEE address or stable flash-UID fallback;
- official PHY initialization and calibration tables;
- two rotating RX DMA buffers and one TX DMA buffer;
- CRC/RSSI validation;
- immediate software ACK handling;
- CCA, unslotted CSMA-CA, frame retries, and TX-done to RX turnaround;
- RAM-resident flash erase/program operations.

The harness application owns startup vectors, linker layout, stacks, and the
reserved diagnostic regions.

The local runtime-sensor path is the current end-to-end hardware proof gate,
not the final production architecture. Replacing its local MAC/radio copy with
the reusable backend remains required.

## Capability boundary

The reusable TLSR8258 backend currently supports the Zigbee end-device path:

- active/passive/energy scan;
- association;
- data request polling;
- unicast data TX/RX;
- local association-state clear and MAC reset;
- mandatory platform timing and delay services.

Coordinator and router roles are not advertised. Secure entropy is not yet
provided by the backend, so operations that require it fail explicitly.

The preferred feature is:

```toml
zigbee-mac = { features = ["tlsr8258"] }
```

The former `telink` feature remains only as a backward-compatible alias for
`tlsr8258`; it does not imply B91 support.

## Toolchain installation

The toolchain is not committed to the repository. Install the current local
default release into the ignored `.toolchains` directory:

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

The scripts also accept an external extraction:

```bash
cd examples/telink-tlsr8258-sensor
TC32_TOOLCHAIN=/path/to/tc32-stage2-toolchain \
  ./scripts/tlsr8258.sh build runtime-sensor
```

The dedicated tc32 GitHub workflow downloads a public modern-tc32 release and
builds the real `tc32-unknown-none-elf` target; it does not use a Cortex-M
stand-in.

## Building

Build the reusable backend with the full stack in the radio harness:

```bash
cd examples/telink-tlsr8258-radio
./scripts/tlsr8258.sh build runtime-join
```

Build the current end-to-end runtime gate separately:

```bash
cd examples/telink-tlsr8258-sensor
./scripts/tlsr8258.sh build runtime-sensor
```

The runtime-gate output files are:

```text
target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor
target/tc32-unknown-none-elf/release/telink-tlsr8258-sensor.bin
```

The current runtime image is larger than the 256 KiB production/OTA slot, so
the post-link checker emits a warning while still enforcing the factory-data,
cache, DMA, BSS, and stack boundaries.

## Hardware status

The TLSR8258 end-device path has been proven with Home Assistant ZHA and an
Ember coordinator:

- active scan and coordinator beacon parsing;
- association and indirect polling;
- secured network-key delivery;
- Request-Key, unique TCLK transport, Verify-Key, and Confirm-Key;
- normal ZDO/ZCL interview;
- battery, temperature, humidity, and Identify entities;
- reset resume with crash-safe journaled security-counter reservations.

Remaining production work includes a clean small application entry point,
factory-reset UI validation, deep-sleep/retention reconfiguration, the Zbit
flash voltage guard, and reducing the image to the OTA slot.

## Telink B91

`examples/telink-b91-sensor` is an unsupported scaffold. The previous
documentation incorrectly described the TLSR8258 backend as a shared B91
driver and claimed that a complete `libdrivers_b91.a` FFI path existed.

A B91 implementation must start with a separately proven B91 radio HAL and
then implement `RadioPhy` plus the mandatory platform services. Until that
work exists, B91 is not built in CI and no firmware artifact is published.
