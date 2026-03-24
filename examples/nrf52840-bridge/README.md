# nRF52840 USB Serial Bridge

Thin firmware for the Nordic nRF52840 that exposes the 802.15.4 radio over a
USB CDC ACM serial port (`/dev/ttyACM0` on Linux, `COMx` on Windows).

The host-side Zigbee stack runs on a PC/Raspberry Pi and talks to this firmware
through the `zigbee-mac` crate's `serial` MAC backend. All MAC-layer intelligence
lives on the host — the dongle is a transparent radio pipe.

## Architecture

```
┌──────────────────────────────────┐       USB serial        ┌──────────────────┐
│  Host (Linux / macOS / Windows)  │◄══════════════════════►│  nRF52840 Dongle │
│                                  │   /dev/ttyACM0          │                  │
│  zigbee-mac serial backend       │   CMD_DATA_REQ ──►      │  802.15.4 radio  │
│  zigbee-nwk / aps / zcl / bdb   │   ◄── CMD_DATA_IND      │  (this firmware) │
└──────────────────────────────────┘                         └──────────────────┘
```

## Serial protocol

All communication uses a simple framed protocol:

```
START(0xF1) | CMD | SEQ | LEN_LO | LEN_HI | PAYLOAD[0..LEN] | CRC_LO | CRC_HI
```

- **START** — fixed `0xF1` sync byte
- **CMD** — command ID (see table in `src/main.rs`)
- **SEQ** — sequence number for request/response matching
- **LEN** — 16-bit LE payload length
- **PAYLOAD** — command-specific data
- **CRC** — CRC16-CCITT (poly `0x1021`, init `0xFFFF`) over CMD..PAYLOAD

### Key commands

| CMD  | Name          | Direction | Purpose                            |
|------|---------------|-----------|------------------------------------|
| 0x01 | RESET_REQ     | host→fw   | Reset radio hardware               |
| 0x02 | SCAN_REQ      | host→fw   | Energy/Active/Passive/Orphan scan  |
| 0x03 | ASSOCIATE_REQ | host→fw   | Send association request           |
| 0x04 | DATA_REQ      | host→fw   | Transmit raw 802.15.4 frame        |
| 0x05 | SET_REQ       | host→fw   | Set PIB attribute (channel, PAN…)  |
| 0x06 | GET_REQ       | host→fw   | Read PIB attribute                 |
| 0x07 | START_REQ     | host→fw   | Start PAN (coordinator mode)       |
| 0xC1 | DATA_IND      | fw→host   | Received 802.15.4 frame            |

Each request `0x0N` has a confirm `0x8N`.

## Building

```bash
cd examples/nrf52840-bridge
cargo build --release
```

Requires the `thumbv7em-none-eabihf` target:

```bash
rustup target add thumbv7em-none-eabihf
```

## Flashing

With a J-Link or DAPLink probe connected:

```bash
cargo run --release          # uses probe-rs (configured in .cargo/config.toml)
```

For the Nordic nRF52840 Dongle (PCA10059) without a debug probe, use
`nrfutil` to flash via the built-in USB bootloader:

```bash
# Convert ELF to hex
cargo objcopy --release -- -O ihex nrf52840-bridge.hex

# Package and flash (hold the dongle RESET button to enter bootloader)
nrfutil pkg generate --hw-version 52 --sd-req 0x00 \
    --application nrf52840-bridge.hex bridge-pkg.zip
nrfutil dfu usb-serial -pkg bridge-pkg.zip -p /dev/ttyACM0
```

## Connection to zigbee-mac serial backend

The `zigbee-mac` crate's `serial` backend (not yet implemented) will:

1. Open the USB serial port (e.g. `/dev/ttyACM0`)
2. Implement the `MacDriver` trait by translating each MAC primitive into the
   corresponding serial command:
   - `mlme_reset()` → `CMD_RESET_REQ`
   - `mlme_scan()` → `CMD_SCAN_REQ`
   - `mcps_data()` → `CMD_DATA_REQ`
   - …etc
3. Listen for `CMD_DATA_IND` indications (received frames) and surface them
   via `mcps_data_indication()`

This means **the full Zigbee stack runs on the host** (x86/ARM Linux), using
the dongle purely as a radio transceiver. This is the same architecture used by
Zigbee2MQTT with a CC2531/CC2652 USB stick.

## Status

🚧 **Skeleton** — the protocol handling and command dispatch logic are complete,
but the actual nRF52840 radio driver calls and USB CDC ACM setup are stubbed
with TODO comments. Contributions welcome!
