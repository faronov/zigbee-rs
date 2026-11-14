# Sniffing & Frame Injection

Radio claims need an independent receiver: ACK timing, a Green Power frame
reaching a proxy, a router answering a beacon request. zigbee-rs uses an
nRF52840 Dongle (PCA10059) driven by [hwlab](https://github.com/faronov/hwlab)
for this. The dongle runs one of two firmwares:

| Firmware | Receive | Transmit | Source |
|---|---|---|---|
| Nordic *nRF Sniffer for 802.15.4* v0.8.0 | yes | no | NordicSemiconductor/nRF-Sniffer-for-802.15.4 |
| zigbee-rs radio lab | yes | yes | `tools/nrf52840-radio-lab` |

The radio lab firmware is a test instrument. It does not run the Zigbee
stack and is not part of any product image. It keeps Nordic's USB identity
(`1915:154B`, "nRF 802154 Sniffer") and its text protocol, so Wireshark's
extcap and hwlab capture work with either firmware.

## Protocol

The radio lab speaks Nordic's protocol over USB CDC ACM:

- `sleep`, `channel N`, `receive`, `shell echo off`;
- received frames are reported as
  `received: <hex incl. FCS> power: <dBm> lqi: <n> time: <µs>`.

The radio receives everything (promiscuous) and never sends an ACK.
`time` is taken at the end of the SFD from a crystal-timed timer.

It adds these commands:

| Command | Purpose |
|---|---|
| `version` | `version: zigbee-rs-radio-lab 0.1.0 protocol: 1 caps: …` |
| `power [dBm]` | TX power, -40…+8 dBm, default 0 dBm |
| `stats` | received, CRC errors, overflows, transmitted |
| `transmit <hex> [cca] [ack] [repeat <n> <ms>]` | send a raw MPDU |

- **`transmit` input.** It takes the MPDU without the FCS, 1–125 octets, and
  the radio appends the FCS.
- **Defaults.** One frame, no CCA and no ACK wait. Reception resumes after
  every frame.
- **Output.** Each frame that goes on air is echoed as
  `transmitted: <hex incl. FCS> channel: N power: P time: T`. A
  `tx: i/n status: sent|acked|no_ack|cca_busy|error` line follows each
  attempt, and a `tx_done:` summary ends the command.
- **Detection.** Nordic's firmware answers `version` with
  `version: command not found`. That is how hwlab tells the two apart.

See `tools/nrf52840-radio-lab/README.md` for the full grammar.

## Building and flashing

```sh
cd tools/nrf52840-radio-lab
cargo build --release
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/nrf52840-radio-lab radio-lab.hex
```

The image keeps the dongle's factory layout:

- MBR at `0x0`;
- application at `0x1000`;
- Nordic's Open USB DFU bootloader at `0xE0000`.

No debugger is needed:

1. Press the dongle's side RESET button. The red LED pulses and the dongle
   enumerates as `1915:521F`.
2. Flash with one of these:
   - nrfutil:

     ```sh
     nrfutil nrf5sdk-tools pkg generate --hw-version 52 --sd-req 0x00 \
       --application-version 1 --application radio-lab.hex radio-lab-dfu.zip
     nrfutil nrf5sdk-tools dfu usb-serial -pkg radio-lab-dfu.zip -p <bootloader port>
     ```

   - hwlab's `FlashProgramming.dfu_program`, which takes the hex directly.
     This path is not yet proven on hardware; see below.

To restore Nordic's firmware, flash
`nrf802154_sniffer/nrf802154_sniffer_nrf52840dongle.hex` from Nordic's
repository (commit `e459feb`) the same way.

## Using it from hwlab

The hwlab `nrf802154` plugin probes the firmware when it opens the port.
`PacketCapture.firmware_info` reports the result. Only the radio lab
firmware gets the `PacketInjection` capability.

**Capture** is unchanged:

```sh
hwlab sniffer capture --channel 15 --duration 10s
```

**Transmit** is a dangerous operation and needs `--yes` (MCP: `confirm`).
For example, a beacon request:

```sh
hwlab invoke nrf802154/<serial> PacketInjection.transmit --yes \
  -a psdu='"030801ffffffff07"' -a channel=26 -a note='why'
```

Quote the PSDU. `-a` values are parsed as YAML, so an all-digit PSDU would
otherwise become a number.

The plugin enforces these guards:

- PSDU of at most 125 octets;
- channel 11–26;
- `repeat` ≤ 100, `interval_ms` ≥ 10 and bursts ≤ 30 s;
- 300 frames per minute per dongle.

Every request is appended to `~/.hwlab/artifacts/nrf802154-tx/audit.jsonl`
before anything is sent, and a result record follows.

While a capture runs on the dongle, transmit goes through that capture.
Omit `channel` or pass the capture's channel; the plugin will not retune a
capture. The injected frames are then recorded in two places:

- the pcapng, as outbound packets commented with the transmit id;
- the session timeline, as packet events with `direction=tx`.

This makes replay and fault-injection tests (for example a Green Power
replay or a bad-MIC copy) reviewable from one capture.

## Validation status

Hardware: one PCA10059 on 2026-11-14.

| Item | Level |
|---|---|
| Layout accepted by the Open DFU bootloader; same USB identity and serial | hardware |
| hwlab firmware detection, Nordic and radio lab | hardware |
| Sniffing on channel 15: PAN 0xDFE9 frames with RSSI, LQI and µs time | hardware |
| Transmit on channel 15 | hardware: a beacon request drew beacons from twelve independent routers within 3–130 ms |
| Transmit on channel 26 | firmware report only (no second receiver on that channel) |
| `cca`, `ack`, power ≠ 0 dBm, long bursts, abort | host-tested only |
| hwlab `dfu_program` | simulator-tested; the real dongle was flashed with the same DFU code in a standalone tool after the plugin call stalled |

Known gap: the interval between repeated frames comes from the dongle's
uncalibrated RC oscillator. It is accurate to about 1 %; 100 ms measured as
99.3 ms. The timestamps are crystal-accurate.
