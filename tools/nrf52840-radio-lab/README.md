# nRF52840 Dongle radio lab (sniffer + raw transmitter)

Lab firmware for the Nordic nRF52840 Dongle (PCA10059). It replaces
Nordic's *nRF Sniffer for 802.15.4* with a pure-Rust image. The image
receives promiscuously, like Nordic's sniffer, and can also transmit raw
IEEE 802.15.4 frames on request.

It is a test instrument, not a shipped image. It is not part of any product
and does not run the Zigbee stack. CI builds it, lints it and checks its
layout. See [Validation](#validation) for what has run on hardware.

## Compatibility

- USB VID:PID `1915:154B` and product string `nRF 802154 Sniffer`, as in
  Nordic's firmware. The manufacturer string is `zigbee-rs`. The serial is
  derived from FICR `DEVICEID` in Zephyr's format.
- The CDC ACM text protocol is a superset of Nordic's v0.8.0 shell, so the
  Wireshark extcap (`nrf802154_sniffer.py`) and hwlab's `nrf802154` capture
  keep working unchanged. Nordic's commands are `sleep`, `channel [N]`,
  `receive` and `shell echo on|off`. Frames are reported as:

  ```text
  received: <psdu hex incl. FCS> power: <rssi dBm> lqi: <lqi> time: <µs since boot>
  ```

  `time` is taken at the end of the SFD (`RADIO.ADDRESS`, captured by PPI on
  TIMER0), which is the same reference point Nordic's firmware uses.
- The radio does not filter addresses and never sends an auto-ACK. Frames
  with a bad FCS are counted (`stats`) and not printed.

## Additional commands

| Command | Reply |
|---|---|
| `version` | `version: zigbee-rs-radio-lab <ver> protocol: 1 caps: receive,transmit,cca,ack-wait,repeat,power,stats` |
| `help` | command summary |
| `power` / `power <dBm>` | `power: N`; setting it is silent. Levels: -40, -20, -16, -12, -8, -4, 0, 2..8. The default is 0 dBm. |
| `stats` | `stats: rx: .. crc_error: .. overflow: .. tx: ..` |
| `transmit <hex> [cca] [ack] [repeat <n> <interval_ms>]` | see below |

`transmit` takes a raw MPDU without the FCS, 1–125 octets. The radio
appends the FCS (CRC-16/ITU-T). By default it does a single shot with no CCA
and no ACK wait.

- `cca` runs an energy-detect CCA first. If the channel is busy, the result
  is `cca_busy` and the frame is not sent.
- `ack` listens for an Imm-Ack whose sequence number matches octet 2 of the
  MPDU.
- `repeat <n> <interval_ms>` sends up to 1000 copies, at least 5 ms apart.
  The interval is timed by embassy-time on RTC1, which runs from the
  uncalibrated internal RC oscillator, so it can be off by about 1 %
  (100 ms measured as 99.3 ms). The `time:` stamps come from TIMER0 on the
  32 MHz crystal and are accurate.
  Any new command line aborts the burst; the summary then says
  `result: aborted`, and the command runs afterwards.

Each attempt prints the frame as sent (only when it actually went on air)
and then its result. Received frames keep being printed in between:

```text
transmitted: <psdu hex incl. FCS> channel: <ch> power: <dBm> time: <µs>
tx: 1/1 status: sent|acked|no_ack|cca_busy|error time: <µs>
tx_done: attempts: 1 sent: 1 acked: 0 no_ack: 0 cca_busy: 0 error: 0 result: ok
```

Reception, when it is on, resumes after every attempt on the same channel.
Nordic's firmware answers `version` with `version: command not found`. Host
tools use that difference to detect whether transmit is available.

Example: a MAC beacon request on channel 26, which is harmless:

```text
channel 26
receive
transmit 0308a5ffffffff07
```

## Build

```sh
cd tools/nrf52840-radio-lab
cargo build --release
rust-objcopy -O ihex target/thumbv7em-none-eabihf/release/nrf52840-radio-lab radio-lab.hex
```

`memory.x` keeps the dongle's factory layout:

| Region | Range |
|---|---|
| MBR | `0x0000`–`0x1000` |
| Application | `0x1000`–`0xE0000` |
| Open USB DFU bootloader | `0xE0000` |
| MBR params and bootloader settings | top two pages |
| RAM | from `0x20000008` (the MBR reserves the first word) |

## Flash (Open USB DFU, no debugger)

1. Press the dongle's side RESET button. The red LED pulses and the device
   enumerates as `1915:521F` (Open DFU bootloader).
2. Flash, using either of these:
   - hwlab: `device_invoke nrf802154 dfu_program` with the hex (see the hwlab
     docs).
   - nrfutil:

     ```sh
     nrfutil nrf5sdk-tools pkg generate --hw-version 52 --sd-req 0x00 \
       --application-version 1 --application radio-lab.hex radio-lab-dfu.zip
     nrfutil nrf5sdk-tools dfu usb-serial -pkg radio-lab-dfu.zip -p <bootloader port>
     ```

## Restore Nordic's sniffer

Nordic *nRF Sniffer for 802.15.4* v0.8.0
(github.com/NordicSemiconductor/nRF-Sniffer-for-802.15.4, commit `e459feb`).
The dongle image is `nrf802154_sniffer/nrf802154_sniffer_nrf52840dongle.hex`
(sha256 `13f957ca685e47b0c87f5088847dc729a8b7d17432327599bf1b61f3d357800a`).
Package and flash it the same way as above. Use an `--application-version`
that is not lower than the version currently installed, in case the
bootloader enforces downgrade protection.

## Validation

Hardware: one PCA10059 dongle (USB serial `7E2F90FB8DD40A98`) on 2026-11-14,
flashed through its Open DFU bootloader. The image was `0.1.0`, built from
this directory.

- **Proven on hardware:**
  - The open bootloader accepted the layout. The device enumerated with the
    same VID:PID, product string and serial as Nordic's firmware.
  - `version` probe: hwlab told the new firmware apart from Nordic's (which
    answered `version: command not found`).
  - Sniffing on channel 15 through hwlab's unchanged capture path: 641 frames
    in 21 s from PAN 0xDFE9, including ACKs, with RSSI, LQI and µs
    timestamps; tshark decoded them.
  - Transmit on channel 15 inside a capture: one beacon request (`sent`,
    FCS `6e21`). Within 3–130 ms twelve PAN 0xDFE9 routers answered with
    beacons, so independent receivers got the frame. The echoed frame
    appeared in the pcapng as an outbound packet.
- **Firmware report only:** transmit on channel 26 (3 frames `sent`, FCS
  `132d`), because no second receiver was on that channel.
- **Not yet run on hardware:** `cca`, `ack`, power levels other than 0 dBm,
  long bursts, the abort path, and the Wireshark extcap.

## Layout

| File | Contents |
|---|---|
| `src/radio.rs` | Register-level RADIO/TIMER0/PPI driver: an RX ring filled from the RADIO interrupt, and bounded polled TX with optional CCA and ACK wait. |
| `src/main.rs` | USB CDC ACM, command dispatch and LEDs. LED1 is the heartbeat: 1 s while asleep, 250 ms while receiving. Blue toggles for each received frame. Red is on during transmit. |
| `../radio-lab-protocol` | Command parser and line formatters, host-tested in the workspace. |
