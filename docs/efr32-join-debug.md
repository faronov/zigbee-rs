# EFR32MG1 Join Debug Runbook

This runbook captures the local EFR32MG1 Zigbee join workflow used with Home
Assistant ZHA, an nRF52840 baseline device, Commander CLI, RTT logs, register
dumps, and an 802.15.4 sniffer.

## Current Goal

The EFR32MG1 device sometimes reaches the coordinator, but ZHA interview and
configuration do not reliably complete. Debug by proving each layer in order:
radio scan, MAC association, NWK join, APS transport-key delivery, ZDO
interview, then ZCL configuration/reporting.

Use Home Assistant ZHA as the primary coordinator backend for this workflow.
Use the nRF52840 example as the first baseline before changing EFR32 behavior.

## Tools

Local repo:

```bash
cd /Users/afaronov/zboss_opensource/zigbee-rs-fork
```

EFR32 Commander CLI:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli
```

EFR32 target:

```text
EFR32MG1P132F256
```

Local Spinel sniffer script:

```bash
/usr/local/bin/sniffer.py
```

Observed serial candidates:

```bash
ls /dev/cu.*
```

Previously seen candidates:

```text
/dev/cu.usbmodem14101
/dev/cu.usbmodem14201
/dev/cu.URT1
```

## Home Assistant ZHA

Confirm ZHA services are available through the Home Assistant MCP tools. The
important service is `zha.permit`.

Open permit-join for a test window:

```text
ha_call_service(domain="zha", service="permit", data={"duration": 254})
```

Collect HA logs before and after each attempt:

```text
ha_system_log()
```

The useful ZHA/zigpy/bellows evidence is the first error after the device joins
or after interview starts. Do not classify the EFR32 failure from Home Assistant
logs alone; always pair HA logs with RTT and sniffer capture.

## nRF52840 Baseline

Run this first. If the nRF baseline cannot join and complete interview, debug
the coordinator/network before changing EFR32 code.

```bash
cd /Users/afaronov/zboss_opensource/zigbee-rs-fork/examples/nrf52840-sensor
cargo run --release
```

Expected baseline:

- ZHA sees the device join.
- Interview completes.
- At least one sensor entity appears.
- Reports arrive after pairing.

## Sniffer Capture

Start capture before flashing or resetting the device. Use the coordinator
channel, starting with channel 15 if that is the known test channel.

```bash
python3 /usr/local/bin/sniffer.py \
  --uart /dev/cu.usbmodem14101 \
  --channel 15 \
  --output /tmp/efr32-diag-join-ch15.pcap \
  --crc \
  --rssi
```

If `/dev/cu.usbmodem14101` is not the sniffer, inspect `/dev/cu.*` and retry
with the other modem device. Wireshark is installed, but the local Wireshark
extcap directory currently only has generic extcaps, so this `sniffer.py` path
is the practical local capture route.

In Wireshark, filter by PAN/channel and add the Zigbee network key if encrypted
NWK/APS decoding is needed.

## EFR32 Builds

Build from the example directory:

```bash
cd /Users/afaronov/zboss_opensource/zigbee-rs-fork/examples/efr32mg1-sensor
```

Beacon-only radio diagnostic:

```bash
cargo build --release --no-default-features --features diag-beacon
```

Full join diagnostic without sensor workload:

```bash
cargo build --release --no-default-features --features diag-join
```

Trace build for targeted instrumentation only:

```bash
cargo build --release --no-default-features --features 'diag-join trace'
```

Do not use the trace build as the default pass/fail test. A previous trace run
changed timing enough to produce repeated scan TX timeouts:

```text
TX timeout: FRC_IF=0x0 RAC_st=6
```

## Flash And RTT

Flash the built EFR32 image:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli flash \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  --device EFR32MG1P132F256
```

Read RTT logs:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli rtt connect \
  --device EFR32MG1P132F256
```

Use `diag-beacon` first. Success means RTT shows beacon counts and parsed PAN
descriptors, and the sniffer shows beacon requests and coordinator beacons.

Then use `diag-join`. Success means RTT reaches `network_steering=ok`, ZHA
sees the join, and the sniffer shows association, transport-key delivery,
device announce, and interview traffic.

## Register Dumps

Use these snapshots when scan or TX/RX state looks wrong:

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli readmem \
  --device EFR32MG1P132F256 \
  --range 0x40080000:+0x200
```

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli readmem \
  --device EFR32MG1P132F256 \
  --range 0x40083000:+0x100
```

```bash
/Applications/Commander-cli.app/Contents/MacOS/commander-cli readmem \
  --device EFR32MG1P132F256 \
  --range 0x40084000:+0x180
```

Address map:

| Block | Base | Purpose |
|-------|------|---------|
| FRC | `0x40080000` | Frame controller |
| SYNTH | `0x40083000` | PLL/frequency synthesizer |
| RAC | `0x40084000` | Radio controller/state machine |

## Failure Classification

Use RTT and PCAP together:

| Symptom | Likely Layer | Next Check |
|---------|--------------|------------|
| No EFR32 TX visible in PCAP | EFR32 radio/TX | FRC/RAC state, TX completion, IRQ enable |
| Beacon requests visible, no beacons received | RF/channel/filtering | Channel, PAN filtering, RX restart after TX |
| Association request visible, no association response handled | MAC RX | Address filtering, RX window, ACK/data timing |
| Association succeeds, no transport-key processed | Poll/indirect APS | Data request timing, pending frame delivery, APS security |
| Device announce visible, interview fails | ZDO/ZCL | Node/simple descriptor responses, Basic/Identify clusters |
| Works once then scans fail | Radio state cleanup | Reset MAC/radio state before retry |

Do not permanently force `rx_on_when_idle=true` for a sleepy end device. It is
acceptable only as a controlled diagnostic variant because it changes how the
coordinator delivers indirect traffic.

## Acceptance Criteria

- `cargo check --workspace` passes after code changes.
- nRF52840 baseline joins ZHA and completes interview.
- EFR32 `diag-beacon` reliably sees coordinator beacons.
- EFR32 `diag-join` either completes ZHA interview or produces a classified
  failing frame/layer in RTT plus PCAP.
- Any future code patch is tied to the failing layer identified above.
