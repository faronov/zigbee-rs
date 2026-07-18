# Telink TLSR8258 Zigbee sensor

This package is the current end-to-end TLSR8258 hardware gate for join,
Trust Center exchange, ZHA interview, reporting, persistence, reset resume,
and secured parent rejoin.

`runtime-sensor` builds the dedicated `telink-tlsr8258-runtime` binary over
the reusable `tlsr8258-hal` radio and `zigbee_mac::telink::TelinkMac`. The
large local diagnostic radio and `Tlsr8258Mac` compile only into the separate
`telink-tlsr8258-lab` binary selected by legacy and diagnostic modes.

`runtime-router` builds `telink-tlsr8258-router` — an **EXPERIMENTAL**,
join/relay-only router firmware over the same reusable
`zigbee_mac::telink::TelinkMac`. See "Router firmware" below before treating
it as a complete Zigbee router.

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
| `runtime-router` | **EXPERIMENTAL** join/relay-only router — see "Router firmware" below |
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
./scripts/tlsr8258.sh build runtime-router
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build diag-assoc
./scripts/tlsr8258.sh build diag-beacon
```

The release profile and helper optimize the complete firmware for size:

```text
-C lto=fat -C opt-level=s -C codegen-units=1
```

The large-image failure previously attributed to compiler code generation was
reduced to delayed TX-done to RX activation. The radio now enters RX
synchronously at TX completion, matching the official Telink ACK-turnaround
sequence.

Outputs:

```text
target/tc32-unknown-none-elf/release/telink-tlsr8258-runtime
target/tc32-unknown-none-elf/release/telink-tlsr8258-runtime.bin
```

The current runtime image is 213,988 bytes (`0x343E4`), below the 256 KiB
production/OTA slot and the security-journal boundary at `0x74000`.

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
./scripts/tlsr8258.sh pgm-dump 0x74000 512
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
- reset resume with monotonic reserved global and TCLK counters;
- secured unicast rejoin and indirect Rejoin Response polling after a parent
  requests rejoin;
- successful ZHA Identify after reset and parent re-registration.

## Remaining work

- Remove the legacy local radio/MAC source after its remaining diagnostic
  modes have moved to the reusable harness.
- Remove the remaining application-owned SRAM markers and HA probe logic.
- Add production factory-reset UI and hardware validation.
- Add retention/deep-sleep radio reconfiguration and the Zbit flash voltage
  guard.
- Reduce the image below the 256 KiB OTA slot.

## Router firmware (`runtime-router`) — EXPERIMENTAL, non-parenting

`telink-tlsr8258-router` is the TLSR8258 analogue of
`examples/nrf52840-router`: an always-on FFD that joins an existing network
with the router capability bit and exercises NWK relay paths. It reuses the
same proven boot vector, IRQ glue, production linker layout
(`memory-runtime.x`), and security journal as `runtime-sensor` — only the
application module (`src/runtime_router.rs`) and device role differ.

**What it does:**

- Joins with the router capability bit (`DeviceType::Router`,
  `PowerMode::AlwaysOn`, `rx_on_when_idle` always on — never sleeps).
- After joining, `zigbee-nwk`'s `Nlme::nlme_start_router()` drives
  `zigbee_mac::telink::TelinkMac::mlme_start` into non-beacon (`BO=SO=15`),
  non-PAN-coordinator, continuous-RX mode.
- Relays unicast NWK frames addressed elsewhere and rebroadcasts
  broadcast/route-request traffic (existing generic `zigbee-nwk` forwarding,
  unchanged by this firmware).
- Sends periodic NWK Link Status broadcasts (existing generic
  `zigbee-runtime`/`zigbee-nwk` tick behavior).
- Persists join/security state across resets and secure-rejoins on request,
  exactly like `runtime-sensor`.

**What it explicitly does NOT do — and must not silently grow:**

- **No child association.** `TelinkMac` never implements
  `MLME-ASSOCIATE.response`. Nothing fakes it here.
- **No beacon transmission.** `macBeaconOrder`/`macSuperframeOrder` are
  fixed at 15 by `TelinkMac::mlme_start`'s parameter validation
  (`zigbee_mac::telink::validate_router_start`); beaconed superframes are
  rejected with `MacError::Unsupported`.
- **No permit-joining.** `macAssociationPermit` is never set `true`.
- **No indirect transmission / pending-frame queue** for sleepy children —
  `TelinkMac::mcps_data` rejects `tx_options.indirect`.
- **No PAN-coordinator / network-forming support.** `mlme_start` rejects
  `pan_coordinator: true` requests.

`zigbee_mac::telink::TelinkMac::capabilities().router` intentionally
reports `false`: that flag describes the ability to admit and serve child
devices, which this backend genuinely lacks, not "can relay frames on an
existing route" (which `mlme_start` now enables).

The router uses a distinct IEEE address from `runtime-sensor`. When switching
the same board between those firmware roles, the runtime clears the stale
commissioned identity while preserving both outgoing security-counter bounds,
then performs a fresh join. This avoids both restoring a journal for the wrong
IEEE address and reusing secured nonce/counter space.

Build and flash exactly like the sensor runtime:

```bash
cd examples/telink-tlsr8258-sensor
./scripts/tlsr8258.sh check runtime-router
./scripts/tlsr8258.sh build runtime-router
./scripts/tlsr8258.sh flash runtime-router
```

Output:

```text
target/tc32-unknown-none-elf/release/telink-tlsr8258-router
target/tc32-unknown-none-elf/release/telink-tlsr8258-router.bin
```

The router image is 212,256 bytes (`0x33D20`), below the 256 KiB
production/OTA slot and the security-journal boundary at `0x74000`.

### Router remaining gaps

- Home Assistant ZHA hardware testing confirms FFD join, Basic/Identify
  interview, Identify commands, and commissioned-state restoration after a
  hardware reset.
- A routed third-node traffic test and long-duration relay stability run are
  still outstanding.
- Child association, beaconing, permit-joining, and indirect transmission
  are not implemented at the MAC layer; adding them requires independent
  hardware bring-up of `MLME-ASSOCIATE.response`, beacon TX, and a pending
  (indirect) frame queue in `tlsr8258-hal`/`zigbee-mac`, which is out of
  scope here.
- Coordinator (PAN-forming) support is out of scope for this backend.
