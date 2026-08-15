# TLSR8258 Zigbee Router

A pure-Rust always-on TLSR8258 router/relay example. It is intentionally a
separate crate from the end-device sensor, matching the Nordic example layout.

## What to read

```text
src/main.rs      reset glue and application selection
src/app.rs       router role, commissioning, receive and maintenance loop
../../boards/tlsr8258-tb04
                 TB-04 LEDs, flash token, and typed resources
../../products/tlsr8258-tb04
                 security/child partitions, journals, and linker layout
```

The firmware joins as an FFD, keeps the radio in continuous receive, relays
NWK traffic, and sends normal router maintenance frames. The bounded parent
path handles beacon requests, permit joining, child association, and indirect
delivery to polling sleepy children.

Authenticated children are restored from a dedicated crash-safe journal before
the router answers orphan notifications. Parent Announce then reconciles stale
records with other routers; route state remains RAM-only.

The application drives the stack from bounded RX events and processes every
`StackEvent`/`TickResult`, including failed commissioning, leave/rejoin, and
`RunAgain` deadlines. The current TLSR8258 radio backend still implements each
bounded RX event with register polling; RF-IRQ wakeup is a separate remaining
power/latency optimization.

Production join/interview timing is exported through the retained
`TELINK_JOIN_METRICS` RAM symbol. It records association, Transport-Key,
security reservation, `Device_annce`, network-up, TCLK completion, and the
first Node/Active Endpoint/Simple Descriptor requests. The appended security
diagnostics include Request-Key/Verify-Key attempts, the exact outgoing
Verify-Key count, the legacy APS security frame-counter field (zero for the
R22-compliant APS-unsecured command), Confirm-Key status/source/security
fields, and ZDO response success/failure counters. Reading SRAM through SWire
halts the CPU and is restart-intrusive on the tested programmer, so stop the
acceptance capture before inspecting RAM or flash and treat every SWire read as
another reset/resume test.

ZiGate v3.23 hardware acceptance confirms the corrected path on the first
association: NWK-secured `Verify-Key` used APS frame control `0x41` with no APS
auxiliary security header, ZiGate returned an authenticated successful
`Confirm-Key`, and commissioning completed in 1.982 seconds without a Leave or
reassociation. The legacy Verify-Key APS security-counter diagnostic remained
zero.

### Receive-queue overload accounting

TLSR8258 has no hardware MAC address filter, so on a busy channel the radio
interrupts for every CRC-valid frame on air regardless of destination. The
last three fields of each queue's group make an overload attributable to one
exact stage, and every field is cumulative since boot (never cleared by a
rejoin):

| Field | Meaning |
|-------|---------|
| `radio_rx_queue_overflow` | frames lost at the HAL interrupt queue |
| `radio_rx_queue_evicted` | subset of the above where a lower-priority queued frame was sacrificed to keep a more valuable newly arrived frame |
| `radio_rx_queue_high_water` | deepest the 16-slot HAL queue ever got |
| `mac_data_queue_overflow` / `mac_data_queue_evicted` / `mac_data_queue_high_water` | the same three for the MAC's own retained `McpsDataIndication` queue |
| `mac_command_queue_overflow` / `mac_command_queue_high_water` | the MAC command-event queue, which has no lower-value class to evict |

`overflow - evicted` is the number of arrivals dropped outright at that
stage; that difference is the number that has to stay at zero. A
`high_water` pinned at the capacity while that difference rises means the
capacity is genuinely too small rather than the drain being too slow.

Home Assistant ZHA and an independent channel-15 sniffer have verified:

- association, Transport-Key, Device_annce, interview, and TCLK exchange;
- normal Link Status transmission and relay of NWK frames from other nodes;
- durable restart on the same short address with monotonic security counters;
- an Identify command and Default Response after restart;
- zero Beacon Requests, Association Requests, and Device_annce broadcasts
  during the accepted silent-resume capture.

Parent mode still requires a child-device sniffer gate. Verify ACK turnaround
and Frame Pending on child polls, deferred Association Response delivery, and
indirect NWK delivery before claiming child interoperability.

## Build

```bash
./scripts/tlsr8258.sh build router
```

The flashable image is:

```text
examples/telink-tlsr8258-router/target/tc32-unknown-none-elf/release/telink-tlsr8258-router.bin
```
