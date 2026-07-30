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
                 security partition, journal, and linker layout
```

The firmware joins as an FFD, keeps the radio in continuous receive, relays
NWK traffic, and sends normal router maintenance frames. The bounded parent
path handles beacon requests, permit joining, child association, and indirect
delivery to polling sleepy children.

The child/neighbor table is currently RAM-only; children must re-associate
after a router reboot. Security/network commissioning state remains
crash-safely persisted.

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
Verify-Key APS frame counter, Confirm-Key status/source/security fields, and
ZDO response success/failure counters. Reading SRAM through SWire halts the
CPU and is restart-intrusive on the tested programmer, so stop the acceptance
capture before inspecting RAM or flash and treat every SWire read as another
reset/resume test.

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
