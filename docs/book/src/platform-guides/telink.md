# Telink TLSR8258

TLSR8258 uses the custom `tc32-1.98.1-20261003-31a272` Rust toolchain
(LLVM 23.1.2), targeting `tc32-unknown-none-elf`. The production
sleepy sensor, always-on End Device and router use the same shared
application frontends as the ARM and RISC-V products.

## Layering

```text
sensor, always-on End Device or parent-router product behavior
        ↓
products/tlsr8258-tb04
        ↓
boards/tlsr8258-tb04
        ↓
tlsr8258-hal + zigbee-mac (TelinkMac / TelinkRouterMac) + tlsr8258-rt
```

The sensor, ED and router `main.rs` files are the short composition roots.
Each role is its own crate (`examples/telink-tlsr8258-{sensor,ed,router}`)
rather than a bin of one board crate: the TC32 build script, the post-link
layout gates, the LOW32K retention features and the CI cache are keyed on the
crate and bin name. The product crate selects the role with its `sensor`,
`end-device` or `router` feature.

The sleepy sensor and the always-on ED use `TelinkMac`, which is the shared
`SoftMacCore<TelinkRadioPhy, EndDevice>`: scan, association, polling, frame
retries and ACK matching run in the portable core, while the HAL performs the
CSMA-CA backoff (with the PIB parameters), the transmission and the Imm-Ack
wait as one radio operation. The Imm-Ack itself is still generated in the
HAL RX-completion path, never from task context. The router uses
`TelinkRouterMac`, the same core in its parent role
(`SoftMacCore<TelinkRadioPhy, Parent<8, 8, 0>>`, eight retained data frames).
Beacon, association and orphan responses, command events and indirect data
come from the core; Frame Pending is armed in the HAL's ISR source-match
table. The shared-core router, ED and sensor images have run on a TB-04
against ZHA (see [Validation](#validation)); the MAC unification section of
the firmware-size chapter has their footprint.

The board owns fitted LEDs, flash/ADC resources, and physical wiring. The
product owns identity, profile, battery/poll behavior, sleep policy,
partitions, linker layout, and which LED state a LOW32K wake restores.

| Layer | Crate / file | Owns |
|---|---|---|
| runtime | `tlsr8258-rt` `retention` (feature `retention-proof`) | `RetainedCell`, the checksummed `RetainedRoot` commit record, retained-state tripwire, the `TELINK_RETENTION_IMAGE` layout marker |
| chip adapter | `apps/tlsr8258-sensor` (`tlsr8258-sensor-app`) | `TelinkWake` (Active/SUSPEND/LOW32K `WakeController`), retained MAC/AES/ADC platform record, wake resume and fail-closed reset, `TelinkSupervisor`, `TelinkNoDiagnostics`, `init_sleep_clocks` |
| product | `products/tlsr8258-tb04` `boot`, `sensor` | TB-04 bring-up (`boot`, `failure`, `halted`) shared by sensor/ED/router; sensor identity, `SENSOR_POLICY`, clusters, synthetic source, RGB status and its `RetainedIndicator` |
| example | `examples/telink-tlsr8258-sensor` | entries (`main.rs`), part list and device builder (`app.rs`), SUSPEND root (`suspend.rs`), LOW32K retained statics and fresh root (`retention.rs`) |

Commissioning, polling, reporting, persistence, and router lifecycle are in
`apps/sensor-sed` and `apps/router`, not a local application state machine.

## Sensor application

The default image constructs `SensorApp` with:

- `TelinkWake` (from `tlsr8258-sensor-app`);
- RGB semantic status;
- synthetic temperature/humidity;
- fixed battery;
- `NoOta`;
- product user actions and supervisor.

### Default SUSPEND policy

```text
fast wait: Active
slow wait: Idle
slow poll: 250 ms
```

`Idle` performs one interrupt-masked transaction:

1. quiesce MAC/RF/DMA;
2. enter full-SRAM timer `SUSPEND` using the calibrated RC32K source;
3. restore clocks, timer, radio, DMA, and MAC state;
4. return to `SensorApp`.

This is the default sensor image. Documentation that says the production
sensor does not enter suspend is obsolete.

### Feature-gated LOW32K images

`retention-proof` changes the slow depth to `Retention` and uses a reset-on-wake
LOW32K path with the existing 250 ms policy.

`retention-proof-10s` uses the same restoration path and a 10-second
steady-state slow poll. Both are one `SENSOR_POLICY` whose slow-poll and
slow-depth fields are selected by `cfg!`.

These are explicit proof images, not silent replacements for the default
SUSPEND product. Retained application/MAC/AES/ADC state is reconstructed before
a fresh root future continues the lifecycle.

A LOW32K wait never returns: the timer wake re-enters through
`_rust_retention_entry`. The fresh root therefore first calls
`SensorApp::resume_after_timer_wake()`, which completes the iteration that
slept (expired join backoff, due joined poll/report work), and only then
calls `step()`. Without it the root re-entered `step()` with the slept wait
still pending, so the next wait shrank until it fell below the LOW32K wake
floor (`MIN_SLEEP_TICKS`, 1.765 ms) and the image halted in `wake_failure`
with `PowerManagement(TooShort)`; an unjoined image also never retried
because its join backoff was never consumed. A wait below the floor is now
busy-waited in `TelinkWake` instead of being reported as an error.

The former independent retention lab (`diag-retention`, completion marker
`0x5254600D`) was removed together with `tools/telink-tlsr8258-lab`. A LOW32K
hardware proof must now be run on the `sensor-retention` images themselves:
repeated reset-on-wake cycles with retained network state, frame counters and
continued ZCL reporting. A successful build or symbol check is not that
proof.

Lab result (2026-11-03, TB-04, `sensor-retention` 250 ms, functional only):
fresh join through a third-party router, ZHA interview, about 21 minutes of
continuous 30 s temperature/humidity reports across LOW32K wakes, silent
resume after a hard reset with monotonic outgoing/TCLK frame counters, and an
unjoined image (permit closed for about 4.5 minutes) that kept retrying and
joined on its own once permit-join opened. Current was not measured.

## Router application

The TB-04 router uses:

```rust,ignore
ParentRouterApp::new(
    ZigbeeNode::new(device, security_store, profile),
    PersistentChildren::new(child_store),
    policy,
    RouterParts::new(status, supervisor, diagnostics),
)?;
```

`TelinkRouterMac` (the shared `SoftMacCore` parent role) implements the
parent primitives needed by `ParentMacDriver`.
`ParentRouterApp` owns steering/resume, bounded receive/tick processing,
security checkpoints, and child-table restore/save/clear.

The TB-04 board has no fitted user button, so nothing calls
`request_commissioning()`. After a Leave without rejoin the router stays
factory-new with the red LED solid and makes no network search; a power cycle
starts Network Steering again, retrying with the 5..60 s policy backoff. A
product built on a module with a button should map its short press to
`request_commissioning()`.

## Always-on End Device application

`examples/telink-tlsr8258-ed` is a mains-powered, RX-on-when-idle,
non-routing End Device:

```rust,ignore
AlwaysOnEndDeviceApp::<_, _, _, _, _, _, EdObserver>::new_observed(
    ZigbeeNode::new(device, security_store, profile),
    &END_DEVICE_POLICY,
    RouterParts::new(status, supervisor, NoDiagnostics),
)?;
```

It uses the end-device `TelinkMac` (no parent MAC, no `router` feature), and
the same IRQ-driven RX/auto-ACK path as the router, because the HAL services
receive in the IRQ only while `macRxOnWhenIdle` is set.
`products/tlsr8258-tb04` (`end-device` feature) owns:

- the identity: model `TLSR8258-ED`, IEEE offset `0x45`, distinct from the
  sensor (`0x33`) and router (`0x52`);
- the Basic + Identify profile: the TB-04 has no measured cluster to report;
- the 15..60 s retry policy;
- the security-journal role tag `END_DEVICE` (`0x5202`).

The sensor and router remain untagged legacy images. Their distinct IEEE
offsets still make a role switch reset the network state while keeping the
frame-counter limits. The ED reserves only the security journal; it has no
child or APS-table journal.

The ED shares the router's LED adapters (`status_leds`). Like the router, it
has no button. After a Leave it stays factory-new until the next power cycle.

`TELINK_ED_METRICS` (magic `TJED`, `#[no_mangle]`) exports the commissioning
results, network identity, uptime and HAL RX counters for debugger reads.

## Flash ownership

The 512 KiB TB-04 product preserves:

```text
0x00000..0x70000  application
0x70000..0x72000  APS binding/group/application-key journal
0x72000..0x74000  child-table journal
0x74000..0x76000  security journal
0x76000..0x77000  factory EUI-64
0x77000..0x78000  factory config and ADC calibration
```

The sensor keeps only the security token and drops the APS/child tokens, so
those persistence paths are removed; the always-on ED does the same. The router
consumes all three independent journal tokens.

Flash geometry is verified before factory-data access on non-512-KiB layouts.
Zbit writes require the ADC/PC5 voltage guard before every page program or
sector erase and fail closed on missing, unstable, or low voltage.

## Build

Install the [pinned release](https://github.com/modern-tc32/rust/releases/tag/tc32-1.98.1-20261003-31a272)
under `.toolchains/tc32-1.98.1-20261003-31a272`, stripping the archive's
top-level directory. Archive checksums are listed in
[`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/master/BUILD.md#telink-tlsr8258).
All build helpers honor `TC32_TOOLCHAIN`; Actions uses the same release and
verifies its Linux archive checksum, compiler version, and TC32 target.
Host tools remain on Rust `1.94.1`.

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
./scripts/tlsr8258.sh build router
./scripts/tlsr8258.sh build ed
```

`tools/tlsr8258-firmware.sh` gates the ED image on:

- no parent or router symbols;
- the End Device Timeout client symbols;
- no sleep or retention path;
- exactly one `tlsr8258_rt::block_on` root;
- the product-wide 20 KiB SVC (64 B guard below it) / 1 KiB IRQ stack layout.

CI also runs `tools/check-role-symbols.sh`: `leaf` for the sensor and the ED,
`parent` for the router.

Production and radio-harness configurations enable LLVM tail merging. The
October 4 rebuild measured **195,688 B** for the default sensor and
**288,508 B** for the router, cumulative reductions of 32.4% and 33.8% from
the old `tc32-45` builds. See the
[tail-merge comparison](../advanced/firmware-size.md#tlsr8258-tail-merging-2026-10-04)
for both LOW32K variants, the separate compiler-only improvement, and physical
headroom. No features or physical memory safeguards were removed.
The current tail-merged `SoftMacCore` images were run on hardware (see
[Validation](#validation)). The later move of the sensors
to `SoftMacCore` cut sensor flash by a further 1,480–1,616 B (see the MAC
unification section of the firmware-size chapter).

Recorded images (sensor baseline 2026-09-06; router refreshed 2026-09-10).
Former regression budgets are historical comparisons, no longer enforced:

| image | bytes | former regression budget | former budget minus bytes |
|---|---:|---:|---:|
| default SUSPEND sensor | 290,616 | 294,912 | 4,296 |
| LOW32K 250 ms proof | 295,548 | 299,008 | 3,460 |
| LOW32K 10 s proof | 295,552 | 299,008 | 3,456 |
| parent router | 436,072 | 430,080 | -5,992 |

The LOW32K images keep only state that must survive deep retention below
`0x848000`: `.ram_code`, `.data`, `.bss`, `.retained`, `.rf_dma` and a
256 B retained tripwire after the last DMA byte, verified before a wake is
accepted. The stacks do not need to survive sleep (every entry reloads both
stack pointers), so they live in the non-retained upper 32 KiB with the same
placement as the default image: a 24 KiB SVC stack `[0x849C00, 0x84FC00)`, a
64 B guard `[0x849BC0, 0x849C00)` re-armed on every cold and retention entry
and checked after every executor step and before every flash erase/program,
and the 1 KiB IRQ stack `[0x84FC00, 0x850000)`. The previous layout gave the
fresh root an 11,008 B SVC stack in LOW32K without a guard; hardware measured
about 11.7 KB on the join path (association, transport-key wait, soft-MAC
poll), which overran `.rf_dma` and retained statics and prevented joining.
`tools/tlsr8258-firmware.sh` now fails the LOW32K build unless the static
analyzer bounds both fresh-stack roots (`_rust_cold_entry`,
`_rust_retention_entry`) within the declared 24 KiB
(`tools/tlsr8258-sensor-retention-stack-gate.args`; ~21 KB analyzed).
Painted high-water on hardware with the new layout, measured at the first
retention transaction: 15,420 B after a cold resume from the journal and
15,444 B after a fresh join, both with the guard intact (about 9.1 KB
headroom). Warm-wake depth cannot be measured by halting, because LOW32K sleep
resets the debug block; it is bounded by the analyzer (20,104 B).

The router's old 356,352 B budget was a pre-R22 baseline. The former
430,080 B (`0x69000`) budget was introduced above the earlier 427,776 B image.
The recorded router exceeded it by 5,992 B and failed that historical gate.
The artificial budget is no longer enforced. Neither the mandatory physical
458,752 B (`0x70000`) application boundary nor any journal moved.
The recorded image has 22,680 B of physical headroom; the 28,672 B difference
between the former budget and physical boundary is not a reserved partition.

Radio bring-up diagnostics live in the standalone
`examples/telink-tlsr8258-radio` harness. It links against the same `0x70000`
application boundary as the product and fails its post-link checks at it, so
flashing it cannot overwrite the journals or factory data listed above.
The former `tools/telink-tlsr8258-lab` (a separate hand-written join stack
with `diag-*` and `lab-sensor` images) was removed; `./scripts/tlsr8258.sh
dump <address>` now forwards to the radio harness script.

## Validation

The recorded sensor images passed build/layout checks. The refreshed router
linked with the physical memory assertions and the durable Bind/Unbind path,
but its build command failed the regression-size gate enforced at the time.
That artificial gate has since been removed; physical limits and the hardware
gates below remain. The existing TC32 toolchain was not modified. Earlier
TB-04 images, all on the hand-written MAC, produced hardware evidence for:

- hardware AES KAT, secured commissioning, TCLK exchange, ZHA interview, and
  sustained traffic;
- deployed security persistence;
- timer SUSPEND primitive;
- router join, durable restart, Link Status, and NWK relay.

The `SoftMacCore` images were then run on one TB-04 against Home Assistant ZHA
(channel 15) through the hwlab SWire programmer:

| image | hardware result |
|---|---|
| sleepy sensor (`telink-tlsr8258-sensor`) | ZHA join, interview, 30 s Temperature/Humidity/Battery reports, resume after three resets |
| always-on ED (`telink-tlsr8258-ed`) | ZHA join, interview (End Device, mains), resume after reset |
| parent router (`telink-tlsr8258-router`) | ZHA join (association 911 ms, TCLK and commissioning 7.5 s), interview (Router, mains, Basic + Identify), resume after reset without a new association, >18 min stable; the SVC stack guard stayed intact and other routers list it as a neighbour |

Router counters 18 minutes after the reset: 41,071 valid frames, 0 ACK
timeouts, 0 channel-access failures, 52 busy CCAs out of 774, 0 DMA-incomplete.
The 16-slot HAL RX queue overflowed 5,566 times (945 by priority eviction)
around the join, resume and ZHA topology-scan bursts, at about 41 frames/s.
In steady state, losses fell to about 2 %. The router loop handles one frame
per iteration plus a tick. The overflow counter does not say whether a dropped
frame was addressed to the router.

A lab build with per-class queue counters on the joined router (ZHA, channel
15) then showed what filled the queue. About 87 % of the queued valid frames
could never reach a consumer: other nodes' MAC ACKs (the MAC consumes its own
ACKs inside the transmit, never from this queue) and unicasts to other nodes or
PANs (the MAC drops them on its destination check). The upper layers left the
queue undrained for up to ~0.8 s, mostly in the busy-wait NWK broadcast and
RREQ relay jitter. The outright drops then hit the broadcasts the router has to
act on: 16.7 % of them were lost. The soft MAC's own 8-slot retained-RX queue
was never the bottleneck (high-water 1, no evictions).

The HAL now classifies each frame after the software ACK and queues only
broadcasts and frames for this node. Promiscuous reception (scans) and an
unknown local identity still admit every well-formed frame. On the same
router, the base image lost 10.3 % of valid frames to overflow (8.6 % outright).
After a 31-minute soak, the fixed image lost 0.15 % of valid frames: 66 of
43,702, all outright, with no evictions. That is about 3 % of the queued
relevant frames, against 16.7 % of broadcasts before. The remaining
overflow comes in bursts with high-water 16. It matched the inline RREQ relay
schedule, which then blocked for up to 2·128 ms jitter plus two 254 ms
retries.

NWK broadcast and RREQ relays are now scheduled instead of waited out inline
(see the broadcast relay schedule in [NWK](../core-concepts/nwk.md)), so the
router loop keeps draining this queue during jitter and retry intervals. The
same router resumed as 0x565F without a new association, and Identify worked.
32 minutes after the reset, it had received 46,429 valid frames with 55
overflows (0.12 %, all outright), no evictions, and high-water 16. The image
it replaced had run for days and lost 744 of 30,524 valid frames (2.44 %, 13
evictions) in the 15 minutes just before the flash. Its lifetime rate was
0.92 %, so the channel traffic varies a lot between runs. A short burst
overflow remains, and no per-phase attribution has been run on the new image.
Frames that arrive while flash erase/write runs with interrupts disabled are
lost without being counted.

Remaining gates:

- repeated application-level default SUSPEND acceptance with network/counter
  checks and measured current;
- LOW32K proof completion on the `sensor-retention` images: measured sleep
  current, a multi-hour run, and ZHA device removal (a `zha.remove` was not
  observed to reach the SED through its third-party parent);
- pad/comparator wake modes;
- corrected-image first-attempt child admission and complete interview;
- the parent router: a child joining through it, its own neighbour table in
  ZHA, a paint-based stack high-water on the persistence paths, and
  attribution of the residual burst RX-queue overflow (0.12 %) that remains
  with the scheduled NWK relays;
- the one-off SED `security persistence` panic seen once and not reproduced;
- long-duration router and sleep stability.
