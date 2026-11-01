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
table. The hand-written router MAC it replaced was hardware-tested; the
shared-core router image is build-, layout- and symbol-checked only (see the
MAC unification section of the firmware-size chapter).

The board owns fitted LEDs, flash/ADC resources, and physical wiring. The
product owns identity, profile, battery/poll behavior, sleep policy,
partitions, linker layout, and retention restoration policy.

The example's `src/app.rs` is composition/storage glue required by TC32 and
reset-on-wake entry. Commissioning, polling, reporting, persistence, and
router lifecycle are in `apps/sensor-sed` and `apps/router`, not a local
application state machine.

## Sensor application

The default image constructs `SensorApp` with:

- `TelinkSuspendWake`;
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
steady-state slow poll.

These are explicit proof images, not silent replacements for the default
SUSPEND product. Retained application/MAC/AES/ADC state is reconstructed before
a fresh root future calls the finite `step()` lifecycle.

The former independent retention lab (`diag-retention`, completion marker
`0x5254600D`) was removed together with `tools/telink-tlsr8258-lab`. A LOW32K
hardware proof must now be run on the `sensor-retention` images themselves:
repeated reset-on-wake cycles with retained network state, frame counters and
continued ZCL reporting. A successful build or symbol check is not that
proof.

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
- the default 16 KiB SVC / 1 KiB IRQ stack layout.

CI also runs `tools/check-role-symbols.sh`: `leaf` for the sensor and the ED,
`parent` for the router.

Production and radio-harness configurations enable LLVM tail merging. The
October 4 rebuild measured **195,688 B** for the default sensor and
**288,508 B** for the router, cumulative reductions of 32.4% and 33.8% from
the old `tc32-45` builds. See the
[tail-merge comparison](../advanced/firmware-size.md#tlsr8258-tail-merging-2026-10-04)
for both LOW32K variants, the separate compiler-only improvement, and physical
headroom. No features or physical memory safeguards were removed.
These new images have not been run on hardware. The later move of the sensors
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

The retained LOW32K fresh-root SVC stack is 8,448 B, 256 B above its 8 KiB
gate.

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
gates below remain. The existing TC32 toolchain was not modified. The
`SoftMacCore` sensor images (commit `34513de`) are build/symbol-checked only;
no TB-04 has run them yet. Earlier TB-04 images, all on the hand-written MAC,
produced hardware evidence for:

- hardware AES KAT, secured commissioning, TCLK exchange, ZHA interview, and
  sustained traffic;
- deployed security persistence;
- timer SUSPEND primitive;
- router join, durable restart, Link Status, and NWK relay.

Remaining gates:

- repeated application-level default SUSPEND acceptance with network/counter
  checks and measured current;
- the sensors on `SoftMacCore`: join, polling, ACK handling, and reporting on
  hardware;
- LOW32K proof completion on the `sensor-retention` images;
- pad/comparator wake modes;
- corrected-image first-attempt child admission and complete interview;
- the always-on ED: join, interview, and resume after reset on hardware;
- the `SoftMacCore` parent router: join, ZHA interview, neighbour/LQI, a child
  joining through it, and the stack high-water on the persistence paths;
- long-duration router and sleep stability.
