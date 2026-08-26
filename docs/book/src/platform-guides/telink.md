# Telink TLSR8258

TLSR8258 uses the custom `tc32-stage2-tc32-45` Rust target. The production
sensor and router use the same shared application frontends as the ARM and
RISC-V products.

## Layering

```text
tlsr8258-hal + TelinkMac + tlsr8258-rt
        ↓
boards/tlsr8258-tb04
        ↓
products/tlsr8258-tb04
        ↓
sensor or router composition root
```

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

The independent retention lab reports:

```text
start: 0x52540000
cycles: 0x52542001..0x52542004
pass: 0x5254600D
failure: 0xDEADxxxx
```

Only `0x5254600D` is the HIL completion marker. A successful build or symbol
check is not the marker.

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

`TelinkMac` implements the parent primitives needed by `ParentMacDriver`.
`ParentRouterApp` owns steering/resume, bounded receive/tick processing,
security checkpoints, and child-table restore/save/clear.

## Flash ownership

The 512 KiB TB-04 product preserves:

```text
0x00000..0x72000  application
0x72000..0x74000  child-table journal
0x74000..0x76000  security journal
0x76000..0x77000  factory EUI-64
0x77000..0x78000  factory config and ADC calibration
```

The sensor drops the child-table token so child persistence code is removed.
The router consumes both independent journal tokens.

Flash geometry is verified before factory-data access on non-512-KiB layouts.
Zbit writes require the ADC/PC5 voltage guard before every page program or
sector erase and fail closed on missing, unstable, or low voltage.

## Build

Install the target toolchain under `.toolchains/tc32-stage2-tc32-45`. Host
tools use Rust `1.94.1`.

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
./scripts/tlsr8258.sh build router
```

Current images:

| image | bytes |
|---|---:|
| default SUSPEND sensor | 279,652 |
| LOW32K 250 ms proof | 284,436 |
| LOW32K 10 s proof | 284,440 |
| parent router | 343,660 |

Independent diagnostics remain under `tools/telink-tlsr8258-lab`:

```bash
./scripts/tlsr8258.sh build diag-pm
./scripts/tlsr8258.sh build diag-retention
```

## Validation

Hardware-proven on TB-04:

- hardware AES KAT, secured commissioning, TCLK exchange, ZHA interview, and
  sustained traffic;
- deployed security persistence;
- timer SUSPEND primitive;
- router join, durable restart, Link Status, and NWK relay.

Remaining gates:

- repeated application-level default SUSPEND acceptance with network/counter
  checks and measured current;
- LOW32K proof completion using the marker above;
- pad/comparator wake modes;
- corrected-image first-attempt child admission and complete interview;
- long-duration router and sleep stability.
