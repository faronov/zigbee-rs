# TLSR8258 Zigbee sensor

Pure-Rust TB-04 environmental sleepy end device using the shared
`sensor_sed_app::SensorApp`.

## Ownership

```text
environmental Sleepy End Device profile
        ↓
products/tlsr8258-tb04
        ↓
boards/tlsr8258-tb04
        ↓
tlsr8258-hal + TelinkMac + tlsr8258-rt
```

This example is the composition/reset root. `src/app.rs` assembles
borrowed/static TC32 resources and reset-on-wake entry.
It does not contain a separate commissioning, polling, reporting, or
persistence state machine.

The profile exposes Basic, Power Configuration, Identify, Temperature, and
Relative Humidity. Temperature/humidity are synthetic.

## Default power path

The default product policy is:

```text
fast wait: Active
slow wait: Idle
slow poll: 250 ms
```

`Idle` is an atomic full-SRAM timer `SUSPEND` transaction:

1. quiesce MAC/RF/DMA;
2. enter SUSPEND using the calibrated RC32K wake source;
3. restore clocks, timer, radio, DMA, AES, and MAC state;
4. return to the shared `step()` lifecycle.

The default sensor does enter SUSPEND. LOW32K retention is a separate
feature-gated image.

## LOW32K proof images

```bash
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
```

- `sensor-retention`: reset-on-wake `Retention`, 250 ms slow cadence.
- `sensor-retention-10s`: the same restore path, 10-second slow cadence.

The independent retention HIL uses:

```text
pass: 0x5254600D
fail: 0xDEADxxxx
```

A successful build or symbol check is not the pass marker.

## Build

Install `tc32-stage2-tc32-45` under
`.toolchains/tc32-stage2-tc32-45`, then from the repository root:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
```

Recorded images and former regression budgets:

| image | bytes | former regression budget |
|---|---:|---:|
| default SUSPEND | 290,616 | 294,912 |
| LOW32K 250 ms | 295,548 | 299,008 |
| LOW32K 10 s | 295,552 | 299,008 |

These artificial budgets are no longer enforced. The physical `0x70000`
application boundary, protected journals/factory data, RAM, and stack checks
remain mandatory.

The retained fresh-root SVC stack is 8,448 B, 256 B above its 8 KiB gate.

## Storage

The product security journal is `0x74000..0x76000`. Factory EUI/config remain
at `0x76000..0x78000`. The sensor drops the router-only APS-table and
child-table partition tokens. The product selects the orthogonal
`compact-single-endpoint` capacity only after asserting that its one endpoint
and report set fit the two-endpoint/four-report limits.

## Validation

The exact images above are build/layout-tested. Earlier hardware evidence
exists for the secured TB-04 Zigbee/AES/persistence path and the timer SUSPEND
primitive. Remaining acceptance includes repeated
application-level SUSPEND with network/counter checks and measured current,
plus LOW32K completion using the marker above.

Hardware diagnostics stay under `tools/telink-tlsr8258-lab`.
