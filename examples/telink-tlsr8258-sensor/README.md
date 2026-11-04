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
tlsr8258-hal + TelinkMac (SoftMacCore<TelinkRadioPhy>) + tlsr8258-rt
```

This example is only the composition root:

- `src/main.rs`: `_rust_entry` / LOW32K wake entry;
- `src/app.rs`: the sensor part list and device builder;
- `src/suspend.rs`: default SUSPEND image root;
- `src/retention.rs`: LOW32K retained statics, cold start and fresh root.

What the sensor *is* (identity, `SENSOR_POLICY`, clusters, synthetic source,
RGB status) lives in `products/tlsr8258-tb04/src/sensor.rs`; TB-04 bring-up
in `products/tlsr8258-tb04/src/boot.rs`. Generic TLSR wake/retention
machinery is in `apps/tlsr8258-sensor` and `tlsr8258-rt` (`retention`).
It does not contain a separate commissioning, polling, reporting, or
persistence state machine.

The profile exposes Basic, Power Configuration, Identify, Temperature, and
Relative Humidity. Temperature/humidity are synthetic.

## Default power path

The default product policy is:

```text
fast poll: 250 ms, Active   (join/rejoin + 120 s, interview, Identify,
                             pending APS ack / ZCL response, OTA)
slow poll: 5 s, Idle        (joined steady state)
reports:   30 s sample cadence, independent of the poll
```

`SLOW_POLL_MS` (product crate) selects the joined slow poll; the production
value is `RESPONSIVE_SLOW_POLL_MS` from `sensor-sed-app`, the shortest common
parent indirect hold (6 s) minus a 1 s margin, so coordinator commands are
fetched before the parent expires them.

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

- `sensor-retention`: reset-on-wake `Retention`, 250 ms slow cadence (wake-cycle
  stress proof).
- `sensor-retention-10s`: the same restore path, 10-second slow cadence.

The former independent retention HIL image (pass marker `0x5254600D`) was
removed with `tools/telink-tlsr8258-lab`; LOW32K hardware proof has to run on
these images directly. A successful build or symbol check is not that proof.

## Build

Install `tc32-1.98.1-20261003-31a272` under
`.toolchains/tc32-1.98.1-20261003-31a272` (see the checksums and extraction
instructions in [`BUILD.md`](../../BUILD.md#telink-tlsr8258)), then from the repository root:

```bash
./scripts/tlsr8258.sh build sensor
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
```

LLVM tail merging is enabled with the pinned compiler. The October 4 local
builds measure 195,688 B (default), 198,764 B (250 ms LOW32K), and
198,768 B (10 s LOW32K). See the
[controlled comparison](../../docs/book/src/advanced/firmware-size.md#tlsr8258-tail-merging-2026-10-04);
these images have not been flashed.

Historical images and former regression budgets:

| image | bytes | former regression budget |
|---|---:|---:|
| default SUSPEND | 290,616 | 294,912 |
| LOW32K 250 ms | 295,548 | 299,008 |
| LOW32K 10 s | 295,552 | 299,008 |

These artificial budgets are no longer enforced. The physical `0x70000`
application boundary, protected journals/factory data, RAM, and stack checks
remain mandatory.

The LOW32K images keep only retained state (code, data, `.retained`, RF DMA
and a 256 B tripwire) below `0x848000`. Their fresh-root SVC stack is
24 KiB at `[0x849C00, 0x84FC00)` in non-retained SRAM, with a 64 B guard below
it that both entries re-arm and the IRQ stack above it; the build fails if the
static analyzer bounds either entry root above 24 KiB
(`tools/tlsr8258-sensor-retention-stack-gate.args`). Hardware paint measured
15,420 B (cold resume) and 15,444 B (fresh join) with the guard intact.

The default SUSPEND image keeps all SRAM retained and uses the leaf 24 KiB
SVC stack `[0x849C00, 0x84FC00)` with a 64 B guard below it, armed at entry
and checked after every executor step and before every flash erase/program.
The build fails if the static analyzer bounds `_rust_entry` above 24 KiB or
the app-level rejoin-failure chain above its regression budget
(`tools/tlsr8258-sensor-stack-gate.args`; 22,236 B analyzed). Hardware paint
on the previous 20 KiB layout measured at most 15,136 B (fresh join and
interview); the analyzed rejoin-failure path was not exercised on hardware.

A LOW32K timer wake re-enters the fresh root, which calls
`SensorApp::resume_after_timer_wake()` before `step()` so the iteration that
slept is serviced; waits shorter than the LOW32K wake floor are busy-waited.

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

Radio bring-up diagnostics live in `examples/telink-tlsr8258-radio`.
