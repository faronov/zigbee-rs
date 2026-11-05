# Power Management

Sleep is part of the product/application contract, not a hidden MAC default.
The shared sleepy-sensor lifecycle decides when it is safe to wait; the
platform adapter performs one atomic wait/restore transition.

## Fast and slow wait depths

`SensorPolicy` selects two independent depths:

```rust,ignore
pub struct SensorPolicy {
    // poll/report/retry policy omitted
    pub fast_sleep_depth: SleepDepth,
    pub slow_sleep_depth: SleepDepth,
}
```

```rust,ignore
pub enum SleepDepth {
    Active,
    Idle,
    Retention,
}
```

- **Active** — await timer/button without entering a platform sleep mode.
- **Idle** — quiesce the radio and use a retained light/suspend wait.
- **Retention** — use a deeper retained/reset-on-wake path with explicit
  restoration.

The fast depth is used during commissioning, interview, commands, reporting
configuration, and other short-poll windows. The slow depth is used only in
the stable joined state. An active OTA session forces active waiting.

## Atomic platform wait

`WakeController::wait(mac, WaitRequest)` must:

1. validate that the requested depth is supported;
2. quiesce MAC/radio work without losing an event;
3. enter a bounded timer/button wait;
4. restore clocks, timers, calibration, radio, DMA, and errata state required
   by that platform;
5. return only when the next normal MAC operation is safe.

Preparation failure returns without sleeping. Required restoration failure is
an error, not silent success.

The shared application also shortens the wait for:

- runtime `RunAgain` deadlines;
- report/sample deadlines;
- fast-poll expiry;
- status/Identify transitions when status hardware exists;
- OTA service;
- watchdog maximum wait;
- rejoin/announce retries.

`NoStatus::PRESENT = false` removes status-only deadlines.

## Parent loss and rejoin recovery

`sensor-sed-app` (all SED products) bounds how long a lost or broken parent
can keep a battery device awake, via `SensorPolicy::recovery`
(`RecoveryPolicy::DEFAULT` unless a product overrides it):

- **Parent-loss evidence.** An APS-acknowledged report that exhausts its
  retries (`report_aps_ack`), or pending protocol work that outlives
  `max_active_ms` (30 s), counts one strike; MAC-level poll failures are
  still handled by the runtime. `parent_loss_strikes` (3) consecutive
  strikes start a rejoin. This catches a parent that ACKs MAC polls but
  delivers nothing end to end.
- **Staged rejoin.** Secure rejoin on the current channel, then on all
  channels, then a Trust Center rejoin. Parent selection normalises LQI
  to the IEEE 0–255 scale and, if every candidate exceeds the link-cost
  limit, retries the selection accepting any valid link cost (still ranked
  by depth, then cost) rather than rejecting every router.
- **Backoff.** Each failed attempt doubles the delay from `join_retry_ms`
  up to `join_retry_max_ms` (15 min), shortened by up to
  `join_retry_jitter_pct` (25 %). The device waits at its slow depth
  (radio off) for the whole delay, not in a radio-on retry loop.
- **Keep-awake limits.** Every reason that holds the fast poll or the
  `Active` depth has a limit in `RecoveryPolicy` (`0` = unbounded):
  pending work `max_active_ms` (30 s; each further period it persists is
  another strike), an OTA session `max_ota_awake_ms` (2 h; the session is
  aborted through `OtaLifecycle::abort`, a suspended download stays
  resumable), Identify `max_identify_awake_ms` (2 min; Identify then runs on
  the slow poll) and back-to-back timed fast-poll windows
  `max_fast_poll_ms` (10 min; the windows are closed). A window that ends
  on time never fires.
- **Missed reports.** With `report_aps_ack`, no APS-acknowledged traffic
  for the shortest periodic reporting maximum interval plus
  `missed_report_grace_ms` (2 min) counts one strike and forces every
  report due, so the next tick produces fresh evidence.
- **Anomaly record.** Each fired limit, missed report and parent-loss
  rejoin is counted in `AnomalyRecord` (last reason, reasons seen,
  saturating per-reason counts). `SensorApp::take_anomaly_update` hands a
  product the changed record; `AnomalyJournal` appends it to a NOR window
  only then, at most N writes per boot, so normal operation writes nothing.
  The ESP32-H2 thermometer journals it in `zbnv` and publishes it as
  Diagnostics (0x0B05) attributes.
- **No self factory reset.** `secure_rejoin_failure_limit = 0` on every SED
  product: the network is kept until a user action (long press) or a Leave
  from the network. A non-zero limit restores the old give-up behaviour.

Router and always-on end-device apps (`apps/router`) follow the same rule:
`RouterPolicy::secure_rejoin_failure_limit` defaults to `0` (never reset) in
`RouterPolicy::DEFAULT` and every product, and a pending secured rejoin is
released only on the `join_retry_initial_ms`..`join_retry_max_ms` exponential
backoff instead of the runtime's fixed 5 s retry. A non-zero limit is an
opt-in give-up. Host-tested in `apps/router/tests/lifecycle.rs`
(`unlimited_secure_rejoin_backs_off_and_keeps_parent_state`).

Status: host-tested (`apps/sensor-sed/tests/lifecycle.rs`); the AEM proof
of the sleep floor between attempts is a hardware gate.

## Current product policies

| product | fast | slow | implementation/status |
|---|---|---|---|
| nRF52840/52833 sensor | `Idle` | `Idle` | radio-off retained System-ON wait; prior sensor-path hardware evidence |
| ESP32-C6/H2 | `Active` | `Active` (`Idle` with opt-in `light-sleep`) | `light-sleep` adds GPIO/RTC-wake light sleep with PHY suspension; no current claim |
| BL702 | `Idle` | `Idle` | RX-off M154 idle + WFI on TIMER CH0 alarm IRQ; no PDS/HBN claim; HIL join/poll/resume OK, current unmeasured |
| CC2340 | `Idle` | `Retention` | radio off + WFE on SYSTIM alarm; slow polls enter TI standby (LFOSC RTC/button wake, radio re-init); LP-EM-CC2340R5 EnergyTrace ~5.6 µA mean at 10 s poll (commits `d6a1cec`, `0ae1560`) |
| PHY6222/6252 | `Idle` | `Idle` | radio sleep + real timer; retention rejected, hardware path unverified |
| EFR32MG1 | `Active` | `Retention` | prior RTCC/LFRCO EM2 path evidence |
| EFR32MG21 | `Idle` | `Idle` | radio-gated WFE + 1 kHz SysTick; not EM2, HIL pending |
| TLSR8258 default | `Active` | `Idle` | full-SRAM timer SUSPEND |
| TLSR8258 retention proof | `Active` | `Retention` | feature-gated LOW32K reset-on-wake |

Routers are always-on. `RelayRouterApp`, `ParentRouterApp`,
`DistributedRouterApp`, `CoordinatorApp`, and `TrustCenterCoordinatorApp` run
bounded receive/tick slices but do not use sleepy end-device parent polling.
`AlwaysOnEndDeviceApp` (the nRF52840 router product) is an rx-on-when-idle
end device and does not sleep either.

These statements describe product policies and prior path evidence. The exact
2026-09-06 release images are build/layout-tested unless an exact-image HIL
rerun is separately recorded.

## Telink variants

The default TB-04 sensor uses atomic full-SRAM timer `SUSPEND`. It is not the
LOW32K image.

```bash
./scripts/tlsr8258.sh build sensor
```

Feature-gated proof images:

```bash
./scripts/tlsr8258.sh build sensor-retention
./scripts/tlsr8258.sh build sensor-retention-10s
```

The default TB-04 sensor polls every 5 s once joined (`RESPONSIVE_SLOW_POLL_MS`,
below the shortest common parent indirect hold). The first proof image keeps
a 250 ms slow cadence as a wake-cycle stress test; the second uses 10 seconds. The
standalone legacy lab stack has been removed, so the LOW32K retention HIL
proof has to run on these images; a successful compile is not that proof.

## Security and flash before sleep

Do not enter a depth that can lose state while:

- secured outgoing counters need a reservation checkpoint;
- a journal commit is in progress;
- OTA staging/verification/activation is active;
- the radio has an outstanding exchange;
- a parent poll or short-poll window is due.

Deep/retention implementations must preserve the no-counter-reuse guarantee.
Reset-on-wake products must distinguish cold boot, valid retained state, and
corrupt retention markers.

## Measuring power

Do not publish current from a datasheet estimate or from seeing `WFE`/sleep
execute. A power acceptance should record:

- board revision and fitted components;
- supply voltage and measurement instrument;
- commissioned role and parent poll policy;
- radio TX power and report traffic;
- minimum/typical/maximum current over repeated cycles;
- wake source and restored peripherals;
- counter/journal state before and after resets;
- long-duration parent child-aging behavior.

No PHY62x2 AON current is claimed. No battery-life estimate is inferred for a
platform whose complete sleep/network path has not passed HIL.
