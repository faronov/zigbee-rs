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

## Current product policies

| product | fast | slow | implementation/status |
|---|---|---|---|
| nRF52840/52833 sensor | `Idle` | `Idle` | radio-off retained System-ON wait; sensor path hardware-proven |
| ESP32-C6/H2 | `Active` | `Active` | no low-power claim |
| BL702 | `Active` | `Active` | polling executor; no PDS/HBN claim |
| CC2340 | `Active` | `Active` | commissioning/radio HIL pending |
| PHY6222/6252 | `Idle` | `Idle` | radio sleep + real timer; retention rejected, hardware path unverified |
| EFR32MG1 | `Active` | `Retention` | RTCC/LFRCO EM2 steady state hardware-proven |
| EFR32MG21 | `Idle` | `Idle` | radio-gated WFE + 1 kHz SysTick; not EM2, HIL pending |
| TLSR8258 default | `Active` | `Idle` | full-SRAM timer SUSPEND |
| TLSR8258 retention proof | `Active` | `Retention` | feature-gated LOW32K reset-on-wake |

Routers are always-on. `RelayRouterApp`, `ParentRouterApp`, and
`CoordinatorApp` run bounded receive/tick slices but do not use sleepy
end-device parent polling.

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

The first retains the 250 ms slow cadence; the second uses 10 seconds. The
independent retention HIL pass marker is `0x5254600D`; a successful compile is
not that marker.

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
