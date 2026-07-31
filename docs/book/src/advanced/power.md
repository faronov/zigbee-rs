# Power Management

Battery-powered Zigbee devices spend most of their life asleep. The
`zigbee-runtime` crate provides a `PowerManager` that decides *when* to sleep,
*how long* to sleep, and *what kind* of sleep to use — while still meeting
Zigbee's poll and reporting deadlines.

---

## PowerMode

Every device declares its power strategy through the `PowerMode` enum
(`zigbee_runtime::power::PowerMode`):

```rust
pub enum PowerMode {
    /// Always on — router or mains-powered end device.
    AlwaysOn,

    /// Sleepy End Device — periodic wake for polling.
    Sleepy {
        /// Poll interval in milliseconds.
        poll_interval_ms: u32,
        /// How long to stay awake after activity (ms).
        wake_duration_ms: u32,
    },

    /// Deep sleep — wake only on timer or external event.
    DeepSleep {
        /// Wake interval in seconds.
        wake_interval_s: u32,
    },
}
```

| Mode | Typical Use | Radio | CPU | RAM |
|------|------------|-------|-----|-----|
| `AlwaysOn` | Routers, mains-powered EDs | On | On | Retained |
| `Sleepy` | Battery sensors, remotes | Off between polls | Halted | Retained |
| `DeepSleep` | Ultra-low-power sensors | Off | Off | Off (RTC only) |

Set the power mode when you build your device:

```rust
use zigbee_runtime::power::{PowerManager, PowerMode};

let pm = PowerManager::new(PowerMode::Sleepy {
    poll_interval_ms: 7_500,   // poll parent every 7.5 s
    wake_duration_ms: 200,     // stay awake 200 ms after activity
});
```

---

## SleepDecision

Each iteration of the event loop calls `PowerManager::decide(now_ms)`. The
manager returns one of three verdicts:

```rust
pub enum SleepDecision {
    /// Stay awake — pending work.
    StayAwake,
    /// Light sleep for the given duration (ms). CPU halted, RAM retained.
    LightSleep(u32),
    /// Deep sleep for the given duration (ms). Only RTC + wake sources active.
    DeepSleep(u32),
}
```

### Decision Logic

The decision tree inside `decide()` works as follows:

1. **Pending work?** — If `pending_tx` or `pending_reports` is set, always
   return `StayAwake`. Outgoing frames and attribute reports must be sent
   before the CPU is halted.

2. **AlwaysOn** — Always `StayAwake`. Routers never sleep.

3. **Sleepy** —
   - If less than `wake_duration_ms` has elapsed since the last activity
     (Rx/Tx, sensor read, user input), stay awake.
   - If a MAC poll is overdue (`since_poll >= poll_interval_ms`), stay awake
     to send the poll immediately.
   - Otherwise, enter `LightSleep` for the time remaining until the next
     poll is due.

4. **DeepSleep** —
   - If the last activity was within the last 1 second, stay awake (brief
     grace period for completing any post-wake work).
   - Otherwise, enter `DeepSleep` for `wake_interval_s × 1000` ms.

```rust
let decision = pm.decide(now_ms);
match decision {
    SleepDecision::StayAwake => { /* process events */ }
    SleepDecision::LightSleep(ms) => mac.sleep(ms),
    SleepDecision::DeepSleep(ms)  => mac.deep_sleep(ms),
}
```

---

## Sleepy End Device (SED) Behavior

A Sleepy End Device is a Zigbee device that spends most of its time with the
radio off. Its parent router buffers incoming frames and releases them when the
SED sends a MAC Data Request (poll).

### Poll Interval

The poll interval determines how often the SED wakes to check for buffered
data. Use `PowerManager::should_poll(now_ms)` to decide when to send a poll:

```rust
if pm.should_poll(now_ms) {
    mac.send_data_request(parent_addr);
    pm.record_poll(now_ms);
}
```

Typical poll intervals:

| Application | Poll Interval | Battery Impact |
|-------------|--------------|----------------|
| Light switch | 250–500 ms | High responsiveness, shorter battery |
| Door sensor | 5–10 s | Moderate |
| Temperature sensor | 30–60 s | Very low power |

### Activity Tracking

Call `record_activity()` whenever something interesting happens — a frame is
received, a sensor is read, or a user presses a button. This resets the
wake-duration timer and prevents premature sleep:

```rust
pm.record_activity(now_ms);  // keep CPU awake for at least wake_duration_ms
```

The `set_pending_tx()` and `set_pending_reports()` methods act as hard locks
that prevent sleep entirely until the work is done:

```rust
pm.set_pending_tx(true);       // acquired before queueing a frame
// ... send the frame ...
pm.set_pending_tx(false);      // release after MAC confirms transmission
```

---

## How MAC Backends Implement Sleep

The `PowerManager` itself does not touch hardware — it only *decides*. The
actual sleep/wake is performed by the MAC backend:

| Platform | Light Sleep | Deep Sleep |
|----------|-----------|------------|
| ESP32-C6/H2 | `esp_light_sleep_start()` | `esp_deep_sleep()` — only RTC memory retained |
| nRF52840 | `TASKS_DISABLE` + `__WFE` (System ON, RAM retained) | System OFF (wake via GPIO/RTC) |
| TLSR8258 | RF/DMA quiesce + restore hooks | HAL suspend/retention entry; timer wake proven, production sensor integration pending |
| PHY6222 | `radio_sleep()` + Embassy timer wait | AON system sleep not enabled |
| EFR32MG1 | `radio_sleep()` — radio clock gating via CMU | — |
| EFR32MG21 | `radio_sleep()` — radio clock gating via CMU | — |
| BL702 | Polling only; PDS not implemented | HBN not implemented |

The runtime event loop integrates the power manager like this (simplified):

```rust
loop {
    // 1. Process all pending events
    process_mac_events(&mut pm);
    process_zcl_reports(&mut pm);

    // 2. Ask the power manager what to do
    let decision = pm.decide(now_ms());

    match decision {
        SleepDecision::StayAwake => continue,
        SleepDecision::LightSleep(ms) => {
            mac.enter_light_sleep(ms);
            // CPU resumes here after wake
        }
        SleepDecision::DeepSleep(ms) => {
            nv.persist_state();          // save everything before deep sleep
            mac.enter_deep_sleep(ms);
            // After deep sleep, device resets — execution restarts from main()
        }
    }
}
```

> **Important:** Before entering `DeepSleep`, all critical state must be
> persisted to NV storage — deep sleep usually causes a full CPU reset and RAM
> is lost. See [NV Storage](./nv-storage.md) for details.

---

## Platform-Specific Power Optimizations

### nRF52840

The nRF52840 sensor example applies several hardware-level optimizations beyond
the basic sleep/wake cycle:

**DC-DC converter** — The nRF52840 has internal LDO regulators that can be
replaced by an on-chip DC-DC converter for ~40% lower current draw. Both
`reg0` (main 1.3 V supply) and `reg1` (radio 1.8 V supply) are enabled:

```rust
config.dcdc = embassy_nrf::config::DcdcConfig {
    reg0: true,
    reg0_voltage: None, // keep UICR default
    reg1: true,
};
```

**TX power reduction** — Default TX power is reduced from +8 dBm to 0 dBm,
saving ~50% TX current while still providing adequate range for home use:

```rust
mac.set_tx_power(0); // 0 dBm — good range, saves ~50% TX current vs +8 dBm
```

**Internal RC oscillator** — The HFCLK source is set to the internal RC
oscillator instead of the external crystal. The radio hardware automatically
requests the XTAL when it needs high accuracy (during TX/RX), then releases
it. This saves ~250 µA when the radio is idle:

```rust
config.hfclk_source = embassy_nrf::config::HfclkSource::Internal;
```

**RAM bank power-down** — Unused RAM banks are powered down during sleep,
saving additional current. On the nRF52840-DK, ~190 KB of unused RAM can be
powered off.

**Polling and reporting** — The sensor uses a two-phase polling scheme:
- Fast poll: 250 ms for 120 seconds after joining/activity (responsive)
- Slow poll: 30 seconds during steady state (low power)
- Report interval: 60 seconds

**Radio sleep** — Between polls, the radio is disabled via `TASKS_DISABLE`
register write and the state machine waits for `DISABLED`. This saves ~4-8 mA
of radio idle current. Before the next TX/RX, `radio_wake()` re-applies the
channel setting and re-enables the radio:

```rust
device.mac_mut().radio_sleep();
Timer::after(Duration::from_millis(poll_ms)).await;
device.mac_mut().radio_wake();
```

### TLSR8258

The production TLSR8258 sensor is still a polling end device; it does **not**
yet enter suspend or retention sleep. The HAL now contains the low-level
pieces needed for that integration:

```rust
device.mac_mut().prepare_for_sleep();
let wake = tlsr8258_hal::pm::cpu_suspend_wakeup_rc(
    tlsr8258_hal::pm::WakeConfig::Timer { wakeup_tick },
)?;
device.mac_mut().resume_after_sleep();
```

The sleep entry is RAM-resident, uses bounded clock/analog waits, returns a
typed wake status after suspend, and restores the RF/DMA configuration
through the MAC hooks. Timer-only suspend behavior has been exercised on
hardware.

The generalized wake API can arm:

- timer;
- pad;
- pad or timer;
- comparator;
- comparator or timer.

Pad wake configuration supports ports A-D. Port E is rejected because the
naively extrapolated analog enable address collides with a hardware-proven PM
control register. Comparator wake only arms the PM source; the external
comparator front end must already be configured by board-specific code.

Pad, comparator, and mixed wake modes are disassembly-derived and TC32-built,
but have not been validated on silicon. The production runtime also still
needs an end-to-end acceptance test proving that RAM state, radio restore,
poll scheduling, frame counters, and flash-backed security state remain
correct over repeated sleeps. No sleep-current or battery-life estimate is
claimed until that work is complete.

### PHY6222

The PHY62x2 example currently uses only radio sleep plus Embassy timer waits
between parent polls:

```rust
device.mac_mut().radio_sleep();
Timer::after(Duration::from_millis(poll_ms)).await;
device.mac_mut().radio_wake();
```

The low-level HAL contains RTC, SRAM-retention, and RAM-resident sleep-entry
primitives, but the application does not call them. AON wake restarts through
ROM; enabling it safely requires hardware proof of the packaged boot image,
flash release, interrupt dispatch, and retained runtime state. No PHY62x2
sleep-current or battery-life claim is made.

---

### EFR32MG1 / EFR32MG21

Both EFR32 platforms use the **CMU (Clock Management Unit)** to gate the radio
peripheral clock, providing radio sleep between polls.

**Radio clock gating** — The MAC driver's `radio_sleep()` method disables the
radio peripheral clock via the CMU, stopping all radio activity and saving
the radio idle current (~5–8 mA). On wake, `radio_wake()` re-enables the
clock and re-applies the channel setting:

```rust
device.mac_mut().radio_sleep();   // CMU clock gate — radio off
Timer::after(Duration::from_millis(poll_ms)).await;
device.mac_mut().radio_wake();    // CMU clock enable, re-apply channel
```

**Series 1 vs Series 2 CMU differences:**

| Feature | EFR32MG1P (Series 1) | EFR32MG21 (Series 2) |
|---------|---------------------|---------------------|
| CMU base | `0x400E4000` | `0x40008000` |
| Clock enable register | `HFPERCLKEN0` | `CLKEN0` |
| Radio blocks gated | RAC, FRC, MODEM, SYNTH, AGC, BUFC | RAC, FRC, MODEM, SYNTH, AGC, BUFC |

Both platforms implement the same `radio_sleep()` / `radio_wake()` interface
despite the different register layouts — the CMU abstraction is handled inside
each platform's MAC driver (`efr32/` for Series 1, `efr32s2/` for Series 2).

> **Series 1:** The production EFR32MG1P sensor additionally uses RTCC/LFRCO
> wakeups and EM2 between 30-second parent polls, including the Series-1 DCDC
> safety gate and PB13 interrupt wake. EFR32MG21 currently uses radio clock
> gating only.

---

## Reportable Change Thresholds

Both the nRF52840 and PHY6222 sensor examples configure **reportable change
thresholds** in the ZCL Reporting Configuration to suppress unnecessary
transmissions. A report is sent only when the attribute value changes by more
than the threshold *or* the maximum reporting interval expires:

| Attribute | Min Interval | Max Interval | Reportable Change |
|-----------|-------------|-------------|-------------------|
| Temperature (0x0402) | 60 s | 300 s | ±0.5 °C (50 centidegrees) |
| Humidity (0x0405) | 60 s | 300 s | ±1% (100 centi-%) |
| Battery (0x0001) | 300 s | 3600 s | ±2% (4 in 0.5% units) |

This means a device that sits at constant temperature will only report every
5 minutes (max interval), and tiny fluctuations (e.g., ±0.1 °C) are
suppressed entirely. This can reduce TX events by 80–90% in stable
environments.

---

## Power Budget Estimates

### nRF52840 (CR2032, 230 mAh)

| State | Current | Duty Cycle | Average |
|-------|---------|------------|---------|
| System ON idle (DC-DC, internal RC, RAM power-down) | ~3 µA | ~99.8% | ~3.0 µA |
| Radio RX (poll, 0 dBm) | ~5 mA | ~0.03% (10 ms / 30 s) | ~1.7 µA |
| Radio TX (report, 0 dBm) | ~5 mA | ~0.005% (3 ms / 60 s) | ~0.25 µA |
| Sensor read | ~1 mA | ~0.003% | ~0.03 µA |
| **Total average** | | | **~5 µA** |
| **Estimated battery life (CR2032)** | | | **~5+ years** |

> With reportable change thresholds suppressing most TX events, practical
> battery life approaches the self-discharge limit of the CR2032.

### PHY6222

No power budget is published yet. The current example uses light sleep only,
and neither radio timing nor AON sleep current has been measured on hardware.

---

## Battery Optimization Tips

1. **Minimize wake time.** Process events as fast as possible, then sleep.
   A typical SED wake cycle should complete in under 10 ms.

2. **Batch sensor reads with polls.** Read the sensor just before sending
   a report, so you don't need a separate wake cycle.

3. **Use appropriate poll intervals.** A door sensor that only reports on
   state change doesn't need 250 ms polls — 30 seconds is fine.

4. **Prefer DeepSleep for long idle periods.** If the device only reports
   every 5 minutes, deep sleep (with NV persistence) uses orders of
   magnitude less power than light sleep.

5. **Disable unused peripherals.** Turn off ADC, I²C, and SPI buses before
   sleeping — stray current through pull-ups adds up.

6. **Use reporting intervals instead of polling.** Configure the server-side
   minimum/maximum reporting intervals in the ZCL Reporting Configuration so
   the device only wakes when it has something new to say.

7. **Keep the network key frame counter in NV.** Frame counters must
   survive reboots. If a device resets its counter to zero, the network
   will reject its frames as replays.

8. **Enable DC-DC converters (nRF52840).** Switching from the internal LDO
   to the DC-DC converter saves ~40% idle current.

9. **Reduce TX power.** For home automation, 0 dBm provides plenty of range
   while halving TX current compared to +8 dBm.

10. **Use reportable change thresholds.** Adding a minimum change threshold
    (e.g., ±0.5 °C for temperature) eliminates unnecessary transmissions
    caused by sensor noise or small fluctuations.

11. **Power down flash (PHY6222).** Put external or on-chip flash into deep
    power-down mode before system sleep — saves ~14 µA.

12. **Prepare GPIOs for sleep (PHY6222).** Set unused pins to input with
    pull-down to prevent floating-pin leakage current.
