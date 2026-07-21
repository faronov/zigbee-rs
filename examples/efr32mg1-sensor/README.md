# EFR32MG1P TRÅDFRI Zigbee/SHT3x Sensor

Pure-Rust firmware for the connected `EFR32MG1P132F256IM32`. The default
profile is the factory-EUI Zigbee sensor with real SHT3x measurements and
crash-safe security persistence. `diag-sht` and `diag-nv` keep sensor and
flash-controller bring-up isolated from the radio stack.

## Fixed board assumptions

| Function | Configuration |
|---|---|
| HFXO/HCLK | 38.4 MHz, CTUNE 360 |
| LED | PA0, active high |
| Button | PB13, active low, pull-up/filter |
| I2C0 SDA/SCL | PC10/PC11, LOC15, open drain |
| I2C speed | 10 kHz with weak internal pull-ups for bring-up margin |
| SHT3x | Probe `0x44`, then `0x45` only |

The controller supports external pull-ups, but this board definition
intentionally uses the known native-project internal-pull-up configuration.

## Bootloader-safe memory map

```text
0x00000000..0x00003FFF  resident Gecko bootloader (not emitted)
0x00004000..0x00036FFF  Rust application
0x00037000..0x00038FFF  security journal (two 4 KiB sectors)
0x00039000..0x00039FFF  Rust application NV (two 2 KiB pages)
0x0003A000..0x0003FFFF  existing native NVM3 (preserved)
0x20000000..0x20007BFF  usable SRAM (0x7C00 bytes)
```

The custom board linker script places the vector table at `0x4000` and writes
the `APP_PROPERTIES` address at vector word 13 (`0x4034`). `cortex-m-rt`'s
`set-vtor` startup writes `SCB->VTOR = 0x4000` before Rust initialization.

## Profiles

- `sensor` (default): factory-EUI Zigbee end device with real SHT3x
  temperature/humidity, ZHA reporting, and crash-safe security restore.
- `diag-sht`: HFXO, RTCC-backed Embassy timers/RTT, PA0 LED, I2C0, and real
  SHT3x only; no NV/radio.
- `diag-nv`: MSC plus the bounded Rust application journal only; no radio.
- `diag-em2`: RTCC-backed EM2 wake timer + DCDC LN safety gate only; no
  NV/I2C/radio/Zigbee. Isolated bring-up for a future SED conversion.
- `diag-rtcc-time`: Embassy executor + `Timer` running on the RTCC/LFRCO
  time driver (`src/time_driver.rs`) — short active waits plus an explicit
  EM2 wait phase, no NV/I2C/radio/Zigbee. Isolated bring-up for the same
  future SED conversion as `diag-em2`.
- `diag-beacon`: one bounded raw TX gate with full RAC/FRC/BUFC/SEQ/PROTIMER
  snapshots; active scan starts only if that TX completes.
- `diag-join`: Zigbee join/poll diagnostic.

Exactly one profile must be enabled.

## Build and verify

Run Cargo from this directory so `.cargo/config.toml` supplies the Cortex-M
target, build-std, and bootloader linker script:

```bash
cd examples/efr32mg1-sensor

cargo build --release
cargo build --release --no-default-features --features diag-sht
cargo build --release --no-default-features --features diag-nv
cargo build --release --no-default-features --features diag-em2
cargo build --release --no-default-features --features diag-rtcc-time
cargo build --release --no-default-features --features diag-beacon,trace
cargo build --release --no-default-features --features diag-join

tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

The ELF artifact is:

```text
target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

Create a range-limited Intel HEX only after layout verification:

```bash
rust-objcopy -O ihex \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex
```

The verifier rejects a first file-backed load below `0x4000`, a load entering
the security journal at `0x37000`, an invalid SP, a missing
application-properties pointer, or a Reset handler without the early VTOR
write.

## `diag-nv` flash gate

The NV diagnostic opens the two-page journal at `0x39000`, writes a fixed
probe record, reads it back, and leaves the LED on after successful
verification. It initializes neither radio nor Zigbee:

```text
[EFR32][diag-nv] BOOT nv=0x00039000..0x0003A000 radio=off
[EFR32][diag-nv] PASS bytes=16 page_size=0x800
```

Use page-limited erase for a clean journal after the diagnostic:

```bash
commander device pageerase \
  --device EFR32MG1P132F256IM32 \
  --range 0x39000:+0x1000
```

## `diag-em2` EM2 power-management gate

Isolated bring-up gate for the reusable RTCC wake-timer / EM2-entry HAL
module (`efr32mg1_hal::pm`), ahead of converting `sensor` to a true Sleepy
End Device. It initializes only clocks, the LED, RTT, and RTCC — **not**
NV, I2C, the security journal, or the radio/Zigbee stack.

What it does, once per boot:

1. Places three SRAM canaries — one in `.bss`, one in the same `.uninit`
   NOLOAD section `FAULT_LOG` uses (near the top of RAM), and one on its
   own stack frame — each filled with a distinct, recomputable pattern
   (`efr32mg1_hal::pm::canary_pattern`).
2. Brings up LFRCO and routes it to RTCC over the CMU "LFE" branch (the
   same source/branch GSDK 4.5.0's `sleeptimer` component and its
   `sli_sleeptimer_set_pm_em_requirement()` EM2-capability check both use
   for this part — see `efr32mg1-hal/src/pm.rs` module docs for the exact
   GSDK cross-references).
3. Unmasks the `RTCC` NVIC line — safe only because this profile defines a
   real, bounded `RTCC()` interrupt handler; every other profile leaves it
   at the linker's `DefaultHandler` (infinite loop) default and never
   unmasks it.
4. Repeats 10 times: arms an RTCC compare-match `~1 s` out, applies the
   Series-1 (SDID 80) DCDC LN-handshake safety workaround, executes `WFI`
   with `SLEEPDEEP` set, and on wake verifies (a) the wake was the RTCC
   compare match, (b) RTCC actually advanced by roughly the requested
   ticks (±10%, since LFRCO is not crystal-accurate), and (c) all three
   canaries are unchanged.
5. Prints one RTT line per iteration and a final `PASS`/`FAIL` summary.

```bash
cargo build --release --no-default-features --features diag-em2
```

Expected RTT output on success:

```text
[EFR32][diag-em2] BOOT lfrco_hz=32768 iterations=10 sleep_ms=1000 tolerance_pct=10
[EFR32][diag-em2] iter=1 cause=RtccCompare deadline=0x00008000 elapsed_ticks=32768 elapsed_ms=1000 progressed=true canary_bss=true canary_uninit=true canary_stack=true => PASS
...
[EFR32][diag-em2] PASS iterations=10 failures=0
```

Every hardware register offset and bit used by this gate (CMU LFE clock
tree, RTCC CC0 compare, and the undocumented `EMU_BASE+0x7C` DCDC LN-running
peek) is cross-checked against GSDK 4.5.0 headers/`em_emu.c`/`sleeptimer`
sources and the native `~/efr32mg1p-bme280-zigbee-sensor` reference
project — see the doc comment at the top of `efr32mg1-hal/src/pm.rs` for the
exact citations. This diagnostic does not flash hardware by itself and does
not change `sensor` behavior: every new code path is behind
`#[cfg(feature = "diag-em2")]`.

## `diag-rtcc-time` RTCC Embassy time-driver gate

Proves the RTCC/LFRCO Embassy time driver (`src/time_driver.rs`, which
replaces the previous SysTick-based driver for every profile) end to end
with a real Embassy executor and `Timer`, ahead of the same future SED
conversion `diag-em2` is isolating. Only clocks, RTCC, LFRCO, RTT, and the
LED — **not** NV, I2C, the security journal, or the radio/Zigbee stack.

Two phases, run once per boot:

1. **Active waits** — 10 iterations of `Timer::after(200 ms).await` through
   the normal Embassy queue path (`embassy_time_queue_utils::Queue`,
   `now()`/`schedule_wake` via RTCC's single CC0 compare), each checked
   against the requested duration within tolerance.
2. **Explicit EM2 wait** — 5 iterations calling
   `efr32mg1_hal::pm::sleep_for_ticks_polled` directly (bypassing `Timer`
   entirely), reusing the exact same bounded apply-DCDC-gate/
   `SLEEPDEEP`/`WFI` primitive `diag-em2` already proved on hardware. Run
   strictly after phase 1 completes, so there is only ever one owner of
   RTCC's CC0 channel at a time. Each iteration also asserts
   `SCB.SCR.SLEEPDEEP` reads back clear immediately after waking (proving
   it is never left armed by accident) and validates two SRAM canaries
   reused from `diag-em2`'s technique (`.bss` + stack).

```bash
cargo build --release --no-default-features --features diag-rtcc-time
```

Expected RTT output on success:

```text
[EFR32][diag-rtcc-time] BOOT lfrco_hz=32768 embassy_tick_hz=32768 active_iterations=10 active_wait_ms=200 em2_iterations=5 em2_wait_ms=500
[EFR32][diag-rtcc-time] active_iter=1 elapsed_ms=200 progressed=true => PASS
...
[EFR32][diag-rtcc-time] em2_iter=1 deadline=0x00004000 elapsed_ticks=16384 elapsed_ms=500 progressed=true sleepdeep_cleared=true canary_bss=true canary_stack=true => PASS
...
[EFR32][diag-rtcc-time] PASS active_iterations=10 em2_iterations=5 failures=0
```

`embassy-time-driver`'s `tick-hz-32_768` feature (enabled in `Cargo.toml`)
pins `embassy_time_driver::TICK_HZ` to exactly RTCC's LFRCO-derived tick
rate, so no fractional tick conversion is needed anywhere in the driver.
Because `embassy-time-driver` is `links = "embassy-time"`, Cargo unifies a
single instance of it across the whole dependency graph — including
`zigbee-mac`'s `efr32` feature, which depends on a separate `embassy-time`
0.4 (this crate uses 0.5) that resolves to the very same driver instance.
This diagnostic does not flash hardware by itself and does not change
`sensor` behavior: every new code path is behind
`#[cfg(feature = "diag-rtcc-time")]`, and `diag-em2` keeps its own,
unmodified `RTCC` handler (see `src/time_driver.rs`'s module doc header for
why the two can never conflict).

## `diag-sht` RTT markers

The diagnostic performs:

1. bounded 38.4 MHz HFXO startup;
2. I2C0 initialization and bus recovery if SDA/SCL are not idle;
3. soft reset (`30 A2`) and 2 ms wait;
4. status read (`F3 2D`) with CRC validation;
5. high-repeatability single shot (`24 00`), 20 ms wait, six-byte read;
6. independent temperature and humidity CRC checks.

Expected markers include:

```text
[EFR32][diag-sht] CLOCK_READY ...
[EFR32][diag-sht] I2C_READY ...
[EFR32][diag-sht] SHT_FOUND address=0x44 status=... crc=ok
[EFR32][diag-sht] MEAS_OK ... temp_centi_c=... humidity_centi_percent=... crc=ok
```

Probe, status, CRC, transfer, and clock failures have distinct RTT markers.

## `diag-beacon` TX gate

The phase-3 diagnostic performs exactly one raw channel-15 Beacon Request
before any active scan. After its three-LED-pulse marker it waits five seconds
for an RTT client to attach. Build it with `trace` so the pre-arm, timeout,
and post-recovery snapshots include the TX-critical registers:

```bash
cargo build --release --no-default-features --features diag-beacon,trace
tools/verify-layout.py target/thumbv7em-none-eabi/release/efr32mg1-sensor
```

Expected gate markers are:

```text
[EFR32][diag-beacon] TX_ONLY_BEGIN ...
[EFR32][diag-beacon] TX_ONLY_PASS scan_gate=open
[EFR32][diag-beacon] ACTIVE_SCAN_ENABLED ...
```

On timeout, `TX_ONLY_FAIL` is emitted once and `ACTIVE_SCAN_BLOCKED` repeats
every five seconds; no scan or additional TX is attempted. The driver first
clears the software RX source,
explicitly clears xG1 `RXOVERFLOW` (RAC state 6), aborts FRC RX, clears BUFC,
and verifies bounded RAC idle. A detailed `tx-timeout-stuck` snapshot is
followed by either `tx-timeout-recovered` or
`tx-timeout-recovery-failed`.

The connected `EFR32MG1P132F256IM32` now passes this gate. The missing step was
the pair of RAC sequencer writes used by GSDK RAIL immediately before `TXEN`:
set `SR0` bit 7 and clear bit 3. With those writes, FRC TXDONE completes and
the following active scan receives channel-15 beacons.

## Hardware-proven `diag-sht`

The isolated diagnostic is proven on the connected
`EFR32MG1P132F256IM32`:

- resident bootloader and `0x3A000..0x3FFFF` native NVM3 remained unchanged;
- HFXO/HCLK started at 38.4 MHz with CTUNE 360;
- SHT3x was detected at `0x44`, status CRC passed;
- a 140-sample 10 kHz stress run completed with zero I2C or CRC errors after
  enabling the Series-1 GPIO input filter;
- observed values were approximately 25.2 °C and 66.6 %RH.

`diag-sht` and the bounded `diag-beacon` gate are hardware-proven.

The no-NV `diag-join` path is also hardware-proven with a locally administered
diagnostic EUI derived from the factory identity:

- association and ACK handling completed through a router;
- Data Request used the assigned short address and received
  `frame_pending=true`;
- the indirect APS Transport-Key frame was received and decrypted;
- the NWK key was installed;
- Device Announce was sent;
- Node Descriptor, Active Endpoints, and Simple Descriptor requests received
  responses.

## Hardware-proven production profile

The default sensor profile is hardware-proven with factory EUI
`00:0d:6f:ff:fe:da:ee:ee`:

- the 8 KiB `SecurityStateJournal` at `0x37000` commissioned through ZHA and
  reserved monotonically increasing global/TCLK counter ranges;
- reset restore completed on channel 15 without erasing the journal;
- a forced secured NWK rejoin changed the parent while atomically clearing
  `rejoin_pending` and preserving the existing counter reservations;
- repeated hardware reset/resume cycles filled all 32 records in the active
  journal sector, then generation 32 was committed to slot 0 of the second
  sector with no invalid records and monotonically advanced counter bounds;
- ZHA completed interview as `Zigbee-RS / EFR32MG1-Sensor`;
- the normal profile reads the SHT3x at `0x44`; ZHA observed real values and a
  later periodic update (`23.87 -> 23.88 °C`, `61.13 -> 61.10 %RH`);
- the resident bootloader, separate application NV, and native
  `0x3A000..0x3FFFF` NVM3 remained byte-identical throughout NV writes,
  commissioning, reset, secure rejoin, and journal rollover.

The initial normal profile exposed an SRAM overflow after commissioning:
the Embassy task and nested runtime tick consumed more stack than remained
above `.bss`. Exact nightly task allocation, smaller fixed cluster stores,
and a flattened runtime tick path leave roughly 3 KiB of measured stack/IRQ
margin in the usable `0x7C00` SRAM.

The board is normally left on the production `sensor` profile.

The isolated `diag-em2` profile is also hardware-proven:

- RTCC runs from the 32.768 kHz LFRCO and wakes the core from EM2;
- ten consecutive one-second sleeps each measured exactly 32768 RTCC ticks;
- every wake arrived through the RTCC compare ISR;
- `.bss`, `.uninit`, and stack canaries remained intact;
- bootloader and `0x37000..0x3FFFF` persistence regions remained byte-identical;
- the production sensor was restored afterward, advanced the security journal
  to generation 33, and resumed real SHT3x reporting in ZHA.

Safe identification/read-only checks remain:

```bash
commander adapter probe
commander device info --device EFR32MG1P132F256IM32
commander readmem --device EFR32MG1P132F256IM32 --range 0x00000000:+0x40
commander readmem --device EFR32MG1P132F256IM32 --range 0x00004000:+0x40
```

Build and verify `diag-sht`, then perform a page-limited flash with no
mass-erase/recover option:

```bash
cd examples/efr32mg1-sensor
cargo build --release --no-default-features --features diag-sht
tools/verify-layout.py target/thumbv7em-none-eabi/release/efr32mg1-sensor
rust-objcopy -O ihex \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex
commander flash \
  target/thumbv7em-none-eabi/release/efr32mg1-sensor.hex \
  --device EFR32MG1P132F256IM32
```

Never use `device masserase` for this layout. Re-read vectors at `0x0` and
`0x4000`, then attach RTT without requesting an erase.

## Architecture

- `efr32mg1-hal`: raw CMU/MSC wait-state, GPIO, I2C0 controller, and RTCC
  wake-timer/EM2-entry (`pm`) code.
- `boards/efr32mg1-tradfri`: HFXO/pin/LOC/I2C speed and application flash map.
- `drivers/sht3x`: generic blocking embedded-hal 1.0 SHT3x protocol.
- this example: profile sequencing, Embassy timers, RTT, and LED patterns.
  `src/time_driver.rs` is the Embassy time driver, backed by
  `efr32mg1_hal::pm`'s RTCC/LFRCO wake timer (it replaces an earlier
  SysTick-based driver, which cannot survive EM2). It owns the `RTCC`
  interrupt for every profile that links it; `diag-em2` does not compile
  this module and keeps its own separate, unmodified `RTCC` handler.

Full DCDC/EM2 integration into the production SED path is still deferred;
`diag-em2` and `diag-rtcc-time` are isolated, unflashed gates towards it and
do not change `sensor` behavior. Power management for the production
profile begins only after these gates — and the SED conversion itself —
are proven on hardware.
