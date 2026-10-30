# BL702 pure-Rust Zigbee sensor

Environmental sleepy-end-device profile for the XT-ZB1 without Bouffalo radio
archives.

## Composition

```text
environmental Sleepy End Device profile
        ↓
products/bl702-xt-zb1
        ↓
boards/bl702-xt-zb1
        ↓
bl702-hal + Bl702RadioPhy + SoftMacCore
```

This example is the composition root.

The root constructs explicit `SensorSedParts`:

- `Bl702Wake` (`Active` and radio-quiesced `Idle`; `Retention` rejected);
- `NoStatus`;
- synthetic temperature/humidity;
- GPADC supply battery;
- `NoOta`;
- no user action;
- BL702 supervisor and diagnostics.

The product selects `Idle` for both fast (250 ms) and slow (10 s) parent-poll
waits. An `Idle` wait:

1. disables M154 receive and waits (bounded) for the M154 state machine to
   report idle; if it does not, the wait returns `WakeReason::Activity`
   instead of sleeping;
2. awaits an Embassy timer, so the executor executes WFI;
3. wakes on the TIMER channel 0 comparator-0 interrupt (CLIC source 52,
   CLIC direct mode `mtvec = entry | 0b10`).

Clocks, the RF analog block, PLL, and per-die calibration stay powered, so
there is no restore step: the next MAC RX/TX starts from the same idle state
used between ordinary operations. The time driver arms the comparator for
the earliest deadline, capped at 30 minutes so the 1 MHz counter's 64-bit
extension never misses its ~71.6-minute wrap. A boot self-test proves alarm
interrupt delivery; on failure the driver reverts to the previous polling
executor and diagnostics report `wfi=false`.

`Retention` (PDS/HBN) is rejected by the wake controller and refused by
`bl702_hal::pm`; no PDS/HBN behavior is claimed. Idle sleep was validated on
one XT-ZB1 against ZHA: the alarm self-test passes (`wfi=true`), Idle waits
show no radio vetoes, and fresh join, interview/reconfigure over the 10 s long
poll, Mgmt_Leave and reset/resume all work. Current draw and behavior across
the ~71.6-minute counter wrap are not yet measured.

With `BL702_DIAGNOSTIC_LOG=1`, the UART (2 Mbaud) shows `alarm irq ready`
after `logger ready` (or `alarm irq FAILED: polling executor`), then an
`idle sleep: waits=…, idle_ms=…, radio_vetoes=…, alarm_irqs=…, wfi=…`
summary on the first and every 64th idle wait.

## Radio and Zigbee validation

Earlier XT-ZB1 images produced hardware evidence for:

- cold ACAL/KCAL/ROSCAL/RCCAL;
- channel selection, CCA/ED, RX, and TX;
- steering and association on channel 15;
- indirect polling and Transport-Key reception;
- ZHA interview, Configure Reporting, TCLK exchange, and live reports.

The production path installs token-owned SEC_ENG AES, runs two startup KATs,
and fails closed rather than using software AES.

## Storage

The board exposes the physical XIP flash resource. The product reserves and
constructs:

```text
0x00000000..0x000FE000  application
0x000FE000..0x00100000  SecurityStateJournal
```

Persistence is integrated. Security-state and replay-counter journal writes
plus reset/resume were exercised on XT-ZB1 hardware; long-term journal
compaction still needs controlled validation.

## Build

```bash
python3 -m pip install --require-hashes --no-deps \
  -r tools/bl702-image-tool/build-requirements.txt
python3 -m pip install --require-hashes --no-deps --no-build-isolation \
  -r tools/bl702-image-tool/requirements.txt
cd examples/bl702-sensor
./build-image.sh
```

Diagnostic logging:

```bash
BL702_DIAGNOSTIC_LOG=1 ./build-image.sh
```

Recorded production images and historical budget comparison:

| artifact | bytes |
|---|---:|
| raw binary | 189,442 |
| former raw regression budget (not enforced) | 192,512 |
| former budget minus raw bytes | 3,070 |
| packaged boot image | 197,648 |
| packager/device physical slot | 1,044,480 |

The artificial regression budget no longer fails builds. Physical image,
protected-journal, RAM, and stack boundaries remain mandatory.
The package script verifies BL702 boot magic and explicit 32 MHz XTAL clock
fields. These exact artifacts are build/package/layout-tested; the earlier
radio evidence is not an exact-image rerun.

## Flash and monitor

Connect the CH340 port, hold BOOT/GPIO28, then:

```bash
./flash.sh
./monitor.sh
```

The flash script programs and verifies the packaged image. Do not infer
persistence validation from a successful program operation.

## Remaining gates

- security-journal erase/program, power interruption, and reset recovery;
- GPADC battery accuracy;
- `Idle` sleep: WFI wake on the TIMER alarm, sleep current, and long-poll
  stability across a 71.6-minute counter wrap;
- PDS/HBN low-power modes (not implemented);
- RF output/spectral qualification;
- hardware address filtering and auto-ACK.
