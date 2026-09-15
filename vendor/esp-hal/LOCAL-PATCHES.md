# esp-hal 1.0.0: retained C6/H2 light-sleep backport

## Provenance

Base: the published `esp-hal-1.0.0.crate`, SHA-256
`54786287c0a61ca0f78cb0c338a39427551d1be229103b4444591796c579e093`.
Every extracted file was compared byte-for-byte with that archive before changes.
The published VCS metadata identifies upstream commit
`5ba50dd2d21136370050dd2f7a767761e01cfeef`, directory `esp-hal`.
The archive omits license files; `LICENSE-MIT` and `LICENSE-APACHE` are copied
from that exact upstream commit. Package version, dependency requirements,
features and the published manifest are unchanged.

The H2 sleep implementation and PMU register structures derive from
[esp-rs/esp-hal#4587](https://github.com/esp-rs/esp-hal/pull/4587),
merge commit `29a3e398abde4c3ef4031c2581ab0309d1100522`.
That initial implementation omitted clock restoration; it is not used unchanged.
Hardware cross-checks use Espressif's **ESP-IDF v5.5.1** sources:
`components/esp_hw_support/port/{esp32c6,esp32h2}/{rtc_clk.c,rtc_time.c,pmu_param.c,pmu_sleep.c}`,
the corresponding `hal/clk_tree_ll.h`, `hal/pmu_ll.h`, `hal/lp_timer_ll.h`,
and H2 `soc/regi2c_bias.h`. These are references, not linked IDF dependencies.

## Local changes

- Add H2 timer/digital-GPIO retained PMU light sleep and active/sleep clock
  configuration. Restore CPU source, CPU/AHB dividers, the bus-update latch
  and ROM delay frequency; re-enable/recalibrate PLL when required. Restore the
  ADC power-detect setting and apply Espressif's post-wake low-temperature bias fix.
  H2 PLL/I2C restoration code is placed in RAM because MSPI can depend on BBPLL.
- Preserve C6 inactive CPU/AHB divider banks and the existing MSPI divider,
  rather than replacing the application's flash-clock configuration on wake.
  Explicitly configure PMU CPU stall and digital-watchdog pause for both chips.
- Add `unsafe Rtc::sleep_light_retained`, with typed admission/calibration errors.
  It accepts only timer/digital-GPIO wake and rejects all power-down flags.
  It does not run C6's unrelated EXT1 pad cleanup. H2 deep sleep and powered-down
  CPU/top-domain configurations are not supported by this backport.
- Use calibrated Q19 RTC periods and checked 48-bit timer arithmetic. Handle H2
  ECO2 RC_FAST calibration division, including REF_TICK enable/restoration.
  Detect zero/invalid calibration and restore clocks before returning an error.
  Retain H2 RC32K as well as XTAL32K/RC_FAST, including when RC32K is the slow clock.
- Correct SYSTIMER high-word reload (`>> 32`, not `<< 32`) for all existing
  setter branches.

## Integration and limits

Patch **all** firmware dependency graphs to this single HAL copy; keep the pinned
compiler and radio-facing HAL APIs. The product `sleep.rs` requires the local
retained entry. It owns calibrated elapsed-time accounting and returns missing
SYSTIMER ticks; the HAL does **not** advance SYSTIMER automatically.

The caller must hold its critical section across radio suspend, GPIO setup,
PMU sleep, one-time SYSTIMER compensation/alarm servicing, and radio resume.
Finish flash/DMA/UART/USB activity, quiesce all other PLL consumers, and budget
watchdogs. The HAL keeps TIMG0 enabled for RTC calibration; do not gate or
concurrently use its calibration circuit. Digital GPIO wake preserves the top
domain; GPIO9 on C6 is not an EXT1/RTC-GPIO wake source.

An `Ok` HAL return may be a hardware rejection: inspect PMU status. The product
reports rejection explicitly and does not invent stopped-clock compensation.
Retained XTAL, RAM, flash and peripheral domains deliberately trade minimum
current for in-place execution/state preservation. There is no WFI substitution,
reset-based resume, RF-state implementation, or secure-boot policy change here.
Linked probes and host regressions are **not hardware/current qualification**.

## Host regressions

From the repository root, with `OUT` set to an existing out-of-tree directory:

```sh
rustup run nightly-2026-03-23 rustc --edition 2024 --test \
  vendor/esp-hal/tests/retained_sleep_host.rs -o "$OUT/hal-sleep-tests"
"$OUT/hal-sleep-tests"
rustup run nightly-2026-03-23 rustc --edition 2024 --test \
  products/esp32-zigbee-devkit/src/sleep.rs -o "$OUT/product-sleep-tests"
"$OUT/product-sleep-tests"
```

No fixture variables or ignore flags are required. The tests exercise the actual
clock-register replay helpers, source/divider decoding, power/wake admission,
calibration arithmetic, timer bounds, counter high words/wrap and elapsed-time
accounting. Cross-link both chip features on `nightly-2026-08-01` with the existing
dependencies before integration.
