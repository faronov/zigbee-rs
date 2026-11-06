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
- H2 CPU source FLASH_PLL (`soc_clk_sel = 3`, 64 MHz derived from the 96 MHz
  BBPLL) is accepted for light sleep. The ESP-IDF 2nd-stage bootloader leaves
  the CPU on it (`CPU /1`, `AHB /2`), so before this change every sleep attempt
  returned `UnsupportedClock` and the device never slept. The PAC labels value 3
  reserved; IDF v5.5.1 `clk_tree_ll.h` defines it. As IDF does
  (`esp_sleep_start`: `rtc_clk_cpu_freq_get_config`,
  `rtc_clk_cpu_freq_set_xtal_for_sleep`, then `rtc_clk_cpu_freq_set_config` after
  wake), the source and CPU/AHB dividers are saved, the CPU runs on XTAL /1 in
  sleep, and on wake BBPLL is re-enabled and recalibrated to 96 MHz for source 1
  **or** 3. Then the saved dividers, source 3, bus latch and 64 MHz ROM delay are
  replayed. The clock is never forced to 96 MHz PLL. Other combinations
  (`cpu_div` not dividing 64, AHB above 32 MHz, AHB not a multiple of CPU) still
  return `UnsupportedClock` before any register is touched.
- The H2 sleep request, the wake/reject wait, the bias/analog restore and the
  CPU clock restore run in one `#[ram]` function (`sleep_and_restore_clocks`),
  with `pmu()`/`lp_aon()`/`restore_sleep_bias`/`SavedSleepAnalog::restore`
  inlined. No flash instruction or `.rodata` fetch happens between the sleep
  request and the restored CPU source; the only external calls are RAM
  functions and the ROM `ets_delay_us`/`ets_update_cpu_frequency`. This is
  stricter than IDF, which keeps `esp_sleep_start` in flash unless
  `CONFIG_PM_SLP_IRAM_OPT` is set. Verified by disassembling the linked
  thermometer ELF. The only flash target in the RAM path is the unreachable
  `regi2c_enable_block` invalid-block panic.
- esp-hal's `Clocks` still reports the 96 MHz configuration that `esp_hal::init`
  requested, while the CPU runs at 64 MHz on FLASH_PLL. This predates the patch
  and is unchanged; sleep timing uses the RTC/SYSTIMER, not `Clocks`.
- Preserve C6 inactive CPU/AHB divider banks and the existing MSPI divider,
  rather than replacing the application's flash-clock configuration on wake.
  Explicitly configure PMU CPU stall and digital-watchdog pause for both chips.
- Add `unsafe Rtc::sleep_light_retained`, with typed admission/calibration errors.
  It accepts only timer/digital-GPIO wake and rejects all power-down flags.
  It does not run C6's unrelated EXT1 pad cleanup. Powered-down CPU/top-domain
  light-sleep configurations are not supported by this backport.
- Add opt-in `unsafe Rtc::sleep_light_retained_power_down(RetainedPowerDown, ..)`
  (`sleep_light_retained` = `RetainedPowerDown::NONE`). **H2 only**: it may stop
  XTAL (`xtal`) and RC_FAST (`rc_fast`) and then programs the HP_SLEEP/LP_SLEEP
  bias and regulator registers the way IDF v5.5.1 `pmu_sleep_config_default`
  does (`pmu_param.c`/`pmu_sleep.c`): XTAL off → `xpd_trx=0`, `pd_cur=1`,
  `bias_sleep=1`, HP sleep dbias 1, LP sleep dbias from `DSLP_DBIAS` eFuse
  (default 6); XTAL on → active dbias from `ACTIVE_HP/LP_DBIAS` (+2, capped,
  only for eFuse block ≥ v0.3). Without this the analog registers stay at
  their reset values (bias on, HP regulator near active voltage) — IDF's
  "XTAL-on" light sleep. The six registers are saved before sleep and
  restored after wake (before clock restore). `pmu_sleep_calculate_hw_wait_time`
  follows IDF (XTAL settle time only when XTAL is stopped). The C6 backport has
  no analog sleep config and rejects any oscillator power-down. CPU, top,
  modem and VDD_SDIO (flash) domains remain powered: CPU/top need REGDMA
  retention, modem power-down would lose 802.15.4 MAC/coex init, and flash
  power-down needs RAM-resident sleep code — none of these are implemented.
- H2 HP_SLEEP clock power sets `i2c_iso_en`/`i2c_retention` (bits 26/27) with
  `xpd_bbpll=0`, as IDF v5.5.1 `PMU_SLEEP_POWER_CONFIG_DEFAULT` does; the
  first port wrote the whole register with both clear, so the analog-I2C
  (BBPLL) configuration was not retained across sleep. LP_SLEEP keeps
  `bod_source_sel=0`/`vddbat_mode=0`: IDF selects the VBAT brownout source
  only for VBAT-powered deep sleep (`pmu_sleep.c`), never for light sleep.
- Use calibrated Q19 RTC periods and checked 48-bit timer arithmetic. Handle H2
  ECO2 RC_FAST calibration division, including REF_TICK enable/restoration.
  Detect zero/invalid calibration and restore clocks before returning an error.
- `RetainedSleepError::InvalidCalibration(CalibrationFailure)` names the failed
  step: `SlowClockPeriodZero` (`LP_AON.store1` or the timer period is 0),
  `FastClockTimeout` (TIMG0 RC_FAST calibration returned 0, i.e.
  `rtc_cali_timeout`), `FastClockOutOfRange` (H2: RC_FAST period outside
  1..40 MHz, or an arithmetic overflow). C6 keeps the generic period check.
- H2 RC_FAST calibration sets `PCR.ctrl_tick_conf.fosc_tick_num = 255` before
  every RC_FAST measurement, and `rtc::init` sets it at boot, as IDF v5.5.1
  `clk_ll_rc_fast_tick_conf()` (`REG_FOSC_TICK_NUM`, "enable a division of 32
  to the fosc clock") does in `rtc_clk_cal_internal`; the C6 init already did.
  A v1.2 (ECO2+) H2 read back the reset value 7, and every light-sleep attempt
  failed with `InvalidCalibration` (cause `0x14000003`).
- H2 `calibrate_internal` restores RC_FAST and RC32K like IDF: after the
  measurement, the digital gate and then the oscillator are switched off again
  if the calibration had enabled them (upstream re-set them only when they had
  been on, which never turned them off). C6 is unchanged.
- H2 `calibrate_internal` no longer forces `icg_hp_xtal32k` and the XTAL32K and
  RC32K oscillators on (the upstream "very hacky" block, kept for C6); only the
  oscillator being calibrated is enabled. H2 light sleep programs LP_ACTIVE and
  LP_SLEEP `xpd_xtal32k`/`xpd_rc32k` as IDF `PMU_SLEEP_POWER_CONFIG_DEFAULT`
  does with `PMU_SLEEP_PD_XTAL32K`/`PMU_SLEEP_PD_RC32K`: on only for the
  oscillator that is the RTC slow clock (off for RC_SLOW). Before, both ran in
  active and sleep (XTAL32K shares GPIO13/14 with the pads).
- H2 deep sleep (`Rtc::sleep_deep`, now enabled for `esp32h2`), LP timer
  wake only: `RtcSleepConfig::start_sleep` routes `deep` to a separate
  `start_deep_sleep`, so the retained light-sleep admission (which still
  rejects `deep`) is unchanged. It follows IDF v5.5.1 `esp_deep_sleep_start`
  → `pmu_sleep_init(dslp)` → `pmu_sleep_start`: all `PMU_SLEEP_PD_*` power
  flags (top, CPU, modem, VDD_SDIO, XTAL, RC_FAST, XTAL32K) from
  `RtcSleepConfig::apply`, `PMU_SLEEP_DIGITAL_DSLP_CONFIG_DEFAULT` (ICG 0,
  pad sleep select off), `PMU_SLEEP_ANALOG_DSLP_CONFIG_DEFAULT` (HP regulator
  off; LP regulator on at the `DSLP_DBIAS` sleep dbias, driver 7), the same
  timing parameters, no reject sources and `store9` deep flag; the CPU spins
  until the request takes effect, and the timer wake resets the chip. Any
  other wake source fails with `UnsupportedWakeSource` (a panic through
  `sleep_deep`) before a register changes. GPIO digital-hold isolation is
  not implemented (IDF only does it when digital hold is enabled). Used only
  by the `esp32h2-floor-lab` image to measure board quiescent current.
  Lab-tested on one Waveshare ESP32-H2-Zero (chip v1.2, WSTK AEM at 3.3 V):
  10 consecutive 30.0 s timer wakes, each a clean reboot; the board draws
  0.73 mA in deep sleep, so that figure bounds the chip's deep-sleep
  current but does not qualify it.
- Correct SYSTIMER high-word reload (`>> 32`, not `<< 32`) for all existing
  setter branches.

## Integration and limits

Patch **all** firmware dependency graphs to this single HAL copy; keep the pinned
compiler and radio-facing HAL APIs. The product `sleep.rs` requires the local
retained entry. It owns calibrated elapsed-time accounting and returns missing
SYSTIMER ticks; the HAL does **not** advance SYSTIMER automatically. With
`xtal` power-down SYSTIMER stops for the whole sleep; the same RTC-calibrated
compensation covers it.

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
