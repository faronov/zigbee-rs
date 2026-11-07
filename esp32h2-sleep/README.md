# esp32h2-sleep

RAM-resident ESP32-H2 PMU light and deep sleep, ported from ESP-IDF v5.5.1.
It replaces the H2 half of the vendor esp-hal retained-sleep backport
(`vendor/esp-hal/LOCAL-PATCHES.md`) with a crate this repository owns, and
adds the IDF light-sleep mode that powers the TOP, CPU and modem domains off.

## Ownership

```text
examples/esp32h2-sensor   thermometer.toml `sleep_mode`, floor_lab.toml `light_mode`
products/esp32-zigbee-devkit
  sleep.rs                calibration check, SYSTIMER compensation, radio
                          suspend/resume, eFuse -> ChipInfo, error mapping
esp32h2-sleep             PMU words, analog/bias, clocks, admission,
                          retention tables, RAM-resident entry and restore
```

The crate has no esp-hal dependency (only the `esp32h2` PAC). It never reads
eFuses: the owner passes `ChipInfo` (PMU DBIAS fuses and chip revision),
because eFuse access is flash code.

## Modes

| Mode | Feature / key | Domains off | Wake | Status |
|------|---------------|-------------|------|--------|
| `Retained` (Step A) | default | none (0.6 V retention, clock-gated); XTAL and RC_FAST may stop | LP timer, digital GPIO | port of the hardware-proven vendor path; **host-tested and built, hardware re-proof pending** |
| `TopPowerDown` (Step B) | `top-pd` + `sleep_mode = "top_pd"` | TOP, CPU, modem; XTAL required off | LP timer, EXT1 (GPIO7..=14) | **compiles, host-tested, RAM-checked; not run on hardware** |
| deep sleep | always | all | LP timer (chip reset) | port of the vendor `sleep_deep`; hardware re-proof pending |

`Retained` remains the default until `TopPowerDown` is proven on hardware.

## Step B: what TOP power-down retains

ESP-IDF H2 retains these with software-triggered REGDMA
(`SOC_PM_RETENTION_SW_TRIGGER_REGDMA`) and software CPU retention
(`sleep_cpu.c`). This crate does both with CPU copies from `.rwtext`.

| Group | Regions (words) | IDF source |
|-------|-----------------|------------|
| Clocks/resets | PCR 85 + reset-event bypass 1 | `system_retention_periph.c` (clock) |
| Flash controller | SPI1/SPI0 MEM 2 x (55 + 41 + 1 + 31) | `spimem_regs_retention` |
| IO | IO_MUX 29, GPIO 61 + func-out 32 + func-in 127 | `iomux_regs_retention` |
| System | INTMTX 69, HP_SYSTEM 12, HP_APM 68, TEE 33 (then M4 mode = 0) | `intr_matrix_*`, `hp_system_*`, `tee_apm_*` |
| Modem clocks | MODEM_SYSCON 7, MODEM_LPCON 7 | `sleep_clock.c` modem |
| Radio | BTBB 93 + 62 + 19, 802.15.4 MAC 99 | `btbb_retention`, `ieee802154_mac_regs_retention` |
| Timer | SYSTIMER 15 (unit/comparator load sequence) | `systimer_regs_retention` |
| CPU domain | PLIC M/U 78, CLINT 12, INTPRI 46, cache wrap 1 | `sleep_cpu.c` `cpu_domain_dev_*` |
| CPU core | 73 non-critical CSRs, 39-word critical frame | `RvCore*SleepFrame` |
| Console pad | USB-Serial-JTAG CONF0 1 | `sleep_console_usj_pad_backup` |

Total **5,296 B** of `.bss` (`FRAMES` 5,140 + `ESP32H2_SLEEP_CPU_FRAME`
156). IDF needs no MMU-table retention on H2 (the table survives).

Not retained, deliberately (IDF does not either): UART, I2C, SPI2, TIMG
(except the MWDT flash-boot bits, cleared on wake), APB_SARADC/TSENS, AES,
SHA, ECC, RMT, LEDC. Users must reconfigure them after a wake with
`Wake::top_powered_down`. In this repository:

* AES (`zigbee-mac` `esp_aes`) writes mode and key on every block;
* TSENS (`chip_temperature`) re-selects its range and powers up per read;
* RTC calibration (TIMG0) programs every field it uses;
* UART0 is unused in quiet builds; a logging build loses its UART0 setup
  (USB-Serial-JTAG is the console and its pad word is restored).

## Wake sequence (TOP power-down)

1. Save CPU frames, CSRs and every region (after the switch to XTAL).
2. `LP_AON_STORE8` = `esp32h2_sleep_cpu_restore`; save the critical frame.
3. Request sleep. A reject returns with nothing lost.
4. Wake: the ROM jumps to the restore stub (critical frame). In order:
   non-critical CSRs and CPU frames (first, as in IDF: the power-down
   clears `mpcer`/`mpcmr`, and the ROM `ets_delay_us` used by the clock
   restore spins on that cycle counter), PCR, MWDT0/1 flash-boot off, modem
   clocks, sleep bias fix, analog, CPU clock (BBPLL), SPI1/SPI0,
   IO_MUX/GPIO, INTMTX, HP_SYSTEM, TEE (+ M4 mode), HP_APM, SYSTIMER, BTBB,
   802.15.4 MAC, USJ CONF0; invalidate and resume the I-cache; return to
   the caller.

The radio is suspended by the product before sleep and resumed after it
(`enable_phy` = `phy_wakeup_init` + digital restore): the IDF modem
power-down flow.

## Differences from ESP-IDF v5.5.1

| Item | IDF | Here | Kind |
|------|-----|------|------|
| Retention engine | REGDMA links, software trigger | CPU copies in `.rwtext` | conscious (same register lists, no link memory) |
| PCR save point | before the XTAL switch | after it | conscious: restoring PCR cannot glitch the bus clock |
| 802.15.4 MAC | from base | from +4, `COMMAND` skipped | conscious: a restored command could start TX/RX |
| SYSTIMER period mode | always toggled | only when set | conscious |
| Restore point | after clock restore, flash usable | before any flash access | conscious, required by in-crate stub |
| MWDT flash-boot | TIMG0 only | TIMG0 and TIMG1 | conscious |
| Pad hold / GPIO isolation in TOP PD | none for H2 | product latches its isolated pads with `LP_AON_GPIO_HOLD0` (`pad_isolation`) | measured: without the hold the H2-Zero `top_pd` floor was 1.02 mA vs 0.80 mA retained; fix not yet measured |
| ROM wake path and flash/cache | ROM-internal | assumed not to touch flash before the stub | **risk**: hardware gate |
| TSENS DAC trim (regi2c) | re-applied by driver | re-applied per read | covered |
| UART0 retention | optional module | none | **risk** for logging builds only |

## Safety checks

* `tools/esp32h2-sleep-ram-check.py ELF...` (CI, `build-esp32h2`): every
  crate function in `.rwtext` and everything it calls stays in SRAM or mask
  ROM; no flash address is built; buffers end below the ROM wake stack
  (0x4084D380); `.dram2_uninit` is empty.
* Host tests (`cargo test -p esp32h2-sleep`): admission, PMU power/analog
  words, timing and the retention tables (no overlap, IDF bounds, budget).

## Cost model (to verify on hardware)

* RAM: 5,296 B `.bss`, only with `top-pd`.
* Wake: ~1,300 words saved and restored (~0.3-0.5 ms CPU) plus the TOP/CPU
  power-up (IDF budgets ~0.5-1 ms): about 5-10 uC per wake.
* Saving: the measured retained-sleep excess is ~70-85 uA (phase B minus
  phase D); the datasheet TOP power-down floor is ~25 uA, so ~60 uA, or
  1.8 mC per 30 s poll. Break-even wake interval: ~0.2 s.

## Validation status

| Gate | Retained | TopPowerDown |
|------|----------|--------------|
| Host tests | yes | yes |
| Links, clippy, RAM check | yes | yes |
| USB self-test (sleep OK, registers, TSENS) | pending | pending |
| Zigbee join/poll/report after wake | pending | pending |
| WSTK current capture | pending | pending |

The vendor esp-hal H2 sleep entry stays in tree, unused by the product,
until both rows are proven; then it is removed.
