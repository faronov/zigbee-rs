# BL702 TSEN experiment

## Status

**Lab-only and blocked. Do not use this as a supported BL702 temperature
source.**

The on-chip TSEN path was tried on one DOIT XT-ZB1 board. Both the pure-Rust
implementation archived here and the official Bouffalo SDK path returned
implausible values, approximately **-220 °C to -239 °C**. Reproducing the
failure with both implementations did not validate the method; it left the
tested sample, its calibration data, and the measurement path unresolved.

Release integration is blocked until the experiment is repeated on a second
BL702 sample and independently checked against a known ambient temperature.

Validation level:

- register setup and conversion helpers: compiled and host-tested while the
  attempt lived in `bl702-hal`;
- one XT-ZB1 sample: lab-tested, failed with implausible readings;
- production support: none;
- remaining gate: repeat on a second BL702 sample, then verify calibration,
  raw dual-bias codes, and temperature against an independent reference.

## Live register probe (same sample)

A second diagnosis drove the GPADC directly through the resident
`eflash_loader` helper over USB ISP (`memory_write` 0x50 /
`memory_read` 0x51), without flashing firmware.

Probe caveats:

- `memory_read` assembles bytes 2–3 with `lhu`/`lbu`. Peripherals return zero
  for those sub-word reads, so only the low 16 bits are trustworthy. Writes
  use `sw` and work across all 32 bits.
- The averaged `RESULT` register and the GPIP FIFO cannot be read
  meaningfully this way. The 12-bit `RAW_RESULT` register (`0x4000F934`) can.

Findings, all from `RAW_RESULT` with Vref 2.0 V:

- eFuse TSEN trim is present: enable=1, refcode=2023, parity valid.
- The ADC itself works. VBAT/2 reads about 3.06 V, GND reads the offset, and
  PGA gain changes scale the reading.
- External mode (`TSEXT_SEL=1`) reacts to `TSVBE_LOW` by about 110 12-bit
  codes on the floating pad.
- Internal diode on channel 14 sits at about 1800 codes. It does not change
  with `TS_EN`, `CHIP_SEN_PU`, `SEN_SEL`, or `SEN_TEST_EN`.
- With PGA chopping off (the Zephyr `adc_bflb` TSEN setup: chop=VREF_AZ,
  VCM 1.2 V, no dither/DWA), toggling `TSVBE_LOW` changes it by only 5–8
  12-bit codes. That is about 80–140 16-bit codes, or 3–4 mV. The calibrated
  formula expects about refcode + 7.753·T ≈ 2200 codes (about 68 mV) at room
  temperature.

Conclusion: the failure is analog, not a formula or calibration-decode bug.
On this sample the internal diode's dual-current bias does not switch, so
`(dV − refcode) / 7.753` lands near −245 °C regardless of software.

## Archived source

The `snapshots/` files preserve the low-level ADC and eFuse source from the
failed attempt (with only the leading module comment changed to mark each file
as archived):

- `snapshots/adc.rs` contains the TSEN register configuration, dual-bias
  sampling, conversion formula, diagnostic probes, and host tests;
- `snapshots/efuse.rs` contains the calibration-word decoder and the eFuse
  shadow-reload probe used during diagnosis.

They are source snapshots, not a standalone crate. After removing the
unverified API from the production HAL, making this a buildable firmware would
either expose experimental register operations through `bl702-hal` again or
duplicate BL702 startup, UART, linker, and MMIO ownership in a new crate. That
would broaden this cleanup without improving the unresolved hardware result.

To resume the work, copy the snapshots into a disposable branch or lab
worktree, add an empty `diagnostic-probes` feature to `bl702-hal`, and drive the
methods from a dedicated `tools/bl702-lab` diagnostic binary. Do not wire the
result into `examples/bl702-sensor` until the second-sample gate passes.

## Production behavior

The production XT-ZB1 example does not expose Device Temperature Configuration
cluster `0x0002` and does not claim BL702 chip-temperature support.
Environmental temperature and humidity remain synthetic until a physical
sensor is selected; the existing VBAT/GPADC path is unaffected by this archive.
