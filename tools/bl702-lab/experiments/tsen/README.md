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
