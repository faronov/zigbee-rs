# BL702 Zigbee Scaffold

This crate is a compile-only experiment around Bouffalo's binary BL702 radio
libraries. It is not a pure-Rust radio backend and is not hardware-proven.

## SDK Audit Result

- `liblmac154.a` contains the IEEE 802.15.4 MAC/PHY implementation.
- `libbl702_rf.a` contains RF initialization and calibration.
- The vendor Zigbee stack is also binary-only, although zigbee-rs does not
  need it.
- The archives retain symbols and DWARF. Static analysis recovered the M154
  register paths plus the RF reset, live-calibration, channel, RX, CCA, and
  TX-power algorithms needed for an independent implementation.
- Startup, CLIC dispatch, boot headers, and image packaging are not yet proven
  by this crate.

A pure-Rust port is technically feasible, but it is not implemented or
hardware-proven yet. Cold boot must run live ACAL, KCAL, ROSCAL, and RCCAL;
a generic precomputed calibration snapshot is not a safe replacement.

The vendor Zigbee linker exposes 112 KiB at `0x42014000..0x42030000`, but
reserves `0x42028000..0x420283ff`. This scaffold places data/heap in the lower
80 KiB and the stack in the upper 31 KiB instead of treating the window as one
flat allocation.

## Compile-Only Check

```bash
cd examples/bl702-sensor
cargo build --release --features stubs
```

The `stubs` feature supplies no-op radio symbols. The resulting ELF checks Rust
integration only and must not be flashed.

## Vendor ABI Experiment

The vendor archives use `rv32imfc/ilp32f`. Any future hybrid build must use:

```bash
BL_IOT_SDK_DIR=/path/to/bl_iot_sdk \
  cargo build --release --target riscv32imafc-unknown-none-elf
```

Stripping the ELF float-ABI flag is invalid because it changes metadata, not
the calling convention. The current runtime/startup, interrupt dispatch,
missing vendor dependencies, and boot image format still need dedicated
bring-up before this path can produce firmware.
