# EFR32MG1P TRÅDFRI hardware lab

Standalone, explicitly named hardware diagnostics for the connected
`EFR32MG1P132F256IM32`. These are not production firmware profiles and the
package has no feature-selected entry points.

## Binaries

| Binary | Purpose |
|---|---|
| `efr32mg1-diag-nv` | Write/read the bounded application-NV journal |
| `efr32mg1-diag-em2` | Ten RTCC/EM2 cycles with retained SRAM canaries |
| `efr32mg1-diag-rtcc-time` | Embassy RTCC timer plus explicit EM2 waits |
| `efr32mg1-diag-radio-em2` | Raw TX before and after repeated EM2 |
| `efr32mg1-diag-sht` | I2C/SHT3x probe and repeated CRC-checked samples |
| `efr32mg1-diag-join` | No-NV join, poll, announce, and descriptor path |
| `efr32mg1-diag-beacon` | Bounded raw TX gate followed by active scans |

Build from this directory so `.cargo/config.toml` supplies the target and
bootloader-safe linker script:

```bash
cargo build --release --bin efr32mg1-diag-nv
cargo build --release --bin efr32mg1-diag-em2
cargo build --release --bin efr32mg1-diag-rtcc-time
cargo build --release --bin efr32mg1-diag-radio-em2
cargo build --release --bin efr32mg1-diag-sht
cargo build --release --bin efr32mg1-diag-join
cargo build --release --bin efr32mg1-diag-beacon
```

Verify each ELF before producing a HEX:

```bash
tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-diag-em2
```

## Memory and flash safety

```text
0x00000000..0x00003FFF  resident Gecko bootloader
0x00004000..0x00036FFF  selected diagnostic image
0x00037000..0x00038FFF  production security journal
0x00039000..0x00039FFF  Rust application NV
0x0003A000..0x0003FFFF  existing native NVM3
```

Only `efr32mg1-diag-nv` intentionally writes flash, and only in
`0x39000..0x39FFF`. No command in this package flashes hardware. Never use a
mass erase with this layout.

The binaries compile shared vector, application-properties, panic/fault-log,
and RTCC time-driver source directly into each final ELF. `diag-em2` excludes
the Embassy time driver and owns its minimal `RTCC` handler; all async
diagnostics include the RTCC Embassy handler. `diag-nv` initializes neither.

Historical `sed-diag` retained milestone records and the one-shot
`sed-migrate` gate were retired during the production/lab split. Their
hardware-proven outcomes remain documented in repository history; they are
not general-purpose diagnostics.
