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
| `efr32mg1-diag-spi` | Read-only USART0/SPI JEDEC-ID probe on the external flash |
| `efr32mg1-diag-pwm` | TIMER0 CC0 PWM fade on the PA0 LED |
| `efr32mg1-diag-ota-storage` | Read-only resident Gecko bootloader/storage/slot probe |
| `efr32mg1-diag-ota-write` | Destructive external OTA-slot erase/write/readback gate |
| `efr32mg1-diag-ota-install` | One-shot GBL staging, verification, and bootloader install gate |

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
cargo build --release --bin efr32mg1-diag-spi
cargo build --release --bin efr32mg1-diag-pwm
cargo build --release --bin efr32mg1-diag-ota-storage
cargo build --release --bin efr32mg1-diag-ota-write
EFR32_GBL_PATH=/absolute/path/update.gbl \
  cargo build --release --bin efr32mg1-diag-ota-install
```

Verify each ELF before producing a HEX:

```bash
tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-diag-em2
tools/verify-layout.py \
  target/thumbv7em-none-eabi/release/efr32mg1-diag-ota-storage
```

## Memory and flash safety

```text
0x00000000..0x00003FFF  resident Gecko bootloader
0x00004000..0x00036FFF  selected diagnostic image
0x00037000..0x00038FFF  production security journal
0x00039000..0x00039FFF  Rust application NV
0x0003A000..0x0003FFFF  existing native NVM3
```

`efr32mg1-diag-nv` writes only `0x39000..0x39FFF`.
`diag-ota-write` intentionally erases and programs external Gecko OTA slot 0,
checks a write crossing offset 256, and erases the slot again before halting.
It never requests activation. `diag-ota-install` is a one-shot destructive
gate: it embeds the GBL named by `EFR32_GBL_PATH`, replaces external slot 0,
verifies it with the resident parser, and immediately resets into the
bootloader to install it. Use it only with a separately verified application-
only GBL that remains below the internal journal boundary. `diag-spi` sends
only the read-JEDEC-ID command.
`diag-ota-storage` calls only Gecko bootloader discovery, init, information,
slot-read, and deinit APIs; it does not erase/program storage, alter the
bootload list, or request installation. No command in this package flashes
hardware. Never use a mass erase with this layout.

The binaries compile shared vector, application-properties, panic/fault-log,
and RTCC time-driver source directly into each final ELF. `diag-em2` excludes
the Embassy time driver and owns its minimal `RTCC` handler; all async
diagnostics include the RTCC Embassy handler. `diag-nv` initializes neither.

Historical `sed-diag` retained milestone records and the one-shot
`sed-migrate` gate were retired during the production/lab split. Their
hardware-proven outcomes remain documented in repository history; they are
not general-purpose diagnostics.
