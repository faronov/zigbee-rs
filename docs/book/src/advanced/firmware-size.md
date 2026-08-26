# Firmware Size

Embedded size is measured from the final release artifact produced by the
target's pinned toolchain. The ELF file size on the host is not a flash-usage
number.

## Current images

| image | measured bytes | format |
|---|---:|---|
| nRF52840 sensor, default | 223,344 | raw binary |
| nRF52840 sensor, BME280 | 230,560 | raw binary |
| nRF52840 sensor, SHT31 | 227,040 | raw binary |
| nRF52833 sensor, default | 223,400 | raw binary |
| nRF52833 sensor, BME280 | 230,976 | raw binary |
| nRF52833 sensor, SHT31 | 227,128 | raw binary |
| nRF52840 always-on End Device | 214,864 | raw binary |
| nRF52840 UF2 variants | 221,736–223,456 | linked image before UF2 |
| ESP32-C6 sensor | 354,512 | application image |
| ESP32-H2 sensor | 339,568 | application image |
| BL702 sensor | 182,786 | raw binary |
| BL702 packaged image | 190,992 | boot image |
| CC2340 sensor, pinned SDK | 212,688 | raw binary |
| PHY6222 sensor | 129,556 | packaged PHY6 image |
| EFR32MG1 sensor | 156,612 | raw binary |
| EFR32MG21 sensor | 201,192 | raw binary |
| TLSR8258 default SUSPEND sensor | 279,652 | raw binary |
| TLSR8258 LOW32K 250 ms | 284,436 | raw binary |
| TLSR8258 LOW32K 10 s | 284,440 | raw binary |
| TLSR8258 parent router | 343,660 | raw binary |

The measurements use:

- `nightly-2026-03-23` for the workspace, nRF, BL702, CC2340, and EFR32;
- `nightly-2026-08-01` for ESP32 and PHY6222;
- `tc32-stage2-tc32-45` for TLSR8258.

Changing the compiler invalidates direct size comparisons.

## Static RAM snapshot

`llvm-size` `.data + .bss`, excluding stack and linker-specific reservations:

| image | bytes |
|---|---:|
| nRF52840 sensor | 37,464 |
| nRF52833 sensor | 37,464 |
| nRF52840 always-on End Device | 4,232 |
| ESP32-C6 | 52,268 |
| ESP32-H2 | 51,848 |
| BL702 | 31,984 |
| PHY6222 | 4,288 |
| EFR32MG1 | 14,912 |
| EFR32MG21 | 17,136 |
| TLSR8258 sensor | 13,468 |
| TLSR8258 router | 24,280 |

These values do not prove stack headroom. Inspect the target map and existing
layout checker. EFR32MG1 has exactly `0x7C00` bytes of usable SRAM; never
replace it with a rounded 32 KiB linker region.

## Two independent limits

Every production build should check:

1. **regression budget** — catches unexpected growth;
2. **physical boundary** — prevents overlap with bootloader, security,
   child-table, factory, or OTA regions.

Passing a growth budget does not prove the linker boundary, and vice versa.
CI checks both for targets with protected partitions.

## Why the application model remains small

The shared applications use concrete generic capabilities:

- `NoStatus` removes status-only timing and indication paths;
- `NoOta` removes OTA only when the profile is statically non-OTA;
- `AlwaysOnEndDeviceApp` removes router and parent/child lifecycle;
- an end-device role removes router/parent maintenance;
- products drop unused peripheral tokens and let dead-code elimination remove
  their drivers.

There is no heap allocator or public trait-object application graph. One
internal pinned `dyn Future` outlining path controls TC32 code duplication
without allocating.

CI uses symbol gates in addition to byte counts. For example:

- sensor images must not contain parent/router maintenance;
- the nRF relay must contain routing but not child-admission or coordinator
  startup;
- hardware-AES products must contain the selected backend and no software
  fallback;
- Telink sensor/router images must preserve their role-specific partitions.

## Measurement commands

Raw binary:

```bash
OBJCOPY=$(find "$(rustc --print sysroot)" -name llvm-objcopy -print -quit)
"$OBJCOPY" -O binary path/to/firmware path/to/firmware.bin
stat -f '%z' path/to/firmware.bin   # macOS
```

Static sections:

```bash
SIZE=$(find "$(rustc --print sysroot)" -name llvm-size -print -quit)
"$SIZE" path/to/firmware
```

Use the platform packager instead of raw `objcopy` for ESP, BL702, PHY62x2,
and UF2 deployment formats. Exact commands and current budgets are in
[`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/experiment/zephyr-app-model/BUILD.md)
and `.github/workflows/ci.yml`.
