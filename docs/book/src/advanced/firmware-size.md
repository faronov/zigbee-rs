# Firmware Size

Embedded size is measured from the final release artifact produced by the
target's pinned toolchain. The ELF file size on the host is not a flash-usage
number.

## Current images

Baseline snapshot: **2026-09-06**. The always-on nRF52840 and TLSR8258
parent-router rows were refreshed **2026-09-07**; EFR32MG1 and ESP32-C6/H2
were refreshed **2026-09-08**. The parent router and ESP32-C6 currently
**fail their regression budgets**; the other refreshed images pass their
build/layout gates. Prior hardware evidence is
not an exact-image HIL rerun unless explicitly stated.

The gate column is a regression budget except for PHY6222 and PHY6252, where
it is the hard XIP slot limit.

| image | measured | gate | headroom | metric |
|---|---:|---:|---:|---|
| PHY6222 sensor | 130,624 | 130,816 | 192 | occupied XIP span |
| PHY6252 feature-selected sensor | 130,464 | 130,816 | 352 | occupied XIP span |
| BL702 sensor | 189,442 | 192,512 | 3,070 | raw binary |
| nRF52840 default / BME280 / SHT31 | 224,472 / 231,792 / 228,216 | 225,280 / 245,760 / 241,664 | 808 / 13,968 / 13,448 | raw binaries |
| nRF52840 always-on End Device | 226,584 | 253,952 | 27,368 | file-backed flash span |
| nRF52840 UF2 ProMicro / MDK / PCA10059 / DK | 222,968 / 222,848 / 224,536 / 224,552 | 237,568 each | 14,600 / 14,720 / 13,032 / 13,016 | linked images before UF2 |
| nRF52833 default / BME280 / SHT31 | 224,464 / 231,784 / 228,208 | 225,280 / 245,760 / 241,664 | 816 / 13,976 / 13,456 | raw binaries |
| EFR32MG1 sensor | 163,236 | 167,936 | 4,700 | raw binary |
| EFR32MG21 sensor | 202,820 | 212,992 | 10,172 | raw binary |
| CC2340R5 pinned-SDK sensor | 223,536 | 225,280 | 1,744 | raw binary |
| ESP32-C6 sensor | 369,248 | 368,640 | -608 | application image; over budget |
| ESP32-H2 sensor | 354,096 | 356,352 | 2,256 | application image |
| TLSR8258 default / LOW32K 250 ms / LOW32K 10 s | 290,616 / 295,548 / 295,552 | 294,912 / 299,008 / 299,008 | 4,296 / 3,460 / 3,456 | raw binaries |
| TLSR8258 parent router | 433,756 | 430,080 | -3,676 | raw binary; over budget |

The measurements use:

- `nightly-2026-03-23` for the workspace, nRF, BL702, CC2340, and EFR32;
- `nightly-2026-08-01` for ESP32 and PHY6222;
- `tc32-stage2-tc32-45` for TLSR8258.

Changing the compiler invalidates direct size comparisons.

Additional artifacts and physical limits:

- BL702 boot image: 197,648 B; physical slot: 1,044,480 B.
- EFR32MG1's prior Zigbee OTA container was 162,538 B; it has not been
  regenerated for the current image. Resident Gecko bootloader assumed.
- EFR32MG21: no OTA packaging path.
- CC2340R5 physical application slot: 516,096 B.
- ESP32-C6/H2 merged flash: 434,784 B / 419,632 B; OTA slot: 2,031,616 B each.
  The prior C6/H2 Zigbee OTA containers (368,098 B / 352,994 B) have not been
  regenerated for the current application images.
- PHY6252 has the separate exact feature-selected occupied-XIP measurement
  shown above; its hardware path remains unverified.
- TLSR8258 physical application boundary: 458,752 B (`0x70000`), followed by
  APS/child/security journals at `0x70000`/`0x72000`/`0x74000`.

The TLSR8258 router's 430,080 B (`0x69000`) gate replaces the pre-R22
356,352 B baseline. It was set above the earlier 427,776 B image.
The current 433,756 B image exceeds it by 3,676 B, blocking release.
The physical 458,752 B (`0x70000`) boundary and journals did not move:
24,996 B of physical headroom remains, but that does not waive the
regression gate. The gate-to-boundary separation is 28,672 B.

## RAM and stack snapshot

| image | `.data` | `.bss` | static total | stack |
|---|---:|---:|---:|---:|
| PHY6222 / PHY6252 | 652 | 4,288 | 4,940 | 54,384 available |
| EFR32MG1 | 260 | 14,720 | 14,980 | 16,760 available |
| EFR32MG21 | 308 | 18,280 | 18,588 | 46,944 available |
| CC2340R5 | 16 | 4,756 | 4,772 | — |
| ESP32-C6 | 3,008 | 50,344 | 53,352 | — |
| ESP32-H2 | 2,652 | 50,272 | 52,924 | — |
| TLSR8258 retained fresh-root SVC | — | — | — | 8,448 linked |

ESP32-H2 initialized data includes 84 B in `.data.wifi`.
EFR32MG1 has exactly `0x7C00` bytes of usable SRAM and 376 B of margin above
its 16 KiB stack gate. TLSR8258's retained SVC stack has 256 B above its 8 KiB
gate. These linked values are not runtime high-water measurements.

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

`compact-single-endpoint` is a product-capacity feature, not a protocol or
role feature. It keeps two application-endpoint slots and four reporting
entries without removing Zigbee behavior. The EFR32MG1, PHY62x2 sensor, and
TLSR8258 sensor/router products currently select it, after profile-capacity
assertions.

There is no heap allocator or public trait-object application graph. One
internal pinned `dyn Future` outlining path controls TC32 code duplication
without allocating.

CI uses symbol gates in addition to byte counts. For example:

- sensor images must not contain parent/router maintenance;
- the nRF always-on End Device must contain neither routing/parent maintenance
  nor coordinator startup;
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
[`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/experiment/r22-bdb-complete/BUILD.md)
and `.github/workflows/ci.yml`.
