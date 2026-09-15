# Firmware Size

Embedded size is measured from the final release artifact produced by the
target's pinned toolchain. The ELF file size on the host is not a flash-usage
number.

## Current images

Baseline snapshot: **2026-09-06**. EFR32MG1 was refreshed **2026-09-08**.
The TLSR8258 parent-router, always-on nRF52840, EFR32MG21, CC2340 fallback,
and ESP32-C6/H2 rows were refreshed
**2026-09-10**. These are named local build snapshots, not the latest remote
CI results; unrefreshed rows do not describe the current working tree.
The recorded parent router and all four ESP variants **fail their regression
budgets**. Prior hardware evidence is not an exact-image HIL rerun unless
explicitly stated.

The gate column is a regression budget except for PHY6222 and PHY6252, where
it is the hard XIP slot limit.

| image | measured | gate | headroom | metric |
|---|---:|---:|---:|---|
| PHY6222 sensor | 130,624 | 130,816 | 192 | occupied XIP span |
| PHY6252 feature-selected sensor | 130,464 | 130,816 | 352 | occupied XIP span |
| BL702 sensor | 189,442 | 192,512 | 3,070 | raw binary |
| nRF52840 default / BME280 / SHT31 | 224,472 / 231,792 / 228,216 | 225,280 / 245,760 / 241,664 | 808 / 13,968 / 13,448 | raw binaries |
| nRF52840 always-on End Device | 231,280 | 253,952 | 22,672 | file-backed flash span |
| nRF52840 UF2 ProMicro / MDK / PCA10059 / DK | 222,968 / 222,848 / 224,536 / 224,552 | 237,568 each | 14,600 / 14,720 / 13,032 / 13,016 | linked images before UF2 |
| nRF52833 default / BME280 / SHT31 | 224,464 / 231,784 / 228,208 | 225,280 / 245,760 / 241,664 | 816 / 13,976 / 13,456 | raw binaries |
| EFR32MG1 sensor | 163,236 | 167,936 | 4,700 | raw binary |
| EFR32MG21 sensor | 202,068 | 212,992 | 10,924 | raw binary |
| CC2340R5 pinned-SDK sensor (pre-static-task snapshot) | 223,536 | 225,280 | 1,744 | historical raw binary; not rebuilt |
| CC2340R5 fallback sensor | 213,160 | 225,280 | 12,120 | raw binary; radio firmware unavailable |
| ESP32-C6 sensor default / `light-sleep` | 381,056 / 392,080 | 368,640 each | -12,416 / -23,440 | application images; over budget |
| ESP32-H2 sensor default / `light-sleep` | 365,920 / 376,640 | 356,352 each | -9,568 / -20,288 | application images; over budget |
| TLSR8258 default / LOW32K 250 ms / LOW32K 10 s | 290,616 / 295,548 / 295,552 | 294,912 / 299,008 / 299,008 | 4,296 / 3,460 / 3,456 | raw binaries |
| TLSR8258 parent router | 436,072 | 430,080 | -5,992 | raw binary; over budget |

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
- ESP32-C6 default/light merged flash: 446,592/457,616 B; H2: 431,456/442,176 B.
  OTA slot: 2,031,616 B each. Their regenerated version-1 Zigbee OTA containers
  are 381,122/392,146 B for C6 and 365,986/376,706 B for H2.
  These combined-fix images have host validation, not hardware execution
  evidence, and all remain over their unchanged regression budgets.
- PHY6252 has the separate exact feature-selected occupied-XIP measurement
  shown above; its hardware path remains unverified.
- TLSR8258 physical application boundary: 458,752 B (`0x70000`), followed by
  APS/child/security journals at `0x70000`/`0x72000`/`0x74000`.

The TLSR8258 router's 430,080 B (`0x69000`) gate replaces the pre-R22
356,352 B baseline. It was set above the earlier 427,776 B image.
The current 436,072 B image exceeds it by 5,992 B, blocking release.
The physical 458,752 B (`0x70000`) boundary and journals did not move:
22,680 B of physical headroom remains, but that does not waive the
regression gate. The gate-to-boundary separation is 28,672 B. The existing
TC32 toolchain linked the persistent-APS composition with its physical
memory assertions; its build command still fails the regression-size gate.

## RAM and stack snapshot

| image | `.data` | `.bss` | static total | stack |
|---|---:|---:|---:|---:|
| PHY6222 / PHY6252 | 652 | 4,288 | 4,940 | 54,384 available |
| EFR32MG1 | 260 | 14,720 | 14,980 | 16,760 available |
| EFR32MG21 | 308 | 18,320 | 18,628 | 46,904 available |
| nRF52840 always-on End Device | 88 | 18,444 | 18,532 | 242,588 available |
| CC2340R5 fallback | 16 | 17,428 | 17,444 | 19,420 available |
| ESP32-C6 default / `light-sleep` | 3,412 / 3,608 | 50,384 | 53,796 / 53,992 | 390,696 / 390,504 linked |
| ESP32-H2 default / `light-sleep` | 3,120 / 3,368 | 50,328 | 53,448 / 53,696 | 201,312 / 200,640 linked |
| TLSR8258 retained fresh-root SVC | — | — | — | 8,448 linked |

ESP32 initialized data includes `.data.wifi` (80 B for C6, 84 B for H2);
the static totals exclude RAM-resident code. The always-on nRF52840 image
also reserves 1,024 B of `.uninit`, for 19,556 B total occupied SRAM.
Its 18,408 B and CC2340's 16,776 B main-task pools now use compiler-sized
static storage rather than the insufficient dynamic arena; both linkers
require at least 16 KiB of stack reserve. CC2340's old pinned-SDK RAM figure
is not a valid measurement of this new task allocation.
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
