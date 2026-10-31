# Firmware Size

Embedded size is measured from the final release artifact produced by the
target's pinned toolchain. The ELF file size on the host is not a flash-usage
number.

Size reports remain measurements, but artificial regression budgets are no
longer enforced. A larger image does not fail a build merely for exceeding a
previous snapshot's budget. Physical Flash/RAM limits, protected partitions,
OTA-slot bounds, stack reserves, and linker/layout checks remain mandatory.

## TLSR8258 tail merging (2026-10-04)

The production and radio-harness Cargo configurations (and, until its removal,
the lab's) explicitly enable
LLVM tail merging (`-enable-tail-merge=true`) with the pinned
`tc32-1.98.1-20261003-31a272` toolchain. This is a separate change from the
compiler-only comparison below; the initial compiler upgrade retained the
old disabled setting.

The macOS x86-64 rebuild with only this flag changed measured:

| image | tail merge off (`19db883`) | tail merge on | additional reduction | physical headroom |
|---|---:|---:|---:|---:|
| default SUSPEND sensor | 203,640 B | 195,688 B | 7,952 B | 263,064 B |
| LOW32K 250 ms proof | 206,904 B | 198,764 B | 8,140 B | 259,988 B |
| LOW32K 10 s proof | 206,908 B | 198,768 B | 8,140 B | 259,984 B |
| parent router | 300,116 B | 288,508 B | 11,608 B | 170,244 B |

The cumulative reduction from the old `tc32-45` builds is **32.4%** for the
default sensor and **33.8%** for the router. Production features, `opt-level=s`,
fat LTO, single codegen unit, linker layout, and all memory checks are unchanged.
Headroom is relative to the `0x70000` application boundary, not an OTA slot.
The LOW32K SVC stack reserve remains 8,448 B.

All four production variants, six lab modes, and the standalone radio harness
passed their local link/layout checks. These are build results, not exact-image
hardware qualification; no device was flashed. Actions builds the same Cargo
configurations, and its firmware cache key includes `.cargo/config.toml` files.

## TLSR8258 compiler comparison (2026-10-04)

The current pinned toolchain is
[`tc32-1.98.1-20261003-31a272`](https://github.com/modern-tc32/rust/releases/tag/tc32-1.98.1-20261003-31a272),
using LLVM 23.1.2. The initial compiler-only upgrade, before enabling tail
merging, used a controlled macOS x86-64 rebuild of the `df6e934`
firmware sources and locked dependencies:

| image | old `tc32-45` | new toolchain | reduction | new physical headroom |
|---|---:|---:|---:|---:|
| default SUSPEND sensor | 289,540 B | 203,640 B | 85,900 B (29.7%) | 255,112 B |
| LOW32K 250 ms proof | 294,444 B | 206,904 B | 87,540 B (29.7%) | 251,848 B |
| LOW32K 10 s proof | 294,448 B | 206,908 B | 87,540 B (29.7%) | 251,844 B |
| parent router | 436,072 B | 300,116 B | 135,956 B (31.2%) | 158,636 B |

These are complete raw binaries, including the unchanged fixed `.text`
placement at `0x8000`. The source, features, linker scripts, `opt-level=s`,
fat LTO, single codegen unit, and disabled tail merge were held constant:
no protocol functionality or memory safeguards were removed for this reduction.
Physical headroom is relative to the unchanged `0x70000` application boundary,
not a new regression budget or a promise of dual-slot OTA support.

All four production variants and the six lab modes plus the standalone radio
harness passed their local link/layout checks. This is build evidence, not
exact-image hardware validation; no device was flashed. The LOW32K SVC stack
reserve remains 8,448 B.

## Local image snapshots

This page is the single source of truth for recorded sizes; `BUILD.md` and the
README link here instead of duplicating tables. CI firmware jobs also upload
`*.size.json` measurements for the exact commit they built.

Baseline snapshot: **2026-09-06**.
The TLSR8258 parent-router, always-on nRF52840, CC2340 fallback,
and ESP32-C6/H2 rows were refreshed
**2026-09-10**. These are named local build snapshots, not the latest remote
CI results; unrefreshed rows do not describe the current working tree.
PHY6222, PHY6252 and PB-03F-Kit occupied-XIP measurements and the BL702 raw
image, and both EFR32 rows, were refreshed **2026-10-31** (local macOS
builds of `7d45e7e`).
The recorded parent router, all four ESP variants and the current EFR32MG1
image exceed the former regression budgets; those former budgets are no longer build
blockers. Both PHY62x2 variants fit their physical XIP slot.
Prior hardware evidence is not an exact-image HIL rerun unless
explicitly stated.

The reference column contains former, unenforced regression budgets except
for PHY6222, PHY6252, PB-03F-Kit and BL702, where it is the mandatory physical
limit (XIP slot, or the BL702 primary-slot application maximum).
The margin is reference minus measured bytes, not physical headroom for the
former-budget rows.

| image | measured | physical limit / former budget | margin vs reference | metric |
|---|---:|---:|---:|---|
| PHY6222 sensor | 135,680 | 187,104 | 51,424 | occupied XIP span |
| PHY6252 feature-selected sensor | 135,744 | 187,104 | 51,360 | occupied XIP span |
| PB-03F-Kit `pb03f-zigbee` (PHY6252) | 193,216 | 200,448 | 7,232 | occupied XIP span |
| BL702 sensor (OTA-capable) | 227,216 | 482,304 | 255,088 | raw binary |
| nRF52840 default / BME280 / SHT31 | 224,472 / 231,792 / 228,216 | 225,280 / 245,760 / 241,664 | 808 / 13,968 / 13,448 | raw binaries |
| nRF52840 always-on End Device | 231,280 | 253,952 | 22,672 | file-backed flash span |
| nRF52840 UF2 ProMicro / MDK / PCA10059 / DK | 222,968 / 222,848 / 224,536 / 224,552 | 237,568 each | 14,600 / 14,720 / 13,032 / 13,016 | linked images before UF2 |
| nRF52833 default / BME280 / SHT31 | 224,464 / 231,784 / 228,208 | 225,280 / 245,760 / 241,664 | 816 / 13,976 / 13,456 | raw binaries |
| EFR32MG1 sensor | 171,792 | 167,936 | -3,856 | raw binary; above former budget |
| EFR32MG21 sensor | 209,028 | 212,992 | 3,964 | raw binary |
| CC2340R5 pinned-SDK sensor (pre-static-task snapshot) | 223,536 | 225,280 | 1,744 | historical raw binary; not rebuilt |
| CC2340R5 fallback sensor | 213,160 | 225,280 | 12,120 | raw binary; radio firmware unavailable |
| ESP32-C6 sensor default / `light-sleep` | 381,056 / 392,080 | 368,640 each | -12,416 / -23,440 | application images; above former budget |
| ESP32-H2 sensor default / `light-sleep` | 365,920 / 376,640 | 356,352 each | -9,568 / -20,288 | application images; above former budget |
| TLSR8258 default / LOW32K 250 ms / LOW32K 10 s | 290,616 / 295,548 / 295,552 | 294,912 / 299,008 / 299,008 | 4,296 / 3,460 / 3,456 | raw binaries |
| TLSR8258 parent router | 436,072 | 430,080 | -5,992 | raw binary; above former budget |

The measurements use:

- `nightly-2026-03-23` for the workspace, nRF, BL702, CC2340, and EFR32;
- `nightly-2026-08-01` for ESP32 and PHY6222;
- `tc32-stage2-tc32-45` for TLSR8258.

These older snapshots use their listed compilers, not the current TC32 pin.
Compiler comparisons require holding source, features, and build flags constant,
as in the controlled TLSR8258 rebuild above.

The PHY62x2 measurements keep `nightly-2026-08-01` and the
existing product feature sets, with target-local identical-code folding and
the `immediate-abort` panic strategy (every panic site is a `udf` trap into
the halting `HardFault` handler instead of a `panic-halt` call with
arguments and `Location` records; this alone saves 3,328 B). They include the
R22/BDB/ZCL conformance and security fixes, which grew the image from
130,752 B to 135,520 B before size work. Both variants link and pass the
layout check locally on macOS. The growth from the October 6 figures
(132,576/132,544 B) to 138,624 B came mostly with the move to `SoftMacCore`
(about +6.2 KB; see "Phase B: PHY62x2" below); the slimmed core brought
them back to 135,680/135,744 B (see "PHY62x2 and TLSR8258 on the slimmed
core").

Additional artifacts and physical limits:

- BL702: overwrite-only loader at `0x2000..0x10000`, primary slot
  `0x10000..0x87000` (application maximum `0x75C00` = 482,304 B after the
  MCUboot header and trailer sector), secondary OTA slot `0x87000..0xFE000`;
  CI rejects a larger raw application.
- EFR32MG1's prior Zigbee OTA container was 162,538 B; it has not been
  regenerated for the current image. Resident Gecko bootloader assumed.
- EFR32MG21: no OTA packaging path.
- CC2340R5 physical application slot: 516,096 B.
- ESP32-C6 default/light merged flash: 446,592/457,616 B; H2: 431,456/442,176 B.
  OTA slot: 2,031,616 B each. Their regenerated version-1 Zigbee OTA containers
  are 381,122/392,146 B for C6 and 365,986/376,706 B for H2.
  These combined-fix images have host validation, not hardware execution
  evidence. Exceeding their former regression budgets no longer blocks builds;
  the physical OTA-slot and other validation checks still apply.
- The PHY6252 EVK image links within its physical XIP slot; its hardware
  path remains unverified. Only the PB-03F-Kit (PHY6252) has hardware
  evidence.
- TLSR8258 physical application boundary: 458,752 B (`0x70000`), followed by
  APS/child/security journals at `0x70000`/`0x72000`/`0x74000`.

The TLSR8258 router's former 430,080 B (`0x69000`) regression budget replaced
the pre-R22 356,352 B baseline and was set above the earlier 427,776 B image.
The recorded 436,072 B image exceeded it by 5,992 B and failed that historical
gate. That artificial threshold is no longer enforced. The mandatory physical
458,752 B (`0x70000`) boundary and journals did not move: this snapshot has
22,680 B of physical headroom. The 28,672 B gap between the former budget and
the physical boundary was not a reserved partition. The existing TC32
toolchain linked the persistent-APS composition with its physical memory
assertions; removing the regression gate does not establish hardware acceptance.

## RAM and stack snapshot

| image | `.data` | `.bss` | static total | stack |
|---|---:|---:|---:|---:|
| PHY6222 / PHY6252 | 1,216 | 16,572 | 17,788 | 41,540 available |
| PB-03F-Kit `pb03f-zigbee` | 1,304 | 16,628 | 17,932 | 41,396 available |
| BL702 sensor | 592 | 16,592 | 17,184 | 15,360 reserved |
| EFR32MG1 | 288 | 15,056 | 15,344 | 16,400 available |
| EFR32MG21 | 316 | 18,888 | 19,204 | 46,328 available |
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
EFR32MG1 has exactly `0x7C00` bytes of usable SRAM and only 16 B of margin
above its 16 KiB stack gate. TLSR8258's retained SVC stack has 256 B above its 8 KiB
gate. These linked values are not runtime high-water measurements.

## Measurement versus physical limits

Every production build must retain:

- **size measurement** — reports the final artifact's bytes for comparison;
- **physical boundary checks** — prevent overlap with bootloader, security,
  child-table, factory, or OTA regions and preserve usable RAM and stack reserves.

There is no arbitrary regression-budget failure. A size report is not proof of
physical fit: product linker assertions, layout checks, and image/OTA
validation remain independent, mandatory checks.

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

## MAC unification footprint baseline

Phase A of the MAC unification (all MAC logic in the shared `SoftMacCore`,
backends reduced to `RadioPhy`) was measured against commit `44d457c`, the
last commit before the core API changed. Every image that CI builds was
rebuilt with its CI toolchain, features and `--locked`, and then measured
with `tools/mac-footprint.py`:

```bash
python3 tools/mac-footprint.py build --keep-going --out target/mac-footprint/now
python3 tools/mac-footprint.py report --out target/mac-footprint/now \
    --json target/mac-footprint/now.json
python3 tools/mac-footprint.py compare tools/mac-footprint-baseline.json \
    target/mac-footprint/now.json
```

`tools/mac-footprint-baseline.json` is the post-Phase-A reference: the
"after" rows below. It is the gate for Phase B backend migrations.
`compare` exits non-zero if `flash`, `data`, `bss`, `pool`, `stack` or
`mac_chain` grows for any image. The other columns are informational, because
they move whenever the inliner shifts code between the MAC and the runtime.

Column meanings (the script's `--help` gives the full definitions):

- `flash`: file-backed allocated bytes.
- `pool`: the largest Embassy task pool or arena static, which holds the MAC
  futures.
- `stack`: the static worst-case direct-call stack.
- `mac_chain`: the deepest direct-call chain that executes `zigbee_mac::`
  code.
- `mac_stack`: the MAC call tree measured from MAC roots alone.
- `mac_frame`: the largest single MAC frame.

All stack figures are derived from function prologues in the linked ELF.
They are static bounds, not runtime high-water marks.

Frames too large for an immediate are allocated through the literal pool on
Thumb-1 (Cortex-M0+: CC2340, PHY62x2) and TC32 (TLSR8258):

```text
ldr  rN, [pc, #..]   @ .word 0xffffd044
add  sp, rN
```

The tool originally counted these frames as 0 B. It now reads the literal
and adds the frame; a non-negative literal marks the frame dynamic. Thumb-2
(nRF52, EFR32) and RISC-V (BL702, ESP32) use immediates, so their rows are
unaffected.

The corrected rows in `tools/mac-footprint-baseline.json` were raised by the
under-count. The `stack` and `mac_chain` values in the `44d457c` table below
are the old, under-counted values. Corrected values (bytes):

| image | `stack` | `mac_chain` | `mac_stack` | `mac_frame` |
|---|---:|---:|---:|---:|
| cc2340-sensor-fallback | 24,072 | 23,576 | 2,456 | 1,024 |
| pb03f-zigbee | 24,256 | 22,944 | 2,152 | 728 |
| phy6222-sensor | 24,312 | 22,912 | 1,984 | 728 |
| phy6252-sensor | 24,312 | 22,912 | 1,984 | 728 |
| tlsr8258-router | 47,424 | 37,376 | 2,992 | 2,192 |
| tlsr8258-sensor | 30,632 | 30,096 | 2,968 | 2,192 |
| tlsr8258-sensor-retention | 31,864 | 31,328 | 2,968 | 2,192 |
| tlsr8258-sensor-retention-10s | 31,864 | 31,328 | 2,968 | 2,192 |

The corrected TLSR8258 bounds exceed the linked SVC stacks (16 KiB for the
default image, 8,448 B for the retained fresh root). The deepest paths are in
persistence, not the MAC:

- router: `app::run` (12,240 B frame) → `activate_network` →
  `ApsTableJournal::store` → `ApsTableJournal::current` (13,136 B frame);
- retained sensor: `cold_run` → `record_failed_rejoin` →
  `SecurityStateJournal::store` → `ensure_replay_cache`.

These are static direct-call bounds and include paths that may not nest at
run time. They still need review against runtime stack high-water
measurements on hardware. The CC2340 bound (24,072 B) fits its 29,244 B
stack region, and PHY62x2 fits its 54,384 B.

The CC2340 image uses
the fallback (no SDK) mode, because SDK mode cannot be built locally. The
PHY62x2 images were measured from the tree at `44d457c`.

Baseline (`44d457c`, bytes):

| image | `flash` | `data` | `bss` | `pool` | `stack` | `mac_chain` | `mac_stack` | `mac_frame` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| bl702-sensor | 228,782 | 0 | 16,592 | 16,388 | 25,168 | 24,304 | 1,872 | 1,008 |
| cc2340-sensor-fallback | 223,000 | 8 | 7,612 | 5,328 | 6,724 | 6,620 | 2,456 | 1,024 |
| efr32mg1-sensor | 171,796 | 0 | 15,056 | 6,928 | 27,892 | 26,156 | 1,332 | 584 |
| efr32mg21-sensor | 208,832 | 0 | 18,888 | 11,040 | 25,524 | 23,612 | 1,732 | 1,072 |
| esp32c6-sensor | 392,892 | 3,592 | 53,476 | 0 | 30,288 | 25,456 | 1,920 | 576 |
| esp32c6-sensor-light-sleep | 403,972 | 3,784 | 53,476 | 0 | 30,544 | 25,744 | 1,952 | 576 |
| esp32h2-sensor | 377,574 | 3,296 | 53,420 | 0 | 30,288 | 25,120 | 1,376 | 576 |
| esp32h2-sensor-light-sleep | 388,246 | 3,544 | 53,420 | 0 | 30,544 | 25,376 | 1,408 | 576 |
| nrf52833-sensor | 234,736 | 88 | 37,376 | 32,772 | 30,036 | 29,988 | 1,100 | 752 |
| nrf52833-sensor-bme280 | 243,200 | 88 | 37,376 | 32,772 | 30,316 | 30,268 | 1,100 | 752 |
| nrf52833-sensor-sht31 | 238,872 | 88 | 37,376 | 32,772 | 30,036 | 29,988 | 1,100 | 752 |
| nrf52840-router | 236,780 | 88 | 19,116 | 18,056 | 28,428 | 26,628 | 4,276 | 4,056 |
| nrf52840-sensor | 234,736 | 88 | 37,376 | 32,772 | 30,036 | 29,988 | 1,100 | 752 |
| nrf52840-sensor-bme280 | 243,200 | 88 | 37,376 | 32,772 | 30,316 | 30,268 | 1,100 | 752 |
| nrf52840-sensor-sht31 | 238,860 | 88 | 37,376 | 32,772 | 30,036 | 29,988 | 1,100 | 752 |
| nrf52840-sensor-uf2-dk | 234,916 | 88 | 37,376 | 32,772 | 30,028 | 29,980 | 1,100 | 752 |
| nrf52840-sensor-uf2-mdk | 233,396 | 88 | 37,376 | 32,772 | 30,012 | 29,964 | 1,100 | 752 |
| nrf52840-sensor-uf2-pca10059 | 234,872 | 88 | 37,376 | 32,772 | 30,028 | 29,980 | 1,100 | 752 |
| nrf52840-sensor-uf2-promicro | 233,556 | 88 | 37,376 | 32,772 | 30,012 | 29,964 | 1,100 | 752 |
| pb03f-zigbee | 190,840 | 0 | 16,628 | 16,388 | 9,516 | 8,508 | 2,152 | 728 |
| phy6222-sensor | 133,464 | 0 | 16,572 | 16,388 | 9,300 | 8,300 | 1,984 | 728 |
| phy6252-sensor | 133,448 | 0 | 16,572 | 16,388 | 9,300 | 8,300 | 1,984 | 728 |
| tlsr8258-router | 270,794 | 2,428 | 19,684 | 0 | 6,488 | 6,488 | 2,992 | 1,464 |
| tlsr8258-sensor | 169,287 | 2,156 | 9,816 | 0 | 9,244 | 9,244 | 2,968 | 752 |
| tlsr8258-sensor-retention | 172,495 | 2,156 | 12,284 | 0 | 9,220 | 9,220 | 2,968 | 752 |
| tlsr8258-sensor-retention-10s | 172,499 | 2,156 | 12,284 | 0 | 9,220 | 9,220 | 2,968 | 752 |

After the nRF migration to `SoftMacCore<NrfRadioPhy>`, the deletion of the
legacy `NrfMac` and `Bl702Mac`, and the addition of the offload and parent
hooks, only these images changed. Every other image is byte-identical in
these metrics.

| image | Δ `flash` | Δ `data` | Δ `bss` | Δ `pool` | Δ `stack` | Δ `mac_chain` |
|---|---:|---:|---:|---:|---:|---:|
| bl702-sensor | -1,540 | 0 | 0 | 0 | -496 | -496 |
| nrf52833-sensor | -556 | 0 | 0 | 0 | -2,720 | -5,236 |
| nrf52833-sensor-bme280 | -636 | 0 | 0 | 0 | -2,720 | -5,236 |
| nrf52833-sensor-sht31 | -468 | 0 | 0 | 0 | -2,720 | -5,236 |
| nrf52840-router | -636 | 0 | -256 | -256 | -16 | -4,620 |
| nrf52840-sensor | -552 | 0 | 0 | 0 | -2,720 | -5,236 |
| nrf52840-sensor-bme280 | -576 | 0 | 0 | 0 | -2,704 | -5,220 |
| nrf52840-sensor-sht31 | -352 | 0 | 0 | 0 | -2,712 | -5,228 |
| nrf52840-sensor-uf2-dk | -668 | 0 | 0 | 0 | -2,720 | -5,236 |
| nrf52840-sensor-uf2-mdk | -624 | 0 | 0 | 0 | -2,704 | -5,220 |
| nrf52840-sensor-uf2-pca10059 | -672 | 0 | 0 | 0 | -2,720 | -5,236 |
| nrf52840-sensor-uf2-promicro | -724 | 0 | 0 | 0 | -2,704 | -5,220 |

The nRF sensor `mac_stack` rose by 884 B and `mac_code` by about 3.8 KB, while
`flash` and `mac_chain` fell. Two effects explain this:

- `mlme_poll` is no longer inlined into its runtime caller, so it now counts
  as a MAC root, even though the real MAC call chain shrank by 5.2 KB.
- The removed bytes were legacy `log::info!` format strings and their
  formatting code.

The nRF main-task future, measured with `-Z print-type-sizes`, shrank from
18,648 B to 18,632 B on the sensor and from 18,024 B to 17,768 B on the router.

### Core slimming before Phase B

Before the remaining backends were ported, `SoftMacCore` itself was slimmed
so that a port does not have to pay for the core's generality. The changes:

- The transmit path is selected by the `RadioPhy::Tx` marker type, so each
  future has the exact size of the one path that is compiled.
- One synchronous-classifying `listen` loop serves every receive window.
  No future holds a received frame across an await.
- The association, leave and start paths write PIB fields directly instead
  of going through the generic `MacPib::set`.
- By-value requests are captured once (`fn -> impl Future { async move }`),
  not twice as with `async fn`.
- Retained frames use a byte-indexed ring instead of `heapless::Deque`.
- The transmit and CSMA paths use the same `async move` form.
- `RadioPhy::receive` writes straight into the next retained-RX slot. No
  receive future carries its own 132-byte frame buffer.

Measured against `157d189`, both trees built in directories with
equal-length paths (bytes):

| image | Δ `flash` | Δ `data` | Δ `bss` | Δ `pool` | Δ `stack` | Δ `mac_chain` | Δ `mac_code` | Δ `mac_frame` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| bl702-sensor | -6,400 | 0 | 0 | 0 | -3,920 | -4,704 | -4,308 | -416 |
| nrf52833-sensor | -7,068 | 0 | 0 | 0 | -64 | -1,764 | -4,270 | -384 |
| nrf52833-sensor-bme280 | -7,288 | 0 | 0 | 0 | -56 | -1,756 | -4,270 | -384 |
| nrf52833-sensor-sht31 | -7,032 | 0 | 0 | 0 | -64 | -1,764 | -4,270 | -384 |
| nrf52840-router | -7,680 | 0 | -216 | -216 | -64 | -2,560 | -4,704 | -744 |
| nrf52840-sensor | -7,072 | 0 | 0 | 0 | -64 | -1,764 | -4,270 | -384 |
| nrf52840-sensor-bme280 | -7,332 | 0 | 0 | 0 | -72 | -1,772 | -4,270 | -384 |
| nrf52840-sensor-sht31 | -7,132 | 0 | 0 | 0 | -72 | -1,772 | -4,270 | -384 |
| nrf52840-sensor-uf2-dk | -7,068 | 0 | 0 | 0 | -64 | -1,764 | -4,270 | -384 |
| nrf52840-sensor-uf2-mdk | -7,028 | 0 | 0 | 0 | -64 | -1,764 | -4,270 | -384 |
| nrf52840-sensor-uf2-pca10059 | -7,064 | 0 | 0 | 0 | -64 | -1,764 | -4,270 | -384 |
| nrf52840-sensor-uf2-promicro | -7,024 | 0 | 0 | 0 | -64 | -1,764 | -4,270 | -384 |

The images that do not use the core (CC2340, EFR32, ESP32, PHY62x2,
TLSR8258) have no source change in their backends. Built in the same
directory, PHY62x2 and TLSR8258 are byte-identical in every metric.

The others move by LTO codegen noise inside the shared `zigbee-mac` crate:

- `cc2340-sensor-fallback`: `flash` +52 and `mac_code` +28
  (`Cc2340Driver::transmit_frame` and the grouping of `Debug` impls).
- `efr32mg1-sensor`: `flash` +4 and `mac_chain` +40, but not the
  worst-case `stack`. In `nlme_network_discovery`, the instructions are
  identical; only the stack slots are packed differently.
- `efr32mg21-sensor`: `flash` +4.
- The four ESP32 images: `flash` -122.

The noise disappears when those backends move onto the core.

The nRF `stack` worst case is the main task's start-up chain
(`DeviceBuilder::assemble`). That chain runs no MAC code, so the nRF MAC
savings show up in `mac_chain`. MAC future and struct sizes on nRF52840
(`-Z print-type-sizes`, bytes):

| future | `157d189` | slim |
|---|---:|---:|
| `mlme_scan` | 1,000 | 992 |
| `mcps_data` | 608 | 416 |
| `mlme_associate` | 592 | 344 |
| `mlme_disassociate` | 472 | 320 |
| `mlme_poll` / `mlme_poll_timeout` | 440 / 464 | 344 |
| `mcps_data_indication` | 352 | 272 |
| `SoftMacCore` struct (`RX` = 8) | 1,252 | 1,240 |

Each of these futures is 8 B larger than in the first slim measurement.
`mac_chain`, which includes the PHY receive future, still fell after
receive-into-slot: by 800 B on BL702, whose receive future no longer
holds a frame of its own, and by 524 B on the nRF router. On the backends
ported temporarily for measurement (EFR32MG1/MG21, TLSR8258 router),
`mac_chain` fell by a further 376–608 B. nRF sensor `flash` rose by about
100 B against the previous slim state; it is still about 7 KB below
`157d189`.

`tools/mac-footprint-baseline.json` rows for these images were lowered by
the measured deltas.

Path dependence: building the same source in a directory with a different
path changes `flash` by roughly 100–250 B (panic location strings). The
codegen noise from crate hashes adds a few tens of bytes, and up to about
200 B. Always measure the parent commit in the same directory, or one with
a path of equal length, before applying the zero-growth rule.

### Phase B: PHY62x2 (approved flash exception)

`zigbee-mac/src/phy6222` now runs on `SoftMacCore<Phy6222RadioPhy>`. Its own
scan, association, poll, CSMA-CA and retry code is gone; the driver and its
SRAM interrupt ACK remain. Deltas against `157d189`, measured in the same
tree:

| image | Δ `flash` | Δ `data` | Δ `bss` | Δ `pool` | Δ `stack` | Δ `mac_chain` |
|---|---:|---:|---:|---:|---:|---:|
| pb03f-zigbee | +6,388 | 0 | 0 | 0 | -4,372 | -4,348 |
| phy6222-sensor | +6,156 | 0 | 0 | 0 | -4,380 | -4,116 |
| phy6252-sensor | +6,176 | 0 | 0 | 0 | -4,380 | -4,116 |

The flash growth breaks the "no growth" gate. It was approved for PHY62x2
only, because the shared core's general paths (`collect_beacons`,
`MacPib::set`, association, `mlme_poll` with the two-pass short/extended
poll) replace the smaller, PHY-specific legacy MAC. In exchange, the static
stack bound falls by about 4.3 KB. `tools/mac-footprint-baseline.json`
carries the new PHY62x2 rows; every other row is unchanged. `pb03f-zigbee`
(196,320 B of XIP) no longer fit the 196,352 B slot with margin, so the
PB-03F slot grew by one 4 KiB sector, to `0x10100..0x40FFF` (200,448 B); the
reserved OTA staging area shrank to `0x41000..0x7DFFF` (244 KiB, still larger
than the slot). The EVK images use 138,624 B of their unchanged 187,104 B
XIP span.

### Phase B: TLSR8258 sensors

The three TLSR8258 sensor images moved to
`SoftMacCore<TelinkRadioPhy, EndDevice>` with full TX offload (now `type Tx = FullTx`): the HAL runs
CSMA-CA with the PIB parameters, the transmission and the Imm-Ack wait in one
radio operation, and the turnaround-critical ACK stays in the HAL
RX-completion path. The router stays on the legacy `TelinkRouterMac`
(`telink-legacy-router` feature), which the sensors no longer link. Measured
against `157d189`:

| image | `flash` | `data` | `bss` | `pool` | `stack` | `mac_chain` | `mac_stack` | `mac_frame` |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| tlsr8258-router | 270,790 (-4) | 2,428 (0) | 19,684 (0) | 0 | 6,488 (0) | 6,488 (0) | 2,992 (0) | 1,464 (0) |
| tlsr8258-sensor | 167,671 (-1,616) | 2,156 (0) | 8,384 (-1,432) | 0 | 4,680 (-4,564) | 4,088 (-5,156) | 1,832 (-1,136) | 896 (+144) |
| tlsr8258-sensor-retention | 171,015 (-1,480) | 2,156 (0) | 10,852 (-1,432) | 0 | 4,656 (-4,564) | 4,064 (-5,156) | 1,832 (-1,136) | 896 (+144) |
| tlsr8258-sensor-retention-10s | 171,019 (-1,480) | 2,156 (0) | 10,852 (-1,432) | 0 | 4,656 (-4,564) | 4,064 (-5,156) | 1,832 (-1,136) | 896 (+144) |

The retained LOW32K SVC stack reservation is unchanged at 8,448 B; the static
worst-case stack under it fell by 4,564 B. The `.bss` saving is the legacy
MAC's data-indication, command-event and association queues, which the
end-device core does not need.

A full router port onto `SoftMacCore<TelinkRadioPhy, Parent<8, 8, 1>>` was
measured at 272,340 B (+1,546) and rejected by the gate. The added
`RadioPhy::set_rx_on_when_idle` hook is zero-cost elsewhere: `bl702-sensor`
is byte-identical, and `nrf52840-sensor` and `nrf52840-router` differ only
in 20 bytes of panic-location line numbers.

The `stack`, `mac_chain`, `mac_stack` and `mac_frame` columns of the two
Phase B sections above come from the tool before it counted literal-pool
stack frames (see "Core slimming before Phase B"). They are not comparable with
the corrected rows below or in `tools/mac-footprint-baseline.json`.

### PHY62x2 and TLSR8258 on the slimmed core

PHY62x2 (`type Tx = AckWaitTx`; the SRAM interrupt Imm-Ack path is
unchanged) and the TLSR8258 sensors (`type Tx = FullTx`) moved to the
slimmed `RadioPhy` API with in-place `receive`. The PHY62x2 composition
roots now build the MAC and the device in a non-inlined `build_device`
function. In `async main`, the `SoftMacCore` value, its `Result` and the
builder were temporaries in the Embassy main task's poll frame, which stays
live under the whole stack chain (2,512 B; 1,504 B with the helper).

Deltas against master `28b1803`, both measured with the corrected tool in
directories with equal-length paths:

| image | `flash` | `data` | `bss` | `pool` | `stack` | `mac_chain` | `mac_code` |
|---|---:|---:|---:|---:|---:|---:|---:|
| pb03f-zigbee | 194,244 (-3,136) | 0 (0) | 16,628 (0) | 16,388 (0) | 24,152 (-2,904) | 22,224 (-3,720) | 10,174 (-2,410) |
| phy6222-sensor | 136,688 (-2,940) | 0 (0) | 16,572 (0) | 16,388 (0) | 24,072 (-2,888) | 22,192 (-3,824) | 9,938 (-2,442) |
| phy6252-sensor | 136,776 (-2,852) | 0 (0) | 16,572 (0) | 16,388 (0) | 24,072 (-2,896) | 22,192 (-3,832) | 9,938 (-2,442) |
| tlsr8258-router | 270,790 (0) | 2,428 (0) | 19,684 (0) | 0 (0) | 47,424 (0) | 37,376 (0) | 18,016 (0) |
| tlsr8258-sensor | 161,355 (-6,316) | 2,156 (0) | 8,368 (-16) | 0 (0) | 30,360 (-3,696) | 29,424 (-4,184) | 9,560 (-3,136) |
| tlsr8258-sensor-retention | 164,579 (-6,436) | 2,156 (0) | 10,836 (-16) | 0 (0) | 29,768 (-3,696) | 28,824 (-4,192) | 9,528 (-3,128) |
| tlsr8258-sensor-retention-10s | 164,583 (-6,436) | 2,156 (0) | 10,836 (-16) | 0 (0) | 29,776 (-3,688) | 28,824 (-4,192) | 9,528 (-3,128) |

Against the legacy, hand-written MACs (`157d189` rows, corrected tool),
master's ports had grown `stack`/`mac_chain` by +2.6–2.8 KB/+3.0–3.1 KB
(PHY62x2) and +1.6–3.4 KB/+1.7–3.5 KB (TLSR8258 sensors). With the slimmed core
every one of these images is now below the legacy MAC on both:

| image | Δ `flash` | Δ `bss` | Δ `stack` | Δ `mac_chain` |
|---|---:|---:|---:|---:|
| pb03f-zigbee | +3,404 | 0 | -104 | -720 |
| phy6222-sensor | +3,224 | 0 | -240 | -720 |
| phy6252-sensor | +3,328 | 0 | -240 | -720 |
| tlsr8258-sensor | -7,932 | -1,448 | -272 | -672 |
| tlsr8258-sensor-retention | -7,916 | -1,448 | -2,096 | -2,504 |
| tlsr8258-sensor-retention-10s | -7,916 | -1,448 | -2,088 | -2,504 |

The remaining PHY62x2 flash growth is about half of the approved
+6.2–6.5 KB exception. Occupied XIP spans: PHY6222 135,680 B, PHY6252
135,744 B (of 187,104 B), `pb03f-zigbee` 193,216 B (of 200,448 B). The
TLSR8258 raw binaries are 190,284 B (default), 193,404 B (LOW32K 250 ms) and
193,408 B (LOW32K 10 s); the retention image still links one
`tlsr8258_rt::block_on` monomorph. `tools/mac-footprint-baseline.json`
carries these PHY62x2/TLSR8258 rows. The new-API paths are build- and
layout-checked only; the PB-03F hardware evidence predates them.

## Measurement commands

Raw binary:

```bash
OBJCOPY=$(find "$(rustc --print sysroot)" -name llvm-objcopy -print -quit)
"$OBJCOPY" -O binary path/to/firmware path/to/firmware.bin
stat -f '%z' path/to/firmware.bin   # macOS
```

Machine-readable artifact measurement, from the repository root:

```bash
./tools/firmware-size-report.sh sensor path/to/firmware.bin path/to/firmware.size.json
```

The three arguments are image name, binary, and output JSON. The report
contains measurements without regression-budget, remaining-budget, or
exceeded-budget fields; it does not enforce an artificial growth threshold.

Static sections:

```bash
SIZE=$(find "$(rustc --print sysroot)" -name llvm-size -print -quit)
"$SIZE" path/to/firmware
```

Use the platform packager instead of raw `objcopy` for ESP, BL702, PHY62x2,
and UF2 deployment formats. Exact commands and physical checks are in
[`BUILD.md`](https://github.com/faronov/zigbee-rs/blob/master/BUILD.md)
and `.github/workflows/ci.yml`.
