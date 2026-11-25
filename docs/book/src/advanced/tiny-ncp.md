# Tiny NCP tier for 64 KiB radio parts (feasibility study)

**Question.** Can the zigbee-rs ZRHP network co-processor
(`apps/ncp-coordinator`, [host protocol](host-protocol.md)) run on the
cheap 64 KiB-RAM Zigbee modules of ESP32-S3 wall panels, so that the
panel's ESP32-S3 runs zigbee-rs-hub as the host?

**Answer (measured, 2026-11-25): not with the current architecture.** At
the smallest table sizes that still make a useful coordinator, the NCP's
*static* RAM alone exceeds 64 KiB on both candidate chips, before any
stack:

| chip | static RAM needed | SRAM | short by (static only) |
|---|---:|---:|---:|
| EFR32MG21 F768 (Tuya ZS3L), 48 TC devices + GP | 78,748 B | 65,532 B usable | 13,412 B (linker) |
| EFR32MG21, 32 TC devices, every parameter at its floor, GP | 69,108 B | 65,532 B | ~3,600 B |
| TLSR8258 (Tuya ZT3L), 48 TC devices + GP | 78,564 B (incl. 5,632 B RAM-code/I-cache reservation) | 65,536 B | 13,028 B |

The steady-state stack needs another ~20 KiB (TC32 static analysis that
resolves `dyn Future` polls; see [Stack](#stack)), so the real gap is
**~25–40 KiB**. The tables that scale with the network (Trust Center
devices, neighbours, address map, routes) are a small part of it:
everything that makes the NCP an NCP (link window, endpoint queues, UART
rings, Trust Center runtime, host passthrough) plus the stack layers is
fixed cost.

Recommendation: keep 64 KiB parts out of the ZRHP NCP line-up. For a
ZS3L panel, run an EmberZNet NCP on the module and use the hub's EZSP
backend (`zigbee-rs-hub/docs/ezsp-backend.md`, running on a SONOFF Dongle
Max); for a ZT3L (TLSR8258) panel there is no zigbee-rs coordinator path.
Details, the reduction roadmap and the update paths follow.

Validation level: **build and static measurement only.** Nothing in this
chapter ran on hardware; no module was available.

## Target hardware

Tuya T3E / Golden Security GS-T3E panels: ESP32-S3-WROOM-1 (16 MB flash,
8 MB octal PSRAM), 480×480 touch LCD, relays, and a Tuya Zigbee module on a
UART. Two module variants are reported:

| module | part | flash / RAM | host UART (module side) | first-flash / recovery |
|---|---|---|---|---|
| ZS3L | EFR32MG21A020F768IM32 (+20 dBm) | 768 KiB / 64 KiB | Tuya gateway route PD00 TX, PD01 RX, PA03 CTS, PA04 RTS (115 200, flow control on in Tuya firmware); or the TXD/RXD pads PA05/PA06 | SWD pads (DIO/CLK, "burning" pins); Gecko bootloader with PC01-low-at-reset entry (Tuya gateway datasheet) |
| ZT3L | TLSR8258F1KET32 | 1 MiB / 64 KiB | PB1 TX, PB7 RX (Tuya datasheet); ESPHome: ZT3L TX → S3 GPIO19, RX ← S3 GPIO20, 115 200 | SWS pad only (no ROM UART bootloader) |

Which ZS3L route a given panel wires, and its reset/boot lines (ESPHome
mentions GPIO4 for reset, unverified), are unknown. The SONOFF Dongle-E
uses the same EFR32MG21A020F768IM32. Note that the ZBT-1 is **not** a
64 KiB part: it is an EFR32MG21A020F1024IM32 with 96 KiB of RAM
(`products/ha-connect-zbt1/link/memory.x`, DEVINFO `MSIZE` on the lab
unit). No further market research was done.

## What was built

### `ncp-capacity-tiny` (cargo feature, all tiers' images unchanged)

One opt-in feature per crate, beside `ncp-capacity-compact`:
`zigbee-nwk`, `zigbee-aps`, `zigbee-bdb`, `zigbee-runtime`,
`router-app` (`ncp-capacity-tiny`) and `ncp-coordinator-app` (`tiny`).
In the stack crates `compact` wins when both are enabled (the CI
`--all-features` runs keep testing `compact`); `ncp-coordinator-app`
refuses both with a compile error. Journal formats are those of
`ncp-capacity` except the child journal, which keeps the router's 512 B
slots.

| table | router | `ncp-capacity` (H2/C6, MG24) | `compact` (CC2652P7) | **`tiny`** |
|---|---:|---:|---:|---:|
| Trust Center devices / APS link keys | 32 / 32 | 256 / 256 | 128 / 128 | **48 / 48** |
| address map | 32 | 512 | 256 | **64** |
| neighbors / NIB `max_children` | 32 / 20 | 96 / 64 | 64 / 32 | **32 / 16** |
| source routes | 32 | 128 | 64 | **32** |
| NWK replay floors | 64 | 128 | 96 | **64** |
| indirect queue / poll estimates | 8 / 16 | 16 / 64 | 16 / 64 | **8 / 16** |
| persisted children / child slot | 32 / 512 B | 64 / 1 KiB | 64 / 1 KiB | **32 / 512 B** |
| GP Sink Table | 8 | 16 | 16 | **8** |
| TC removal intents | 32 | 32 | 32 | **8** |
| ZRHP link window (each way) / endpoint queue | — | 16 / 1 536 B | 8 / 1 024 B | **4 (spec minimum) / 1 024 B** |
| `max_aps_inflight` / retry cache | — | 16 / 64 | 16 / 64 | **8 / 16** |
| restore staging (keys / children) | — | 64 / 64 | 64 / 64 | **48 / 32** |

The negotiated capacities (`SYS_NEGOTIATE`) report the tiny sizes. The
largest tiny Trust Center snapshot (3 909 B) fits one 4 KiB chunk slot of
`trust_center_stream`, so the tiny tier never streams more than one chunk;
the security snapshot (48 keys, 64 floors) fits an 8 KiB logical sector
(`SecurityStateJournal::NCP_SECTOR_FITS`).

Host tests: `zigbee-nwk`, `zigbee-aps`, `zigbee-bdb` and `zigbee-runtime`
with `ncp-capacity-tiny` (table capacities, a 48-entry key table and TC
table that refuse the 49th, a one-chunk TC snapshot that migrates from the
single-slot record and falls back on a torn commit), and
`ncp-coordinator-app --features tiny` (all integration tests, negotiated
capacities). Byte-identical images without the feature (rebuilt in one
worktree before and after, `cmp` of the loadable bytes): MG21 router and
router-core, MG24 NCP, CC2652P7 NCP, ESP32-H2 Thread-BR NCP, TLSR8258
router and router-core.

### EFR32MG21 NCP composition (build-only)

- `boards/tuya-zs3l`: the module's pins (both UART routes by feature, the
  flow-control pins and PC01 claimed), HFXO, typed resources. Host-tested.
- `products/efr32mg21-ncp`: identity (`ZS3L-NCP`, ZRHP chip 4), the
  memory map for F768 and F512 (Gecko bootloader `0x0..0x4000` kept, 96 KiB
  of NV journals at the top: 6 × (2 × 8 KiB)), journals and opening policy,
  UART policy (460 800 default; 115 200 / 921 600), Gecko
  `ApplicationProperties`, fatal-fault policy. Host-tested.
- `examples/efr32mg21-ncp`: the composition root (the EFR32MG24 NCP's).
  **It does not link with the real map**: `.bss` overflows the 64 KiB RAM
  by 13,412 B (F768), and on F512 the image also overflows the 400 KiB
  application window by 41,152 B. `EFR32MG21_NCP_MEASURE=1` links a
  measurement-only map (256 KiB of "RAM"); the image carries
  `_ZRS_MEASUREMENT_ONLY_IMAGE` and `tools/verify-layout.py` rejects it.
- `efr32s2-hal`: `se-trng` (opt-in) builds the Secure Engine mailbox TRNG
  on xG21 (same `SEMAILBOX_HOST` interface and command as xG24; xG21 has no
  bus-clock gate); `usart0-host-small` selects 4 KiB / 1 KiB host-link
  rings. `zigbee-mac`: `efr32mg21-se-trng` makes `fill_random` use it.
  Without them xG21 has **no entropy source** (`fill_random` returns
  `Unsupported`), so a coordinator could neither form with its own keys
  nor generate unique TCLKs; with an always-failing entropy source the
  optimiser would also have removed key-generation code and the
  measurement would have been too small. Not run on xG21 silicon.

### TLSR8258 measurement composition (not committed)

The tiny NCP composed for TC32 in a scratch crate outside the repository
(the router's `TelinkRouterMac` with `pan-coordinator` and hardware AES,
the TC + GP combo, NCP-tier journals on `FlashRegion`, a critical-section
implementation over `REG_IRQ_EN`, the HAL's polled non-DMA UART on PB1/PB7
with a 4 KiB software ring, `tlsr8258_rt::block_on_pinned`), built with the
repository's TC32 toolchain and `opt-level = "s"`, fat LTO, linked against
the TB-04 map with an oversized RAM region. It is a measurement, not a
product: the transport is not interrupt- or DMA-driven.

Building it found one portability bug, fixed here: `gp_remote`'s ZCL TSN
used `AtomicU8::fetch_add`, which TC32 (and Cortex-M0+) do not have; it now
goes through `increment_u8` (a critical section on targets without atomic
read-modify-write; the same instruction sequence elsewhere).

## Measurements

Release builds, repository toolchains (`nightly-2026-03-23` for ARM,
`tc32-1.98.1-20261003-31a272`), measured 2026-11-25 on branch
`ncp-tiny-tier` (base `62ab4b44`). "Flash" is the file-backed image
(`tools/mac-footprint.py`; TLSR8258: the `.bin`, which includes the
~28 KiB below `.text` at 0x8000). "Bound" is the static worst-case stack
over direct calls (`mac-footprint.py`, which does not follow indirect
`dyn Future` polls) or, for TC32, `tools/tlsr8258-stack-report.py` (which
resolves them).

| image | flash | `.data` | `.bss` | static RAM | linked stack | stack bound |
|---|---:|---:|---:|---:|---:|---:|
| MG21 router + GP (BRD4181A, 64 KiB) — baseline | 387,460 | 472 | 34,064 | 34,536 | 30,960 | 20,236 |
| MG21 router-core — baseline | 356,500 | 472 | 32,016 | 32,488 | 33,008 | 20,180 |
| **MG21 tiny NCP, 48 TC devices, GP** (F768) | **450,744** | 1,336 | 77,412 | **78,748** | **none: link fails, −13,412** | 33,132 (boot) |
| MG21 tiny NCP, 32 TC devices, GP | 450,824 | 1,336 | 73,316 | 74,652 | link fails | 31,340 |
| MG21 tiny NCP, parameter floor¹, GP | 450,384 | 912 | 68,196 | 69,108 | link fails | 31,340 |
| MG21 tiny NCP on F512 (400 KiB window) | overflows by 41,152 | | | | | |
| MG24 NCP, `ncp-capacity` — baseline | 452,904 | 3,888 | 168,536 | 172,424 | 89,512 | 71,744 |
| CC2652P7 NCP, `compact` — baseline | 451,416 | 5,176 | 116,012 | 121,188 | 26,268 | 50,512 ² |
| TLSR8258 router + GP (TB-04) — baseline | 320,872 | 336 | 32,540 | 33,452 ³ | 20,480 (declared) | gate passes |
| TLSR8258 router-core — baseline | 293,692 | 336 | 30,180 | 31,092 ³ | 20,480 | gate passes |
| **TLSR8258 tiny NCP, 48 TC devices, GP** (measurement) | **512,340** | 896 | 71,460 | **72,932 ³ + 5,632 reserved** | none: needs ~102 KiB of 64 | 20,624 steady state ⁴ |

The 32-device and floor rows were measured on an earlier revision of the
branch (before the tier-precedence change, the TSN helper and comment
edits); the 48-device image of that revision was 176 B smaller in flash
with identical RAM, so compare those rows by RAM.

¹ 32 TC devices, 768 B endpoint queues, 2 KiB UART receive ring, restore
staging 16/16, 4 APS requests in flight, 8 retry entries, 2 log records
(temporary edits, not committed).
² Boot composition chain (see [side findings](#side-findings)).
³ `.data` + `.bss` + `.rf_dma` (576 B); the TLSR8258 also reserves
`.ram_code` (rounded up to 256 B) and the 2.25 KiB I-cache tag/data at the
bottom of SRAM, plus the mandatory 4 KiB gap and the 1 KiB IRQ stack.
⁴ `block_on_pinned` + `serve` (dyn polls resolved); 28,264 B from
`_rust_entry` including the measurement root's 7,640 B composition frame.
IRQ path 516 B.

Measured per-device cost: 48 → 32 Trust Center devices saves 4,096 B, i.e.
**256 B of RAM per TC device** (TC runtime table 54 B, persisted state
88 B, APS link key ~51 B, restore staging, alignment).

### Where the RAM goes (MG21 tiny NCP, 48 devices)

From `llvm-nm` and `-Zprint-type-sizes` (thumbv8m):

| static | bytes | largest parts |
|---|---:|---|
| `ZigbeeDevice` (MAC → BDB, ZCL) | 29,712 | NWK 9,832 (MAC 1,656, neighbors 1,284, indirect 1,156, replay floors 1,072, source routes 896, broadcast TX 660, address map 644, RREQ 576, routes 576); APS +7,056 (link keys 2,452, ACK table 1,348, fragment RX 1,312, bindings 708, fragment TX 316); ZDO +3,216 (endpoint descriptors 2,564); device +9,240 (GP proxy/sink 4,180, reporting 1,544, Identify 648, scratch 644, ZCL endpoints 644, Basic 568, pending responses 564) |
| `Ncp<4>` | 17,896 | link 6,528 (RX 2,608, TX 2,720, COBS accumulator), endpoint queues 5 × 1 024 = 5,140, restore staging 2,336, op queue 1,456, TX frame 657, deferred 588, active scan 468, retry 368 |
| `TrustCenterCoordinatorApp` | 11,000 | Trust Center runtime 8,008 (persisted state 4,544, TC table 2,600, initial-key attempts 580), coordinator core 2,968 (child and APS-table journal caches) |
| host UART rings | 5,136 | RX 4 KiB, TX 1 KiB |
| main task future | 4,624 | the `run` loop's awaited futures |
| security journal | 3,232 | |
| radio (`isr-ack`) | 2,308 | RX queue 1 KiB, RX buffer 1 KiB, TX 256 |
| host passthrough, GP NCP state, port, GP store | 2,888 | 1,332 + 968 + 480 + 108 |
| `.data` | 1,336 | log ring (4 records) and initialisers |

The MG21 router's static RAM is 34,536 B; the NCP adds 44,212 B: the NCP
struct 17.9 KB, the Trust Center app +8.6 KB, the device +7.4 KB (Trust
Center, global APS counter, host passthrough, host endpoints, GP sink),
the UART rings 5.1 KB, host passthrough and GP NCP state 2.3 KB, the rest
~2.9 KB.

### Stack

- **Boot**: the composition builds the stack layers by value
  (`compose` 12,840 B frame → `compose_device` 6,032 → `assemble_into`
  2,240 → APS `new` 3,268 → NWK `new` 4,040 → `NeighborTable::new` 1,320):
  33,132 B on MG21. It can be cut to the steady-state depth by
  constructing in place, as the ESP32 NCP already does for `Ncp`.
- **Steady state**: the TC32 analysis, which resolves `dyn Future` polls,
  bounds the NCP loop at **20,624 B**. On ARM the direct-call part is
  14.4 KB (`run` poll frame 2,728 + `install_network` 11,660); the
  ESP32-H2 NCP's `serve` chain budget is 31 KiB. A 64 KiB NCP therefore
  needs about 20 KiB of stack plus margin, so static RAM must stay at or
  below **~40 KiB**.

### Green Power

Measured on the routers (proxy only): MG21 +30,960 B flash, +2,048 B RAM;
TLSR8258 +27,180 B flash, +2,360 B RAM. The NCP's GP combo (proxy + sink +
`gp`/`gp_remote`) cannot be compiled out of `ncp-coordinator-app` today
(its GP features are unconditional), so **the GP-off NCP numbers are
projected**: RAM −5,257 B (device GP 4,180 + GP NCP state 968 + GP store
108 + reply state; exact type sizes), flash −56…62 KB (50,222 B of
GP-named text in the tiny image, scaled by the router's
attribution-to-delta ratio 1.12, plus rodata). Projected GP-off tiny NCP:
~390–395 KB flash (fits F512's 400 KiB window, barely), ~73.5 KB static
RAM (48 devices) or ~63.8 KB (parameter floor): still no room for a stack.

### Flash fit

| part | application window | tiny NCP + GP | GP off (projected) |
|---|---:|---:|---:|
| EFR32MG21 F768 (ZS3L, Dongle-E) | 656 KiB | 450,744 B: fits, 216 KiB spare | fits |
| EFR32MG21 F512 | 400 KiB | overflows by 41,152 B | ~390–395 KB: fits with < 20 KB |
| TLSR8258 1 MiB (ZT3L) | up to ~896 KiB | 512,340 B: fits | fits |
| TLSR8258 512 KiB (TB-04) | 440 KiB (`0x6E000`) | does not fit | ~455 KB: does not fit |

TC32/ARM code ratio: routers 287,765 / 323,840 B of `.text` = 0.89 (both
`opt-level = "s"`); the tiny NCP is 478,675 B on TC32 (`s`) vs 414,932 B on
ARM (`z`).

## What would close the gap

Estimated from the breakdown above (each item is a code change, none is
implemented):

| change | static RAM saved | notes |
|---|---:|---|
| ZRHP small-MTU profile: a negotiated `rx_mtu` (reserved in v1), e.g. 256 B | ~8–9 KiB | link 6.5 → ~2.5 KiB, endpoint queues 5.1 → ~2.5 KiB, UART RX ring 4 → 2 KiB; backup/TBL pages already negotiate their size |
| one Trust Center device record (merge the persisted state, the TC table, the APS key and restore staging) | ~190 B per device: 6–9 KiB at 32–48 | measured 256 B per device today |
| no RAM snapshot caches for the security, APS-table and child journals | ~4–5 KiB | read-modify-write from flash instead |
| NCP-only runtime profile: no device-local ZCL server state (reporting, Identify, Basic, scratch), two host endpoints, no APS fragmentation | ~5–7 KiB | the host serves ZCL and descriptors |
| `ncp-coordinator-app` `green-power` feature (or a host-side sink, `GP_SHADOW`) | ~5.3 KiB, ~60 KB flash | |
| restore staged straight into the TC table | ~2 KiB | |
| in-place construction of the stack layers | stack 33 → ~20 KiB | needed on every NCP, see side findings |

Together ~30–37 KiB: about 42–49 KiB of static RAM at 48 devices, 38–45
KiB at 32. With ~20 KiB of stack that is **marginal on the EFR32MG21 at 32
devices and does not fit at 48**; on the TLSR8258 (about 53 KiB left for
static RAM plus the SVC stack after the RAM-code, cache, gap and IRQ
reservations) it does not fit at all. This is several weeks of runtime
restructuring for a part that would still be the weakest NCP, so it is
not recommended now.

## Host-assisted tables

ZRHP already specifies paging (8.1–8.8): a bounded NCP cache, the host
holding the complete TC-device/key store (`TBL_LOOKUP`, `TBL_WRITEBACK`,
`TBL_SYNC`, epochs, `rev`, invariants I1–I5). The NCP implements only the
standalone mode (`tables.rs`).

**RAM it would save on the tiny tier.** TC devices: a 16-entry cache
(children, keys in flight) instead of 48 saves ~8 KiB at the measured
256 B per device. The address map (64 × 10 B = 644 B) is already an
evictable cache; source routes (896 B) are already optional when the host
sends relays (`HOST_SOURCE_ROUTES`). Paging adds a miss queue
(`tbl_miss_queue_len` 8 × 128 B = 1 KiB) and writeback state (`rev`,
dirty/pinned, ~6 B per entry). **Net ~7–8 KiB**, a third of the gap at
best.

**Latency.** A `TBL_LOOKUP` exchange is ~33 B out and ~75 B back (DEVICE
record) plus link overhead: ~10 ms of wire time at 115 200 Bd, ~2.5 ms at
460 800, ~1.3 ms at 921 600, plus the 1 ms ack delay and the host's
PSRAM lookup (< 1 ms). Join and rejoin timers are seconds
(`bdbcTCLinkKeyExchangeTimeout` 5 s, APS ACK wait ~1.5 s with retries,
Transport-Key delivery to a polling child up to the poll interval), so a
lookup fits easily even at 115 200 Bd; the spec's 250 ms lookup timeout
holds.

**What the NCP would need.** No new ZRHP messages for TC devices (they
exist); on the NCP: per-entry `rev`/dirty/pinned and the NV-persisted
`host_acked_rev`; a key-provider hook in `zigbee-aps` decryption and in
`zigbee-bdb` admission (Update-Device, association, Request-Key) that can
park a frame or a join until the host answers; writeback with retry and
eviction only after a durable host commit (I1); `TBL_SYNC` re-sending
dirty entries after either side resets. Security: the keys cross the
on-board UART in the clear, as `KEY_EXPORT` backups already do; paging
makes the host a permanent key holder, which contradicts
`SEC_LOCK_KEY_EXPORT` and must be documented as such (link
authentication, spec section 13, stays reserved). With the host down,
devices whose keys are not cached cannot rejoin or be decrypted
(availability, never safety).

**Recommendation: against, for the tiny tier.** It saves too little to
make a 64 KiB NCP fit and adds the most delicate state machine of the
stack (security paths that wait for the host). It remains the right way
to scale the compact and full tiers beyond 128–256 devices later.

## Update path per chip

**EFR32MG21 (ZS3L).**

- First flash and recovery: SWD pads (back up the whole flash first; the
  stock NVM3 and Tuya tokens are overwritten).
- Field updates: keep the resident Gecko bootloader. The host reboots the
  NCP into it (`SYS_RESET` mode 1 → `reboot_to_bootloader`, or PC01 held
  low across a RESET pulse with Tuya's gateway bootloader, if the panel
  wires both lines to the S3) and uploads a GBL over XMODEM at 115 200
  (`tools/efr32-gbl`, as for MG24). The NV journals sit above every GBL.
  Whether the module's bootloader enforces signed GBLs, and that it stays
  in bootload mode when the application is invalid, are unverified.
- A/B with the zigbee-rs loader (`zrs-boot`, chained as on the ZBT-1):
  768 − 16 (Gecko) − 16 (loader) − 96 (NV) = 640 KiB, i.e. two 320 KiB
  slots, below the 450 KB image (and the ~392 KB GP-off projection). Not
  possible on F768; a single-slot overwrite through Gecko is the path.

**TLSR8258 (ZT3L).**

- First flash and recovery: SWS only (hwlab Telink SWire plugin or
  `TlsrPgm`); there is no ROM UART bootloader.
- Field updates need our own resident loader at `0x0` (the ROM always
  starts there): UART receive with ZRHP-compatible framing, verification
  with the shared `zrs-boot`/`mcuboot-image` code, and an application
  linked above it. Telink documents ROM multi-address boot (an alternate
  image address selected by the `0x4B` flag at image offset 8); its
  address choices on 1 MiB parts are **not proven** in this repository
  ([bootloader](bootloader.md): "Telink remap not proven"). With a
  `0x80000` alternate, slot A could hold the 512 KB NCP image but slot B
  (448 KiB below the NV and factory sectors) could not; GP off would fit
  with ~1 KiB to spare. A/B is not realistic for this image.
- The host link would need a DMA receive path: the non-DMA UART has a
  4-byte receive buffer, and the flash routines mask interrupts for the
  whole erase, so bytes are lost during every journal erase (ARQ recovers
  them, but every flash write costs a retransmission round).

## Capacity verdict

| chip | zigbee-rs ZRHP NCP | realistic devices | basis |
|---|---|---:|---|
| EFR32MG21 F768/F512 (64 KiB) | **no** (static RAM 69–79 KiB measured) | 0 today; ~32 after the roadmap, with thin margins | this chapter |
| TLSR8258 (64 KiB) | **no** (static 78.6 KiB measured incl. reservations) | 0 | this chapter |
| CC2652P7 (144 KiB) | yes, `compact` | 128 | [CC26x2 guide](../platform-guides/cc26x2.md); see the boot-stack finding below |
| EFR32MG24, ESP32-H2/C6 (256 KiB+) | yes, `ncp-capacity` | 256 | [host protocol 8.9](host-protocol.md) |

Vendor firmware on the same silicon does better because its stacks are
C with shared buffer pools: Tuya's ZS3L-based multimode gateway (an
RTL8197FS host with the ZS3L as an EmberZNet NCP) is specified for up to
128 Zigbee sub-devices (Tuya "Multi-Mode Gateway PCBA"). SONOFF's
EmberZNet 7.4.5 NCP on the MG24 Dongle Max reports neighbors 26, children
32, address table 8 (`zigbee-rs-hub/docs/ezsp-backend.md`). Telink's
TLSR8258 coordinator table sizes were not found.

**For the panels:** a ZS3L panel can use an EmberZNet NCP build for the
EFR32MG21A020F768 (the Dongle-E part) with the module's UART route, and
the hub's existing EZSP backend. A ZT3L panel has no supported
coordinator path; the module could run the zigbee-rs TLSR8258 router
(fits, lab-tested on the TB-04) only as a router of another network.

## Side findings

- **CC2652P7 NCP boot stack.** Its static direct-call bound is
  50,512 B (`TaskStorage::poll` 2,728 → `compose` 21,616 → `compose_device`
  6,304 → `assemble_into` 2,232 → APS `new` 6,480 → NWK `new` 7,888 →
  `NeighborTable::new` 2,600 …) against a linked stack of 26,268 B. If the
  bound is reachable, the image overflows its stack at boot. It has never
  run on hardware; in-place construction of the stack layers (and a stack
  gate like `tools/esp32-ncp-stack-gate.py` for the ARM NCPs) should come
  before its first hardware run. The MG24 NCP's bound (71,744 B) is below
  its 89,512 B stack.
- **xG21 entropy.** Without `se-trng`, MG21 images have no entropy source;
  a coordinator role on xG21 needs the Secure Engine TRNG proven first.
- **TC32 and Cortex-M0+ atomics.** NCP code must not use atomic
  read-modify-write (fixed for `gp_remote`'s TSN).

## Reproducing

```bash
# Host tests of the tier
cargo test -p zigbee-nwk --features router,ncp-capacity-tiny
cargo test -p zigbee-aps --features ncp-capacity-tiny
cargo test -p zigbee-bdb --features ncp-capacity-tiny
cargo test -p zigbee-runtime --features ncp-capacity-tiny,green-power-ncp,host-passthrough
cargo test --manifest-path apps/ncp-coordinator/Cargo.toml --features tiny
# The real F768 link (fails: .bss overflows RAM by 13,412 B)
(cd examples/efr32mg21-ncp && cargo build --release)
# Measurement-only link (never flash it)
(cd examples/efr32mg21-ncp && EFR32MG21_NCP_MEASURE=1 cargo build --release --target-dir target/measure)
python3 tools/mac-footprint.py report tiny=examples/efr32mg21-ncp/target/measure/thumbv8m.main-none-eabihf/release/efr32mg21-ncp
```

## Open questions

- Which ZS3L UART route, reset and boot lines a ZS3L-equipped panel wires
  to the ESP32-S3, and whether its Gecko bootloader enforces signed GBLs.
- The ZS3L's HFXO CTUNE (no `MFG_CTUNE` token known; the board uses the
  ZBT-1's 140).
- Whether the xG21 Secure Engine answers the TRNG command as xG24 does
  (`se-trng` is build-only).
- TLSR8258 ROM multi-address boot on 1 MiB parts: the alternate address and
  how it is selected.
- Whether the roadmap's ZRHP small-MTU profile is worth a protocol
  revision for other constrained hosts too (embedded hosts with 4–8 KiB
  link budgets).
- Whether a ~32-device NCP on 64 KiB is worth weeks of runtime
  restructuring, given that EmberZNet already covers the ZS3L.
