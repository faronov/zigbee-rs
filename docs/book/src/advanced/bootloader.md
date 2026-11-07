# Bootloader Core

zigbee-rs ships one pure-Rust bootloader design across platforms. Every
chip runs the same verified boot sequence. Each chip contributes only a thin
shell: flash driver, entry check, hand-over and status output. Products own
the layout, the trusted key and the loader binary.

This page is both the design and its current status. **Phase 1** (this
revision) extracted the shared core from the two existing loaders with no
behaviour change. Everything marked *Phase 2* or later is a proposal awaiting
a decision. It is not an implemented or hardware-proven capability.

## Current state (Phase 1)

| crate | role | validation |
|---|---|---|
| `mcuboot-image` | image format, SHA-256 + ECDSA P-256 verification, trailer magic, the overwrite-only install engine (`boot`), `LoaderInfo`, the app-side `SlotWriter` (`firmware-writer`) | host-tested, including power loss after every flash operation |
| `bootloaders/core` (`zrs-boot-core`) | `Platform` trait and the `launch`/`run` sequence, `BootFlash` slot guard, RISC-V fixed-entry and Cortex-M vector-table checks, console `Reporter`, LED `Indicator`, `sim::SimFlash` power-cut simulator | host-tested (`--features sim`) |
| `bootloaders/bl702-boot` | BL702 shell: XIP mapping, L1C invalidation, riscv-rt jump, UART lines | host-tested, release-built; production path proven on one XT-ZB1 (0.1.0→0.1.1 OTA, commit `8aeee49`) **before** this refactor; the refactored binary is not yet re-run on hardware |
| `bootloaders/cc2340r5-boot` | CC2340R5 shell: vector-table check, VTOR/MSP jump, fault blinks and SHUTDOWN | host-tested, thumbv6m build only; no product loader binary exists |

```text
product loader binary   layout, key (build-time), LoaderInfo placement, console/LEDs
        |
chip shell (*-boot)     impl Platform: entry check, cache/clock hand-over, jump, fail
        |
zrs-boot-core           launch/run, BootFlash, entry checks, Reporter/Indicator, sim
        |
mcuboot-image           header/TLV, SHA-256, P-256, trailer, overwrite engine, writer
```

## Core API

```rust,ignore
pub trait Platform {
    fn progress(&mut self, event: BootEvent);          // status output
    fn kick_watchdog(&mut self) {}                       // before every event
    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {}
    fn prepare(&mut self, layout: &Layout, bootable: &Bootable) -> Option<u32>;
    unsafe fn start(&mut self, entry: u32) -> !;
    fn fail(&mut self) -> !;
}

pub fn launch<F: NorFlash, P: Platform>(flash, layout, key, platform) -> Option<u32>;
pub fn run<F: NorFlash, P: Platform>(flash, layout, key, platform) -> !;
```

- **Flash** is any `embedded_storage::nor_flash::NorFlash`. It is not a
  method of `Platform`, so a HAL's existing driver plugs in directly.
  `ERASE_SIZE` must divide `Layout::sector_size`. `BootFlash` wraps the
  driver and refuses any erase or program outside the two slots. The
  loader, the boot header and the persistence partitions are therefore
  unreachable from the copy. The BL702 shell uses it. The CC2340 shell
  keeps its unguarded `InternalFlash` so that Phase 1 changes no
  behaviour; adopting `BootFlash` there is a Phase 2 item.
- **Page size** is `Layout::sector_size`, the largest erase unit of the
  two slots. Write granularity comes from `NorFlash::WRITE_SIZE`.
- **Jump to the application** is `prepare` (entry check, cache
  invalidation) followed by `start`. The core never calls `start` with
  anything `prepare` did not return for the image it validated in the same
  boot.
- **Watchdog**: `kick_watchdog` runs before every progress event. No
  current loader arms a watchdog. Validating a full slot is one step with
  no events, so an armed watchdog must last longer than one full-slot
  validation. A per-chunk hook inside `validate` would be a Phase 2
  `mcuboot-image` change.
- **Crypto accelerator** (Phase 2, optional): an `ImageCrypto` trait for
  streaming SHA-256 and P-256 verify, with today's software implementation
  as the default. Candidate engines exist on BL702 (SEC_ENG), nRF52840
  (CC310), EFR32MG1 (CRYPTO) and MG21/MG22 (SE/CRYPTOACC). None is used
  or verified yet. The software path stays the reference, and the host
  tests compare the two.

## Image format and versioning

- MCUboot/imgtool images: a `0x100`-byte header (`imgtool --header-size
  0x100`), with the application vector table or entry at
  `slot + 0x100`. The TLV area carries `SHA256`, `KEYHASH` and
  `ECDSA256`. Header flags must be zero: no encrypted, compressed or RAM-load
  images.
- The version is `major.minor.revision+build`. The downgrade rule compares
  `major.minor.revision` and ignores `build`, as MCUboot's
  `MCUBOOT_DOWNGRADE_PREVENTION` does.
- The Zigbee OTA file version is `0x25MMmmpp` (major, minor, patch), so
  ZCL "newer file" decisions and the loader's downgrade rule agree.
- `LoaderInfo` (32 bytes, "ZRSBOOT1", CRC-32) sits at a product-fixed
  address. It identifies the loader version and the hash of the trusted
  key, so the application can refuse an image the loader would reject
  before downloading it.

## Signature scheme and keys

- **Today**: ECDSA P-256 over SHA-256 (`imgtool sign --key ... -t
  ecdsa-p256`), checked by the compact verify-only `mcuboot_image::p256`.
  The loader trusts exactly one public key.
- **Keys are build-time inputs, never constants in the core or shell
  crates.** The shells take a `TrustedKey` from the product. The BL702
  product reads `BL702_OTA_KEY_XY` (hex, or a file path) in
  `products/bl702-xt-zb1/layout/build.rs`. Its default is the committed
  *public* key `keys/own-ec-p256.xy.hex`. Private keys never enter the
  repository. CI generates an ephemeral key for each run, so CI images
  are test-signed.
- Every new product loader follows the same pattern: one
  `<PRODUCT>_OTA_KEY_XY` input, consumed by the product's layout crate. The
  loader and the application's `SlotWriter` share that key, and so do
  `LoaderInfo` and the OTA policy.
- Open decisions: listed below.

## Install strategy: overwrite, swap or ping-pong

Today every loader is **overwrite-only**. A pending request is the MCUboot
magic in the secondary trailer. The application writes it only when
`FirmwareWriter::activate` succeeds. The loader then:

1. validates the secondary image (hash, signature, key hash, flags) and
   applies the downgrade rule against the primary header;
2. erases the primary, copies the image in 256-byte chunks, skipping
   all-`0xFF` chunks, and re-validates the copy;
3. erases the secondary trailer sector, then the secondary header sector.

The request survives until the copied primary validates. A power cut at
any point resumes the copy from the intact secondary on the next boot. A
rejected request is dropped in the same order, so the drop is also
resumable.

Overwrite-only has no rollback. A signed image that crashes after boot
stays installed until a later OTA replaces it. Options:

| strategy | rollback | extra flash | complexity | fits |
|---|---|---|---|---|
| overwrite-only (today) | no | none | lowest; proven on BL702 | every chip; the only option when the secondary is external SPI flash (MG1) |
| overwrite + confirm flag | no rollback; adds "booted but never confirmed" detection and a retry/recovery policy | one trailer word | low | stepping stone for every chip |
| swap-move (MCUboot) | yes | one sector per slot plus a status area | high; per-sector state machine, twice the flash wear | CC2340, nRF52, MG21/MG22 if rollback is required |
| ping-pong (execute in place from either slot) | yes, natively | none | low in the loader; the image must run from either slot | ESP (MMU plus `otadata`); candidates to verify: BL702 (XIP image-offset remap), Telink (ROM dual-boot address) |

**Recommendation for Phase 2:**

- Keep overwrite-only as the default for every chip.
- Add a confirm flag to the trailer, written by the application after it
  rejoins its network. The application then reports a "not confirmed"
  state, which feeds the OTA policy.
- Evaluate ping-pong on BL702 only after proving the XIP remap on
  hardware.
- Treat swap-move as opt-in per product.

The decision is per product, and the per-chip table below records it.

### Known hazard (characterised, not yet fixed)

An erase interrupted on the primary header sector can leave the header
magic intact while the version bytes read higher, because erase only sets
bits. The engine then applies the downgrade rule against an image that no
longer validates. It drops the request and nothing is bootable. MCUboot's
overwrite-only mode compares headers the same way.
`known_hazard_torn_primary_header_reads_as_a_downgrade` pins the current
behaviour. The fix changes behaviour, so it is a Phase 2 decision. Two
options:

- apply the downgrade rule only against a primary that validates, and
  treat an invalid primary as "anything signed may be installed";
- before erasing the primary, write a `copy-started` mark into the
  secondary trailer, using MCUboot's trailer field offsets, and skip the
  downgrade check while the mark is present.

## Per-chip layouts and field migration

Existing layouts do not change. New layouts are proposals for Phase 2 and
need the product's linker post-link checks before any firmware uses them.

| chip / product | today | proposed loader | slots (proposal) | migration |
|---|---|---|---|---|
| BL702 XT-ZB1 | `bl702-boot` at `0x2000..0xFFE0`, `LoaderInfo` `0xFFE0`, primary `0x10000..0x87000`, secondary `0x87000..0xFE000`, journal `0xFE000..0x100000` | unchanged | unchanged | none; the refactored loader is binary-compatible with the slots, trailer, key and `LoaderInfo` (reflash over ISP to adopt it) |
| CC2340R5 | app at `0x0` (504 KiB), journal `0x7E000..0x80000`, `NoOta`; loader crate exists, no product binary | 16 KiB at `0x0` | primary `0x4000`, secondary in the upper half, journal unchanged at `0x7E000` | first install over SWD only; it moves the application, so the journal is kept and the device stays commissioned |
| nRF52840/52833 bare | app `0x0..0xFE000`, journal `0xFE000`, `NoOta` | 16 KiB at `0x0` | two equal slots below the journal | SWD |
| nRF52840 UF2 boards | MBR `0x0..0x1000`, app `0x1000..0xF2000`, journal `0xF2000`, UF2 `0xF4000..0xFE000`, MBR params `0xFE000` | zrs loader as the UF2 "application" at `0x1000` (UF2 and MBR untouched) | two slots in `0x5000..0xF2000` | flash the loader and a signed image over UF2; UF2 remains the recovery path |
| PHY6222 / PB-03F | ROM boots the PHY6 header at `0x10000`, app `0x10100..0x3E000`, `NoOta` | the image the ROM boots | slots after the loader | first prove that ROM boot of a plain image works (open hardware gate) |
| EFR32MG22 | no bootloader, app at `0x0`, `NoOta` | 16 KiB at `0x0` | internal slots, persistence unchanged | SWD |
| EFR32MG21 | Gecko `0x0..0x4000`, app `0x4000..0x74000`, router tables `0x74000..0x7C000`, persistence `0x7C000..0x80000`, `NoOta` | replaces Gecko in `0x0..0x4000` | internal slots below `0x74000` | SWD for development devices; field devices as for MG1 |
| EFR32MG1 TRADFRI | Gecko `0x0..0x4000`; app `0x4000..0x37000`; journal `0x37000..0x39000`; app NV `0x39000..0x3A000`; NVM3 `0x3A000..0x40000`; OTA staged as GBL in external SPI flash via the bootloader API | replaces Gecko in `0x0..0x4000` | primary internal `0x4000..0x37000`; secondary in **external SPI flash** (USART0) | high risk, see below |
| ESP32-C6/H2 | IDF second stage, `otadata`, `ota_0`/`ota_1`, `zbnv` | keep IDF | keep | the app-side writer verifies the shared signature before `activate` (Phase 2) |
| TLSR8258 | ROM boot, NV `0x70000..0x78000`, `NoOta` | keep ROM dual-boot | keep | as ESP |

### EFR32MG1 leaving the Gecko bootloader

- **One-way and high risk.** The device can only be updated over the air
  through Gecko's GBL path. Replacing the bootloader means delivering a
  GBL whose bootloader-upgrade payload is the zrs loader, or an
  application that rewrites `0x0..0x4000` itself.
- A power cut while that region is erased leaves a device that only SWD
  can recover.
- Series 1 splits the Gecko bootloader into a first stage and a main
  stage. Before choosing a path, verify against Silicon Labs UG266 and on
  a sacrificial board whether the first stage verifies, or must keep, the
  main stage it boots.
- The secondary slot lives in external SPI flash. The core therefore needs
  a split-device `NorFlash` adapter (internal primary, external secondary)
  in Phase 2. The board crate owns the USART0 flash pins. The Gecko API
  and direct SPI access must never run at the same time.
- NVM3 (`0x3A000..0x40000`), the journal and the application NV must stay
  untouched, and `BootFlash` must guard them.
- Do this chip last, and only after the loader is hardware-proven on MG22
  or MG21.

## Size budget

Measured with `llvm-size -A` on release builds. File-backed means
`.text + .rodata + .data` plus `.vector_table` or `.init.rust`. Before and
after were built from equal-length worktree paths.

| loader | region | before (`32c17d8`) | after (Phase 1) | delta |
|---|---|---|---|---|
| BL702 `bl702-xt-zb1-boot` | `0x2000..0xFFE0` (57,312 B) | 14,070 B (`.text` 12,084, `.rodata` 1,376, `.data` 572, `.init.rust` 38); `.bss` 1 | 14,202 B (`.text` 12,320, `.rodata` 1,272); `.bss` 1 | **+132 B** flash, +0 RAM |
| CC2340R5 (size harness; no product binary) | 16 KiB | 8,796 B (`.vector_table` 192, `.text` 7,888, `.rodata` 716); `.bss` 4 | 8,828 B (`.text` 7,920) | **+32 B** flash, +0 RAM |

The growth comes from three changes:

- the shared `Reporter` writes the console prefix as a separate call
  instead of storing one literal per line;
- the trait-method split of `fail`;
- the watchdog-hook closure.

`.rodata` shrank because the prefix is now stored once.

Budget for new shells: at most 16 KiB of flash for Cortex-M loaders, which
matches the Gecko and CC2340 loader regions, and BL702's existing region.
Stack use has not been measured yet. BL702 reserves 15 KiB. Measuring the
worst-case stack of `validate`, which holds P-256 state, is a Phase 2
gate.

## Testing strategy

1. **Host, simulated flash** (`zrs-boot-core`, feature `sim`):
   - `SimFlash` models NOR semantics: erase sets `0xFF`, programming can
     only clear bits, the erase and write alignment of the real part is
     enforced, and `strict()` refuses to reprogram a unit that is not
     erased.
   - A power cut can be injected before any erase or program operation, in
     three modes:
     - `Before`: nothing happens;
     - `Half`: a prefix is applied;
     - `Torn`: noisy bits.
     After a cut, every operation fails until `power_cycle()`.
   - `replay_with_cuts` records the operations of a clean boot. It then
     replays the boot once per operation and per cut mode. After each cut
     it checks that:
     - the loader and NV regions are untouched;
     - the next boot finishes the job;
     - the boot after that is a plain boot.
   - This covers the install of both a requested and a padded image, and
     dropping a rejected and a downgrade request.
   - The same vectors and a second power-loss suite run in `mcuboot-image`.
2. **Host, shells**: entry and vector checks, the XIP mapping, the BL702
   console lines byte-for-byte, the indicator blink pattern, and
   `BootFlash` against the real BL702 map.
3. **Build gates** (CI):
   - `cargo test` and `clippy -D warnings` for the core (`sim,compact-sha256`)
     and for each shell, on host and target;
   - the BL702 product loader, and `build-image.sh` with an ephemeral key;
   - the CC2340 shell for thumbv6m.
4. **Hardware**, one chip at a time, using the existing lab, not in Phase 1:
   1. boot a signed image;
   2. refuse an unsigned image and a downgrade;
   3. OTA from a version N to N+1, end to end;
   4. cut power with a controllable supply at random points during the
      copy, at least 50 cycles;
   5. confirm that the commissioned network state and security counters
      survive.

## Open decisions

1. **Install strategy per chip:** overwrite-only, overwrite with a
   confirm flag, swap-move, or ping-pong (BL702, ESP, Telink).
2. **Signature algorithm.** Keep ECDSA P-256, which imgtool and the
   proven BL702 key already use, or add Ed25519, which needs a smaller
   verifier but a different imgtool key type.
3. **Key handling.** One key or several trusted keys per loader, and
   rotation. A key change needs a new loader, and no loader self-update
   path exists or is proposed.
4. **The torn-header downgrade hazard.** Which fix to apply (see above).
5. **Watchdog.** Whether loaders arm one, and with what period.
6. **CC2340 `BootFlash`.** Whether the CC2340 shell adopts the slot guard.
   It costs a little flash and changes behaviour only for an out-of-slot
   write.
7. **The MG1 migration path.** A GBL-delivered loader, or app-side
   self-replacement. Alternatively, keep Gecko on MG1 permanently and
   share only the image format and signature.

## Proposed Phase 2 order

1. Engine fixes in `mcuboot-image` and the core: the torn-header hazard
   and an optional confirm flag. Host tests only. Then re-run the BL702
   OTA and power-cut tests on hardware with the refactored loader.
2. A CC2340R5 product loader binary and a `SlotWriter`-based product OTA
   path. Hardware: SWD install, boot, OTA, power cuts.
3. nRF52840: first the bare DK layout, then UF2 coexistence. Hardware:
   both boards.
4. EFR32MG22 (no bootloader), then MG21 (replacing Gecko over SWD).
   Hardware: both.
5. PHY6222, after proving its ROM boot.
6. ESP32 and Telink: app-side verification of the shared signature
   before activation, with no loader change.
7. EFR32MG1 leaving Gecko, last, after a sacrificial-board migration
   rehearsal.
