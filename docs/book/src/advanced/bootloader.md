# Bootloader Core

zigbee-rs uses one pure-Rust bootloader design on every platform. Every
chip runs the same verified boot sequence. Each chip adds only a thin
shell: the flash driver, the entry check, the hand-over and the status
output. Each product owns its layout, its trusted keys, its trial policy and
its loader binary.

This page is both the design and its current status:

- **Phase 1** (`0efac9b`) extracted the shared core from the two existing
  loaders without changing their behaviour.
- **Phase 2 step 1** (this revision) adds:
  - the brick fix;
  - trial boots with an application confirm;
  - up to three trusted keys;
  - optional Ed25519;
  - opt-in swap-move with revert.

  All of it is **host-tested only**. The refactored BL702 loader has not
  yet been run on hardware; the plan is in
  [BL702 hardware re-validation](#bl702-hardware-re-validation-plan).

Anything marked *later phase* is a proposal. It is not an implemented or
hardware-proven capability.

## Current state

| crate | role | validation |
|---|---|---|
| `mcuboot-image` | Image format, SHA-256, ECDSA P-256 and optional Ed25519 verification, the install engine (`boot`), the status/trial log, opt-in swap-move, `LoaderInfo` and `LoaderKeys`, and the application-side `SlotWriter` (`firmware-writer`) | Host-tested, including power loss at every flash operation |
| `bootloaders/core` (`zrs-boot-core`) | The `Platform` trait and the `launch`/`run` sequence, the `BootFlash` slot guard, the RISC-V fixed-entry and Cortex-M vector-table checks, the console `Reporter`, the LED `Indicator`, and the `sim::SimFlash` power-cut simulator | Host-tested (`--features sim`) |
| `bootloaders/bl702-boot` | BL702 shell: XIP mapping, L1C invalidation, riscv-rt jump, UART lines | Host-tested and release-built. The v1 loader's production path was proven on one XT-ZB1 (0.1.0→0.1.1 OTA, `8aeee49`). The v2 loader (this revision) **has not been run on hardware** |
| `bootloaders/cc2340r5-boot` | CC2340R5 shell: vector-table check, VTOR/MSP jump, fault blinks, SHUTDOWN | Host-tested; thumbv6m build only. No product loader binary exists |
| `zigbee-runtime` `FirmwareWriter`/`OtaLifecycle` | `confirm_image` hook; `FirmwareWriter::confirm_image` | Host-tested |
| `apps/sensor-sed` | Confirms the running image on its first APS-acknowledged report, or once joined when reports are not acknowledged | Host-tested |

```text
product loader binary   layout, BootConfig (keys, trial boots, mode), loader block, console/LEDs
        |
chip shell (*-boot)     impl Platform: entry check, cache/clock hand-over, jump, fail
        |
zrs-boot-core           launch/run, BootFlash, entry checks, Reporter/Indicator, sim
        |
mcuboot-image           header/TLV, SHA-256, P-256/Ed25519, status log, overwrite/swap-move, writer
```

## Core API

```rust,ignore
pub struct BootConfig<'a> {
    pub layout: Layout,              // primary, secondary, slot_size, sector_size
    pub keys: &'a [TrustedKey],      // 1..=MAX_TRUSTED_KEYS (3)
    pub trial_boots: u8,             // 0 = no trial accounting
    pub mode: Mode,                  // Overwrite | SwapMove { status } (feature)
}

pub trait Platform {
    fn progress(&mut self, event: BootEvent);          // status output
    fn kick_watchdog(&mut self) {}                       // before every event
    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {}
    fn prepare(&mut self, layout: &Layout, bootable: &Bootable) -> Option<u32>;
    unsafe fn start(&mut self, entry: u32) -> !;
    fn fail(&mut self) -> !;
}

pub fn launch<F: NorFlash, P: Platform>(flash, config: &BootConfig, platform) -> Option<u32>;
pub fn run<F: NorFlash, P: Platform>(flash, config: &BootConfig, platform) -> !;

// Application side (mcuboot-image):
pub fn trial_state<F: NorFlash>(flash, layout, mode) -> Result<TrialState, BootError>;
pub fn confirm<F: NorFlash>(flash, layout, mode) -> Result<TrialState, BootError>;
pub const fn trial_log_area(layout) -> (u32, u32);   // overwrite-mode log, for flash views
```

- **Flash.** Flash is any `embedded_storage::nor_flash::NorFlash`. It is
  not a method of `Platform`, so a HAL's existing driver plugs in
  directly.
  - `ERASE_SIZE` must divide `Layout::sector_size`.
  - `WRITE_SIZE` may be at most 16 bytes; a status record takes one unit.
  - `BootFlash::for_config` wraps the driver. It refuses any erase or
    program outside the two slots and, in swap-move mode, outside the two
    status sectors. The loader, the boot header and the persistence
    partitions are therefore unreachable.
  - The BL702 shell uses `BootFlash`. The CC2340 shell still uses its
    unguarded `InternalFlash` (an open decision).
- **Page size** is `Layout::sector_size`, the largest erase unit of the two
  slots.
- **Jump to the application.** `prepare` performs the entry check and
  cache invalidation, then `start` jumps. The core never calls `start`
  with anything that `prepare` did not return for the image validated in
  the same boot.
- **Watchdog.** `kick_watchdog` runs before every progress event. No
  current loader arms a watchdog. Validating a full slot is one step that
  emits no events, so an armed watchdog must outlast one full-slot
  validation.
- **Crypto accelerator** (later phase, optional). This would be an
  `ImageCrypto` trait for streaming SHA-256 and signature verification,
  with the software path as the default and as the host-test reference.
  Candidates: BL702 SEC_ENG, nRF52840 CC310, EFR32 CRYPTO/SE. None is used
  yet.

## Image format and versioning

- **MCUboot/imgtool images.**
  - The header is `0x100` bytes (`imgtool --header-size 0x100`). The
    application's vector table or entry sits at `slot + 0x100`.
  - The TLV area carries `SHA256` (`0x10`), `KEYHASH` (`0x01`), and either
    `ECDSA256` (`0x22`) or `ED25519` (`0x24`, over the SHA-256 digest,
    not `--pure`).
  - Header flags must be zero: encrypted, compressed and RAM-load images
    are refused.
- **Version.** The format is `major.minor.revision+build`. The downgrade
  rule compares `major.minor.revision` and ignores `build`, as MCUboot's
  `MCUBOOT_DOWNGRADE_PREVENTION` does. The Zigbee OTA file version is
  `0x25MMmmpp`, so ZCL "newer file" decisions and the loader agree.
- **Loader block.** This is a 64-byte block at the end of the loader
  region (BL702: `0xFFC0..0x10000`):
  - `LoaderKeys` ("ZRSKEYS1", CRC-32, `0xFFC0`) holds the 8-byte ids of
    the extra trusted keys (key 2 and key 3). It stays erased when the
    loader trusts one key.
  - `LoaderInfo` ("ZRSBOOT1", 32 bytes, CRC-32, `0xFFE0`) holds the loader
    version and the hash of the first key. Flag `0x0001` means a
    `LoaderKeys` record precedes it.
  - A v1 loader's record (flags `0`, no keys record) decodes as one trusted
    key, so existing BL702 devices are read correctly.
- **Key id.** A key's id is the first 8 bytes of the public key (the P-256
  X coordinate, or the Ed25519 key). The application trusts only the
  intersection of its build-time keys and the ids the loader publishes.
  With no loader record it trusts nothing and declines every OTA offer.
- `mcuboot-image/tests/vectors` holds imgtool-generated golden vectors:
  P-256 with key A and key B, and Ed25519. Their keys are throwaway files
  named `TEST-ONLY-*`. `tools/bl702_ota.py` produces the loader block, and
  a host test pins it against the Rust encoder.

## Signature scheme and keys

- **Default: ECDSA P-256 over SHA-256** (`imgtool sign -t ecdsa-p256`),
  verified by the compact verify-only `mcuboot_image::p256`. Existing
  BL702 field devices use it and keep working.
- **Optional: Ed25519** (feature `ed25519` on `mcuboot-image`, the core and
  both shells, using `ed25519-compact` with `opt_size`). The verifier is
  large; see the [size budget](#size-budget). It fits BL702's 56 KiB
  loader region but not a 16 KiB Cortex-M region. Making it fit there
  needs a compact 32-bit-limb verifier (an open decision).
- **Keys are build-time inputs, never constants in the core or shell
  crates.**
  - A product's layout crate reads them and builds `BootConfig::keys`. The
    loader, the application's `SlotWriter` policy and the loader block all
    use that same list.
  - BL702 reads `BL702_OTA_KEY_XY` (default: the committed *public* key
    `keys/own-ec-p256.xy.hex`) and the optional `BL702_OTA_KEY2_XY` and
    `BL702_OTA_KEY3_XY`. Each value is hex or a file path.
  - Private keys never enter the repository. CI signs with an ephemeral
    key generated for each run.
- **Key selection.** The image's `KEYHASH` TLV selects the matching
  trusted key, which must also be of the right type. An image signed with
  any trusted key validates.

### Key rotation

Up to three keys per loader allow rotation without bricking devices that
are behind:

1. **Add the new key.** Build a loader that trusts *old + new*
   (`<PRODUCT>_OTA_KEY_XY` = old, `<PRODUCT>_OTA_KEY2_XY` = new). Build the
   application with the same list.
2. **Keep signing with the old key** until every device you care about
   runs the old+new loader. An application signed with the old key boots
   under both the old and the new loader.
3. **Switch signing to the new key.** Devices on the old+new loader
   accept the new images. On a device whose loader trusts only the old
   key, the writer verifies a staged image against only the keys that
   both the application and the loader trust. An image signed with the new
   key therefore fails verification before `activate` and never becomes an
   upgrade request. An application that shares no key with its loader
   declines every offer (`[ota] offer declined: channel …`).
4. **Retire the old key** in a later loader. Before doing so, confirm that
   no device still needs it.

Loaders are not updatable over the air: no loader self-update path exists,
and none is proposed. Rotation therefore happens at ISP/SWD re-flash or
factory programming. A BL702 device that still has the v1 loader trusts
only its single original key. It must keep receiving images signed with
that key until someone re-flashes it over ISP.

## Install strategy

Overwrite stays the default for every chip. Swap-move is a build-time
opt-in.

### Overwrite (default)

A pending request is the MCUboot magic in the secondary trailer. The
application writes it only when `FirmwareWriter::activate` succeeds. The
loader then:

1. validates the secondary image (hash, signature, key, flags) and applies
   the downgrade rule (see [Brick fix](#brick-fix));
2. erases the primary trailer sector (which resets the trial log), erases
   the primary image sectors and copies the image in 256-byte chunks,
   skipping all-`0xFF` chunks, then re-validates the copy;
3. with trial accounting, appends `Done` to the trial log;
4. erases the secondary trailer sector, then the secondary header sector.

The request survives until the copied primary validates. A power cut at
any point resumes the copy from the intact secondary on the next boot. A
rejected request is dropped in the same order, so dropping it is also
resumable.

### Trial boots and confirm

`BootConfig::trial_boots = N` (BL702: `BL702_BOOT_TRIAL_BOOTS`, default 3;
`0` turns it off). The rules:

- **Starting the trial.** An image installed by this loader starts on
  trial (`Done` written).
- **Counting boots.** Each boot of an unconfirmed image appends `Attempt`
  and prints `trial boot n`.
- **Confirming.** The application calls `confirm` (`FirmwareWriter::
  confirm_image`, driven by `OtaLifecycle::confirm_image`). That appends
  `Confirmed` only while the image is on trial, at most one flash write.
- **Expiry in overwrite mode.** After N unconfirmed boots, overwrite has
  nothing to revert to. The loader prints `image unconfirmed` and keeps
  booting the image without further writes. The state is visible to the
  application and to the lab, and a later OTA can replace the image.
- **Images not installed by this loader.** With no `Done` record, an image
  is *permanent*. That covers factory images, images installed by a v1
  loader, and loaders with N = 0. Field devices are never put on trial
  retroactively.
- **When `sensor-sed` confirms.** It confirms on the first APS ACK of an
  attribute report, or as soon as it is joined when reports are not
  acknowledged. If the write fails, it retries at the next proof.
  `OtaLifecycle::confirm_image` defaults to `true` (nothing to confirm)
  for products without a trial log.

**Where the log lives.** In overwrite mode the log sits at the start of
the primary slot's trailer sector. That sector never holds image bytes.
The last 128 bytes stay free for MCUboot trailer fields. On BL702 the
log is `0x86000..0x86F80`. The application's flash view must allow
*writes, never erases* there: `storage.rs` `OtaFlash` admits
`trial_log_area` only.

**Record encoding.** A record is a 16-bit code followed by its bitwise
complement, written as one unit of `max(4, WRITE_SIZE)` bytes.

- Records are appended in order. The first erased unit ends the log.
- An interrupted program only clears bits, and an interrupted erase only
  sets them. Either way a damaged record fails its complement check; it
  can never turn into a different valid record.
- A damaged unit is skipped. After `Done` it counts as a boot attempt, so
  a trial always ends.

### Swap-move with revert (opt-in: feature `swap-move`, `Mode::SwapMove`)

This mode is for CC2340, nRF52 and EFR32MG21/MG22, whose two slots share
one internal flash.

- **The exchange.** The slots are exchanged sector by sector through one
  spare sector, the primary's last. The previous image ends up in the
  secondary slot. Move steps shift the primary up by one sector, then
  swap steps exchange the slots. Every step can be repeated after a power
  cut, and running the exchange again with the same sector count reverts
  it.
- **The status area.** Two sectors outside both slots (`Mode::SwapMove {
  status }`) hold the exchange state.
  - Each starts with a header: "ZRSSWAP1", generation, revert flag and
    sector count, each word followed by its complement. It is written in
    one program operation, so a torn header never validates.
  - A new exchange erases the *other* sector, so the active record is
    never erased.
  - The log after the header holds `Step`, `Done`, `Cleared` and the trial
    records.
- **Revert.** The loader reverts when an unconfirmed install has used up
  its trial boots, or when its copy fails validation while the previous
  image validates. The loader prints `reverting update` and `unconfirmed
  update reverted`, and the reverted image is permanent.
- **Writer protection.** The application's `SlotWriter` refuses to erase
  the secondary slot while the image is on trial, because that slot holds
  the revert image.
- **Cost.** Each exchange costs about three sector writes per image
  sector, and the status area costs two sectors.

| strategy | rollback | extra flash | state |
|---|---|---|---|
| overwrite + trial/confirm (default) | no; unconfirmed images are detected and reported | none (the log is in the existing trailer sector) | host-tested |
| swap-move (opt-in) | yes, automatic on expiry or a broken install | 2 status sectors + 1 spare sector per slot | host-tested; no product uses it yet |
| ping-pong (execute in place from either slot) | yes, natively | none | ESP (IDF `otadata`); BL702 and Telink remap not proven; not implemented |

### Brick fix

**The hazard.** Phase 1 applied the downgrade rule to the primary *header*
alone. Erase only sets bits, so an erase interrupted on the primary header
sector can leave the magic intact while the version reads higher. The
request was then dropped as a "downgrade", and nothing bootable remained.

**The fix.** The downgrade rule now applies only against a primary that
validates fully (hash and signature, using the trusted keys). A primary
that does not validate, whether torn, erased or signed with an unknown
key, never blocks a valid pending update. A flash read error still
aborts the boot rather than being treated as "not a downgrade".

**Tests that pin it:**

- `invalid_primary_never_blocks_a_valid_update` (engine);
- `torn_primary_header_survives_power_loss_and_installs` (engine);
- `torn_primary_header_never_blocks_a_valid_update` (core);
- `torn_primary_install_survives_a_power_cut_at_every_operation` (core).

The last one injects a cut at every flash operation of the torn-header
install, in each of the three cut modes, and requires the next boots to
finish the install.

## Per-chip layouts and field migration

Existing layouts do not change. New layouts are proposals for later phases.
They need the product's post-link checks before any firmware uses them.

| chip / product | today | proposed loader | slots (proposal) | migration |
|---|---|---|---|---|
| BL702 XT-ZB1 | `bl702-boot` at `0x2000..0xFFC0`; loader block `0xFFC0..0x10000` (`LoaderKeys` `0xFFC0`, `LoaderInfo` `0xFFE0`); primary `0x10000..0x87000` (trial log `0x86000..0x86F80`); secondary `0x87000..0xFE000`; journal `0xFE000..0x100000` | v2 loader, overwrite + 3 trial boots, P-256 | unchanged | Re-flash over ISP to adopt it. Slots, trailer, key and `LoaderInfo` stay compatible, and a v1-installed image is permanent. A v2 application on a v1 loader reads one key and never writes the log |
| CC2340R5 | app at `0x0` (504 KiB), journal `0x7E000..0x80000`, `NoOta`; the loader crate exists, but there is no product binary | 16 KiB at `0x0`; overwrite by default, swap-move opt-in | primary `0x4000`, secondary in the upper half, journal unchanged at `0x7E000` | First install over SWD only. It moves the application but keeps the journal, so the device stays commissioned |
| nRF52840/52833 bare | app `0x0..0xFE000`, journal `0xFE000`, `NoOta` | 16 KiB at `0x0` | two equal slots below the journal | SWD |
| nRF52840 UF2 boards | MBR `0x0..0x1000`, app `0x1000..0xF2000`, journal `0xF2000`, UF2 `0xF4000..0xFE000`, MBR params `0xFE000` | zrs loader as the UF2 "application" at `0x1000`; UF2 and MBR untouched | two slots in `0x5000..0xF2000` | Flash the loader and a signed image over UF2. UF2 remains the recovery path |
| PHY6222 / PB-03F | ROM boots the PHY6 header at `0x10000`, app `0x10100..0x3E000`, `NoOta` | the image the ROM boots | slots after the loader | First prove that the ROM boots a plain image (open hardware gate) |
| EFR32MG22 | no bootloader, app at `0x0`, `NoOta` | 16 KiB at `0x0` | internal slots; persistence unchanged | SWD |
| EFR32MG21 | Gecko `0x0..0x4000`, app `0x4000..0x74000`, router tables `0x74000..0x7C000`, persistence `0x7C000..0x80000`, `NoOta` | replaces Gecko in `0x0..0x4000` | internal slots below `0x74000` | SWD for development devices; field devices as for MG1 |
| EFR32MG1 TRADFRI | Gecko `0x0..0x4000`; app `0x4000..0x37000`; journal `0x37000..0x39000`; app NV `0x39000..0x3A000`; NVM3 `0x3A000..0x40000`; OTA staged as GBL in external SPI flash | **out of scope until last**, after a sacrificial-board rehearsal | primary internal, secondary in external SPI flash | High risk; see below |
| ESP32-C6/H2 | IDF second stage, `otadata`, `ota_0`/`ota_1`, `zbnv` | keep IDF | keep | The app-side writer verifies the shared signature before `activate` (later phase) |
| TLSR8258 | ROM boot, NV `0x70000..0x78000`, `NoOta` | keep ROM dual-boot | keep | as ESP |

### EFR32MG1 leaving the Gecko bootloader (deferred)

The user deferred this migration until the last phase, after a rehearsal
on a sacrificial board. The constraints that rehearsal must settle:

- **Risk.** The migration is one-way. The bootloader can only be replaced
  by a GBL bootloader-upgrade payload, or by an application that rewrites
  `0x0..0x4000`. A power cut in that window leaves a device that only SWD
  can recover.
- **Gecko's two stages.** Series 1 splits the Gecko bootloader into a
  first stage and a main stage. Whether the first stage verifies, or must
  keep, the main stage it boots has to be checked against UG266 and on
  hardware.
- **External secondary slot.** The secondary is in external SPI flash, so
  the core needs a split-device `NorFlash` adapter. The Gecko API and
  direct SPI access must never run at the same time.
- **Protected regions.** NVM3, the journal and the application NV must
  stay behind `BootFlash`.

## Size budget

Measured with `llvm-size -A` on release builds, with equal-length worktree
paths. File-backed means `.text + .rodata + .data` plus `.vector_table`,
`.init.rust` and `.loader_info`.

**Loaders**

| loader | region | `32c17d8` | Phase 1 (`0efac9b`) | Phase 2 step 1 | delta vs Phase 1 |
|---|---|---|---|---|---|
| BL702 `bl702-xt-zb1-boot` (P-256) | `0x2000..0xFFC0` (57,280 B) | 14,070 B | 14,202 B | **15,286 B** (`.text` 13,316, `.rodata` 1,360) | **+1,084 B**; `.bss` 1 B (unchanged) |
| BL702, `--features ed25519` | same | n/a | n/a | 41,306 B (`.text` 37,472, `.rodata` 3,224) | +27,104 B |
| CC2340R5 size harness (1 key, 3 trial boots) | 16 KiB | 8,796 B | 8,828 B | **9,396 B** | **+568 B**; RAM unchanged |
| CC2340R5 `swap-move` | 16 KiB | n/a | n/a | 10,848 B | +2,020 B |
| CC2340R5 `ed25519` | 16 KiB | n/a | n/a | 28,540 B | does **not** fit 16 KiB |
| CC2340R5 `ed25519,swap-move` | 16 KiB | n/a | n/a | 29,996 B | does **not** fit 16 KiB |

The BL702 growth breaks down as:

- the boot sequence with trial accounting: about +460 B;
- `BootFlash::write` with sub-unit status programming: about +250 B;
- `Log::append`: about +160 B;
- the validation path with a key set: about +80 B;
- the 64-byte loader block, up from 32 bytes.

Ed25519 costs about 26 KB on RV32 and 19 KB on Cortex-M0+. Most of that
is `ed25519-compact`'s fiat 64-bit-limb field arithmetic: `carry_mul` is
4.4 KB and `carry_square` 3 KB on RV32. Loader stack use with Ed25519 has
not been measured.

**Applications** (the confirm hook and multi-key channel)

| image | flash (file-backed) | RAM (`.data` + `.bss`) | stack |
|---|---|---|---|
| BL702 `bl702-sensor` (SED) | 222,922 → 225,070 B (+2,148) | unchanged | worst-case unchanged; boot chain +96 B (within 5,120), OTA chain −48 B |
| BL702 ED and router images | unchanged | unchanged | unchanged |
| EFR32MG1 sensor | `.text` +136 B | unchanged | not gated |
| nRF52840 sensor | `.text` +228 B | unchanged | not gated |
| CC2340 sensor (no-SDK fallback) | `.text` +48 B | `.bss` −8 B | not gated |

The BL702 loader reserves a 15 KiB stack. A static measurement of the
loader's worst-case stack (P-256 and Ed25519) is still open.

## Testing strategy

1. **Host, simulated flash** (`zrs-boot-core`, feature `sim`):
   - **The flash model.** `SimFlash` models NOR semantics: erase sets
     `0xFF`, programming only clears bits, the real part's erase and
     write alignment is enforced, and `strict()` refuses to reprogram a
     unit that is not erased.
   - **Cut injection.** A power cut can be injected before any erase or
     program, in three modes: `Before` (nothing happens), `Half` (a prefix
     is applied) and `Torn` (noisy bits). After a cut, every operation
     fails until `power_cycle()`.
   - **Replays.** `replay_with_cuts` records a clean boot. It then replays
     the boot once per operation and per cut mode, and checks that:
     - the loader and NV regions are untouched;
     - the next boot finishes the job;
     - the boot after that is a plain boot (or, for trial and swap
       replays, the expected trial or revert state).
   - **Core suites** (`bootloaders/core/src/tests.rs`):
     - install;
     - dropping a rejected request;
     - torn-primary install;
     - trial install;
     - trial expiry and confirm;
     - with `swap-move`: install, revert, and a confirmed install kept;
     - `BootFlash` admitting the status sectors only in swap mode.
   - **Engine suites** (`mcuboot-image`):
     - `upgrade_survives_power_loss_at_every_flash_operation`;
     - `trial_install_survives_power_loss_at_every_operation`;
     - `confirm_survives_power_loss`;
     - the swap-move install and revert at every operation;
     - status records that damage can never forge;
     - multi-key rotation;
     - Ed25519 vectors, with and without the feature.
2. **Host, products and apps**:
   - **BL702 product.** The channel and writer key intersection, a writer
     without a loader trusting no key, and an installed image confirming
     itself through the OTA view. `OtaFlash` admits writes in the trial
     log only.
   - **`sensor-sed` lifecycle.**
     - the first ACK confirms once;
     - a failed confirm retries;
     - unacknowledged reports never confirm;
     - joining confirms when reports are not acknowledged.
   - **`tools/test_bl702_ota.py`** checks the loader block against the
     Rust vector.
3. **Build gates** (CI):
   - `cargo test` and `clippy -D warnings` for the core and both shells,
     host and target, with default features and with `ed25519,swap-move`;
   - `mcuboot-image` with both feature sets;
   - the BL702 product loader, including `--features ed25519`;
   - the CC2340 shell for thumbv6m with `--all-features`;
   - the BL702 sensor static stack gate.
4. **Hardware**, one chip at a time; see the BL702 plan below.

## BL702 hardware re-validation plan

**Nothing below has been run.** Each flashing step needs the user's
approval first.

**Target.** One XT-ZB1 already commissioned to the lab's ZHA network, with:

- the loader UART on GPIO14 at 2 Mbaud;
- ISP over CH340 (`bflb`; BOOT/GPIO28 held at plug-in);
- serial ports `cu.usbserial-14130` and `cu.usbserial-1430`.

### Lab capability (checked read-only)

- hwlab's `device_list` and `capability_registry` show only *fake*
  `PowerSource` devices and no PPK2. **hwlab cannot cut power to the
  BL702 today.**
- hwlab's `uart` (including DTR/RTS line control) and the `bflb` ISP path
  (REAL_HW_VERIFIED for XT-ZB1) are available.

### Steps

1. **Back up the device.** Read the whole 1 MiB flash over ISP: loader,
   slots, journal and the v1 `LoaderInfo`. Keep the file as the recovery
   image.
2. **Build.** Run `build-image.sh` twice: version N (for example 0.2.0) and
   N+1 (0.2.1). Both use the production key and `BL702_BOOT_TRIAL_BOOTS=3`.
   Then run `bl702_ota.py check` on the N+1 `.zigbee` file.
3. **Flash the v2 loader and image N** over ISP (`-full.bin` at `0x0`;
   it covers `0x0..0x87000`, so the journal is kept).
   - **Expected:** `bl702-boot: v2`, then `app v0.2.0+0`, then **no**
     trial lines (a factory-flashed image is permanent).
   - The device rejoins ZHA with its existing network state. That checks
     that the journal survived.
4. **OTA N→N+1 through ZHA.** Serve the file with the zigpy local
   provider (`/config/zigpy_ota/index.json`) and install it from the
   update entity.
   - **Expected UART:** `installing update ……`, `update installed`,
     `trial boot 1`, `app v0.2.1+0`.
   - The device rejoins and its next report is APS-acknowledged. After
     that, a power cycle shows **no** `trial boot` line (confirmed).
   - ZHA shows the new version. The NWK frame counter keeps increasing
     across the reset (check with a sniffer capture).
5. **Negative checks.** Each is one OTA.
   - An image signed with a wrong key is declined by the app channel. If
     it is forced in, the loader rejects it with `bad signature`.
   - An N−1 image is refused as a downgrade.
6. **Power cuts during the copy, 50 or more.**
   - **Arming.** For each cycle, re-arm the request: serve an alternating
     N+1/N+2 pair with distinct builds, or re-stage the same N+1 by
     re-downloading.
   - **Timing.** The copy starts at the `installing update` line. Cut
     power at a random delay of 0–2 s after that line; the copy of a
     ~225 KB image takes roughly 1–2 s, so measure it in the first run.
     Spread the cuts over the erase and the copy, and over the `Done`
     write and the request drop.
   - **Expected after each cut:** the next boot prints `installing
     update` again (or `update installed` if the cut came after the
     copy), then `trial boot 1`, the app version, and a rejoin without a
     new commissioning.
   - **Failure criteria:** no UART output, `no bootable image: …`,
     `halted`, a fresh commissioning, a frame-counter regression, or an
     ISP-only recovery.
   - **Logging.** Record each cycle with its delay and the observed
     lines.
7. **Trial expiry.** Install an image whose app never confirms (a
   test-only build with `report_aps_ack` off and no join, or the network
   powered down).
   - **Expected:** `trial boot 1..3`, then `image unconfirmed` on every
     later boot, and no flash writes. Overwrite has no revert, so this
     checks only that the device stays bootable.
8. **Recovery path.** Restore the step 1 backup over ISP and check that
   the device rejoins.

### What the user must do by hand

- **Approve each flashing step** (steps 3, 4 and 8).
- **Provide power control for step 6.** Choose one of:
  - a USB power relay or switchable hub port that the lab can drive;
  - a PPK2 attached and registered in hwlab;
  - cutting power by hand on a prompted random delay. Manual cuts land
    with roughly 100–300 ms of human jitter, which is acceptable because
    the copy window is longer.
- **Enter ISP mode.** Hold BOOT (GPIO28) while plugging in, for the
  backup (step 1) and for recovery.
- **Confirm the setup.** Check that the XT-ZB1 is the one commissioned to
  ZHA and that the ZHA local OTA provider is configured. Changing
  `extra_providers` needs a Home Assistant restart.

## Open decisions

1. **Ed25519 on 16 KiB loaders.** Either keep Ed25519 BL702-only (or
   large-region-only), or write a compact 32-bit-limb verifier, estimated
   at 4–6 KB.
2. **CC2340 `BootFlash`.** Should the CC2340 shell adopt the slot guard?
   It is required before swap-move is enabled there.
3. **Watchdog.** Should loaders arm one, and with what period? It must
   outlast a full-slot validation.
4. **Trial expiry in overwrite mode.** Keep booting and report it (the
   current behaviour), or additionally refuse further OTA until a confirm?
5. **Swap-move per product.** Which of CC2340, nRF52 and MG21/MG22 turn
   it on, and with how many trial boots.

## Proposed Phase 2 order

1. **Done (host only):** the brick fix, trial/confirm, multi-key,
   optional Ed25519 and opt-in swap-move.
   **Next:** BL702 hardware re-validation (above).
2. **CC2340R5:** a product loader binary (`BootFlash`, overwrite and
   swap-move builds) and a `SlotWriter`-based product OTA path.
   Hardware: SWD install, boot, OTA, power cuts, and a swap-move revert.
3. **nRF52840:** the bare DK layout first, then UF2 coexistence.
   Hardware: both boards.
4. **EFR32MG22** (no bootloader), then **MG21** (replacing Gecko over
   SWD). Hardware: both.
5. **PHY6222**, after proving its ROM boot.
6. **ESP32 and Telink:** app-side verification of the shared signature
   before activation, with no loader change.
7. **EFR32MG1 leaving Gecko:** last, after a sacrificial-board rehearsal.
