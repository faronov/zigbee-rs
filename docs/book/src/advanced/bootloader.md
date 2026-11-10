# Bootloader Core

zigbee-rs uses one pure-Rust bootloader design on every platform. Every
chip runs the same verified boot sequence. Each chip adds only a thin
shell: the flash driver, the entry check, the hand-over and the status
output. Each product owns its layout, its trusted keys, its trial policy and
its loader binary.

This page is both the design and its current status:

- **Phase 1** extracted the shared core from the two existing loaders
  without changing their behaviour.
- **Phase 2 step 1** adds:
  - the brick fix;
  - trial boots with an application confirm;
  - up to three trusted keys;
  - optional Ed25519;
  - opt-in swap-move with revert.
- **Phase 2 step 2** adds the loader watchdog, the CC2340 slot guard and
  the BL702 lab soak.

The BL702 v2 loader passed a hardware run on one XT-ZB1 on 2026-11-08:
OTA through ZHA, confirm-after-rejoin and 46 manual power cuts with no
failure; see [BL702 hardware results](#results-2026-11-08). Every other
chip shell, swap-move, Ed25519 and multi-key rotation remain
**host-tested only**.

Anything marked *later phase* is a proposal. It is not an implemented or
hardware-proven capability.

## Current state

| crate | role | validation |
|---|---|---|
| `mcuboot-image` | Image format, SHA-256, ECDSA P-256 and optional Ed25519 verification, the install engine (`boot`), the status/trial log, opt-in swap-move, `LoaderInfo` and `LoaderKeys`, and the application-side `SlotWriter` (`firmware-writer`) | Host-tested, including power loss at every flash operation |
| `bootloaders/core` (`zrs-boot-core`) | The `Platform` trait and the `launch`/`run` sequence, the `BootFlash` slot guard, the RISC-V fixed-entry and Cortex-M vector-table checks, the console `Reporter`, the LED `Indicator`, and the `sim::SimFlash` power-cut simulator | Host-tested (`--features sim`) |
| `bootloaders/bl702-boot` | BL702 shell: XIP mapping, L1C invalidation, riscv-rt jump, UART lines | Host-tested and release-built. The v2 loader (overwrite, one P-256 key, 3 trial boots, 16 s watchdog) is **lab-tested on one XT-ZB1**: ZHA OTA 0.2.0→0.2.1, confirm-after-rejoin, 168 soak installs and 46 power cuts with no failure ([results](#results-2026-11-08)). Trial expiry (`image unconfirmed`, keeps booting) is lab-tested; wrong-key and downgrade images are refused on hardware by the application before arming ([results](#results-2026-11-0910-refusals-trial-expiry-resume)). The loader's own downgrade and wrong-key rejection are host-tested only |
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
  - Both shells (BL702 and CC2340R5) wrap their driver in `BootFlash`.
- **Page size** is `Layout::sector_size`, the largest erase unit of the two
  slots.
- **Jump to the application.** `prepare` performs the entry check and
  cache invalidation, then `start` jumps. The core never calls `start`
  with anything that `prepare` did not return for the image validated in
  the same boot.
- **Watchdog.** `kick_watchdog` runs before every progress event. The
  engine emits enough events that the longest unkicked step is bounded:
  - `Copying` after every 256-byte chunk (`COPY_CHUNK`);
  - `Busy` before every full-slot validation (`validate_slot`) and before
    every erase step. Slot erases are split into `ERASE_STEP` (64 KiB)
    aligned steps, or one sector per step when the sector size does not
    divide 64 KiB.

  The longest gap is therefore one full-slot validation (SHA-256 over the
  image plus one signature check) or one 64 KiB erase. A host test meters
  the flash I/O between kicks for plain boots, installs, downgrades,
  rejected requests, padded images and swap-move install/revert, and fails
  if any gap exceeds one validation (mutation-checked).

  | loader | config | default | range | after the loader |
  |---|---|---|---|---|
  | BL702 | `BL702_BOOT_WATCHDOG_MS` (layout build script) | 16,000 ms | 0 (off) or 1..=63,999 | disarmed before the jump and before the `halted` loop |
  | CC2340R5 | `Config::watchdog` (`watchdog_ms()`) | product choice; keep `None` until the app feeds it | 1..=131,071,000 ms | **cannot be stopped**; kicked once before the jump, so the app gets a full period |

  - **BL702** uses the TIMER block's watchdog on the 32 kHz clock with
    divider 32 (1,024 ticks/s, 16-bit match). An out-of-range value fails
    the build. The loader prints `watchdog reset` when the previous reset
    came from the watchdog (WSR.WTS, then cleared). Disarming before the
    halt keeps a broken device from reset-looping and wearing flash.
  - **Default 16 s rationale (BL702).** At the ROM's 32 MHz, one full-slot
    validation (software SHA-256 over at most `0x76000` bytes plus P-256)
    is estimated at 2–3 s and a 64 KiB erase at 1–2 s. 16 s gives at
    least 5× margin and covers RC32K tolerance.
    - **Measured on XT-ZB1** (2026-11-08, 238 KB soak image):
      - secondary validation before an install: 1.78–1.81 s (184
        boots);
      - install: 6.40–6.53 s from `installing update` to
        `update installed` (171 installs), so 8.21–8.33 s from
        `bl702-boot: v2` to the jump;
      - the longest gap between printed progress lines: 3.39 s (the
        post-copy check).
    - Even without the per-event kicks, the whole install fits the 16 s
      period about 1.9×, and the longest measured gap fits about 4.7×.
  - **CC2340R5** uses the CKMD watchdog (`WDTCNT`, stall on debug halt).
    The current `cc2340-sensor` does not feed a watchdog, so a product
    that arms it in the loader would reset its application every period.
    Products keep `watchdog: None` until the application feeds it.
  - If the BL702 32 kHz clock does not run, the watchdog never fires
    (fails open). Whether WSR.WTS survives a watchdog reset is unverified.
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
  large; see the [size budget](#size-budget). It is supported **only on
  loaders with a large region (BL702, 56 KiB)**. On 16 KiB loaders the
  feature is a build error: `cc2340r5-boot --features ed25519` fails with
  `compile_error!` ("Ed25519 is not supported on CC2340R5 … P-256 only"),
  and CI checks that it keeps failing. There is no compact verifier.
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

**Phase 2 step 2: loader watchdog and the CC2340 slot guard.** The deltas
are against Phase 2 step 1. The CC2340 rows are the size harness; there is
no product loader yet. Its step 1 column was re-measured from `5e29b18` in
the same harness; the step 1 table above differs from it by 4 to 28 bytes.

| loader | step 1 | step 2 | delta |
|---|---|---|---|
| BL702, watchdog armed (default 16,000 ms) | 15,286 B | **15,834 B** (`.text` 13,808, `.rodata` 1,416) | **+548 B** |
| BL702, `BL702_BOOT_WATCHDOG_MS=0` | 15,286 B | 15,674 B | +388 B |
| BL702, `ed25519` + watchdog | 41,306 B | 41,850 B | +544 B (fits 57,280) |
| CC2340 overwrite, watchdog `None` | 9,400 B | 9,880 B | +480 B (about 124 B is `BootFlash`) |
| CC2340 overwrite, watchdog armed | 9,400 B | 9,952 B | +552 B |
| CC2340 `swap-move`, watchdog `None` | 10,876 B | 11,332 B | +456 B |
| CC2340 `swap-move`, watchdog armed | 10,876 B | 11,400 B | +524 B |
| CC2340 `ed25519` | 28,540 B | build error | n/a |

- **RAM.** BL702 loader `.data` (572 B) and `.bss` (1 B) are unchanged.
- **Disarmed growth.** Even with the watchdog compiled out or set to
  `None`, the images grow. The cause is the engine's `Busy` events and
  64 KiB erase steps, and on CC2340 the `BootFlash` monomorphization
  (`validate_slot<BootFlash>` and `run<BootFlash>`).
- **Applications.** `bl702-sensor` (225,070 B), `bl702-ed` (191,918 B) and
  `bl702-router` (311,950 B) are unchanged in flash and RAM. The
  sensor's stack gates still pass.
- **Lab soak.** The lab-only `bl702-ota-soak` is 237,546 B file-backed,
  and its `.bss` is 18 B. CI builds and lints it but never publishes it.

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
   - the CC2340 shell for thumbv6m, with default features and with
     `swap-move`;
   - a negative build: CC2340 with `--features ed25519` must fail with
     "Ed25519 is not supported on CC2340R5";
   - the BL702 sensor static stack gate;
   - the lab-only `bl702-ota-soak` build and clippy, and the
     `tools/test_bl702_soak_monitor.py` classifier tests.
4. **Watchdog gap meter** (core tests). A simulated watchdog counts flash
   operations between `Busy` events. The replays fail if any validation
   or erase step runs without a kick in front of it. This was
   mutation-checked: deleting either kick fails the test.
5. **Hardware**, one chip at a time; see the BL702 plan below.

## BL702 hardware re-validation plan

**Run on 2026-11-08; see [Results](#results-2026-11-08).** Each
flashing step needed the user's approval first, and every future run
does too. The plan below is kept as it was executed.

### Device and lab facts (checked read-only)

- **Device.** ZHA's XT-ZB1 is IEEE `7c:b9:4c:61:98:b9:00:00`, NWK
  `0xE8E2`. It runs the **router** role image:
  - model `XT-ZB1-Router`, mains-powered;
  - endpoint 1 with only Basic and Identify, and **no OTA client
    cluster**;
  - Basic `sw_build_id` `0.1.0`, date code `20260402`.
- **Which commit it runs.** `SW_BUILD` is the crate version, not a git
  hash, so Zigbee and the UART banner cannot prove commit `bff2377`. The
  device is only consistent with it. Proof needs an ISP readback compared
  with a local `bff2377` router build.
- **On-flash state.**
  - The newest ISP backup predates the router flash. It shows the **v1**
    loader (one key, `LoaderInfo` at `0xFFE0`).
  - The router keeps its child and APS journals at
    `0xF9000..0xFD000`, **inside the secondary slot**.
  - The security journal at `0xFE000` is shared by every role.
  - Moving to the sensor role keeps only the outgoing frame-counter
    limits, so **the sensor needs one fresh commissioning**. It steers by
    itself when factory-new, with a 15 s retry that backs off to 15 min.
- **OTA provider.** ZHA has a local provider configured:
  `zigpy_config.ota.extra_providers` of type `advanced`, path
  `/config/zigpy_ota`, with `zigpy.ota` debug logging.
  - The stale `1234-0702-25000101-bl702-xt-zb1-diagnostic-0.1.1.zigbee`
    was moved to `/config/zigpy_ota_backup/`. The backup folder is a
    sibling, not a subfolder: the provider scans its path recursively.
  - zigpy caches the provider index for 24 h
    (`INDEX_EXPIRATION_TIME`). After adding a file, reload the ZHA
    config entry, or the new file may not be offered until the cache
    expires.
  - No XT-ZB1 update entity exists yet, because the router has no OTA
    client.
- **hwlab.**
  - Only *fake* `PowerSource` devices are registered, and there is no
    PPK2. **Power cuts are manual.**
- **Cabling.** The XT-ZB1 has two micro-USB connectors.
  - The CH340 USB-UART (`cu.usbserial-1430` or `-14130`) is the **only**
    power path in normal operation. It also carries the loader and app
    UART (GPIO14 TX, GPIO15 RX, 2 Mbaud). A power cut therefore means
    unplugging the USB-UART, and the serial port disappears with it.
  - The right micro-USB is the BL702's own USB. ISP (`bflb`) runs there,
    with BOOT (GPIO28) held at plug-in, while the ROM enumerates as a
    CDC device.
  - Each ISP session swaps cables: unplug the USB-UART, then hold BOOT
    and plug in the chip USB. Afterwards, unplug the chip USB and plug
    the USB-UART back into **the same host port**. macOS names
    `cu.usbserial-*` by USB location.

### Images

All images are signed with the production key, with
`BL702_BOOT_TRIAL_BOOTS=3` and the default 16 s loader watchdog. The
versions are TEST-ONLY overrides, so none of them may be published.

| image | build | purpose |
|---|---|---|
| N = 0.2.0 sensor | `BL702_OTA_VERSION_OVERRIDE=0.2.0 ./build-image.sh` | ISP #1, `-full.bin` |
| N+1 = 0.2.1 sensor | `…=0.2.1` | OTA through ZHA |
| soak 0.2.2 | `BL702_LAB_SOAK=1 …=0.2.2` | OTA through ZHA, then the loop |
| final 0.2.3 sensor | `…=0.2.3` | ISP #2, `-full.bin` |

The final image must be **newer than the soak**. A soak request may
still be staged when the run ends; against 0.2.3 it is refused as a
downgrade and dropped, instead of re-installing the soak.

### The loader soak (`bl702-ota-soak`, lab only)

- **The loop.** On every boot the soak:
  1. confirms itself;
  2. prints CRC-32s of the loader region (`0x0..0x10000`) and the
     security journal (`0xFE000..0x100000`);
  3. waits one second;
  4. re-stages its own signed image into the secondary slot through the
     product `SlotWriter`;
  5. requests an install and resets.

  The loader then erases the primary and copies the image back. An equal
  version is not a downgrade.
- **Why it exists.** Every boot is a full install, so a cut at any moment
  lands in the loader's validation, erase or copy, or in the app's
  re-staging. The user does not have to time anything.
- **Size.** A padding blob makes the image about as large as the sensor
  (237,958 B signed), so the copy time is realistic.
- **Monitoring.** `tools/bl702_soak_monitor.py` reads the UART and
  classifies every reset.
  - The power cut removes the USB-UART, so the monitor treats a
    vanished port as the cut, not as silence. It polls for the port to
    re-enumerate and reopens it with DTR and RTS deasserted (RTS drives
    the XT-ZB1 reset).
  - Each cut is classified by the phase it interrupted: `cut-boot`,
    `cut-loader`, `cut-copy`, `cut-install`, `cut-app`, `cut-staging`
    or `cut-rearmed`.
  - `cycle`: the soak re-armed and reset, with no cut.
  - `reset-<phase>`: any other banner without a port loss. It is
    reported but is not a failure.
- **Failure criteria.** The monitor fails the run on:
  - `soak: FAIL`;
  - any loader error (`no bootable image`, `installed copy corrupt`,
    `flash error`, `update rejected`, `halted`);
  - a loader watchdog reset;
  - a changed loader or journal CRC;
  - no `soak: crc` line within 60 s after the port returns (a possible
    brick);
  - no UART output for 30 s while the port is present.
- **Pass criteria.** At least 50 cuts, including cuts during the copy.

### Steps

1. **ISP #1 (user holds BOOT).**
   - Read back the whole 1 MiB flash. This is the recovery image, and it
     also identifies the router build.
   - Program the N `-full.bin` (`0x0..0x87000`: the v2 loader, its key
     block and the sensor).
   - Blank the secondary slot `0x87000..0xFE000` with `0xFF`. This
     removes the router journals and any stale request.
   - Leave `0xFE000..0x100000` (the journal) untouched.
   - **Expected:** `bl702-boot: v2`, then `app v0.2.0+0`, and no trial
     lines.
2. **Commission.** The agent opens permit-join in ZHA (the "XT-ZB1"
   device entry changes from the router model to the sensor). The sensor
   joins by itself.
3. **OTA N→N+1 through ZHA.**
   - Put the 0.2.1 file in `/config/zigpy_ota`, reload the ZHA config
     entry so zigpy re-reads the index, wait for the first query (about
     300 s after boot), and install it from the update entity.
   - **Expected:** `installing update`, `update installed`,
     `trial boot 1`, `app v0.2.1+0`, then a rejoin and an APS-acknowledged
     report.
   - Power-cycling once afterwards shows no `trial boot` line.
4. **Serve the soak (0.2.2) through ZHA.** This reuses the same OTA path
   with no extra ISP. From then on the board cycles by itself, about
   once every 3–5 s (to be measured). The agent runs
   `tools/bl702_soak_monitor.py --port … --log …`.
5. **Power cuts (one user session).** The user unplugs and replugs at
   random moments until the agent says stop; see below. The agent stops
   at ≥ 50 cuts with some during the copy, or at the first failure.
6. **ISP #2 (user holds BOOT).**
   - Read back and compare the loader region and the journal with the
     step 1 readback. They must match the soak's CRC line.
   - Program the final 0.2.3 `-full.bin`.
   - **Expected:**
     - one `update rejected: downgrade`, if a soak request was staged;
     - then `app v0.2.3+0`;
     - the sensor resumes on the network **without** commissioning;
     - its frame counter keeps increasing (sniffer).
7. **Negative and trial checks** (optional, through ZHA):
   - a wrong-key image is declined;
   - an N−1 image is refused;
   - an image that never confirms boots `trial boot 1..3`, then
     `image unconfirmed`.
8. **Recovery** (only if something fails): restore the step 1 readback
   over ISP.

The UART timestamps from step 5 gave the real validation and copy times
and the watchdog margin; see [the watchdog notes](#core-api) and the
results below.

### What the user must do by hand

- **Before the run:**
  - Confirmed: the CH340 USB-UART is the single power path.
  - Approve each flash, moving the stale OTA file, and replacing the
    router with the sensor in ZHA.
- **ISP #1 and ISP #2:**
  1. unplug the USB-UART;
  2. hold BOOT (GPIO28) while plugging in the right (chip) micro-USB,
     then release it;
  3. wait while the agent drives ISP;
  4. when told, unplug the chip USB and plug the USB-UART back into the
     same host port.
- **Steps 2–4:** nothing.
- **Step 5, one session:**
  - unplug the USB-UART at random moments, never in a rhythm;
  - wait about 2 s with the board unpowered;
  - plug it back into the same host port;
  - vary the time powered between about 0.5 s and 6 s;
  - repeat until the agent says **stop**.

  Each cut takes about 5–10 s, so 50+ cuts take about 5–10 minutes. The
  agent reports the count as it goes, and says **stop at once** if it
  sees a stall or failure.

### Results (2026-11-08)

One XT-ZB1 (BL702, MAC `0000b998614cb97c`, ZHA NWK `0xE8E2`), the v2
loader built from this branch: overwrite, one P-256 production key, 3
trial boots, 16 s watchdog. **Lab-tested on one board**; this is not a
production rollout.

| step | result |
|---|---|
| ISP #1 | Full-flash readback kept as the recovery image. The N `-full.bin` was programmed and verified, slot 2 was blanked, and the journal was left untouched. First boot: `bl702-boot: v2`, then `app v0.2.0+0`, with no trial lines |
| Commission | The sensor replaced the router entry at `0xE8E2` with one re-pair, as planned |
| OTA 0.2.0→0.2.1 through ZHA | `installing update` to `update installed` took 6.10 s, and 7.86 s from `v2` to the jump. Then `trial boot 1`, `app v0.2.1+0`, rejoin and confirm. An RTS reset afterwards booted with **no** `trial boot` line, so the confirm persisted |
| Soak 0.2.2 through ZHA | Installed by the loader, and then cycled every 16.3 s by itself |
| Power cuts | Two manual sessions: **46 verified cuts, 0 failures** (accepted by the user; see below) |
| ISP #2 | Readback compared offline with the soak image. 0.2.3 `-full.bin` programmed and verified. Slot 2 and the journal were read back unchanged |
| Final boot | `bl702-boot: v2`, then `app v0.2.3+0`; the unarmed soak image in slot 2 was ignored (see below). Same ZHA device and NWK, with **no re-pair**. Reports resumed, and the first `QueryNextImage` carried `current_file_version 0x25000203`, which ZHA's update entity shows as installed |

**Cut tally.** The monitor classifies each cut by the phase the board
was in when the port vanished:

| phase | cuts |
|---|---|
| loader validation (`cut-loader`) | 4 |
| loader erase/copy (`cut-copy`/`cut-install`) | 13 |
| application start (`cut-app`) | 2 |
| app re-staging slot 2 (`cut-staging`) | 27 |
| **total** | **46** |

- Replaying the timestamps puts 15 cuts inside the loader's install
  window: 13 during erase/copy and 2 in the post-copy check.
- Over the whole soak (199 boots), 168 installs completed. Every boot
  printed the same CRCs: loader region `08eab8f8`, security journal
  `bf22f1b1`.
- There were no `update rejected`, `no bootable image`,
  `installed copy corrupt` or `flash error` lines, no `trial boot 2`, and
  no watchdog reset.
- The user counted about 60 cuts. Quick re-cuts, where the board was
  powered for less than the host needs to re-enumerate the port, merge
  into one host-visible loss, so only 46 are counted.
- The ISP #2 readback agrees: the loader is byte-identical to the
  programmed images, and the journal CRC matches. The primary equals the
  signed soak image except 9 trial/confirm bytes in its trailer sector
  (`0x86000`).

**Timing.** These are the figures in
[the watchdog notes](#core-api). A boot with no request validates the
primary in 1.55–1.56 s.

**What this run did not exercise on hardware:**

- **Downgrade rejection.** The power was last removed while the soak was
  re-staging slot 2: the image was complete but the request magic was
  not yet written. The loader therefore saw no request and ignored slot
  2, so `update rejected: downgrade` never ran. This run proves only "an
  unarmed slot 2 is ignored". The downgrade path remains host-tested.
- Wrong-key refusal, trial expiry (`image unconfirmed`), multi-key
  rotation, Ed25519, swap-move, and watchdog expiry on hardware (step 7
  was skipped).
- Frame-counter continuity on a sniffer. The journal CRC is unchanged
  and the sensor rejoined without commissioning, but no capture was taken.

**Lessons for the next run:**

- **Opening the CH340 port resets the BL702.** The monitor has to open
  the port with DTR and RTS deasserted, and every open is a reset that
  shows up in the log.
- **zigpy caches the provider index.** After adding a file, reload the
  ZHA config entry. After an HA restart the index is re-read.
- **One unexplained stall.** The first soak `update.install` stopped at
  about 51 % when zigpy got no block request for 30 s. The cause is
  unknown (HA was busy, and debug logging was off), and a retry
  completed.
- **ZHA cannot reach an idle sensor.** `update.install` sends a unicast
  `image_notify`. The sensor's 10 s slow poll is longer than the parent's
  indirect hold time (about 7.7 s), so the notify gets no APS ack and the
  install fails with `FAILURE`.
  - The workaround was to reset the board and press install within the
    120 s fast-poll window after a restore.
  - This is a product poll-policy limitation, not a loader issue. The
    10 s poll is kept; the documented procedure is to start the install
    within the 120 s fast-poll window after a reset or rejoin (see
    [Reaching a sleepy device](./ota.md#reaching-a-sleepy-device)).
- **Throughput.** OTA ran at about 77–120 B/s (48 B blocks through
  parent `0x43FD`), so a 238 KB image took about 33 minutes.

### Results (2026-11-09/10): refusals, trial expiry, resume

These runs used the same XT-ZB1 (`0xE8E2`, parent `0x43FD`), the v2 loader
and test-only diagnostic images. Every image went out through ZHA
`update.install`, started inside the 120 s fast-poll window after a reset.
No ISP session was used.

- **Install, trial and confirm.** 0.2.4 and 0.2.5 installed with
  `installing update`, `update installed`, `trial boot 1`, the new
  `app v…`, and then `[ota] image confirmed`. The device kept its NWK
  address with no re-pairing.
- **Confirm survives a power cut.** After a USB-UART unplug of about 5 s,
  0.2.5 booted with no trial line.
- **Unknown key refused (app level).** A 0x25000206 OTA file was signed
  with a throwaway P-256 key. It downloaded completely, then
  `[OTA] Verify failed: VerifyFailed` and `[ota] failed`. No request was
  armed, and the device stayed on 0.2.5.
- **Disguised downgrade refused (app level).** The signed 0.2.4 MCUboot
  image was wrapped in an OTA file version 0x25000207, which is newer than
  the running 0x25000205 at the Zigbee level, so ZHA offered it. The
  download was interrupted by a power cut and resumed. After completion it
  gave `Verify failed` and `[ota] failed`, and the device stayed on 0.2.5.
  The product `SlotWriter` compares MCUboot versions before arming.
- **Resume after reset or power cut.** See
  [Resume after a reset](./ota.md#resume-after-a-reset).
- **Trial expiry.** A lab-only 0.2.8 built with `lab-no-confirm`
  (`BL702_LAB_NO_CONFIRM=1`) never confirms itself. Installed through
  ZHA, it booted with `trial boot 1` and then `[ota] LAB: image confirm
  skipped`. Three RTS resets gave `trial boot 2`, `trial boot 3` and then
  `image unconfirmed`, as did every later reset. In overwrite mode the
  loader kept booting 0.2.8, which rejoined as `0xE8E2` each time. A
  normal 0.2.9 then installed over it and confirmed.

The loader's own `update rejected: downgrade` and unknown-key paths are
still host-tested only: the app refuses those images before it arms a
request, so OTA cannot reach them. A loader-level check needs an ISP
session to arm slot 2 by hand, and that was declined. The NV-journal
power-cut soak was also declined. Each frame-counter reservation covers
1024 counter values (`FRAME_COUNTER_RESERVATION_SIZE`), so an OTA writes
only a handful of journal records.

**Radio.** From about 22:24 on 2026-11-09, bursts of MAC NoAck and route
errors via parent `0x43FD` (RSSI about −83 dBm) aborted downloads at the
server three times, which is the environment, not the firmware. The retries
completed.

**ZHA note.** After a config-entry reload the update entity shows no
`latest_version` until the device queries again. Calling
`homeassistant.update_entity` on it re-reads the provider index at once.

## Open decisions

Resolved in Phase 2 step 2:

- Ed25519 is BL702-only; on CC2340 it is a build error.
- CC2340 uses `BootFlash`.
- Loaders arm a watchdog (BL702 default 16 s).
- An expired trial in overwrite mode keeps booting and is reported.

Still open:

1. **CC2340 watchdog default.** The CC2340 WDT cannot be stopped once
   armed, and `cc2340-sensor` does not feed it. Product configs therefore
   keep `watchdog: None` until the app feeds the watchdog; then they arm
   it with a period that covers a full-slot validation.
2. **Swap-move per product.** Which of CC2340, nRF52 and MG21/MG22 turn
   it on, and with how many trial boots.
3. **XT-ZB1 slow poll versus OTA reachability.** The 10 s poll is kept
   (product decision); an idle sensor is reachable for OTA only in the
   120 s fast-poll window after a reset or rejoin (see
   [Reaching a sleepy device](./ota.md#reaching-a-sleepy-device)).

The BL702 run approvals (the soak image, the router-to-sensor swap,
moving the stale OTA file and both ISP sessions) were given and used on
2026-11-08.

## Proposed Phase 2 order

1. **Done (host only):**
   - the brick fix, trial/confirm, multi-key, optional Ed25519 and opt-in
     swap-move;
   - the loader watchdog, the CC2340 `BootFlash` guard and the lab soak.

   **Done (lab, one board):** the BL702 hardware run
   ([results](#results-2026-11-08)). Still open on BL702 hardware: a
   downgrade rejection, wrong-key refusal and trial expiry.
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
