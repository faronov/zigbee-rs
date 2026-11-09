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
- **Phase 3** ports the loader to more chips, with layouts read from
  product files: nRF52840 (bare DK and three UF2 boards), EFR32 Series 2
  (MG22, MG21, ZBT-1 chained above Gecko) and PHY62x2 (PB-03F). The
  nRF52840 and EFR32 loaders arm a watchdog, and the nRF52840 and
  EFR32MG21/MG22 applications have a `zrs-boot` link variant. ESP32
  gets an application-side signature and version check; its ROM and IDF
  boot stay. Telink has the configuration only. See
  [Phase 3](#phase-3-more-chips).

The BL702 v2 loader passed a hardware run on one XT-ZB1 on 2026-11-08:
OTA through ZHA, confirm-after-rejoin and 46 manual power cuts with no
failure; see [BL702 hardware results](#results-2026-11-08). Every other
chip shell, swap-move, Ed25519 and multi-key rotation remain
**host-tested only**. Of the Phase 3 loaders, only the EFR32MG22 loader has
run on hardware. It is **lab-tested on one BRD4183A** (2026-11-09): boot,
two upgrades, trial expiry, downgrade and wrong-key refusal, and journal
retention in ZHA; see [MG22 hardware results](#mg22-results-2026-11-09).
The other Phase 3 loaders are **built only**.

Anything marked *later phase* is a proposal. It is not an implemented or
hardware-proven capability.

## Current state

| crate | role | validation |
|---|---|---|
| `mcuboot-image` | Image format, SHA-256, ECDSA P-256 and optional Ed25519 verification, the install engine (`boot`), the status/trial log, opt-in swap-move, `LoaderInfo` and `LoaderKeys`, and the application-side `SlotWriter` (`firmware-writer`) | Host-tested, including power loss at every flash operation |
| `bootloaders/core` (`zrs-boot-core`) | The `Platform` trait and the `launch`/`run` sequence, the `BootFlash` slot guard, the RISC-V fixed-entry and Cortex-M vector-table checks, the console `Reporter`, the LED `Indicator`, and the `sim::SimFlash` power-cut simulator | Host-tested (`--features sim`) |
| `bootloaders/bl702-boot` | BL702 shell: XIP mapping, L1C invalidation, riscv-rt jump, UART lines | Host-tested and release-built. The v2 loader (overwrite, one P-256 key, 3 trial boots, 16 s watchdog) is **lab-tested on one XT-ZB1**: ZHA OTA 0.2.0→0.2.1, confirm-after-rejoin, 168 soak installs and 46 power cuts with no failure ([results](#results-2026-11-08)). Downgrade rejection, wrong-key refusal and trial expiry are not yet exercised on hardware |
| `bootloaders/cc2340r5-boot` | CC2340R5 shell: vector-table check, VTOR/MSP jump, fault blinks, SHUTDOWN | Host-tested; thumbv6m build only. No product loader binary exists |
| `zigbee-runtime` `FirmwareWriter`/`OtaLifecycle` | `confirm_image` hook; `FirmwareWriter::confirm_image` | Host-tested |
| `apps/sensor-sed` | Confirms the running image on its first APS-acknowledged report, or once joined when reports are not acknowledged | Host-tested |
| `bootloaders/layout` (`zrs-boot-layout`) | Build-time parser and checker of product `boot/*.layout` and `boot/app-verify.conf` files; emits `BootConfig`, keys, loader and application `MEMORY` regions | Host-tested |
| `bootloaders/cortex-m-boot` (`zrs-boot-cortex-m`) | Cortex-M shell: entry check through the flash driver, VTOR/MSP hand-over, `Hooks` for status, watchdog and failure | Host-tested; built in the three loaders below |
| `products/nrf52840-sensor/bootloader` | nRF52840 loader, one build per board layout | Release-built and linted; **not run on hardware** |
| `bootloaders/efr32s2-boot` | EFR32MG22/MG21 loader; on MG21 chained above the Gecko Bootloader | Release-built and linted. The MG22 devkit build is **lab-tested on one BRD4183A** ([results](#mg22-results-2026-11-09)). MG21, ZBT-1 and `gecko-recovery` are **not run on hardware** |
| `efr32s2-hal::watchdog` | EFR32 Series 2 WDOG0 driver (period rounding, arm/kick/disarm) | Host-tested (period maths). On one MG22 it was seen armed while the loader ran and disarmed in the application. Its period and an actual expiry reset are **not measured** |
| `bootloaders/phy62x2-boot` | PHY62x2 loader started by the boot ROM | Release-built, linted, ROM-boot image generated; **not run on hardware** |
| `mcuboot-image::envelope` | Application-side verification of a signed MCUboot envelope (`EnvelopeWriter`) for chips whose ROM/SDK boot stays | Host-tested with imgtool vectors; ESP32 product tests |

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
  | nRF52840 | `watchdog-ms` in `boot/*.layout` (`<PREFIX>_BOOT_WATCHDOG_MS`) | 8,000 ms | 0 (off) or 1..=131,071,999 | **cannot be stopped**; reloaded before the jump; the `zrs-boot` app adopts it (`NrfWatchdogSupervisor`) |
  | EFR32 Series 2 | `watchdog-ms` in `boot/*.layout` (`<PREFIX>_BOOT_WATCHDOG_MS`) | 8,000 ms | 0 (off) or 1..=238,313 | stopped before the jump and before the halt; the app needs nothing |
  | PHY62x2 | none | 0 | 0 only | no loader watchdog |

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
  - **nRF52840** uses the WDT on the 32.768 kHz LFCLK
    (CRV = ⌈ms·32768/1000⌉, reload register `RR[0]` only), with
    SLEEP and HALT set to **pause**. Consequences:
    - Once started it runs until a pin, power-on, brown-out or watchdog
      reset; a soft reset (OTA, `sys_reset`) keeps it running. A loader
      that finds it already running keeps its configuration and only
      reloads it.
    - The `zrs-boot` application must feed it. `NrfWatchdogSupervisor`
      adopts it right after `embassy_nrf::init`: it writes every reload
      register on every heartbeat (so it also feeds a WDT started with
      another register set), and limits the sleep length to half
      the period only if the WDT counts in sleep. With the loader's
      pause-in-sleep setting a sleepy device is never woken by it, but a
      hang inside `WFE` is not caught either.
    - With nothing bootable the loader halts in `WFI`; the paused WDT
      never fires, so it does not reset-loop.
    - The direct-map (non-`zrs-boot`) images never see a running WDT.
    - **Unverified:** a soft reset into the Adafruit UF2 bootloader with
      the WDT running (UF2 boards). If UF2 does not feed it, DFU mode
      resets after one period; recovery then is a double-tap reset, which
      is a pin reset and stops the WDT.
  - **EFR32 Series 2** uses WDOG0 (`efr32s2-hal::watchdog`) on the
    ULFRCO. The period rounds up to the next `PERSEL` step that is at
    least the requested time at an assumed worst-case ULFRCO of 1.1 kHz
    (8 s → `PERSEL` 11: ≥14.9 s, nominally 16.4 s). The 1.1 kHz bound is
    an assumption that is not yet measured. WDOG0 is disabled before the
    jump, so the application does not have to feed it, and before the
    halt or the `gecko-recovery` reset. On one MG22, `WDOG0_EN` read 1
    while the loader ran and 0 in the application. A whole install plus
    application start took at most about 5 s, a third of the minimum
    period ([results](#mg22-results-2026-11-09)).
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

Existing direct-link layouts do not change. Phase 3 adds loader layouts as
product files (see [Layout files](#layout-files)); the table below is the
pre-Phase 3 plan. The implemented maps are in
[Phase 3 layouts](#phase-3-layouts).

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

## Phase 3: more chips

### Layout files

Each loader deployment is one text file in its product, by convention
`products/<product>/boot/<deployment>.layout`. The loader's build script and
the application's build script both read it through `zrs-boot-layout`, so the
loader region, the slots, the journal and every protected region are written
once. The format (keys, units and defaults) is documented in the crate. In
short:

```text
name      = nrf52840-mdk-uf2
flash     = 0x0..0x100000
sector    = 0x1000
reserved  = 0x0..0x1000 mbr            # repeatable, labelled, never written
loader    = 0x1000..0x5000             # last 64 B: LoaderKeys + LoaderInfo
primary   = 0x5000..0x7B000
secondary = 0x7B000..0xF1000
journal   = 0xF2000..0xF4000           # product persistence, never written
ram       = 0x20000000..0x20040000
header    = 0x200                      # imgtool --header-size
trial-boots = 3                        # <PREFIX>_BOOT_TRIAL_BOOTS overrides
watchdog-ms = 0                        # <PREFIX>_BOOT_WATCHDOG_MS overrides
mode      = overwrite
env-prefix = NRF52840_MDK
key-file  = ../../../bootloaders/keys/own-ec-p256.xy.hex
```

The build refuses, with a message naming the file:

- overlapping regions, a protected region between the slots, unequal or
  misaligned slots, a slot or journal not on erase boundaries;
- a vector table that `VTOR` cannot address (`vector-align`), or a header
  size other than the image's;
- a loader region too small for its 64-byte block, and a linked loader
  that overflows its region (post-link `ASSERT`). The 16 KiB regions
  therefore reject Ed25519 with an explanation;
- a watchdog period the chip cannot provide;
- trusted keys that are missing or malformed. Keys come from
  `<PREFIX>_BOOT_KEYn` environment variables or the `key-file`; private
  keys are never read.

The repository key `bootloaders/keys/own-ec-p256.xy.hex` is a
**development key**. Production images need product keys passed through the
environment.

### Cortex-M loaders

`zrs-boot-cortex-m` is the shared shell. One boot:

1. installs a pending, valid, non-downgrade image and validates the
   primary, inside `BootFlash`;
2. checks the entry: the header has exactly the configured size, and the
   initial SP and reset vector, read through the flash driver, point into
   SRAM and into the image;
3. masks interrupts, sets `VTOR` where the core has one, loads `MSP` and
   jumps.

Each chip loader only composes its flash driver, its generated `Config` and
its `Hooks`. None of them configures clocks, GPIO or the radio. The nRF52840
and EFR32 Series 2 loaders arm their watchdog for the boot decision (see
[the watchdog notes](#core-api)); PHY62x2 has no loader watchdog.

With nothing bootable, the loaders halt in `WFI` (recovery over SWD), apart
from the EFR32 `gecko-recovery` option below.

**nRF52840** (`products/nrf52840-sensor/bootloader`, features `dk`,
`mdk-uf2`, `pca10059-uf2`, `promicro-uf2`):

- The bare DK loader sits at `0x0`.
- On the UF2 boards the loader is the "application" that the MBR and the
  Adafruit UF2 bootloader start. The MBR, the UF2 bootloader and its
  settings are reserved and never written, so UF2 remains the recovery
  path.
- On ProMicro the S140 SoftDevice region is reserved as well.
- The application gets a link variant, `examples/nrf52840-sensor-uf2`
  feature `zrs-boot`. It links into the selected layout's primary slot
  after the 0x200-byte header, and its build checks that the security
  journal stays where the direct map has it. The direct map remains the
  default and is unchanged. Under `zrs-boot` the application adopts the
  loader's WDT (`NrfWatchdogSupervisor`); the default build keeps
  `NrfSupervisor`.

**EFR32 Series 2** (`bootloaders/efr32s2-boot`, features
`efr32mg22-devkit`, `efr32mg21-devkit`, `ha-connect-zbt1`):

- **MG22** has no resident bootloader. The Secure Engine starts the
  vector table at `0x0`, so the loader lives there.
- **MG21 and ZBT-1 are chained.** The resident Gecko Bootloader at
  `0x0..0x4000` is reserved and never written. Gecko starts whatever is at
  `0x4000`, which is the zigbee-rs loader.
- **`gecko-recovery`.** With this feature, when nothing is bootable the
  loader writes the BOOTLOAD reset cause and resets into Gecko's UART
  recovery. Gecko then rewrites everything from `0x4000` up. The build
  refuses this feature unless Gecko is reserved below the loader. It is
  **not verified on hardware**.
- **Gecko secure boot.** If a device's Gecko Bootloader enforces it, Gecko
  would refuse to start the unsigned loader. This must be read from the
  device before any install.
- **Application link variant.** `examples/efr32mg21-sensor` and
  `examples/efr32mg22-sensor` have a feature `zrs-boot`. `build.rs` loads
  `products/<product>/boot/devkit.layout`, links the image into the
  primary slot after the 0x200-byte header, checks that the layout's
  journal is the direct map's `PERSISTENCE`, and keeps the direct map's
  stack and RAM `ASSERT`s. The default build still uses the product
  `link/memory.x` unchanged. Roles:
  - `sed` and `ed` build for both chips (CI);
  - MG22 `router` is refused by `build.rs`, because the layout hands the
    router tables to the slots (the router does not fit MG22's 32 KiB SRAM
    in any case);
  - MG21 `router` (355,680 B) does not fit the 252,672 B cap; the link
    fails.
  ZBT-1 (`examples/ha-connect-zbt1`) has no `zrs-boot` variant yet.

**PHY62x2** (`bootloaders/phy62x2-boot`, feature `pb03f-kit`):

- **Start.** The boot ROM loads the loader through its own ROM-boot table
  at `0x2000`. The jump table and the run descriptor are SRAM load data at
  `0x5000`. The loader's code runs XIP from `0x6000..0xA000`.
- **No VTOR.** The Cortex-M0 has no `VTOR`, so the image ABI is:
  `[SP, Reset]` at the start of the image, followed directly by the
  application's 0x400-byte ROM jump table. The loader copies that table to
  `0x1fff0000` before the hand-over, after checking that it lies inside the
  image.
- **Address.** Applications run at `0x11010100`, the same address as a
  direct ROM-boot image.
- **Application cap.** The application is capped at 200,448 B
  (`app-max = 0x30F00`), the XIP application slot of the PB-03F direct
  ROM-boot map (`products/pb03f-kit/link/memory.x`); a layout test checks
  that the cap and the run address agree with that map. Each slot is
  0x33000: the cap plus the 0x100 header, the 0x300 TLV reserve and the
  trailer sector, in whole sectors.
- **Cold boot only.** PHY system sleep wakes through the ROM, which would
  rerun the loader and clobber retained RAM. Sleeping products need a
  ROM warm-boot path that bypasses it. That is an open hardware question.
- **Building.** The build uses `ESP_PHY_RUST_TOOLCHAIN` with
  `panic = immediate-abort`. CI turns the ELF into a ROM-boot table and
  `app.bin` with `phy62x2-image --rom-boot`.

### Phase 3 layouts

| deployment (file) | reserved, never written | loader | primary | secondary | journal | application cap |
|---|---|---|---|---|---|---|
| nRF52840 DK (`nrf52840-sensor/boot/dk.layout`) | none | `0x0..0x4000` | `0x4000..0x81000` | `0x81000..0xFE000` | `0xFE000..0x100000` | 506,624 B |
| nRF52840 MDK / PCA10059 UF2 (`mdk-uf2`, `pca10059-uf2`) | MBR `0x0..0x1000`, UF2 `0xF4000..0xFE000`, MBR params `0xFE000..0x100000` | `0x1000..0x5000` | `0x5000..0x7B000` | `0x7B000..0xF1000` | `0xF2000..0xF4000` | 477,952 B |
| nRF52840 ProMicro UF2 (`promicro-uf2`) | MBR+S140 `0x0..0x26000`, boot guard `0xF0000..0xF4000`, UF2, MBR params | `0x26000..0x2A000` | `0x2A000..0x8C000` | `0x8C000..0xEE000` | `0xEE000..0xF0000` | 396,032 B |
| EFR32MG22 devkit (`efr32mg22-sensor/boot/devkit.layout`) | none (router tables reclaimed) | `0x0..0x4000` | `0x4000..0x40000` | `0x40000..0x7C000` | `0x7C000..0x80000` | 236,288 B |
| EFR32MG21 devkit (`efr32mg21-sensor/boot/devkit.layout`), F1024 only | Gecko `0x0..0x4000`, router tables `0x74000..0x7C000` | `0x4000..0x8000` | `0x80000..0xC0000` | `0xC0000..0x100000` | `0x7C000..0x80000` | 252,672 B |
| ZBT-1 (`ha-connect-zbt1/boot/zbt1.layout`) | Gecko `0x0..0x4000`, router tables `0xF4000..0xFC000` | `0x4000..0x8000` | `0x8000..0x7E000` | `0x7E000..0xF4000` | `0xFC000..0x100000` | 473,856 B |
| PB-03F (`pb03f-kit/boot/pb03f.layout`) | ROM/factory `0x0..0x2000`, ROM-boot table `0x2000..0x3000`, factory data `0x3000..0x5000`, load data `0x5000..0x6000`, router tables `0x7A000..0x7E000` | `0x6000..0xA000` | `0x10000..0x43000` | `0x43000..0x76000` | `0x7E000..0x80000` | 200,448 B (`app-max`) |

The application cap is the slot minus its last sector (the trailer), the
0x200-byte header (0x100 on PHY62x2) and the 0x300-byte TLV reserve, or
`app-max` when that is smaller. The layout tests assert every cap. (The
earlier "slot capacity" column, and the 244,480 B figure for a reclaimed
MG22 slot, omitted the trailer sector.) All deployments use overwrite mode
with 3 trial boots. The nRF52840 and EFR32 layouts set `watchdog-ms = 8000`;
PB-03F sets 0.

### Application-side verification (ESP32, Telink)

On these chips the ROM or SDK boot stays. The application authenticates what
it accepts over Zigbee OTA:

- **Input.** The OTA payload is a plain
  `imgtool sign --header-size 0x20 --pad-header` image:
  `[32-byte header][native image][protected TLVs][TLVs]`.
- **Staging.** `mcuboot_image::envelope::EnvelopeWriter` wraps the product
  `FirmwareWriter`. It keeps the header and the TLV trailer in RAM and
  stages only the native image, so the ESP image the IDF bootloader
  checks is unchanged.
- **Before any payload byte is written,** the header is parsed and
  refused if malformed or larger than the slot.
- **At `verify`:**
  1. the inner writer checks its own format;
  2. the SHA-256 (over header + image + protected TLVs, as MCUboot
     defines it) and the ECDSA P-256 signature are checked against the
     trusted keys, reading the staged image back;
  3. the digest must match the Zigbee `expected_hash` when one is given;
  4. a version older than the running one is refused (`cmp_release`).
- **`activate`** is refused unless `verify` succeeded.
- **Keys** come from the product file `boot/app-verify.conf` (`name`,
  `env-prefix`, `key-file`) through `zrs-boot-layout::AppVerify`, with the
  same `<PREFIX>_BOOT_KEYn` override as the loaders.

**ESP32-C6/H2.** The product feature `signed-ota` and the example feature
`signed-ota` select `product::profile::OtaWriter = SignedEspWriter`. The
default build is unchanged. Version mapping: Zigbee OTA file version
`0xMMmmrrrr` is MCUboot version `MM.mm.rrrr`, so sign with
`imgtool sign --version MM.mm.rrrr`.

**Limit.** This does not make the ESP32 secure-boot. Anything written over
UART or JTAG still boots; use ESP secure boot for that.

**TLSR8258.** `products/tlsr8258-tb04/boot/app-verify.conf` exists, but
the product has no OTA writer yet, so nothing uses it.

### Phase 3 blockers

- **EFR32MG22 headroom.** With the router tables reclaimed, the
  `zrs-boot` sensor (234,164 B) fits the 236,288 B cap with about 2.1 KB
  to spare; the End Device is 229,788 B.
- **EFR32MG21 devkit.** The layout needs an F1024 part; there is no room
  for two image-sized slots below the router tables on F512.
- **No confirm in the `zrs-boot` nRF and EFR32 applications.** They use
  `NoOta`, so nothing confirms a trial install; it reports `unconfirmed`
  and keeps booting (the approved overwrite behaviour).
- **PHY62x2.** `pb03f-zigbee` (195,168 B) fits the 200,448 B PB-03F cap.
  The PHY62x2 EVK (`phy6222-sensor`, 187,104 B direct slot) has no loader
  layout. There is no PHY `zrs-boot` application build yet. System sleep
  warm-boots through the ROM (see above).
- **Watchdog bounds.** The EFR32 ULFRCO worst case (1.1 kHz) and the nRF
  UF2-bootloader interaction are unverified.
- **CC2340R5 product loader:** blocked on SDK `lpf3-9.20.01.21`.
- **EFR32MG1:** deferred until a sacrificial-board rehearsal.

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

**Phase 3 loaders** (ECDSA P-256, one key, overwrite, 3 trial boots; flash
is file-backed `.vector_table + .text + .rodata + .data + .loader_info`):

| loader | region | flash (`d35436f`) | flash (watchdog) | RAM (`.data` + `.bss`) |
|---|---|---|---|---|
| nRF52840 `dk` | 16,384 B | 9,084 B | 9,264 B (+180) | 0 B |
| nRF52840 `mdk-uf2` | 16,384 B | 9,088 B | 9,260 B (+172) | 0 B |
| nRF52840 `pca10059-uf2` | 16,384 B | 9,084 B | 9,260 B (+176) | 0 B |
| nRF52840 `promicro-uf2` | 16,384 B | 9,080 B | 9,256 B (+176) | 0 B |
| EFR32MG22 devkit | 16,384 B | 9,232 B | 9,536 B (+304) | 280 B |
| EFR32MG21 devkit, ZBT-1 | 16,384 B | 9,124 B | 9,412 B (+288) | 280 B |
| ZBT-1 `gecko-recovery` | 16,384 B | 9,192 B | 9,480 B (+288) | 280 B |
| PB-03F (no watchdog) | 16,384 B XIP + 4 KiB load data | 9,048 B XIP + 1,032 B load data | unchanged | 708 B |

The loader rows were measured in one worktree; the watchdog column is the
loader with `watchdog-ms = 8000`.

**`zrs-boot` application builds** (not shipped; flash is file-backed):

| image | flash | cap |
|---|---|---|
| nRF52840 UF2 `board-nrf-dk,zrs-boot` | 230,276 B (+232 for `NrfWatchdogSupervisor`) | 506,624 B |
| nRF52840 UF2 `board-promicro,zrs-boot` | 229,156 B (+336) | 396,032 B |
| EFR32MG22 `sed,zrs-boot` | 234,164 B | 236,288 B |
| EFR32MG22 `ed,zrs-boot` | 229,788 B | 236,288 B |
| EFR32MG21 `sed,zrs-boot` | 234,272 B | 252,672 B |
| EFR32MG21 `ed,zrs-boot` | 229,988 B | 252,672 B |

Shipped default images: with these changes reverted in the same worktree,
the nRF52840 UF2 (ProMicro, 228,840 B) and EFR32MG22 sensor (234,160 B)
images are the same size. The other defaults (MG22 ed/router, MG21
roles, other UF2 boards) were compared only across worktrees, where
builds differ by 4–20 B in both directions from crate-hash layout noise;
no growth was attributable to the changes.

Phase 3 application images (equal-length worktree paths, against `d4c4023`):

- **ESP32-C6 sensor** (default): `.text` −8 B, `.data` and `.bss`
  unchanged.
- **ESP32-H2 sensor** (default): `.text` −8 B, `.data` and `.bss`
  unchanged.
- **ESP32-H2 thermometer** (default): `.text` +16 B, `.data` and `.bss`
  unchanged. Per function, the delta is `esp_sync` lock inlining
  (+20/+16/−34 B) plus +8 B in `main` and +6 B in one critical section.
  No OTA or writer symbol changed; the crate disambiguators changed.
- **nRF52840 UF2** (default direct map): padding only.

**`signed-ota` cost** (opt-in):

| image | `.text` | `.data` | `.bss` |
|---|---|---|---|
| ESP32-C6 sensor | +12,252 B | +160 B | +352 B |
| ESP32-H2 sensor | +12,168 B | +168 B | +352 B |
| ESP32-H2 thermometer | +12,888 B | +168 B | −168 B |

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
  - This is a product poll-policy defect, tracked separately. It is not a
    loader issue.
- **Throughput.** OTA ran at about 77–120 B/s (48 B blocks through
  parent `0x43FD`), so a 238 KB image took about 33 minutes.

### MG22 results (2026-11-09)

One EFR32MG22C224F512IM40 on a BRD4183A radio board in a WSTK (J-Link
440206286, SWD through hwlab). The loader is the `efr32mg22-devkit` build
from this branch (9,536 B): overwrite, one P-256 key, 3 trial boots, 8 s
watchdog request (`PERSEL` 11). The application is
`examples/efr32mg22-sensor` with `sed,zrs-boot` (234,164 B). Every test
image was signed with imgtool 2.4.0 and the options `--header-size 0x200
--pad-header --align 4 --slot-size 0x3C000`. Upgrade images also had
`--pad`, which writes the request magic. **Lab-tested on one board**;
over-the-air (ZHA OTA) delivery into the secondary slot is not yet tested.

| step | result |
|---|---|
| Preparation | DEVINFO read: EFR32MG22C224, 512 KiB flash and 32 KiB RAM. Full 512 KiB backup (two identical reads, SHA-256 `6d7b5b3b…`) and the user-data page saved. The main flash was erased and the loader written at `0x0` |
| First boot | Unpadded 0.1.0 written at `0x4000`. Loader → application: banner, then a scan of PAN `0xDFE9`. Fresh persistence |
| ZHA join | Joined the home network as `0x8C8F` under the existing device entry (`Zigbee-RS EFR32MG22-Sensor`). Interview and reporting set up (3 of 3 reports configured) |
| Upgrade 0.1.0→0.1.1 | Padded image written to the secondary over SWD. After the reset the primary header read 0.1.1 and the trial log `Done, Attempt`. The secondary header and magic were erased. The application printed `valid security journal preserved`, then resumed on the network with no re-pair |
| Trial expiry | Three more resets: the log stopped at `Done` + 3 `Attempt` (`trial_boots`), and each later boot kept starting the image with its journal (overwrite mode: the image stays and is reported unconfirmed) |
| Downgrade 0.0.9 | Refused. The primary stayed 0.1.1, the log was unchanged, and the request (secondary header and magic) was dropped |
| Wrong key (0.1.2 signed with a throw-away P-256 key) | Refused in the same way |
| Upgrade 0.1.1→0.1.3 | Installed; the log restarted at `Done, Attempt`; the device rejoined |
| Watchdog | `WDOG0_EN` = 1 while the loader ran (sampled twice), and 0 in the application after both installs and after plain resets. The application banner came 4.3–5.0 s after the programming call returned, including J-Link Commander exit, verification, install and 0.4 s of application start. The minimum period is 14.9 s |
| Retention in HA | The sensor kept reporting in HA after every reflash, without a re-pair |

Notes:

- **The SWD attach resets the board in EM2.** The application leaves
  `EMU_CTRL.EM2DBGEN` clear. A J-Link attach while the SED sleeps in EM2
  therefore resets the chip, which counts as one more boot (one more trial
  attempt) during tests. Two of the `WDOG0_EN = 1` samples come from such
  attaches.
- **Not exercised on this board:** delivery over ZHA OTA, power cuts
  during an install, a watchdog expiry reset (needs a hung loader), the
  ULFRCO period, and a confirmed image. `zrs-boot` builds use `NoOta` and
  never confirm. MG21, ZBT-1 and `gecko-recovery` were also not exercised.
- **Restoring the board:** the original image is the backup `6d7b5b3b…`
  in the hwlab artifacts, taken before the erase.

## Open decisions

Resolved in Phase 2 step 2:

- Ed25519 is BL702-only; on CC2340 it is a build error.
- CC2340 uses `BootFlash`.
- Loaders arm a watchdog (BL702 default 16 s).
- An expired trial in overwrite mode keeps booting and is reported.

Resolved in Phase 3:

- MG22 reclaims the router-tables reservation for the slots (end-device
  roles only under the loader).
- nRF52840 and EFR32 Series 2 loaders arm a watchdog (8 s in the
  committed layouts); on nRF the `zrs-boot` application feeds it.
- The PB-03F application cap is its direct map's 200,448 B XIP slot.
- ESP32 version mapping: file version `0xMMmmrrrr` is `MM.mm.rrrr`.

Still open:

0. **Phase 3:**
   - **PHY62x2 sleep.** The warm-boot path for sleeping PHY62x2
     products.
1. **CC2340 watchdog default.** The CC2340 WDT cannot be stopped once
   armed, and `cc2340-sensor` does not feed it. Product configs therefore
   keep `watchdog: None` until the app feeds the watchdog; then they arm
   it with a period that covers a full-slot validation.
2. **Swap-move per product.** Which of CC2340, nRF52 and MG21/MG22 turn
   it on, and with how many trial boots.
3. **XT-ZB1 slow poll versus OTA reachability.** The product poll
   policy should let ZHA's `image_notify` reach an idle sensor. This is
   tracked separately from the loader.

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

Phase 3 status:

- **Lab-tested on one board:** the EFR32MG22 part of step 4
  ([results](#mg22-results-2026-11-09)).
- **Built and host-tested only:** steps 3, 5 and 6, and the MG21 part of
  step 4, all awaiting hardware runs.
- **Skipped:** step 2 (SDK blocker) and step 7.

Each hardware run needs the user's approval of the board, the probe, the
erase scope and the backup first.
