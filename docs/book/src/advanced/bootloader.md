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
  EFR32MG21/MG22 applications have a `zrs-boot` link variant. The MG22
  `sed,zrs-boot` sensor also receives Zigbee OTA images into the
  secondary slot and confirms them after rejoining
  ([MG22 OTA receive](#efr32mg22-ota-receive)). ESP32
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
| `bootloaders/bl702-boot` | BL702 shell: XIP mapping, L1C invalidation, riscv-rt jump, UART lines | Host-tested and release-built. The v2 loader (overwrite, one P-256 key, 3 trial boots, 16 s watchdog) is **lab-tested on one XT-ZB1**: ZHA OTA 0.2.0→0.2.1, confirm-after-rejoin, 168 soak installs and 46 power cuts with no failure ([results](#results-2026-11-08)). Trial expiry (`image unconfirmed`, keeps booting) is lab-tested; wrong-key and downgrade images are refused on hardware by the application before arming ([results](#results-2026-11-0910-refusals-trial-expiry-resume)). The loader's own downgrade and wrong-key rejection are host-tested only |
| `bootloaders/cc2340r5-boot` | CC2340R5 shell: vector-table check, VTOR/MSP jump, fault blinks, SHUTDOWN | Host-tested; thumbv6m build only. No product loader binary exists |
| `zigbee-runtime` `FirmwareWriter`/`OtaLifecycle` | `confirm_image` hook; `FirmwareWriter::confirm_image` | Host-tested |
| `apps/sensor-sed` | Confirms the running image on its first APS-acknowledged report, or once joined when reports are not acknowledged | Host-tested |
| `bootloaders/layout` (`zrs-boot-layout`) | Build-time parser and checker of product `boot/*.layout` and `boot/app-verify.conf` files; emits `BootConfig`, keys, loader and application `MEMORY` regions | Host-tested |
| `bootloaders/cortex-m-boot` (`zrs-boot-cortex-m`) | Cortex-M shell: entry check through the flash driver, VTOR/MSP hand-over, `Hooks` for status, watchdog and failure | Host-tested; built in the three loaders below |
| `products/nrf52840-sensor/bootloader` | nRF52840 loader, one build per board layout | Release-built and linted; **not run on hardware** |
| `bootloaders/efr32s2-boot` | EFR32MG22/MG21 loader; on MG21 chained above the Gecko Bootloader | Release-built and linted. The MG22 devkit build is **lab-tested on one BRD4183A** ([results](#mg22-results-2026-11-09)). MG21, ZBT-1 and `gecko-recovery` are **not run on hardware** |
| `efr32s2-hal::watchdog` | EFR32 Series 2 WDOG0 driver (period rounding, arm/kick/disarm) | Host-tested (period maths). On one MG22 it was seen armed while the loader ran and disarmed in the application. Its period and an actual expiry reset are **not measured** |
| `bootloaders/phy62x2-boot` | PHY62x2 loader started by the boot ROM | Release-built, linted, ROM-boot image generated; **not run on hardware** |
| `mcuboot-image::envelope` | Application-side verification of a signed MCUboot envelope (`EnvelopeWriter`) for chips whose ROM/SDK boot stays | Host-tested with imgtool vectors; ESP32 product tests; ESP32-H2 ZHA upgrade plus wrong-key and downgrade refusal on hardware ([results](#esp32-h2-signed-zha-ota-2026-11-10)); C6 and Telink not run on hardware |
| `mcuboot-image::staging` | Application side of a zigbee-rs loader: `LoaderChannel` (own / foreign / no loader), `shared_keys` (keys both the application and the installed loader trust) and `StagingFlash` (erase/write only in the secondary slot, write-only in the running image's trial log) | Host-tested |
| `apps/sensor-sed` `ota_client` (feature `ota`) | `ImageOtaClient`: query schedule, Image Notify, offer gating on the loader channel, activation after the security checkpoint, reset, confirm through the writer | Host-tested (through the MG22 product tests) |
| `tools/zrs_boot_ota.py` | Signs a loader-linked application from its product `.layout` and the ELF's `ZRS_OTA_IDENTITY`; writes the signed image and the Zigbee OTA file; `check`, `key-xy`, zigpy `index` | Python unit tests; real imgtool 2.4.0 signing of the MG22 build (local and CI with an ephemeral key) |
| `products/efr32mg22-sensor` `ota` (`sed,zrs-boot`) | MG22 identity, flash split, `SlotWriter` over `StagingFlash`, OTA profile and client | Host-tested and release-built. Confirm-after-rejoin of an SWD-staged 0.2.0 is **lab-tested on one BRD4183A** ([results](#mg22-ota-confirm-results-2026-11-09)); delivery over ZHA OTA is **not yet run on hardware** |

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
  `<PREFIX>_OTA_KEY{,2,3}_XY` environment variables or the `key-file`; private
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
- **Discard.** Any `verify` failure (format, signature, hash or version)
  aborts the inner writer, so the platform drops the staged image instead
  of keeping a bootable copy (see the ESP behaviour below).
- **Keys** come from the product file `boot/app-verify.conf` (`name`,
  `env-prefix`, `key-file`) through `zrs-boot-layout::AppVerify`, with the
  same `<PREFIX>_OTA_KEY{,2,3}_XY` override as the loaders.

**ESP32-C6/H2.** The product feature `signed-ota` and the example feature
`signed-ota` select `product::profile::OtaWriter = SignedEspWriter`. The
default build is unchanged. Version mapping: Zigbee OTA file version
`0xMMmmrrrr` is MCUboot version `MM.mm.rrrr`, so sign with
`imgtool sign --version MM.mm.rrrr`.

**Refused images are discarded (ESP, signed and unsigned).** The shared
`EspFirmwareWriter` erases the first 4 KiB sector of the target slot, so
the ESP image magic `0xE9` is gone and no IDF bootloader fallback can start
the image, when:

- `verify` fails (size, ESP structure, checksum, SHA-256 or expected hash),
  or the envelope check fails above it; or
- `abort()` drops a transfer: an explicit abort, another offer replacing a
  suspended download, or the OTA engine ending the session.

It does not erase:

- when the transfer never programmed the slot (a staged image from an
  earlier transfer may still be there);
- once `otadata` programming has started, because the slot may already be
  selected;
- when the runtime only suspends a transfer (server ABORT or silence with
  staged data), which never calls `abort()` and keeps the data for resume.

A second abort is a no-op. If the erase itself fails, `abort()` reports
`EraseFailed`, and a verify failure keeps its own error. Host-tested
(`products/esp32-zigbee-devkit/src/ota.rs` and
`mcuboot-image/src/envelope_tests.rs`); not yet rerun on hardware. Size:
+390 B flash on the default C6 sensor, +388 B on the default H2 sensor and
+84 B on the quiet H2 thermometers, with no bss change.

`tools/create-esp32-ota.py` builds, signs and packages in one step:

```bash
tools/create-esp32-ota.py 0x00020001 out --chip esp32h2 \
    --variant thermometer --sign-key own-ec-p256.pem
```

- `--sign-key` adds the `signed-ota` feature and wraps the ESP image with
  `imgtool sign --header-size 0x20 --pad-header --align 4 --slot-size 0x1F0000
  --version MM.mm.rrrr+0`.
- The result is checked with `imgtool verify`, and the inner image must be
  byte-identical to the espflash image.
- The key's public point must be one of the firmware's trusted keys
  (`boot/app-verify.conf`, including `ESP32_DEVKIT_OTA_KEY{,2,3}_XY`). Otherwise the
  tool refuses; `--test-only-untrusted-key` exists only to build refusal-test
  images.
- `--variant thermometer` takes the model and image type (0x0012) from
  `examples/esp32h2-sensor/thermometer.toml` and builds the
  `esp32h2-thermometer` binary with `thermometer,light-sleep`.
- The container and `index.json` checks accept the envelope: SHA-256 TLV,
  signature TLV, no trailing bytes, inner ESP image valid for the chip.

Signed-OTA size (thermometer, `light-sleep`, non-quiet): +11,892 B text,
+376 B rodata, +160 B data, +352 B bss over the same build without
`signed-ota`. The stack is about 192 KiB, so P-256 verification has ample
headroom. Default builds do not enable it.

**Limit.** This does not make the ESP32 secure-boot. Anything written over
UART or JTAG still boots; use ESP secure boot for that.

**TLSR8258.** `products/tlsr8258-tb04/boot/app-verify.conf` exists, but
the product has no OTA writer yet, so nothing uses it.

### EFR32MG22 OTA receive

`examples/efr32mg22-sensor` with `sed,zrs-boot` downloads Zigbee OTA
images into the loader's secondary slot. The pieces, from shared to
product-specific:

- **`mcuboot_image::staging`** (shared).
  - `LoaderChannel::read` reads the installed loader's `LoaderInfo` and
    `LoaderKeys` records at the layout's `LOADER_INFO_ADDR`. It returns
    `Own` when the loader trusts at least one of the application's keys,
    `Foreign` when it trusts none, and `NoLoader` when there are no
    readable records.
  - Offers are accepted only on `Own`. A direct-mapped or foreign-keyed
    device declines without touching flash.
  - `shared_keys` gives the writer only the keys both sides trust, so an
    image the loader would refuse is refused before activation.
  - `StagingFlash` limits the writer's flash view:
    - reads stop at the end of the higher slot;
    - erase and write are allowed only in the secondary slot;
    - in the running image's trial log, writes only (the confirm).
- **`sensor_sed_app::ota_client::ImageOtaClient`** (shared, feature `ota`).
  - Queries the coordinator 300 s after joining, then daily, and answers
    Image Notify.
  - Downloads through the runtime `OtaManager`.
  - Activates only after `SensorApp` has checkpointed the security
    journal, then resets into the loader.
  - Confirms through the writer: `SensorApp` calls `confirm_image` on
    the first APS-acknowledged report after the rejoin.
- **`products/efr32mg22-sensor::ota`** (product).
  - Identity: manufacturer `0x1234`, image type `0x0022`.
  - File version `0x25MMmmpp`, generated from the package version by
    `zrs-boot-layout::ReleaseVersion`. The TEST-ONLY
    `EFR32MG22_OTA_VERSION_OVERRIDE` marks the image `TEST-ONLY` in its
    banner.
  - 48-byte blocks; `auto_accept` is off.
  - The flash split: the OTA writer gets `0x0..JOURNAL_START` and the
    security journal keeps its own driver. The product's const asserts
    tie the split to `boot/devkit.layout`.
  - The writer is a `SlotWriter` over `StagingFlash`.
- **Composition** (`main.rs`):
  - picks `OtaSensorProfile` and `ImageOtaClient` under `zrs-boot`;
  - prints one raw VCOM line with the version, the loader channel and
    the OTA identity;
  - exports `ZRS_OTA_IDENTITY` for the release tool.
  - The default builds keep `NoOta` and `SensorProfile`.

**Logs.** OTA receive costs about 32 KB of flash:

| part | flash |
|---|---|
| `OtaManager` | 8.6 KB |
| `mcuboot-image` (P-256, image hash) | 6.2 KB |
| `sensor-sed-app` | 2.5 KB |
| `sha2` | 2.4 KB |
| ZCL OTA cluster | 2.2 KB |

With info-level logs the image would be 265,908 B, over the 236,288 B
cap. The `zrs-boot` feature therefore compiles `log` to warnings
(`log/release_max_level_warn`, 233,872 B); the version line is printed
raw. `opt-level = "z"` (222,144 B with warnings) was not used, because it
is profile-wide and its MAC timing is unproven.

**Release.**

```text
python3 tools/zrs_boot_ota.py release \
  --layout products/efr32mg22-sensor/boot/devkit.layout \
  --elf examples/efr32mg22-sensor/target/zrs-boot/thumbv8m.main-none-eabihf/release/efr32mg22-sensor \
  --out <dir> --key <P-256 PEM> --name efr32mg22-sensor
```

- The tool checks:
  - the ELF is linked at the primary slot plus header;
  - the key is one the layout trusts (`key-file` or
    `<PREFIX>_OTA_KEY_XY`).
- It signs unpadded (`--header-size 0x200 --pad-header --align 4
  --slot-size 0x3C000 --version MM.mm.pp+0`), runs `imgtool verify`, and
  writes `<name>-<version>.signed.bin` (SWD into the primary) and
  `<name>-<version>.zigbee`.
- The 0.1.0 image is 234,534 B, which leaves 3,034 B of slot margin.
- CI signs with a per-run key, as for BL702.

**Validation.** Host tests (MG22 product, 23 with `sed,zrs-boot`):

- **Staged and activated:** an image is staged and requested on
  activation; padded images are handled.
- **Refused:** wrong key, downgrade and truncation.
- **Install and confirm:** the shared boot engine installs the staged
  image, which then confirms through the trial log, with the loader and
  the journal untouched.
- **Staging view:** it protects the loader, the image and the journal.
- **Declined offers:** a declined offer leaves flash untouched.
- **Other:** channel detection.

**Hardware.** On one BRD4183A, the confirm-after-rejoin of an SWD-staged
TEST-ONLY 0.2.0 is lab-tested; see
[MG22 OTA confirm results](#mg22-ota-confirm-results-2026-11-09). These
are **not run on hardware**: ZHA OTA delivery into the secondary (the
`StagingFlash` write path and the OTA-time signature check) and the
stack headroom of the verify path.

### Phase 3 blockers

- **EFR32MG22 headroom.** With the router tables reclaimed and OTA
  receive added, the `sed,zrs-boot` sensor (233,872 B, warn-level logs)
  fits the 236,288 B cap with 2,416 B to spare; the End Device is
  204,484 B. Any further OTA feature (resume checkpoints) needs
  headroom first; see [MG22 OTA receive](#efr32mg22-ota-receive).
- **EFR32MG21 devkit.** The layout needs an F1024 part; there is no room
  for two image-sized slots below the router tables on F512.
- **No confirm in the other `zrs-boot` applications.** The nRF52840,
  EFR32MG21 and MG22 `ed` builds use `NoOta`, so nothing confirms a trial
  install; it reports `unconfirmed` and keeps booting (the approved
  overwrite behaviour). Only MG22 `sed,zrs-boot` has OTA and confirm.
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
| EFR32MG22 `sed,zrs-boot` (OTA receive, warn-level logs) | 233,872 B (was 234,164 B without OTA, info logs); `.bss` 16,136 B (+1,536) | 236,288 B |
| EFR32MG22 `ed,zrs-boot` (warn-level logs) | 204,484 B (was 229,788 B) | 236,288 B |
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

MG22 OTA receive (`boot-mg22-ota`, equal-length worktrees against
`493955d`; the shared `sensor-sed-app` gained an optional `log`
dependency, which changes crate hashes everywhere): no function changed
size in the default images. Totals: EFR32MG22 sensor −8 B, End Device
+8 B, nRF52840 DK sensor +16 B (alignment padding), BL702 sensor ±0;
`.bss` unchanged.

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
  ULFRCO period, and a confirmed image. The `zrs-boot` build tested here
  used `NoOta` and never confirmed. OTA receive and confirm came later
  ([MG22 OTA receive](#efr32mg22-ota-receive)); the confirm is now
  lab-tested ([below](#mg22-ota-confirm-results-2026-11-09)), but ZHA OTA
  delivery is not. MG21, ZBT-1 and `gecko-recovery` were also not
  exercised.
- **Restoring the board:** the original image is the backup `6d7b5b3b…`
  in the hwlab artifacts, taken before the erase.

### MG22 OTA confirm results (2026-11-09)

The same board was used, still joined as `0x8C8F`. It started with the
loader and 0.1.3 (`NoOta`). The application under test was
`examples/efr32mg22-sensor` with `sed,zrs-boot` and the OTA receive
path, built with `EFR32MG22_OTA_VERSION_OVERRIDE=0.2.0` (a TEST-ONLY
build; Zigbee file version `0x25000200`; 233,904 B payload, 234,567 B
signed). It was signed with the options above plus `--pad --version
0.2.0+0`. `tools/zrs_boot_ota.py release` accepted the same ELF with
3,001 B of slot margin. **Lab-tested on one board.**

| step | result |
|---|---|
| Backup | Full 512 KiB, two identical reads, SHA-256 `70f166ba…`. Decoded: loader present, primary 0.1.3, secondary header erased, trial log `Done, Attempt` (0.1.3 never confirmed), journal in use |
| Stage | Padded 0.2.0 programmed at `0x40000..0x7C000` only (30 sectors written and read back). The loader and journal sectors were not in the programmed range |
| Install and banner | After the reset, VCOM printed `MG22 app 0.2.0 (TEST-ONLY version): loader Own, OTA 0x1234/0x0022 file 0x25000200` |
| Rejoin | ZHA kept the same device entry: nwk `0x8C8F`, available, LQI 112. A report arrived about 70 s after the reset, with no availability gap and no re-pair |
| Confirm | The primary trailer log read `Done (0x6000), Attempt (0xC000), Confirmed (0xA000)`. The app appended the confirm record after the rejoin |
| Post-state | Backup SHA-256 `9cd8c870…`. Loader bytes unchanged. Primary equals the signed image (header, payload and the 151-byte TLV). Secondary header and trailer sectors erased. Journal grew from 1,547 to 1,785 non-`0xFF` bytes (persistence continued). Reports continued after the SWD session |

**Not exercised:** delivery over ZHA OTA (block transfer into the
secondary through `StagingFlash`, the OTA-time signature check, and
activation by the upgrade-end response), a power cut during staging, and
the stack headroom of the verify path. These wait for the ZHA OTA step.

### MG22 ZHA OTA (2026-11-10)

**Re-pair.** ZHA had interviewed the board while it ran 0.1.3, which has no
OTA client. Its stored descriptor therefore listed no output clusters, and
ZHA created no update entity after the SWD upgrade to 0.2.0 (see
[OTA: reaching a sleepy device](ota.md#reaching-a-sleepy-device)).

1. An SWD reset opened the 120 s fast-poll window, then `zha.remove`.
2. A 60 s `zha.permit`, then another SWD reset. The device rejoined after
   a cold start with no button press.

The interview recorded output cluster `0x0019`, and ZHA created
`update.zigbee_rs_efr32mg22_sensor_firmware` with installed version
`0x25000200`. The board is now NWK `0x9F38` (40760), still IEEE
`08:6b:d7:ff:fe:c3:90:12`; the sensor entity IDs were kept.

**Images.** All were built from the rebased branch, with the
`sed,zrs-boot` feature set and a version override, then signed:

| run | file version | inner image | `.zigbee` size | SHA-256 |
|---|---|---|---|---|
| A upgrade | `0x25000201` | 0.2.1, own key | 234,797 B | `8c387f7d…` |
| B wrong key | `0x25000202` | 0.2.2, throwaway P-256 key | 234,797 B | `3cedd8a0…` |
| C disguised downgrade | `0x25000203` | 0.1.9, own key | 234,798 B | `8fe8600b…` |

Each signed image is 234,735 B, leaving 2,833 B of slot margin.
`zrs_boot_ota.py check` refuses C because its inner version is not its
outer file version. The device must refuse B and C when it verifies the
download.

**Run A, attempt 1.**
- **Offer:** HA showed `latest_version 0x25000201`.
- **Install:** started 20 s after an SWD reset.
- **Stall:** at offset 7,584 (about 3%) the device stopped receiving
  blocks. zigpy aborted after its 30 s no-progress limit
  (`MAX_TIME_WITHOUT_PROGRESS`; the timer restarts only after a block
  response is delivered).
- **Device log:** `Server aborted the download at offset 7584; keeping
  staged data`.
- **Outcome:** 0.2.0 kept running; no upgrade request was written.
- **Context:** the install started 2.5 min after a ZHA reload and 9 min
  after an HA restart, during the post-start traffic burst.

**Attempt 2** was cut off by another HA restart before the install began.

Each later run followed the same procedure:
- swap the single test file in `/config/zigpy_ota` and check its SHA-256;
- reload the ZHA entry, settle at least 10 min, then
  `homeassistant.update_entity` until `latest_version` showed the file;
- start a VCOM capture, reset over SWD, and start `update.install` inside
  the 120 s fast-poll window;
- after the run, read flash over SWD.

**Run A (0.2.0 → 0.2.1): pass.**
- **Install:** started 15 s after the SWD reset, about 10 min after the
  reload. The download took about 25 min.
- **Result:** HA reported `installed_version 0x25000201`.
- **SWD after the install:**
  - the trial log at `0x3E000` read `Done, Attempt, Confirmed`;
  - the primary header at `0x4000` showed 0.2.1;
  - the secondary header and trailer were erased.
- **Device log during the download:**
  - 363 `[NWK] Frame counter replay from 0x0000`;
  - 57 `NoAck` and 23 `ChannelAccessFailure`;
  - 44 NWK route errors, each surfacing as `ZCL frame send failed:
    NoShortAddress`;
  - one block for another offset, ignored.

  The stop-and-wait retries absorbed all of them.

**Run B (wrong key, `0x25000202`): pass, refused.**
- **Download:** completed in about 24 min.
- **Result:** HA raised `Update was not successful: Status.INVALID_IMAGE`;
  `installed_version` stayed `0x25000201`.
- **SWD:**
  - the secondary slot held the fully staged 0.2.2 image (MCUboot magic);
  - the upgrade request trailer (ending at `0x7BFE0`) was erased;
  - the primary was still 0.2.1 and the trial log was unchanged.

**Run C (disguised downgrade, `0x25000203`): pass, refused.**
- **Attempt 1:** started 15 s after an SWD reset. It failed within 1 s
  with `Status.FAILURE` because zigpy could not deliver Image Notify.
  - The device had sent no frame for 62 s after the reset.
  - Shortly before, it had rejoined on its marginal link (RSSI −81 dBm)
    and repeated announces with identical TSNs.
- **Attempt 2:** started only after HA logged the first frame from
  `0x9F38` after the reset.
  - **Download:** about 25 min.
  - **Result:** HA reported `Status.FAILURE`. zigpy reports that when the
    Upgrade End exchange fails, not only on a device status.
    `installed_version` stayed `0x25000201`.
  - **SWD:**
    - the secondary slot held the staged 0.1.9 image;
    - the trailer was erased, so no install was requested;
    - the primary was still 0.2.1 and the trial log was unchanged.

**Final state.**
- **Board:** NWK `0x9F38`, running TEST-ONLY 0.2.1 (confirmed). The
  secondary slot holds an unrequested 0.1.9 image, which the loader
  ignores and the next accepted download overwrites.
- **HA:** `/config/zigpy_ota` restored to its original four files
  (SHA-256 checked), and the ZHA entry reloaded.

**Findings, not fixed:**
- **Frame-counter replays.** Frames from the coordinator were rejected as
  NWK frame-counter replays, most likely the parent's retransmissions after
  a missed MAC ACK.
- **Misleading error mapping.** A NWK route error reaches the ZCL layer as
  `NoShortAddress`.
- **Image Notify needs a gated start.** On a marginal link, start an install
  only after HA has heard from the device following the reset.
- **No resume checkpoints on MG22.** An interrupted download restarts from
  offset 0.
- **Not tested:** a power cut during staging, and stack headroom in the
  verify path.

**Probe artefact (not a firmware fault).** The loader-linked image sleeps
in EM2 with the EM2 debug domain off. A J-Link connect then resets the
target to reach the debug port. Two signs of this:
- Right after the connect, an SWD read sees reset-state GPIO (port A
  `MODEL` = 0, VCOM pins off).
- A fresh boot banner follows on the VCOM.

This was reproduced on purpose, with a back-to-back pair of reads:
- the first read saw `MODEL` 0;
- the second saw PA4/PA5 configured;
- one new banner was captured.

Reads taken while the device polls fast during a download do not reset
it. Expect one extra reboot from any SWD read of an idle MG22. Build with
`em2-debug` if that matters.

### ESP32-H2 signed ZHA OTA (2026-11-10)

**Board.**
- Waveshare ESP32-H2-Zero on USB, reached through the hwlab `espusbjtag`
  probe `10:51:DB:63:BB:2E`. This is the first real-hardware use of that
  probe's H2 flash path; the H2 profile was listed as simulation-only.
- Console: `/dev/cu.usbmodem141101`.
- Secure boot and flash encryption are off.
- Joined to the home ZHA network as NWK `0x3FBB` (unchanged by the
  campaign).

**Backup.** A full 4 MiB backup was taken before any write, SHA-256
`5c325d9f…4bdf9`. It held 0.1.0 in `ota_0` and an erased `ota_1`. It must
never be restored as a whole, because its `zbnv` holds old frame counters.

**Base.** TEST-ONLY 0.2.0 (`thermometer,light-sleep,signed-ota`,
non-quiet) was programmed into `ota_0` (`0x10000`) only. It resumed on
the network without a rejoin.

**Images** (`tools/create-esp32-ota.py --variant thermometer`):

| run | file version | inner image | `.ota` size | SHA-256 |
|---|---|---|---|---|
| A upgrade | `0x00020001` | 0.2.1, own key | 413,114 B | `00612be5…` |
| B wrong key | `0x00020002` | 0.2.2, throwaway P-256 key (`--test-only-untrusted-key`) | 413,113 B | `d1d3db38…` |
| C disguised downgrade | `0x00020003` | 0.1.9, own key; container version rewritten | 413,114 B | `cf1b9704…` |

**Procedure for each run:**
- swap one test file into `/config/zigpy_ota` and check its SHA-256;
- reload the ZHA entry and settle at least 10 min;
- start a UART capture, then a JTAG system reset (`ndmreset`; USB stays
  up);
- run `homeassistant.update_entity` and `update.install` inside the 60 s
  fast-poll window. `latest_version` appears only after the device answers
  in that window; while it long-polls it stays empty.

The H2 stays awake on USB ("native USB session vetoes PMU sleep"). An
active OTA session holds fast poll.

**Run A, attempt 1: aborted by the server, not by the device.**
- **Rejoin after reset:** the old parent `0x1F0F` did not answer, so the
  device rejoined via `0x43FD` with the same address.
- **Parent loss:** at 41% (offset 170,064, 22 min in), polls to `0x43FD`
  failed and the staged rejoin ran:
  - the secure rejoin failed;
  - the Trust Center rejoin via `0x4A76` was accepted but delivered no
    key;
  - a 14.7 s backoff followed;
  - the secure rejoin via `0x43FD` then succeeded.
- **Abort:** the device re-announced, and zigpy aborted with
  `Status.TIMEOUT`.
- **Device side:** `Server aborted the download at offset 170064; keeping
  staged data`. `otadata` was unchanged and 0.2.0 kept running.
- **Cause:** a lossy link to the parent, not the OTA path. The retry used
  the same procedure, about 4 min later.

**Run A, attempt 2 (0.2.0 → 0.2.1): pass.**
- **Download:** about 44 min.
- **Device log:**
  - `slot 1 verified (412864 bytes)`;
  - `Server says upgrade NOW`;
  - `otadata sector 1 -> seq 2 (slot 1)`;
  - reboot, then `Loaded app from partition at offset 0x200000`;
  - resumed as `0x3FBB` with no rejoin;
  - `running image confirmed`.
- **HA:** `installed_version 0x00020001`.
- **JTAG:** `otadata` sector 0 had seq 1 (VALID); sector 1 had seq 2,
  state 2 (VALID).
- **Link counters during the download:**
  - 1,237 MAC `NoAck` to the parent, about 16% of transmissions, each
    surfacing as a NWK route error and `NoShortAddress`;
  - 11 frame-counter replays.

  Stop-and-wait retries absorbed them all.

**Run B (wrong key, `0x00020002`): pass, refused.**
- **Download:** the full image, about 45 min, staged into slot 0.
- **Device log:** `slot 0 verified (412864 bytes)` (ESP image check),
  then `Verify failed: VerifyFailed` (signature). The device then sent
  Upgrade End with status `0x96` (INVALID_IMAGE): `aborted`, `transfer
  failed`.
- **HA:** `Update was not successful: Status.FAILURE`; `installed_version`
  stayed `0x00020001`. As on MG22, zigpy does not pass the device's
  status through.
- **JTAG:** both `otadata` sectors were unchanged; slot 1 stays selected.

**Run C (disguised downgrade, `0x00020003`): pass, refused.**
- **Download:** the full image, about 50 min, staged into slot 0. One
  block for another offset was ignored.
- **Device log:** `slot 0 verified (412864 bytes)`, then `Verify failed:
  VerifyFailed`. The signature is valid, but the envelope's MCUboot
  version 0.1.9 is older than the running 0.2.1. The device then sent
  Upgrade End with status `0x96`.
- **HA:** `Status.FAILURE`; `installed_version` stayed `0x00020001`.
- **JTAG:** both `otadata` sectors were unchanged.
- **Link:** NoAck, route-error and replay counts were similar to run A
  (1,269 NoAck, 12 replays).

**Final state.**
- **Board:** H2 `0x3FBB` running TEST-ONLY 0.2.1 from `ota_1`
  (confirmed).
  - `ota_1` is byte-identical to run A's inner image.
  - After the run, `ota_0` held run C's refused 0.1.9 inner image, which
    was not selected. Its first sector (`0x10000`, 4 KiB) was later
    erased over JTAG, so `ota_0` no longer carries a valid image header.
    The erase was a region erase with no protected overlap, taken after a
    verified backup (`2c6e8108…`). `otadata` was byte-identical before and
    after. The hart was resumed without a reset, and the app kept
    reporting.
  - Final backup SHA-256 after the campaign: `4ad0bbf0…29be53`.
- **HA:** `/config/zigpy_ota` restored to its original four files
  (SHA-256 checked), and the ZHA entry reloaded.

**Findings:**
- **A refused image stays bootable (fixed after this run).** In the
  firmware tested here, `abort()` reset only RAM state. After run B the
  inactive slot held the 0.2.2 inner ESP image (magic `0xE9`), and the
  older `otadata` sector still pointed at that slot. It was not selected,
  because the newer sector wins, but it would boot if the newer `otadata`
  sector became invalid. The writer now erases the slot's first sector on
  a failed verify and on abort ([discard rules](#application-side-verification-esp32-telink)).
  The board still runs the pre-fix 0.2.1 test image.
- **Parent link.** A lossy parent link with about 16% MAC NoAck makes a
  45 min download vulnerable to a parent-loss rejoin. zigpy then aborts
  on the re-announce.

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

Phase 3 status:

- **Lab-tested on one board:** the EFR32MG22 part of step 4
  ([results](#mg22-results-2026-11-09)).
- **Built and host-tested only:** steps 3, 5 and 6, and the MG21 part of
  step 4, all awaiting hardware runs.
- **Skipped:** step 2 (SDK blocker) and step 7.

Each hardware run needs the user's approval of the board, the probe, the
erase scope and the backup first.
