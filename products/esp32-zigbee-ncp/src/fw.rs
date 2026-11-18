//! Firmware update of the NCP itself (spec 7.9 `FW_*`) on the ESP-IDF
//! two-slot layout of [`crate::layout::ACTIVE`].
//!
//! This is the transport-neutral engine behind [`crate::platform`]. It
//! reuses the devkit product's ESP pieces (`otadata` selection records and
//! rollback rules, ESP application image validation, SHA-256, the trusted
//! envelope keys and the version mapping of `signed-ota`) and the generic
//! MCUboot envelope check of `mcuboot-image`; only the slot addresses, the
//! partition table and the `FW_*` transaction rules are the NCP's.
//!
//! # Staging (`FW_BEGIN`, `FW_WRITE`)
//!
//! * The target is the slot the image is *not* running from, identified by
//!   live MMU evidence ([`OtaFlash::running_slot`]), never by `otadata`.
//!   No new image is staged while the running image is on trial: the other
//!   slot is its rollback target.
//! * Nothing is erased by `FW_BEGIN`. Each 4 KiB sector is erased right
//!   before the first byte lands in it; one `FW_WRITE` carries at most
//!   [`MAX_CHUNK`] bytes, so it programs at most 1 KiB and erases at most
//!   one sector (`esp-storage` masks interrupts around every ROM flash call).
//! * Writes are sequential. A write below `next_offset` whose bytes equal
//!   what is stored succeeds without writing (idempotent retries); any
//!   different byte is `CONFLICT`. A write that overlaps `next_offset`
//!   compares the stored part and appends the rest.
//! * The first 32 bytes decide the format-1 flavour: an `imgtool sign
//!   --header-size 0x20 --pad-header` **envelope** (MCUboot header, ESP
//!   image, TLVs) or a **plain** ESP application image. Of an envelope only
//!   the ESP image is staged (the bootloader needs it at the slot start);
//!   the header and up to [`TRAILER_CAPACITY`] bytes of TLVs stay in RAM.
//!
//! # Verification (`FW_FINISH`)
//!
//! Length, then the SHA-256 of the transferred file, then the staged ESP
//! image read back from flash (structure, chip, revisions, checksum and its
//! appended SHA-256), then the signature policy ([`FwPolicy`]):
//!
//! * an envelope must verify with a trusted P-256 key (the devkit's
//!   `signed-ota` keys from `boot/app-verify.conf`), its MCUboot version
//!   must equal the `FW_BEGIN` version and must not be older than the
//!   running image;
//! * a plain (unsigned) image is `ACCESS_DENIED` unless the firmware is a
//!   development build (feature `dev-unsigned-fw`);
//! * a downgrade (`FW_BEGIN` version, and the envelope version, below the
//!   running version) is `ACCESS_DENIED` unless `flags` bit0 is set *and*
//!   the firmware is a development build.
//!
//! A refused image has its first sector erased so its ESP magic is gone
//! and no bootloader fallback can start it.
//!
//! # Activation, trial and confirmation
//!
//! `FW_ACTIVATE` re-checks the staged image and programs one
//! `ESP_OTA_IMG_NEW` `otadata` entry into the spare sector. The bundled
//! ESP-IDF bootloader has no rollback support of its own, so rollback is
//! application driven, exactly as in the devkit product (`ota.rs`,
//! `otadata.rs`): [`check_boot`] runs before the stack on every boot, marks
//! the first boot of a new image (trial) and rolls back on a second boot
//! without confirmation; [`confirm_boot`] (`FW_CONFIRM`) marks it valid;
//! the platform rolls back with [`roll_back`] when the trial window expires.
//!
//! Staging only ever touches `otadata` and the inactive application slot
//! ([`crate::layout::is_fw_writable`]): never the bootloader, the partition
//! table, the NV journals or the identity item.

use embedded_storage::nor_flash::{ErrorType, NorFlashError, NorFlashErrorKind, ReadNorFlash};
use esp32_zigbee_devkit_product::esp_image::{
    APP_DESC_MAGIC, DIGEST_SIZE, EXPECTED_CHIP_ID, EspImageHeader, HEADER_SIZE, ImageReadError,
    MIN_IMAGE_SIZE, SEGMENT_HEADER_SIZE, verify_image,
};
use esp32_zigbee_devkit_product::ota::{BootCheck, BootCheckError, OtaFlash, RunningImageError};
use esp32_zigbee_devkit_product::otadata::{
    Activation, BOOT_MARK, BOOT_MARK_OFFSET, BootAction, ENTRY_SIZE, OtaData, OtaSelectEntry,
    STATE_VALID,
};
use esp32_zigbee_devkit_product::sha256::Sha256;
use esp32_zigbee_devkit_product::signed_ota::image_version;
use mcuboot_image::{
    ENVELOPE_HEADER_LEN, ImageError as EnvelopeError, ImageHeader, KeySet, parse_envelope_header,
    validate_envelope,
};
use ncp_coordinator_app::platform::{FwBegin, FwImage, FwSlot, PlatformError, slot_state};
use zigbee_runtime::firmware_writer::FirmwareError;

use crate::layout::{
    ACTIVE, OTA_SLOT_COUNT, OTA_SLOT_SIZE, PARTITION_TABLE_OFFSET, SECTOR_SIZE, WORD_SIZE,
    ota_slot_offset, otadata_sector_offset,
};

/// Largest `FW_WRITE` payload (`FW_BEGIN.max_chunk`).
pub const MAX_CHUNK: u16 = 1024;
/// RAM kept for an envelope's TLVs (as the devkit's `signed-ota`).
pub const TRAILER_CAPACITY: usize = esp32_zigbee_devkit_product::signed_ota::TRAILER_CAPACITY;
/// `FW_BEGIN.format` of an ESP application image, optionally in a signed
/// MCUboot envelope. Format 2 (bare MCUboot) cannot be booted by the
/// ESP-IDF bootloader and is refused.
pub const FORMAT_ESP_APP: u8 = 1;
/// `FW_BEGIN.flags` bit0: allow a downgrade (development builds only).
pub const FLAG_ALLOW_DOWNGRADE: u8 = 0x01;
/// Largest file `FW_BEGIN` accepts: a full slot in an envelope.
pub const MAX_FILE_SIZE: u32 = OTA_SLOT_SIZE + (ENVELOPE_HEADER_LEN + TRAILER_CAPACITY) as u32;

const PREFIX: u32 = ENVELOPE_HEADER_LEN as u32;
const _: () = assert!(PREFIX as usize <= MIN_IMAGE_SIZE as usize);
const _: () = assert!((MAX_CHUNK as u32) < SECTOR_SIZE);

/// Firmware version word `0xMMmmrrrr` (the devkit's OTA file version, and
/// MCUboot `MM.mm.rrrr` through `signed_ota::image_version`).
pub const fn version_word(major: u8, minor: u8, revision: u16) -> u32 {
    (major as u32) << 24 | (minor as u32) << 16 | revision as u32
}

/// Parse a leading `major.minor.patch` (as in `CARGO_PKG_VERSION` and the
/// ESP app descriptor's version string) into a [`version_word`]. Parsing
/// stops at the first other byte (`-dev`, NUL padding); missing parts are
/// zero; a component out of range gives 0.
pub const fn parse_version(text: &[u8]) -> u32 {
    let mut parts = [0u32; 3];
    let mut part = 0;
    let mut digits = 0;
    let mut index = 0;
    while index < text.len() {
        let byte = text[index];
        if byte.is_ascii_digit() {
            parts[part] = parts[part] * 10 + (byte - b'0') as u32;
            digits += 1;
            if parts[part] > 0xFFFF || digits > 5 {
                return 0;
            }
        } else if byte == b'.' && part < 2 && digits > 0 {
            part += 1;
            digits = 0;
        } else {
            break;
        }
        index += 1;
    }
    if parts[0] > 0xFF || parts[1] > 0xFF {
        return 0;
    }
    version_word(parts[0] as u8, parts[1] as u8, parts[2] as u16)
}

/// The product's acceptance rules for `FW_FINISH`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FwPolicy {
    /// Keys a signed envelope must verify with. Empty trusts nothing.
    pub keys: KeySet,
    /// Version word of the running image.
    pub running_version: u32,
    /// Development build (feature `dev-unsigned-fw`): unsigned images and
    /// (with `flags` bit0) downgrades are accepted.
    pub dev_build: bool,
}

// ── Partition table ────────────────────────────────────────────────────

const PARTITION_ENTRY_SIZE: usize = 32;

/// `(type, subtype, offset, size, label)` of the NCP table's four rows.
const EXPECTED_PARTITIONS: [(u8, u8, u32, u32, &[u8]); 4] = [
    (
        0x01,
        0x00,
        ACTIVE.otadata_offset,
        ACTIVE.otadata_size,
        b"otadata",
    ),
    (
        0x00,
        0x10,
        ACTIVE.ota_0_offset,
        ACTIVE.ota_slot_size,
        b"ota_0",
    ),
    (
        0x00,
        0x11,
        ACTIVE.ota_1_offset,
        ACTIVE.ota_slot_size,
        b"ota_1",
    ),
    (0x01, 0x06, ACTIVE.zbnv_offset, ACTIVE.zbnv_size, b"zbnv"),
];

/// ESP-IDF partition-table entry for row `index` of the NCP table.
pub fn partition_entry(index: usize) -> [u8; PARTITION_ENTRY_SIZE] {
    let (kind, subtype, offset, size, label) = EXPECTED_PARTITIONS[index];
    let mut entry = [0u8; PARTITION_ENTRY_SIZE];
    entry[..2].copy_from_slice(&[0xAA, 0x50]);
    entry[2] = kind;
    entry[3] = subtype;
    entry[4..8].copy_from_slice(&offset.to_le_bytes());
    entry[8..12].copy_from_slice(&size.to_le_bytes());
    entry[12..12 + label.len()].copy_from_slice(label);
    entry
}

/// Check that the flashed partition table is the one every address here is
/// derived from (a 2 MB table under a 4 MB image, or a devkit table, would
/// otherwise make staging write into the wrong partitions).
pub fn validate_partition_table<F: OtaFlash>(flash: &mut F) -> Result<(), FirmwareError> {
    let mut entry = [0u8; PARTITION_ENTRY_SIZE];
    for index in 0..EXPECTED_PARTITIONS.len() {
        let address = PARTITION_TABLE_OFFSET + (index * PARTITION_ENTRY_SIZE) as u32;
        flash.read(address, &mut entry)?;
        if entry != partition_entry(index) {
            log::error!("[FW] partition table row {} is not the NCP layout", index);
            return Err(FirmwareError::HardwareError);
        }
    }
    Ok(())
}

// ── otadata and boot verification (NCP addresses, devkit rules) ────────

/// Read both redundant `otadata` entries.
pub fn read_otadata<F: OtaFlash>(flash: &mut F) -> Result<OtaData, FirmwareError> {
    let mut first = [0u8; ENTRY_SIZE];
    let mut second = [0u8; ENTRY_SIZE];
    flash.read(otadata_sector_offset(0), &mut first)?;
    flash.read(otadata_sector_offset(1), &mut second)?;
    Ok(OtaData::decode([&first, &second]))
}

/// Erase one `otadata` sector, program `activation` and read it back.
fn program_entry<F: OtaFlash>(flash: &mut F, activation: &Activation) -> Result<(), FirmwareError> {
    let address = otadata_sector_offset(activation.sector);
    flash.erase_sector(address)?;
    flash.write(address, &activation.entry.encode())?;
    let mut readback = [0u8; ENTRY_SIZE];
    flash.read(address, &mut readback)?;
    if OtaSelectEntry::decode(&readback) != activation.entry {
        return Err(FirmwareError::ActivateFailed);
    }
    Ok(())
}

fn running_slot<F: OtaFlash>(flash: &mut F) -> Result<u8, RunningImageError> {
    let running = flash.running_slot()?;
    if running >= OTA_SLOT_COUNT {
        return Err(RunningImageError::OutsideOtaSlots);
    }
    Ok(running)
}

fn running_otadata<F: OtaFlash>(flash: &mut F) -> Result<(u8, OtaData), BootCheckError> {
    let running = running_slot(flash)?;
    validate_partition_table(flash)?;
    Ok((running, read_otadata(flash)?))
}

/// Select the previous slot first (after which a power failure already
/// boots it), then mark the abandoned entry `ABORTED`.
fn apply_rollback<F: OtaFlash>(
    flash: &mut F,
    select: &Activation,
    abandon: &Activation,
) -> Result<u8, BootCheckError> {
    program_entry(flash, select)?;
    let slot = read_otadata(flash)?.active_slot();
    let Some(slot) = slot.filter(|slot| select.entry.slot() == Some(*slot)) else {
        return Err(FirmwareError::ActivateFailed.into());
    };
    if let Err(error) = program_entry(flash, abandon) {
        log::warn!(
            "[FW] could not mark the rolled-back image aborted: {:?}",
            error
        );
    }
    Ok(slot)
}

/// The once-per-boot pending-verification step (before the stack starts).
/// [`BootCheck::RolledBack`] requires a reset; [`BootCheck::PendingVerify`]
/// starts the trial window.
pub fn check_boot<F: OtaFlash>(flash: &mut F) -> Result<BootCheck, BootCheckError> {
    let (running, data) = running_otadata(flash)?;
    match data.boot_action(running)? {
        BootAction::None => Ok(BootCheck::Confirmed),
        BootAction::MarkBootAttempt { sector } => {
            // Only clears bits inside the unused `seq_label`; no erase.
            flash.write(otadata_sector_offset(sector) + BOOT_MARK_OFFSET, &BOOT_MARK)?;
            match read_otadata(flash)?.active_entry() {
                Some((active, entry))
                    if active == sector && entry.boot_attempted() && entry.is_unconfirmed() =>
                {
                    Ok(BootCheck::PendingVerify { slot: running })
                }
                _ => Err(FirmwareError::ActivateFailed.into()),
            }
        }
        BootAction::RollBack { select, abandon } => {
            let slot = apply_rollback(flash, &select, &abandon)?;
            Ok(BootCheck::RolledBack { slot })
        }
        BootAction::AdoptRunning(adopt) => {
            program_entry(flash, &adopt)?;
            Ok(BootCheck::FallbackRecorded { slot: running })
        }
    }
}

/// Mark the running image `VALID`. `false` when nothing was pending.
pub fn confirm_boot<F: OtaFlash>(flash: &mut F) -> Result<bool, BootCheckError> {
    let (running, data) = running_otadata(flash)?;
    let Some(confirmation) = data.confirmation(running) else {
        return Ok(false);
    };
    program_entry(flash, &confirmation)?;
    if read_otadata(flash)?.active_slot() != Some(running) {
        return Err(FirmwareError::ActivateFailed.into());
    }
    Ok(true)
}

/// Abandon the running unconfirmed image and reselect the previous slot.
/// `Some(slot)` requires a reset; `None` when nothing is on trial.
pub fn roll_back<F: OtaFlash>(flash: &mut F) -> Result<Option<u8>, BootCheckError> {
    let (running, data) = running_otadata(flash)?;
    match data.rollback(running)? {
        Some(BootAction::RollBack { select, abandon }) => {
            apply_rollback(flash, &select, &abandon).map(Some)
        }
        _ => Ok(None),
    }
}

// ── Cheap slot description (FW_GET_SLOTS) ──────────────────────────────

fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

/// Walk the ESP image headers at `base` without hashing it: the image's
/// appended SHA-256 (what `esptool image_info` prints and the bootloader
/// checks) and the app descriptor's version. `None` when the slot does
/// not start with a plausible image for this chip. Bounded: at most 16
/// segment headers and two small reads.
pub fn describe_slot<F: OtaFlash>(flash: &mut F, base: u32) -> Option<([u8; 32], u32)> {
    let mut header = [0u8; HEADER_SIZE];
    flash.read(base, &mut header).ok()?;
    let parsed = EspImageHeader::parse(&header, EXPECTED_CHIP_ID).ok()?;
    let mut cursor = HEADER_SIZE as u32;
    let mut version = 0;
    for index in 0..parsed.segment_count {
        let mut segment = [0u8; SEGMENT_HEADER_SIZE];
        flash.read(base + cursor, &mut segment).ok()?;
        let data = cursor + SEGMENT_HEADER_SIZE as u32;
        let end = data
            .checked_add(u32_at(&segment, 4))
            .filter(|end| *end <= OTA_SLOT_SIZE - 16 - DIGEST_SIZE as u32)?;
        if index == 0 {
            let mut descriptor = [0u8; 48];
            flash.read(base + data, &mut descriptor).ok()?;
            if u32_at(&descriptor, 0) != APP_DESC_MAGIC {
                return None;
            }
            version = parse_version(&descriptor[16..48]);
        }
        cursor = end;
    }
    let mut digest = [0u8; DIGEST_SIZE];
    flash.read(base + ((cursor + 16) & !15), &mut digest).ok()?;
    Some((digest, version))
}

// ── Staging transaction ────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    Idle,
    Receiving,
    /// `FW_FINISH` succeeded.
    Verified,
    /// `otadata` selects the staged slot; the NCP reboots next.
    Activated,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// Fewer than 32 bytes received.
    Undecided,
    /// Plain ESP image: file offset = slot offset.
    Plain,
    /// MCUboot envelope: the ESP image is file `32..32 + img_size`.
    Envelope { img_size: u32 },
}

/// One `FW_*` transaction over the inactive slot; see the module docs.
pub struct FwUpdate<F: OtaFlash> {
    flash: F,
    policy: FwPolicy,
    running: u8,
    target: u8,
    stage: Stage,
    image: FwImage,
    /// Next expected file offset (`FW_WRITE.next_offset`).
    next: u32,
    kind: Kind,
    prefix: [u8; ENVELOPE_HEADER_LEN],
    trailer: [u8; TRAILER_CAPACITY],
    trailer_len: usize,
    /// ESP image bytes programmed (word aligned).
    flushed: u32,
    tail: [u8; WORD_SIZE as usize],
    tail_len: usize,
    /// Leading slot sectors erased by this transaction.
    sectors_erased: u32,
    hasher: Sha256,
    /// ESP image digest of the verified image.
    staged_digest: [u8; DIGEST_SIZE],
}

fn storage<E: core::fmt::Debug>(error: E) -> PlatformError {
    log::error!("[FW] flash: {:?}", error);
    PlatformError::Storage
}

impl<F: OtaFlash> FwUpdate<F> {
    /// The engine for the running image. Fails without executing-slot
    /// evidence or with a partition table that is not the NCP's: the
    /// product then reports no `FW_UPDATE` capability.
    pub fn new(mut flash: F, policy: FwPolicy) -> Result<Self, (F, BootCheckError)> {
        let running = match running_slot(&mut flash) {
            Ok(running) => running,
            Err(error) => return Err((flash, error.into())),
        };
        if let Err(error) = validate_partition_table(&mut flash) {
            return Err((flash, error.into()));
        }
        Ok(Self {
            flash,
            policy,
            running,
            target: (running + 1) % OTA_SLOT_COUNT,
            stage: Stage::Idle,
            image: FwImage {
                image_size: 0,
                format: 0,
                sha256: [0; 32],
                version: 0,
                flags: 0,
            },
            next: 0,
            kind: Kind::Undecided,
            prefix: [0xFF; ENVELOPE_HEADER_LEN],
            trailer: [0; TRAILER_CAPACITY],
            trailer_len: 0,
            flushed: 0,
            tail: [0xFF; WORD_SIZE as usize],
            tail_len: 0,
            sectors_erased: 0,
            hasher: Sha256::new(),
            staged_digest: [0; DIGEST_SIZE],
        })
    }

    pub fn flash_mut(&mut self) -> &mut F {
        &mut self.flash
    }

    pub fn running_slot(&self) -> u8 {
        self.running
    }

    pub fn target_slot(&self) -> u8 {
        self.target
    }

    /// Slot sectors erased by the current transaction (bounded-work tests).
    pub fn sectors_erased(&self) -> u32 {
        self.sectors_erased
    }

    fn slot_base(&self) -> u32 {
        ota_slot_offset(self.target)
    }

    fn allow_downgrade(&self) -> bool {
        self.policy.dev_build && self.image.flags & FLAG_ALLOW_DOWNGRADE != 0
    }

    fn reset(&mut self) {
        self.stage = Stage::Idle;
        self.next = 0;
        self.kind = Kind::Undecided;
        self.prefix = [0xFF; ENVELOPE_HEADER_LEN];
        self.trailer_len = 0;
        self.flushed = 0;
        self.tail = [0xFF; WORD_SIZE as usize];
        self.tail_len = 0;
        self.sectors_erased = 0;
        self.hasher = Sha256::new();
        self.staged_digest = [0; DIGEST_SIZE];
    }

    /// Drop the transaction; if it wrote into the slot, erase the slot's
    /// first sector so no partial or refused image keeps its ESP magic.
    fn discard(&mut self) -> Result<(), PlatformError> {
        let invalidate = self.sectors_erased > 0 && self.stage != Stage::Activated;
        let base = self.slot_base();
        self.reset();
        if invalidate {
            self.flash.erase_sector(base).map_err(storage)?;
        }
        Ok(())
    }

    /// Discard and report `error` (the discard's own failure is logged).
    fn refuse(&mut self, error: PlatformError) -> PlatformError {
        if let Err(discard) = self.discard() {
            log::error!("[FW] discard failed: {:?}", discard);
        }
        error
    }

    fn check_running(&mut self) -> Result<(), PlatformError> {
        match running_slot(&mut self.flash) {
            Ok(slot) if slot == self.running => Ok(()),
            other => {
                log::error!("[FW] running-image evidence changed: {:?}", other);
                Err(PlatformError::Storage)
            }
        }
    }

    /// The running image still awaits `FW_CONFIRM`.
    pub fn running_on_trial(&mut self) -> Result<bool, PlatformError> {
        Ok(read_otadata(&mut self.flash)
            .map_err(storage)?
            .running_unconfirmed(self.running))
    }

    // ── FW_BEGIN ──

    pub fn begin(&mut self, image: &FwImage) -> Result<FwBegin, PlatformError> {
        if self.stage == Stage::Activated {
            return Err(PlatformError::InvalidState);
        }
        if image.format != FORMAT_ESP_APP {
            return Err(PlatformError::InvalidParameter);
        }
        if image.image_size < MIN_IMAGE_SIZE {
            return Err(PlatformError::InvalidParameter);
        }
        if image.image_size > MAX_FILE_SIZE {
            return Err(PlatformError::TooLarge);
        }
        let downgrade_allowed = self.policy.dev_build && image.flags & FLAG_ALLOW_DOWNGRADE != 0;
        if image.version < self.policy.running_version && !downgrade_allowed {
            log::warn!(
                "[FW] downgrade {:#010x} -> {:#010x} refused",
                self.policy.running_version,
                image.version
            );
            return Err(PlatformError::AccessDenied);
        }
        self.check_running()?;
        if self.running_on_trial()? {
            // The other slot is the rollback target until FW_CONFIRM.
            return Err(PlatformError::InvalidState);
        }
        // A new FW_BEGIN replaces any earlier transaction.
        self.discard()?;
        self.image = *image;
        self.stage = Stage::Receiving;
        log::info!(
            "[FW] staging {} bytes into slot {} (running {})",
            image.image_size,
            self.target,
            self.running
        );
        Ok(FwBegin {
            max_chunk: MAX_CHUNK,
            slot: self.target,
        })
    }

    // ── FW_WRITE ──

    pub fn write(&mut self, offset: u32, data: &[u8]) -> Result<u32, PlatformError> {
        if self.stage != Stage::Receiving {
            return Err(PlatformError::InvalidState);
        }
        if data.len() > MAX_CHUNK as usize {
            return Err(PlatformError::InvalidParameter);
        }
        let end = offset
            .checked_add(data.len() as u32)
            .filter(|end| *end <= self.image.image_size)
            .ok_or(PlatformError::InvalidParameter)?;
        if offset > self.next {
            return Err(PlatformError::InvalidParameter);
        }
        let stored = (self.next.min(end) - offset) as usize;
        if !self.matches_stored(offset, &data[..stored])? {
            return Err(PlatformError::Conflict);
        }
        let fresh = &data[stored..];
        if !fresh.is_empty() {
            if let Err(error) = self.append(fresh) {
                return Err(self.refuse(error));
            }
            self.hasher.update(fresh);
        }
        Ok(self.next)
    }

    /// Whether `data` equals what is stored at file `offset`
    /// (`offset + data.len() <= next`).
    fn matches_stored(&mut self, offset: u32, data: &[u8]) -> Result<bool, PlatformError> {
        let mut at = offset;
        let mut rest = data;
        let mut buffer = [0u8; 64];
        while !rest.is_empty() {
            let take = if at < PREFIX {
                let take = ((PREFIX - at) as usize).min(rest.len());
                if self.prefix[at as usize..at as usize + take] != rest[..take] {
                    return Ok(false);
                }
                take
            } else {
                let (base, body_len) = match self.kind {
                    Kind::Undecided => return Ok(false),
                    Kind::Plain => (0, self.image.image_size),
                    Kind::Envelope { img_size } => (PREFIX, img_size),
                };
                let body_end = base + body_len;
                if at < body_end {
                    let take = ((body_end - at) as usize).min(rest.len()).min(buffer.len());
                    self.stored_body(at - base, &mut buffer[..take])?;
                    if buffer[..take] != rest[..take] {
                        return Ok(false);
                    }
                    take
                } else {
                    let start = (at - body_end) as usize;
                    let take = rest.len();
                    if self.trailer[start..start + take] != *rest {
                        return Ok(false);
                    }
                    take
                }
            };
            at += take as u32;
            rest = &rest[take..];
        }
        Ok(true)
    }

    /// Staged ESP image bytes at `offset` (programmed or in the tail).
    fn stored_body(&mut self, offset: u32, out: &mut [u8]) -> Result<(), PlatformError> {
        let from_flash = (self.flushed.saturating_sub(offset) as usize).min(out.len());
        if from_flash > 0 {
            self.flash
                .read(self.slot_base() + offset, &mut out[..from_flash])
                .map_err(storage)?;
        }
        for (index, byte) in out[from_flash..].iter_mut().enumerate() {
            *byte = self.tail[(offset + (from_flash + index) as u32 - self.flushed) as usize];
        }
        Ok(())
    }

    /// Accept new bytes at `next`.
    fn append(&mut self, data: &[u8]) -> Result<(), PlatformError> {
        let mut rest = data;
        while !rest.is_empty() {
            if self.next < PREFIX {
                let at = self.next as usize;
                let take = (PREFIX as usize - at).min(rest.len());
                self.prefix[at..at + take].copy_from_slice(&rest[..take]);
                self.next += take as u32;
                rest = &rest[take..];
                if self.next == PREFIX {
                    self.classify()?;
                }
                continue;
            }
            let (base, body_len) = match self.kind {
                Kind::Undecided => return Err(PlatformError::InvalidState),
                Kind::Plain => (0, self.image.image_size),
                Kind::Envelope { img_size } => (PREFIX, img_size),
            };
            let body_end = base + body_len;
            let take = if self.next < body_end {
                let take = ((body_end - self.next) as usize).min(rest.len());
                self.write_body(&rest[..take])?;
                take
            } else {
                let start = self.trailer_len;
                let take = rest.len();
                // Sized at classification; `end <= image_size` holds.
                self.trailer[start..start + take].copy_from_slice(rest);
                self.trailer_len += take;
                take
            };
            self.next += take as u32;
            rest = &rest[take..];
        }
        Ok(())
    }

    /// Decide plain image or envelope from the first 32 bytes.
    fn classify(&mut self) -> Result<(), PlatformError> {
        let size = self.image.image_size;
        if self.prefix[0] == esp32_zigbee_devkit_product::esp_image::IMAGE_MAGIC {
            if size > OTA_SLOT_SIZE {
                return Err(PlatformError::TooLarge);
            }
            self.kind = Kind::Plain;
            let prefix = self.prefix;
            return self.write_body(&prefix);
        }
        let header = parse_envelope_header(&self.prefix).map_err(|error| {
            log::warn!("[FW] neither an ESP image nor an envelope: {:?}", error);
            PlatformError::VerifyFailed
        })?;
        if header.img_size > OTA_SLOT_SIZE {
            return Err(PlatformError::TooLarge);
        }
        let trailer = size
            .checked_sub(PREFIX)
            .and_then(|rest| rest.checked_sub(header.img_size))
            .filter(|trailer| *trailer >= 4)
            .ok_or(PlatformError::VerifyFailed)?;
        if trailer as usize > TRAILER_CAPACITY {
            return Err(PlatformError::TooLarge);
        }
        self.kind = Kind::Envelope {
            img_size: header.img_size,
        };
        Ok(())
    }

    /// Append ESP image bytes: whole words are programmed, a sub-word rest
    /// waits in `tail`.
    fn write_body(&mut self, data: &[u8]) -> Result<(), PlatformError> {
        let word = WORD_SIZE as usize;
        let mut rest = data;
        if self.tail_len > 0 {
            let take = (word - self.tail_len).min(rest.len());
            self.tail[self.tail_len..self.tail_len + take].copy_from_slice(&rest[..take]);
            self.tail_len += take;
            rest = &rest[take..];
            if self.tail_len == word {
                let tail = self.tail;
                self.program(&tail)?;
                self.tail = [0xFF; WORD_SIZE as usize];
                self.tail_len = 0;
            }
        }
        if !rest.is_empty() {
            let whole = rest.len() - rest.len() % word;
            if whole > 0 {
                self.program(&rest[..whole])?;
            }
            let remainder = &rest[whole..];
            self.tail[..remainder.len()].copy_from_slice(remainder);
            self.tail_len = remainder.len();
        }
        Ok(())
    }

    /// Program word-aligned bytes at `flushed`, erasing each sector right
    /// before its first byte.
    fn program(&mut self, data: &[u8]) -> Result<(), PlatformError> {
        let mut cursor = self.flushed;
        let mut rest = data;
        while !rest.is_empty() {
            let sector = cursor / SECTOR_SIZE;
            while self.sectors_erased <= sector {
                let address = self.slot_base() + self.sectors_erased * SECTOR_SIZE;
                self.flash.erase_sector(address).map_err(storage)?;
                self.sectors_erased += 1;
            }
            let room = ((sector + 1) * SECTOR_SIZE - cursor) as usize;
            let take = room.min(rest.len());
            self.flash
                .write(self.slot_base() + cursor, &rest[..take])
                .map_err(storage)?;
            cursor += take as u32;
            rest = &rest[take..];
        }
        self.flushed = cursor;
        Ok(())
    }

    // ── FW_FINISH ──

    pub fn finish(&mut self) -> Result<u8, PlatformError> {
        match self.stage {
            Stage::Receiving => {}
            // A retried FW_FINISH after a lost response.
            Stage::Verified => return Ok(0),
            Stage::Idle | Stage::Activated => return Err(PlatformError::InvalidState),
        }
        if self.next != self.image.image_size {
            // Incomplete: the host may still send the missing chunks.
            return Err(PlatformError::VerifyFailed);
        }
        match self.verify() {
            Ok(()) => {
                self.stage = Stage::Verified;
                log::info!("[FW] slot {} verified", self.target);
                Ok(0)
            }
            Err(error) => Err(self.refuse(error)),
        }
    }

    fn verify(&mut self) -> Result<(), PlatformError> {
        let digest = core::mem::take(&mut self.hasher).finalize();
        if digest != self.image.sha256 {
            log::warn!("[FW] SHA-256 of the transferred file differs");
            return Err(PlatformError::VerifyFailed);
        }
        // Program the last partial word, padded with the erased value.
        if self.tail_len > 0 {
            let tail = self.tail;
            self.program(&tail)?;
            self.tail_len = 0;
        }
        self.staged_digest = self.verify_esp_image()?;
        match self.kind {
            Kind::Undecided => Err(PlatformError::VerifyFailed),
            Kind::Plain if self.policy.dev_build => {
                log::warn!("[FW] unsigned image accepted (development build)");
                Ok(())
            }
            Kind::Plain => {
                log::warn!("[FW] unsigned image refused");
                Err(PlatformError::AccessDenied)
            }
            Kind::Envelope { img_size } => self.verify_envelope(img_size),
        }
    }

    fn body_len(&self) -> u32 {
        match self.kind {
            Kind::Envelope { img_size } => img_size,
            _ => self.image.image_size,
        }
    }

    /// Structure, chip, checksum and appended SHA-256 of the staged ESP
    /// image, read back from flash.
    fn verify_esp_image(&mut self) -> Result<[u8; DIGEST_SIZE], PlatformError> {
        let compatibility = self.flash.image_compatibility().map_err(storage)?;
        let base = self.slot_base();
        let size = self.body_len();
        let flash = &mut self.flash;
        verify_image(size, base, compatibility, |offset, buffer| {
            flash.read(base + offset, buffer)
        })
        .map_err(|error| match error {
            ImageReadError::Read(error) => storage(error),
            ImageReadError::Image(error) => {
                log::warn!("[FW] staged ESP image rejected: {:?}", error);
                PlatformError::VerifyFailed
            }
        })
    }

    fn verify_envelope(&mut self, img_size: u32) -> Result<(), PlatformError> {
        let base = self.slot_base();
        let prefix = self.prefix;
        let mut view = SlotView {
            flash: &mut self.flash,
            base,
            len: img_size,
        };
        let valid = validate_envelope(
            &prefix,
            &self.trailer[..self.trailer_len],
            &mut view,
            0,
            &self.policy.keys,
        )
        .map_err(|error| {
            log::warn!("[FW] envelope rejected: {:?}", error);
            match error {
                EnvelopeError::BadSignature => PlatformError::AccessDenied,
                EnvelopeError::Flash => PlatformError::Storage,
                EnvelopeError::NoImage
                | EnvelopeError::Malformed
                | EnvelopeError::Unsupported
                | EnvelopeError::HashMismatch => PlatformError::VerifyFailed,
            }
        })?;
        self.check_envelope_version(&valid.header)
    }

    fn check_envelope_version(&self, header: &ImageHeader) -> Result<(), PlatformError> {
        let signed = header.version;
        if !signed
            .cmp_release(&image_version(self.image.version))
            .is_eq()
        {
            log::warn!("[FW] envelope version differs from FW_BEGIN.version");
            return Err(PlatformError::VerifyFailed);
        }
        if signed
            .cmp_release(&image_version(self.policy.running_version))
            .is_lt()
            && !self.allow_downgrade()
        {
            log::warn!("[FW] signed downgrade refused");
            return Err(PlatformError::AccessDenied);
        }
        Ok(())
    }

    // ── FW_ACTIVATE ──

    pub fn activate(&mut self) -> Result<(), PlatformError> {
        match self.stage {
            Stage::Verified => {}
            Stage::Activated => return Ok(()),
            Stage::Idle | Stage::Receiving => return Err(PlatformError::InvalidState),
        }
        self.check_running()?;
        // Re-check what the bootloader will start (flash errors or
        // corruption after FW_FINISH).
        if self.verify_esp_image()? != self.staged_digest {
            return Err(self.refuse(PlatformError::VerifyFailed));
        }
        let data = read_otadata(&mut self.flash).map_err(storage)?;
        let activation = data.activation_for(self.target).map_err(storage)?;
        // From here on the slot may be selected: never discarded silently.
        self.stage = Stage::Activated;
        program_entry(&mut self.flash, &activation).map_err(storage)?;
        if read_otadata(&mut self.flash)
            .map_err(storage)?
            .active_slot()
            != Some(self.target)
        {
            return Err(PlatformError::Storage);
        }
        log::info!(
            "[FW] slot {} selected for a trial boot (otadata seq {})",
            self.target,
            activation.entry.seq
        );
        Ok(())
    }

    // ── FW_ABORT ──

    /// Discard the transaction. After `FW_ACTIVATE` (before the reboot) the
    /// running slot is selected again first, so abort is always safe.
    pub fn abort(&mut self) -> Result<(), PlatformError> {
        if self.stage == Stage::Activated {
            let data = read_otadata(&mut self.flash).map_err(storage)?;
            let mut reselect = data.activation_for(self.running).map_err(storage)?;
            reselect.entry = reselect.entry.with_state(STATE_VALID);
            program_entry(&mut self.flash, &reselect).map_err(storage)?;
            if read_otadata(&mut self.flash)
                .map_err(storage)?
                .active_slot()
                != Some(self.running)
            {
                return Err(PlatformError::Storage);
            }
            // Now the staged slot may be invalidated like any other.
            self.stage = Stage::Verified;
        }
        if self.stage != Stage::Idle {
            log::info!("[FW] aborted at offset {}", self.next);
        }
        self.discard()
    }

    // ── FW_GET_SLOTS ──

    /// Both slots: the running one (confirmed or trial) and the other one
    /// (staged by this transaction, a valid-looking older image, or empty).
    pub fn slots(&mut self, trial: bool, out: &mut [FwSlot]) -> usize {
        let mut entries = [FwSlot {
            slot: 0,
            state: slot_state::EMPTY,
            version: 0,
            sha256: [0; 32],
        }; OTA_SLOT_COUNT as usize];
        for (slot, entry) in entries.iter_mut().enumerate() {
            let slot = slot as u8;
            entry.slot = slot;
            if slot == self.running {
                entry.state = if trial {
                    slot_state::RUNNING_TRIAL
                } else {
                    slot_state::RUNNING
                };
                entry.version = self.policy.running_version;
                if let Some((digest, _)) = describe_slot(&mut self.flash, ota_slot_offset(slot)) {
                    entry.sha256 = digest;
                }
                continue;
            }
            match self.stage {
                Stage::Receiving => {}
                Stage::Verified | Stage::Activated => {
                    entry.state = slot_state::STAGED;
                    entry.version = self.image.version;
                    entry.sha256 = self.staged_digest;
                }
                Stage::Idle => {
                    if let Some((digest, version)) =
                        describe_slot(&mut self.flash, ota_slot_offset(slot))
                    {
                        entry.state = slot_state::INACTIVE;
                        entry.version = version;
                        entry.sha256 = digest;
                    }
                }
            }
        }
        let count = out.len().min(entries.len());
        out[..count].copy_from_slice(&entries[..count]);
        count
    }
}

/// The staged ESP image as byte-readable flash for `validate_envelope`.
struct SlotView<'a, F> {
    flash: &'a mut F,
    base: u32,
    len: u32,
}

#[derive(Debug)]
struct ViewFault;

impl NorFlashError for ViewFault {
    fn kind(&self) -> NorFlashErrorKind {
        NorFlashErrorKind::OutOfBounds
    }
}

impl<F> ErrorType for SlotView<'_, F> {
    type Error = ViewFault;
}

impl<F: OtaFlash> ReadNorFlash for SlotView<'_, F> {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), ViewFault> {
        offset
            .checked_add(bytes.len() as u32)
            .filter(|end| *end <= self.len)
            .ok_or(ViewFault)?;
        self.flash
            .read(self.base + offset, bytes)
            .map_err(|_| ViewFault)
    }

    fn capacity(&self) -> usize {
        self.len as usize
    }
}

#[cfg(test)]
#[path = "fw_tests.rs"]
mod tests;
