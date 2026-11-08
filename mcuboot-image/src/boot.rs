//! Boot decision, install and trial accounting.
//!
//! The default mode ([`Mode::Overwrite`]) is compatible with MCUboot's
//! `MCUBOOT_OVERWRITE_ONLY` single-image mode. Every boot:
//!
//! 1. If the secondary slot ends with [`BOOT_MAGIC`](crate::trailer::BOOT_MAGIC),
//!    validate the secondary image (hash, signature, geometry) and apply
//!    downgrade prevention: refuse an image whose release version is lower
//!    than that of a primary image that *itself validates fully*. A torn,
//!    half-erased or otherwise invalid primary never blocks a valid update.
//! 2. An accepted image is installed: erase the primary sectors it needs,
//!    copy it, re-validate the primary, record the install in the trial log
//!    (when trial boots are configured), then erase the secondary trailer
//!    sector (dropping the request) and the secondary header sector. A
//!    rejected image is dropped the same way without touching the primary.
//! 3. Validate the primary, account one trial boot if it is an unconfirmed
//!    install, and hand back its vector table address.
//!
//! **Power-fail safety.** The request (the magic) survives until the primary
//! holds a complete, validated copy, so an interrupted install simply runs
//! again on the next boot: the secondary is never modified before that, and
//! a partly erased or partly written primary does not validate and so cannot
//! trigger the downgrade rule. Interrupting the final erases leaves a valid
//! primary and a secondary without a request.
//!
//! **Trial.** With [`BootConfig::trial_boots`] `> 0` an installed image runs
//! on trial until the application confirms it ([`confirm`]). Overwrite mode
//! cannot go back (the previous image is gone), so an image still
//! unconfirmed after that many boots keeps booting and is reported as
//! [`Trial::Unconfirmed`]. [`Mode::SwapMove`] (feature `swap-move`) keeps
//! the previous image in the secondary slot and reverts to it.
//!
//! **Watchdog.** `progress` is called before every long step: each full
//! image validation, each [`ERASE_STEP`] of an install erase and each
//! [`COPY_CHUNK`] copied. A loader that kicks its watchdog there needs a
//! period longer than the longest single step (one full-slot validation or
//! one `ERASE_STEP` erase), not the whole install.

use embedded_storage::nor_flash::NorFlash;

use crate::image::{ImageError, ImageHeader, TrustedKey, ValidImage, validate};
use crate::status::{Log, LogState, Record};
use crate::trailer::{MagicState, read_magic};

/// Copy unit; bounded stack use.
pub const COPY_CHUNK: usize = 256;
/// Largest erase between two progress calls: one 64 KiB block (or one
/// sector, if larger). Aligned steps keep block erases available to drivers
/// that use them.
pub const ERASE_STEP: u32 = 64 * 1024;

/// Two equally sized slots in one flash device, by absolute address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layout {
    pub primary: u32,
    pub secondary: u32,
    pub slot_size: u32,
    pub sector_size: u32,
}

impl Layout {
    /// The longest image (header through TLVs) a slot may hold: the last
    /// sector is reserved for the trailer.
    pub const fn max_image_len(&self) -> u32 {
        self.slot_size - self.sector_size
    }

    pub const fn trailer_sector(&self, slot: u32) -> u32 {
        slot + self.slot_size - self.sector_size
    }

    pub(crate) const fn round_up(&self, len: u32) -> u32 {
        len.div_ceil(self.sector_size) * self.sector_size
    }
}

/// How an accepted update reaches the primary slot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// Copy over the primary; the previous image is lost.
    Overwrite,
    /// Exchange the slots through one spare sector, keeping the previous
    /// image in the secondary slot for revert. The status log lives in
    /// `status`: two sectors outside both slots.
    #[cfg(feature = "swap-move")]
    SwapMove { status: u32 },
}

/// Everything the engine needs; build-time configuration of a product.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootConfig<'a> {
    pub layout: Layout,
    /// Keys an image may be signed with (at most
    /// [`MAX_TRUSTED_KEYS`](crate::image::MAX_TRUSTED_KEYS) are useful).
    pub keys: &'a [TrustedKey],
    /// Boots an installed image may take to confirm itself; `0` disables
    /// trial accounting (every image is permanent).
    pub trial_boots: u8,
    pub mode: Mode,
}

impl<'a> BootConfig<'a> {
    /// Overwrite mode without trial accounting.
    pub const fn overwrite(layout: Layout, keys: &'a [TrustedKey]) -> Self {
        Self {
            layout,
            keys,
            trial_boots: 0,
            mode: Mode::Overwrite,
        }
    }
}

/// What happened to a pending upgrade request on this boot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Update {
    /// No request magic in the secondary slot.
    None,
    /// The secondary image was installed in the primary slot.
    Installed,
    /// The request was dropped because the image failed validation.
    Rejected(ImageError),
    /// The request was dropped by downgrade prevention.
    Downgrade,
    /// An unconfirmed image was replaced by the previous one.
    #[cfg(feature = "swap-move")]
    Reverted,
}

/// Trial state of the image being started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Trial {
    /// Not on trial: trial accounting is off, the image was not installed
    /// by this loader (factory image, older loader), or it was reverted to.
    Permanent,
    /// Unconfirmed install; this is its `boot`-th start (from 1).
    Testing { boot: u16 },
    /// The application confirmed the image.
    Confirmed,
    /// Still unconfirmed after the configured trial boots and no revert was
    /// possible; it keeps booting.
    Unconfirmed,
}

/// Progress for status indication; the loader may blink an LED.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootEvent {
    Validating,
    Erasing,
    Copying {
        done: u32,
        total: u32,
    },
    Dropping,
    /// A long step continues (a full validation or an erase step starts);
    /// nothing to show. Kick a watchdog here.
    Busy,
    /// A revert to the previous image starts (or resumes).
    #[cfg(feature = "swap-move")]
    Reverting,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bootable {
    pub image: ValidImage,
    /// Absolute address of the application's vector table.
    pub vector_table: u32,
    pub update: Update,
    pub trial: Trial,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootError {
    /// A flash erase, write or read failed.
    Flash,
    /// The installed copy does not validate although the source did. The
    /// request is kept; retrying needs a reset.
    CopyCorrupt,
    /// The status log ran out of room (more interrupted record writes than
    /// its slack allows).
    StatusFull,
    /// The configuration is inconsistent: the swap status area overlaps a
    /// slot, or is too small for a full exchange plus the trial records.
    /// Detected before anything is written.
    Config,
    /// Nothing valid to boot.
    NoBootableImage { primary: ImageError, update: Update },
}

/// Run one boot decision. `progress` is called before each long step.
pub fn boot<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    mut progress: impl FnMut(BootEvent),
) -> Result<Bootable, BootError> {
    match config.mode {
        Mode::Overwrite => overwrite(flash, config, &mut progress),
        #[cfg(feature = "swap-move")]
        Mode::SwapMove { status } => crate::swap::boot(flash, config, status, &mut progress),
    }
}

fn overwrite<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    progress: &mut impl FnMut(BootEvent),
) -> Result<Bootable, BootError> {
    let update = process_request(flash, config, progress)?;
    let image = validate_primary(flash, config, update, progress)?;
    let trial = if config.trial_boots == 0 {
        Trial::Permanent
    } else {
        let log = Log::trailer(&config.layout);
        match log.read(flash) {
            Ok(mut state) => account_boot(flash, &log, &mut state, config.trial_boots),
            Err(_) => Trial::Unconfirmed,
        }
    };
    Ok(bootable(&config.layout, image, update, trial))
}

pub(crate) fn bootable(
    layout: &Layout,
    image: ValidImage,
    update: Update,
    trial: Trial,
) -> Bootable {
    Bootable {
        image,
        vector_table: layout.primary + image.header.hdr_size as u32,
        update,
        trial,
    }
}

pub(crate) fn validate_primary<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    update: Update,
    progress: &mut impl FnMut(BootEvent),
) -> Result<ValidImage, BootError> {
    progress(BootEvent::Validating);
    match validate_slot(flash, config, config.layout.primary, progress) {
        Ok(image) => Ok(image),
        Err(ImageError::Flash) => Err(BootError::Flash),
        Err(primary) => Err(BootError::NoBootableImage { primary, update }),
    }
}

/// Account one start of a valid primary whose install `state` describes.
/// Accounting never blocks a boot: a log that cannot be written reports
/// [`Trial::Unconfirmed`].
pub(crate) fn account_boot<F: NorFlash>(
    flash: &mut F,
    log: &Log,
    state: &mut LogState,
    trial_boots: u8,
) -> Trial {
    if !state.done {
        Trial::Permanent
    } else if state.confirmed {
        Trial::Confirmed
    } else if state.attempts >= u16::from(trial_boots) {
        Trial::Unconfirmed
    } else {
        match log.append(flash, state, Record::Attempt) {
            Ok(()) => Trial::Testing {
                boot: state.attempts,
            },
            Err(_) => Trial::Unconfirmed,
        }
    }
}

/// A pending upgrade request, examined.
pub(crate) enum Request {
    /// No request magic in the secondary slot.
    None,
    /// Drop the request without touching the primary.
    Drop(Update),
    /// Install this image.
    Install(ValidImage),
}

/// Look at the secondary slot: request magic, image validity and
/// downgrade prevention.
pub(crate) fn examine_request<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    progress: &mut impl FnMut(BootEvent),
) -> Result<Request, BootError> {
    let layout = &config.layout;
    let magic =
        read_magic(flash, layout.secondary, layout.slot_size).map_err(|_| BootError::Flash)?;
    if magic != MagicState::Good {
        return Ok(Request::None);
    }
    progress(BootEvent::Validating);
    Ok(
        match validate_slot(flash, config, layout.secondary, progress) {
            Err(ImageError::Flash) => return Err(BootError::Flash),
            Err(error) => Request::Drop(Update::Rejected(error)),
            Ok(candidate) if is_downgrade(flash, config, &candidate, progress)? => {
                Request::Drop(Update::Downgrade)
            }
            Ok(candidate) => Request::Install(candidate),
        },
    )
}

fn process_request<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    progress: &mut impl FnMut(BootEvent),
) -> Result<Update, BootError> {
    let layout = &config.layout;
    let update = match examine_request(flash, config, progress)? {
        Request::None => return Ok(Update::None),
        Request::Drop(update) => update,
        Request::Install(candidate) => {
            install(flash, config, &candidate, progress)?;
            Update::Installed
        }
    };
    progress(BootEvent::Dropping);
    erase_sector(
        flash,
        layout.trailer_sector(layout.secondary),
        layout.sector_size,
    )?;
    erase_sector(flash, layout.secondary, layout.sector_size)?;
    Ok(update)
}

/// Downgrade prevention against the primary, applied only when the primary
/// validates fully (hash and signature). The header alone is not evidence:
/// an interrupted erase can leave a header whose version reads higher.
fn is_downgrade<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    candidate: &ValidImage,
    progress: &mut impl FnMut(BootEvent),
) -> Result<bool, BootError> {
    let layout = &config.layout;
    let claimed = ImageHeader::read(flash, layout.primary).map_err(|_| BootError::Flash)?;
    if !claimed.is_some_and(|header| {
        candidate
            .header
            .version
            .cmp_release(&header.version)
            .is_lt()
    }) {
        return Ok(false);
    }
    match validate_slot(flash, config, layout.primary, progress) {
        Ok(primary) => Ok(candidate
            .header
            .version
            .cmp_release(&primary.header.version)
            .is_lt()),
        Err(ImageError::Flash) => Err(BootError::Flash),
        Err(_) => Ok(false),
    }
}

fn install<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    image: &ValidImage,
    progress: &mut impl FnMut(BootEvent),
) -> Result<(), BootError> {
    let layout = &config.layout;
    let total = image.end.div_ceil(F::WRITE_SIZE as u32) * F::WRITE_SIZE as u32;
    progress(BootEvent::Erasing);
    let log = Log::trailer(layout);
    if config.trial_boots != 0 {
        // A fresh trial log; the install record follows the copy.
        erase_sector(
            flash,
            layout.trailer_sector(layout.primary),
            layout.sector_size,
        )?;
    }
    erase_range(
        flash,
        layout,
        layout.primary,
        layout.primary + layout.round_up(total),
        progress,
    )?;

    copy(
        flash,
        layout.secondary,
        layout.primary,
        total,
        0,
        total,
        progress,
    )?;

    progress(BootEvent::Validating);
    match validate_slot(flash, config, layout.primary, progress) {
        Ok(copy) if copy.digest == image.digest => {}
        Err(ImageError::Flash) => return Err(BootError::Flash),
        _ => return Err(BootError::CopyCorrupt),
    }
    if config.trial_boots != 0 {
        let mut state = LogState {
            next: log.start,
            ..LogState::default()
        };
        log.append(flash, &mut state, Record::Done)?;
    }
    Ok(())
}

/// Copy `len` bytes from `from` to the erased `to`, skipping erased chunks.
/// Progress reports `base + offset` of `total`.
pub(crate) fn copy<F: NorFlash>(
    flash: &mut F,
    from: u32,
    to: u32,
    len: u32,
    base: u32,
    total: u32,
    progress: &mut impl FnMut(BootEvent),
) -> Result<(), BootError> {
    let mut buffer = [0u8; COPY_CHUNK];
    let mut offset = 0;
    while offset < len {
        progress(BootEvent::Copying {
            done: base + offset,
            total,
        });
        let chunk = core::cmp::min(COPY_CHUNK as u32, len - offset) as usize;
        let data = &mut buffer[..chunk];
        flash
            .read(from + offset, data)
            .map_err(|_| BootError::Flash)?;
        if data.iter().any(|byte| *byte != 0xFF) {
            flash
                .write(to + offset, data)
                .map_err(|_| BootError::Flash)?;
        }
        offset += chunk as u32;
    }
    Ok(())
}

/// Validate the image in `slot`, reporting [`BootEvent::Busy`] first: a
/// watchdog sees at most one full validation between two kicks.
pub(crate) fn validate_slot<F: NorFlash>(
    flash: &mut F,
    config: &BootConfig<'_>,
    slot: u32,
    progress: &mut impl FnMut(BootEvent),
) -> Result<ValidImage, ImageError> {
    progress(BootEvent::Busy);
    validate(flash, slot, config.layout.max_image_len(), config.keys)
}

/// Erase the sector-aligned `[from, to)` in [`ERASE_STEP`]-aligned steps
/// (one sector each if sectors are larger, or do not divide the step),
/// reporting [`BootEvent::Busy`] before each step.
pub(crate) fn erase_range<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    from: u32,
    to: u32,
    progress: &mut impl FnMut(BootEvent),
) -> Result<(), BootError> {
    let step = if ERASE_STEP.is_multiple_of(layout.sector_size) {
        ERASE_STEP
    } else {
        layout.sector_size
    };
    let mut start = from;
    while start < to {
        let end = core::cmp::min(to, start - start % step + step);
        progress(BootEvent::Busy);
        flash.erase(start, end).map_err(|_| BootError::Flash)?;
        start = end;
    }
    Ok(())
}

pub(crate) fn erase_sector<F: NorFlash>(
    flash: &mut F,
    address: u32,
    size: u32,
) -> Result<(), BootError> {
    flash
        .erase(address, address + size)
        .map_err(|_| BootError::Flash)
}

/// Trial state as the application sees it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrialState {
    /// Not on trial (see [`Trial::Permanent`]).
    Permanent,
    /// Installed and not yet confirmed; started `attempts` times so far.
    Testing {
        attempts: u16,
    },
    Confirmed,
}

fn trial_log<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    mode: Mode,
) -> Result<Option<(Log, LogState)>, BootError> {
    match mode {
        Mode::Overwrite => {
            let log = Log::trailer(layout);
            let state = log.read(flash).map_err(|_| BootError::Flash)?;
            Ok(Some((log, state)))
        }
        #[cfg(feature = "swap-move")]
        Mode::SwapMove { status } => crate::swap::trial_log(flash, layout, status),
    }
}

/// `[start, end)` of the overwrite-mode trial log (inside the primary
/// slot's trailer sector, which never holds image bytes). An application's
/// flash view must admit writes — never erases — here for [`confirm`].
pub const fn trial_log_area(layout: &Layout) -> (u32, u32) {
    let log = Log::trailer(layout);
    (log.start, log.end)
}

/// The trial state of the running (primary) image.
pub fn trial_state<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    mode: Mode,
) -> Result<TrialState, BootError> {
    Ok(match trial_log(flash, layout, mode)? {
        Some((_, state)) if state.done && state.confirmed => TrialState::Confirmed,
        Some((_, state)) if state.done => TrialState::Testing {
            attempts: state.attempts,
        },
        _ => TrialState::Permanent,
    })
}

/// Confirm the running image: append the confirm record if, and only if, it
/// is on trial. Returns the state found before. One flash write at most; an
/// image that is permanent or already confirmed costs nothing.
pub fn confirm<F: NorFlash>(
    flash: &mut F,
    layout: &Layout,
    mode: Mode,
) -> Result<TrialState, BootError> {
    match trial_log(flash, layout, mode)? {
        Some((log, mut state)) if state.done && !state.confirmed => {
            let before = TrialState::Testing {
                attempts: state.attempts,
            };
            log.append(flash, &mut state, Record::Confirmed)?;
            Ok(before)
        }
        Some((_, state)) if state.done => Ok(TrialState::Confirmed),
        _ => Ok(TrialState::Permanent),
    }
}
