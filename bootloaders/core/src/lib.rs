//! Chip-independent core of the zigbee-rs bootloaders.
//!
//! Every zigbee-rs loader runs the same sequence, implemented once here on
//! top of [`mcuboot_image`] (image format, signature check and the install
//! engine):
//!
//! 1. install a pending, valid, non-downgrade image from the secondary slot
//!    ([`mcuboot_image::boot`], power-fail safe and resumable; overwrite by
//!    default, swap-move with revert as the `swap-move` option);
//! 2. validate the primary image (SHA-256 and an ECDSA P-256 or, with
//!    feature `ed25519`, Ed25519 signature by one of up to three trusted
//!    keys) and account a trial boot of an unconfirmed install;
//! 3. let the chip check the entry of the validated image and start it, or
//!    report the failure and stop.
//!
//! A chip shell (`bootloaders/<chip>-boot`) implements [`Platform`]: status
//! output, the architecture entry check ([`entry`]), cache or clock
//! hand-over and the jump. A product's loader binary supplies the slot
//! [`BootConfig`] (layout, trusted keys — build-time inputs, never
//! constants in this crate — trial boots and install mode), the flash
//! driver and the status hardware, then calls the shell.
//!
//! Shared building blocks:
//!
//! * [`flash::BootFlash`] — a flash view that refuses to erase or program
//!   anything outside the two slots (and the swap status area);
//! * [`entry`] — RISC-V fixed-entry and Cortex-M vector-table checks;
//! * [`console`] and [`indicator`] — status lines and LED blinking from
//!   [`BootEvent`]s;
//! * [`loader_info`] and [`loader_block`] — the identity records
//!   applications read;
//! * `sim` (feature `sim`) — a NOR flash simulator with power-cut
//!   injection for host tests.

#![cfg_attr(not(any(test, feature = "sim")), no_std)]

pub mod console;
pub mod entry;
pub mod flash;
pub mod indicator;
#[cfg(any(test, feature = "sim"))]
pub mod sim;

#[cfg(test)]
mod tests;

pub use embedded_storage;
use embedded_storage::nor_flash::NorFlash;
pub use mcuboot_image::{
    self, BootConfig, BootError, BootEvent, Bootable, ERASE_STEP, ImageError, ImageVersion, Layout,
    LoaderInfo, Mode, Trial, TrustedKey, Update, loader_block,
};

/// The chip-specific half of a loader.
///
/// Flash access is not part of this trait: the engine takes any
/// [`NorFlash`] (normally wrapped in [`flash::BootFlash`]), whose
/// `ERASE_SIZE` must divide the layout's `sector_size`.
pub trait Platform {
    /// Engine progress, for status output. Called before every long step
    /// and once per copied chunk while installing.
    fn progress(&mut self, event: BootEvent);

    /// Service an armed watchdog. Called before every [`Platform::progress`]
    /// event. The default does nothing: today's loaders leave the watchdog
    /// disarmed. Image validation is a single step, so a platform that arms
    /// its watchdog needs a period longer than one full-slot validation.
    fn kick_watchdog(&mut self) {}

    /// The install and validation decision is complete. Called exactly once
    /// per boot, before any entry check.
    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {
        let _ = outcome;
    }

    /// Check the entry of the validated primary image and prepare the
    /// hand-over (for example invalidate an instruction cache). Return the
    /// address [`Platform::start`] receives, or `None` to refuse the image.
    fn prepare(&mut self, layout: &Layout, bootable: &Bootable) -> Option<u32>;

    /// Transfer control to the application.
    ///
    /// # Safety
    ///
    /// `entry` must be a value [`Platform::prepare`] returned during this
    /// boot for an image validated during this boot.
    unsafe fn start(&mut self, entry: u32) -> !;

    /// Nothing can be started: indicate it and stop (halt, sleep or reset
    /// into recovery).
    fn fail(&mut self) -> !;
}

/// End of the validated primary image payload (header plus code, without
/// TLVs), as an absolute address.
pub const fn image_end(layout: &Layout, bootable: &Bootable) -> u32 {
    layout.primary + bootable.image.header.hdr_size as u32 + bootable.image.header.img_size
}

/// Encode the record applications read to detect a zigbee-rs loader. Place
/// the result with `#[link_section]` at the address the product's
/// application expects (by convention the last 32 bytes of the loader
/// region).
pub const fn loader_info(loader_version: u32, key_xy: &[u8; 64]) -> [u8; 32] {
    LoaderInfo::new(0, loader_version, key_xy).encode()
}

/// Run the boot decision and the entry check without starting anything.
/// Returns the address to start, if any. Host tests drive this directly.
pub fn launch<F: NorFlash, P: Platform>(
    flash: &mut F,
    config: &BootConfig<'_>,
    platform: &mut P,
) -> Option<u32> {
    let outcome = mcuboot_image::boot(flash, config, |event| {
        platform.kick_watchdog();
        platform.progress(event)
    });
    platform.decided(&outcome);
    outcome
        .ok()
        .and_then(|bootable| platform.prepare(&config.layout, &bootable))
}

/// One complete boot: [`launch`], then [`Platform::start`] or
/// [`Platform::fail`].
pub fn run<F: NorFlash, P: Platform>(
    flash: &mut F,
    config: &BootConfig<'_>,
    platform: &mut P,
) -> ! {
    match launch(flash, config, platform) {
        // SAFETY: `launch` returns only what `prepare` returned for the
        // image validated in this boot.
        Some(entry) => unsafe { platform.start(entry) },
        None => platform.fail(),
    }
}
