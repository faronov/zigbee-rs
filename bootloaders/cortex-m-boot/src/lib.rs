//! Cortex-M shell of the zigbee-rs bootloader core ([`zrs_boot_core`]),
//! shared by the nRF52840, EFR32 Series 2 and PHY62x2 loaders.
//!
//! A chip loader supplies its flash driver (any [`NorFlash`]), the
//! [`Config`] generated from the product's layout file
//! (`zrs-boot-layout`), and [`Hooks`] for status, watchdog and the failure
//! policy, then calls `run`. One boot:
//!
//! 1. installs a pending, valid, non-downgrade image from the secondary slot
//!    and validates the primary image ([`zrs_boot_core::launch`]), with
//!    every erase and program confined to the slots by [`BootFlash`];
//! 2. checks the entry ([`entry`]): the image header must have exactly the
//!    configured size (so the vector table sits where the application was
//!    linked), and the initial stack pointer and reset handler read
//!    *through the flash driver* must point into SRAM and into the image as
//!    mapped at [`Config::xip_base`];
//! 3. hands over: interrupts masked, `VTOR` set when the core has one
//!    ([`Config::vtor`]; a Cortex-M0 dispatches through ROM instead),
//!    `MSP` loaded, branch to the reset handler with `PRIMASK` clear (the
//!    reset state `cortex-m-rt` expects). Clocks and peripherals are left
//!    as the loader found them, apart from what the chip shell documents.
//!
//! With nothing bootable, [`Hooks::fail`] decides: halt, sleep, or re-enter
//! a resident ROM/vendor bootloader.
//!
//! **Signatures.** ECDSA P-256 only: feature `ed25519` fails the build (the
//! verifier does not fit the 16 KiB loader regions).

#![cfg_attr(not(test), no_std)]

#[cfg(feature = "ed25519")]
compile_error!(
    "zrs-boot-cortex-m: Ed25519 is not supported by the Cortex-M loaders. The Ed25519 verifier \
     (~19 KB on Cortex-M0+/M4) does not fit their 16 KiB loader regions; they verify ECDSA \
     P-256 signatures only. Build without feature `ed25519`."
);

use zrs_boot_core::Platform;
use zrs_boot_core::embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
pub use zrs_boot_core::entry::{VectorError, check_vector_table};
pub use zrs_boot_core::flash::{BootFlash, BootFlashError};
pub use zrs_boot_core::{
    BootConfig, BootError, BootEvent, Bootable, Layout, LoaderInfo, Mode, Trial, TrustedKey,
    Update, embedded_storage, loader_block, loader_info, mcuboot_image,
};

/// Loader configuration, normally the constants the product's layout file
/// generates.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// Slots, trusted keys, trial boots and install mode.
    pub boot: BootConfig<'static>,
    /// `imgtool --header-size` the application is linked for.
    pub header_size: u32,
    /// SRAM `[start, end)` for the initial stack pointer check.
    pub ram: (u32, u32),
    /// Memory address of physical flash offset 0 (`0` when flash is mapped
    /// at its offsets).
    pub xip_base: u32,
    /// The core has `VTOR` (Cortex-M0+/M3/M4/M33); `false` on a Cortex-M0
    /// that dispatches interrupts through ROM.
    pub vtor: bool,
}

/// Chip and product policy around the core sequence. Every method except
/// [`Hooks::fail`] defaults to nothing.
pub trait Hooks {
    /// Engine progress (status LEDs or console).
    fn progress(&mut self, event: BootEvent) {
        let _ = event;
    }
    /// Service an armed loader watchdog.
    fn kick_watchdog(&mut self) {}
    /// The install/validation decision, once per boot.
    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {
        let _ = outcome;
    }
    /// The entry check refused a validated image.
    fn refused(&mut self, error: EntryError) {
        let _ = error;
    }
    /// Last step before the hand-over to the checked `entry` (indicators
    /// off, watchdog kicked, platform start-up data copied out of the
    /// validated image).
    fn before_start(&mut self, entry: &Entry) {
        let _ = entry;
    }
    /// Nothing can be started.
    fn fail(&mut self) -> !;
}

/// Why a validated image was not started.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntryError {
    /// The image header size differs from the one the application layout
    /// is linked for.
    HeaderSize,
    /// The vector table could not be read.
    Flash,
    /// Stack pointer or reset handler out of range.
    Vector(VectorError),
}

/// A checked application entry, in memory addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    pub vector_table: u32,
    pub initial_sp: u32,
    pub reset: u32,
    /// End of the validated payload as mapped (`vector_table + img_size`):
    /// anything a hook reads from the image must lie below it.
    pub image_end: u32,
}

/// Check the entry of the validated primary image whose header is
/// `hdr_size` bytes and payload `img_size` bytes: the vector table is read
/// through `flash` at physical `primary + hdr_size`, its addresses are
/// compared against the image as mapped at `config.xip_base`.
pub fn entry<F: ReadNorFlash>(
    flash: &mut F,
    config: &Config,
    hdr_size: u32,
    img_size: u32,
) -> Result<Entry, EntryError> {
    if hdr_size != config.header_size {
        return Err(EntryError::HeaderSize);
    }
    let physical = config.boot.layout.primary + hdr_size;
    let mut words = [0u8; 8];
    flash
        .read(physical, &mut words)
        .map_err(|_| EntryError::Flash)?;
    let initial_sp = u32::from_le_bytes([words[0], words[1], words[2], words[3]]);
    let reset = u32::from_le_bytes([words[4], words[5], words[6], words[7]]);
    let vector_table = config.xip_base + physical;
    let image_end = vector_table + img_size;
    check_vector_table(initial_sp, reset, vector_table, image_end, config.ram)
        .map_err(EntryError::Vector)?;
    Ok(Entry {
        vector_table,
        initial_sp,
        reset,
        image_end,
    })
}

/// Records the decision for [`decide`]; the vector table is read after the
/// engine has released the flash.
struct Decision<'a, H> {
    hooks: &'a mut H,
    image: Option<(u32, u32)>,
}

impl<H: Hooks> Platform for Decision<'_, H> {
    fn progress(&mut self, event: BootEvent) {
        self.hooks.progress(event);
    }

    fn kick_watchdog(&mut self) {
        self.hooks.kick_watchdog();
    }

    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {
        self.hooks.decided(outcome);
    }

    fn prepare(&mut self, layout: &Layout, bootable: &Bootable) -> Option<u32> {
        let header = &bootable.image.header;
        let _ = layout;
        self.image = Some((u32::from(header.hdr_size), header.img_size));
        Some(bootable.vector_table)
    }

    unsafe fn start(&mut self, _entry: u32) -> ! {
        unreachable!("decide never starts")
    }

    fn fail(&mut self) -> ! {
        self.hooks.fail()
    }
}

/// The boot decision and entry check, without starting anything (host tests
/// drive this). `flash` is normally a [`BootFlash`].
pub fn decide<F: NorFlash, H: Hooks>(
    flash: &mut F,
    config: &Config,
    hooks: &mut H,
) -> Option<Entry> {
    let mut decision = Decision { hooks, image: None };
    zrs_boot_core::launch(flash, &config.boot, &mut decision)?;
    let (hdr_size, img_size) = decision.image?;
    match entry(flash, config, hdr_size, img_size) {
        Ok(entry) => Some(entry),
        Err(error) => {
            decision.hooks.refused(error);
            None
        }
    }
}

#[cfg(target_os = "none")]
mod target;
#[cfg(target_os = "none")]
pub use target::{jump, run};

#[cfg(test)]
mod tests;
