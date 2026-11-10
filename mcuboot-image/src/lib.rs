//! MCUboot-compatible image handling for zigbee-rs devices.
//!
//! * [`image`] — header/TLV parsing and validation (SHA-256 plus an ECDSA
//!   P-256 signature, or Ed25519 with feature `ed25519`; the format
//!   `imgtool sign` emits) against up to three trusted keys;
//! * [`p256`] — a compact verify-only P-256 implementation for bootloaders;
//! * [`trailer`] — the upgrade-request magic;
//! * [`boot`] — the install decision and copy, power-fail safe and
//!   compatible with MCUboot's `MCUBOOT_OVERWRITE_ONLY` mode, plus trial
//!   boots and [`confirm`];
//! * `swap` (feature `swap-move`) — opt-in exchange install with revert;
//! * [`loader_info`] — the identity record a zigbee-rs bootloader exposes
//!   to applications;
//! * `writer` (feature `firmware-writer`) — the OTA
//!   `FirmwareWriter` staging images in the secondary slot;
//! * [`envelope`] — app-side verification of signed images for platforms
//!   whose ROM/SDK boot chain stays (the native image staged without the
//!   MCUboot header; `EnvelopeWriter` with feature `firmware-writer`).
//!
//! Everything works on `embedded_storage` flash with absolute addresses and
//! uses no heap.

#![cfg_attr(not(test), no_std)]

pub mod boot;
pub mod envelope;
pub mod image;
pub mod loader_info;
pub mod p256;
pub mod staging;
mod status;
#[cfg(feature = "swap-move")]
mod swap;
pub mod trailer;
#[cfg(feature = "firmware-writer")]
pub mod writer;

#[cfg(test)]
mod envelope_tests;
#[cfg(test)]
mod tests;

pub use boot::{
    BootConfig, BootError, BootEvent, Bootable, COPY_CHUNK, ERASE_STEP, Layout, Mode, Trial,
    TrialState, Update, boot, confirm, trial_log_area, trial_state,
};
pub use envelope::{ENVELOPE_HEADER_LEN, parse_envelope_header, validate_envelope};
#[cfg(feature = "firmware-writer")]
pub use envelope::{EnvelopePolicy, EnvelopeWriter, StagedImage};
pub use image::{
    ImageError, ImageHeader, ImageVersion, KeySet, MAX_TRUSTED_KEYS, TrustedKey, ValidImage,
    validate, validate_with_set,
};
pub use loader_info::{LoaderInfo, LoaderKeys, TrustedIds, loader_block};
pub use staging::{LoaderChannel, StagingError, StagingFlash, loader_ids, shared_keys};
pub use trailer::{BOOT_MAGIC, MagicState};
#[cfg(feature = "firmware-writer")]
pub use writer::{SlotPolicy, SlotWriter};
