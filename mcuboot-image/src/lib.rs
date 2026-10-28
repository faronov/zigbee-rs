//! MCUboot-compatible image handling for zigbee-rs devices.
//!
//! * [`image`] — header/TLV parsing and validation (SHA-256 plus ECDSA
//!   P-256 signature, the format `imgtool sign` emits);
//! * [`p256`] — a compact verify-only P-256 implementation for bootloaders;
//! * [`trailer`] — the upgrade-request magic;
//! * [`boot`] — the overwrite-only install decision and copy, power-fail
//!   safe and compatible with MCUboot's `MCUBOOT_OVERWRITE_ONLY` mode;
//! * [`loader_info`] — the identity record a zigbee-rs bootloader exposes
//!   to applications;
//! * `writer` (feature `firmware-writer`) — the OTA
//!   `FirmwareWriter` staging images in the secondary slot.
//!
//! Everything works on `embedded_storage` flash with absolute addresses and
//! uses no heap.

#![cfg_attr(not(test), no_std)]

pub mod boot;
pub mod image;
pub mod loader_info;
pub mod p256;
pub mod trailer;
#[cfg(feature = "firmware-writer")]
pub mod writer;

#[cfg(test)]
mod tests;

pub use boot::{BootError, BootEvent, Bootable, Layout, Update, boot};
pub use image::{ImageError, ImageHeader, ImageVersion, TrustedKey, ValidImage, validate};
pub use loader_info::LoaderInfo;
pub use trailer::{BOOT_MAGIC, MagicState};
#[cfg(feature = "firmware-writer")]
pub use writer::{SlotPolicy, SlotWriter};
