//! Signed OTA images with the ESP-IDF bootloader kept (feature `signed-ota`).
//!
//! An update is an `imgtool sign --header-size 0x20 --pad-header` envelope
//! around the ESP application image. [`SignedEspWriter`] stages only the ESP
//! image into the inactive slot (where the second-stage bootloader expects
//! it), keeps the 32-byte MCUboot header and the TLVs in RAM, and refuses to
//! select the slot unless:
//!
//! 1. the ESP image passes [`EspFirmwareWriter`]'s own checks (structure,
//!    chip, checksum, appended SHA-256);
//! 2. the envelope's ECDSA P-256 signature verifies with one of
//!    [`TRUSTED_KEYS`] (from `boot/app-verify.conf`);
//! 3. its MCUboot version (major.minor.revision) is not older than the
//!    running image's.
//!
//! Versions: the Zigbee OTA file version `0xMMmmrrrr` of this build is
//! MCUboot version `MM.mm.rrrr` ([`image_version`]); sign with
//! `imgtool sign --version MM.mm.rrrr`.
//!
//! This authenticates what the application accepts over Zigbee. The ESP-IDF
//! bootloader still boots whatever is in the slot (no secure boot): flashing
//! over UART/JTAG bypasses it.

use mcuboot_image::{EnvelopePolicy, EnvelopeWriter, ImageVersion, KeySet};

use crate::ota::{EspFirmwareWriter, OtaFlash};

mod keys {
    include!(concat!(env!("OUT_DIR"), "/app_verify.rs"));
}
pub use keys::TRUSTED_KEYS;

/// RAM kept for the envelope's TLVs (an ECDSA P-256 envelope uses ~150
/// bytes; protected TLVs such as a security counter add a few more).
pub const TRAILER_CAPACITY: usize = 256;

pub type SignedEspWriter<F> = EnvelopeWriter<EspFirmwareWriter<F>, TRAILER_CAPACITY>;

/// The signed-envelope writer trusting `keys`, refusing envelopes older than
/// `running`.
pub fn signed_writer_with_keys<F: OtaFlash>(
    inner: EspFirmwareWriter<F>,
    keys: KeySet,
    running: ImageVersion,
) -> SignedEspWriter<F> {
    EnvelopeWriter::new(inner, EnvelopePolicy { keys, running })
}

/// [`signed_writer_with_keys`] with this build's [`TRUSTED_KEYS`].
pub fn signed_writer<F: OtaFlash>(
    inner: EspFirmwareWriter<F>,
    running: ImageVersion,
) -> SignedEspWriter<F> {
    signed_writer_with_keys(inner, KeySet::new(&TRUSTED_KEYS), running)
}

/// The MCUboot version matching Zigbee OTA file version `0xMMmmrrrr`.
pub const fn image_version(file_version: u32) -> ImageVersion {
    ImageVersion::new(
        (file_version >> 24) as u8,
        (file_version >> 16) as u8,
        file_version as u16,
        0,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn file_versions_map_to_release_versions() {
        assert_eq!(image_version(1), ImageVersion::new(0, 0, 1, 0));
        assert_eq!(
            image_version(0x0102_0304),
            ImageVersion::new(1, 2, 0x0304, 0)
        );
        // Newer file versions are newer releases.
        assert!(
            image_version(0x0100_0000)
                .cmp_release(&image_version(0x00FF_FFFF))
                .is_gt()
        );
    }
}
