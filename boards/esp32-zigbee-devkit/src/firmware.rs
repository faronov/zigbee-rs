//! Firmware version helpers shared by the OTA cluster and the Basic cluster.
//!
//! The examples get their OTA file version from a build script
//! (`ESP32_OTA_VERSION`, default `1`). The same number has to show up in three
//! places or ZHA and the device will disagree about what is installed:
//!
//! * `OtaConfig::current_version` — what `QueryNextImageRequest` advertises,
//! * `Basic::ApplicationVersion` — the u8 ZHA displays,
//! * `Basic::SWBuildID` — the string ZHA displays.
//!
//! These helpers derive the latter two from the former so they cannot drift.

/// Zigbee reserves `0xFFFF_FFFF`: it is the "no image" marker in
/// `QueryNextImageResponse`, so it can never be a real firmware version.
pub const RESERVED_VERSION: u32 = u32::MAX;

/// Whether `version` is usable as an OTA file version.
pub const fn is_valid_version(version: u32) -> bool {
    version != RESERVED_VERSION
}

/// `Basic::ApplicationVersion` for a firmware version.
///
/// The attribute is a u8, so it carries the low byte of the OTA file version.
pub const fn application_version(version: u32) -> u8 {
    (version & 0xFF) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn application_version_is_the_low_byte() {
        assert_eq!(application_version(1), 1);
        assert_eq!(application_version(2), 2);
        assert_eq!(application_version(0x0000_0102), 0x02);
        assert_eq!(application_version(0xDEAD_BE7F), 0x7F);
    }

    #[test]
    fn the_zigbee_no_image_marker_is_not_a_version() {
        assert!(is_valid_version(0));
        assert!(is_valid_version(1));
        assert!(is_valid_version(0xFFFF_FFFE));
        assert!(!is_valid_version(RESERVED_VERSION));
    }
}
