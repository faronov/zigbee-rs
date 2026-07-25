//! ESP application image validation.
//!
//! An image produced by `espflash save-image` starts with a 24-byte
//! `esp_image_header_t`:
//!
//! ```text
//! offset  size  field
//!      0     1  magic (0xE9)
//!      1     1  segment_count
//!      2     1  spi_mode
//!      3     1  spi_speed / spi_size
//!      4     4  entry_addr
//!      8     1  wp_pin
//!      9     3  spi_pin_drv
//!     12     2  chip_id
//!     14     1  min_chip_rev (deprecated)
//!     15     2  min_chip_rev_full
//!     17     2  max_chip_rev_full
//!     19     4  reserved
//!     23     1  hash_appended
//! ```
//!
//! and ends with a SHA-256 of every preceding byte when `hash_appended` is set,
//! which `espflash` always does. Checking the magic, the chip ID and that
//! digest against the bytes actually staged in flash is what turns "the OTA
//! transfer finished" into "this slot contains a bootable image for *this*
//! chip".

/// Size of `esp_image_header_t`.
pub const HEADER_SIZE: usize = 24;

/// Size of the appended SHA-256.
pub const DIGEST_SIZE: usize = 32;

/// First byte of every ESP application image.
pub const IMAGE_MAGIC: u8 = 0xE9;

/// `ESP_CHIP_ID_ESP32C6`.
pub const CHIP_ID_ESP32C6: u16 = 0x000D;

/// `ESP_CHIP_ID_ESP32H2`.
pub const CHIP_ID_ESP32H2: u16 = 0x0010;

/// Chip ID this board build expects staged images to carry.
#[cfg(feature = "esp32c6")]
pub const EXPECTED_CHIP_ID: u16 = CHIP_ID_ESP32C6;
/// Chip ID this board build expects staged images to carry.
#[cfg(feature = "esp32h2")]
pub const EXPECTED_CHIP_ID: u16 = CHIP_ID_ESP32H2;

/// Why a staged image was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageError {
    /// Fewer bytes than a header plus digest.
    TooSmall,
    /// First byte is not [`IMAGE_MAGIC`].
    BadMagic,
    /// Image was built for a different chip.
    ChipMismatch {
        /// Chip ID found in the header.
        found: u16,
    },
    /// Image was built without an appended SHA-256.
    NoAppendedHash,
    /// Header declares no segments.
    NoSegments,
}

/// Fields of `esp_image_header_t` that matter for OTA validation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EspImageHeader {
    /// Number of segments in the image.
    pub segment_count: u8,
    /// Target chip.
    pub chip_id: u16,
    /// Whether a SHA-256 follows the image data.
    pub hash_appended: bool,
}

impl EspImageHeader {
    /// Parse the header, rejecting anything this board cannot boot.
    pub fn parse(header: &[u8; HEADER_SIZE], expected_chip_id: u16) -> Result<Self, ImageError> {
        if header[0] != IMAGE_MAGIC {
            return Err(ImageError::BadMagic);
        }
        let segment_count = header[1];
        if segment_count == 0 {
            return Err(ImageError::NoSegments);
        }
        let chip_id = u16::from_le_bytes([header[12], header[13]]);
        if chip_id != expected_chip_id {
            return Err(ImageError::ChipMismatch { found: chip_id });
        }
        let hash_appended = header[23] == 1;
        if !hash_appended {
            return Err(ImageError::NoAppendedHash);
        }
        Ok(Self {
            segment_count,
            chip_id,
            hash_appended,
        })
    }
}

/// Smallest image that can possibly be valid.
pub const MIN_IMAGE_SIZE: u32 = (HEADER_SIZE + DIGEST_SIZE) as u32;

/// Byte range covered by the appended SHA-256 for an image of `size` bytes.
pub fn hashed_range(size: u32) -> Result<(u32, u32), ImageError> {
    if size < MIN_IMAGE_SIZE {
        return Err(ImageError::TooSmall);
    }
    Ok((0, size - DIGEST_SIZE as u32))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(chip_id: u16, hash_appended: u8, segments: u8) -> [u8; HEADER_SIZE] {
        let mut header = [0u8; HEADER_SIZE];
        header[0] = IMAGE_MAGIC;
        header[1] = segments;
        header[12..14].copy_from_slice(&chip_id.to_le_bytes());
        header[23] = hash_appended;
        header
    }

    #[test]
    fn accepts_a_well_formed_header() {
        let parsed =
            EspImageHeader::parse(&header(CHIP_ID_ESP32C6, 1, 4), CHIP_ID_ESP32C6).unwrap();
        assert_eq!(parsed.segment_count, 4);
        assert_eq!(parsed.chip_id, CHIP_ID_ESP32C6);
        assert!(parsed.hash_appended);
    }

    #[test]
    fn rejects_foreign_and_malformed_images() {
        let mut bad_magic = header(CHIP_ID_ESP32C6, 1, 4);
        bad_magic[0] = 0x7F; // ELF
        assert_eq!(
            EspImageHeader::parse(&bad_magic, CHIP_ID_ESP32C6),
            Err(ImageError::BadMagic)
        );

        assert_eq!(
            EspImageHeader::parse(&header(CHIP_ID_ESP32H2, 1, 4), CHIP_ID_ESP32C6),
            Err(ImageError::ChipMismatch {
                found: CHIP_ID_ESP32H2
            })
        );

        assert_eq!(
            EspImageHeader::parse(&header(CHIP_ID_ESP32C6, 0, 4), CHIP_ID_ESP32C6),
            Err(ImageError::NoAppendedHash)
        );

        assert_eq!(
            EspImageHeader::parse(&header(CHIP_ID_ESP32C6, 1, 0), CHIP_ID_ESP32C6),
            Err(ImageError::NoSegments)
        );
    }

    #[test]
    fn hashed_range_excludes_the_trailing_digest() {
        assert_eq!(hashed_range(1024), Ok((0, 1024 - 32)));
        assert_eq!(hashed_range(MIN_IMAGE_SIZE), Ok((0, HEADER_SIZE as u32)));
        assert_eq!(hashed_range(MIN_IMAGE_SIZE - 1), Err(ImageError::TooSmall));
    }
}
