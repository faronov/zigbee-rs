//! Validation of unsigned, plaintext ESP-IDF application images.
//!
//! Format and compatibility rules follow ESP-IDF v5.5.1
//! `components/bootloader_support/src/{esp_image_format,bootloader_common_loader}.c`
//! and the C6/H2 `soc.h` memory ranges. This checks structure, segment bounds,
//! mapping alignment, application descriptor, revisions, XOR checksum and the
//! appended SHA-256. It is NOT signature verification, anti-rollback enforcement,
//! or proof that the program will run correctly. Signed/encrypted OTA payloads
//! and trailing data are not supported.

pub const HEADER_SIZE: usize = 24;
pub const DIGEST_SIZE: usize = 32;
pub const IMAGE_MAGIC: u8 = 0xE9;
pub const CHIP_ID_ESP32C6: u16 = 0x000D;
pub const CHIP_ID_ESP32H2: u16 = 0x0010;
pub const MAX_SEGMENTS: usize = 16;
pub const SEGMENT_HEADER_SIZE: usize = 8;
pub const APP_DESC_SIZE: usize = 256;
pub const APP_DESC_MAGIC: u32 = 0xABCD_5432;

#[cfg(feature = "esp32c6")]
pub const EXPECTED_CHIP_ID: u16 = CHIP_ID_ESP32C6;
#[cfg(feature = "esp32h2")]
pub const EXPECTED_CHIP_ID: u16 = CHIP_ID_ESP32H2;

/// Actual hardware revision information, not the update's claimed revisions.
/// Revision encoding is `major * 100 + minor`, as in ESP-IDF.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ImageCompatibility {
    pub chip_id: u16,
    pub chip_revision: u16,
    pub efuse_block_revision: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageError {
    TooSmall,
    BadMagic,
    ChipMismatch { found: u16 },
    NoAppendedHash,
    NoSegments,
    TooManySegments,
    SegmentBounds,
    SegmentLength,
    LoadAddress,
    OverlappingSegments,
    MappingAlignment,
    EntryAddress,
    AppDescriptor,
    UnsupportedMmuPageSize,
    ChipRevision,
    EfuseBlockRevision,
    UnsupportedSecureVersion,
    ImageLength,
    Checksum,
    Hash,
}

/// Preserve flash I/O errors separately from malformed-image errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImageReadError<E> {
    Read(E),
    Image(ImageError),
}

impl<E> From<ImageError> for ImageReadError<E> {
    fn from(value: ImageError) -> Self {
        Self::Image(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EspImageHeader {
    pub segment_count: u8,
    pub chip_id: u16,
    pub hash_appended: bool,
    pub entry_addr: u32,
    pub min_chip_revision: u16,
    pub max_chip_revision: u16,
}

fn u16_at(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap())
}

fn u32_at(bytes: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
}

impl EspImageHeader {
    /// Parse only the header. A successful parse is NOT image validation.
    pub fn parse(header: &[u8; HEADER_SIZE], expected_chip_id: u16) -> Result<Self, ImageError> {
        if header[0] != IMAGE_MAGIC {
            return Err(ImageError::BadMagic);
        }
        if header[1] == 0 {
            return Err(ImageError::NoSegments);
        }
        if header[1] as usize > MAX_SEGMENTS {
            return Err(ImageError::TooManySegments);
        }
        let chip_id = u16_at(header, 12);
        if chip_id != expected_chip_id {
            return Err(ImageError::ChipMismatch { found: chip_id });
        }
        if header[23] != 1 {
            return Err(ImageError::NoAppendedHash);
        }
        Ok(Self {
            segment_count: header[1],
            chip_id,
            hash_appended: true,
            entry_addr: u32_at(header, 4),
            // IDF 5.5.1 uses the full revision, not deprecated byte 14.
            min_chip_revision: u16_at(header, 15),
            max_chip_revision: u16_at(header, 17),
        })
    }
}

pub const MIN_IMAGE_SIZE: u32 =
    ((HEADER_SIZE + SEGMENT_HEADER_SIZE + APP_DESC_SIZE + 16 + DIGEST_SIZE) & !15) as u32;

/// Candidate hash range only; call [`verify_image`] to establish the actual
/// image end by walking the segments and checksum trailer.
pub fn hashed_range(size: u32) -> Result<(u32, u32), ImageError> {
    if size < MIN_IMAGE_SIZE {
        return Err(ImageError::TooSmall);
    }
    Ok((0, size - DIGEST_SIZE as u32))
}

fn revision_allowed(actual: u16, min: u16, max: u16) -> bool {
    // IDF IS_FIELD_SET excludes both zero and 0xffff for the maximum.
    // Conservatively enforce max even on chips with an eFuse override.
    actual >= min && (max == 0 || max == u16::MAX || actual <= max)
}

#[derive(Clone, Copy, Default)]
struct Segment {
    start: u32,
    end: u32,
    data_offset: u32,
    mapped: bool,
}

/// Validate an entire staged application, reading offsets relative to its
/// beginning. `partition_offset` is its real physical flash offset.
///
/// Bounded memory (no allocation), at most 16 segments. Every read stays within
/// `size`; lengths and the hash position come from parsed structure, never
/// simply "the last 32 bytes". Standard espflash zero-address padding segments
/// and IDF zero-length segments are supported.
pub fn verify_image<E>(
    size: u32,
    partition_offset: u32,
    compatibility: ImageCompatibility,
    mut read: impl FnMut(u32, &mut [u8]) -> Result<(), E>,
) -> Result<[u8; DIGEST_SIZE], ImageReadError<E>> {
    let (_, hash_end) = hashed_range(size)?;
    partition_offset
        .checked_add(size)
        .ok_or(ImageError::ImageLength)?;
    let mut raw_header = [0; HEADER_SIZE];
    read(0, &mut raw_header).map_err(ImageReadError::Read)?;
    let header = EspImageHeader::parse(&raw_header, compatibility.chip_id)?;
    let (ram_end, rtc_end) = match header.chip_id {
        CHIP_ID_ESP32C6 => (0x4088_0000, 0x5000_4000),
        CHIP_ID_ESP32H2 => (0x4085_0000, 0x5000_1000),
        found => return Err(ImageError::ChipMismatch { found }.into()),
    };
    if !revision_allowed(
        compatibility.chip_revision,
        header.min_chip_revision,
        header.max_chip_revision,
    ) {
        return Err(ImageError::ChipRevision.into());
    }

    let mut hasher = crate::sha256::Sha256::new();
    hasher.update(&raw_header);
    let mut checksum = 0xEF;
    let mut cursor = HEADER_SIZE as u32;
    let mut page_size = 0;
    let mut segments = [Segment::default(); MAX_SEGMENTS];
    let mut entry_found = false;
    let mut buffer = [0; 256];

    for index in 0..header.segment_count as usize {
        let data_offset = cursor
            .checked_add(SEGMENT_HEADER_SIZE as u32)
            .filter(|end| *end <= hash_end)
            .ok_or(ImageError::SegmentBounds)?;
        let mut raw = [0; SEGMENT_HEADER_SIZE];
        read(cursor, &mut raw).map_err(ImageReadError::Read)?;
        hasher.update(&raw);
        let load = u32_at(&raw, 0);
        let len = u32_at(&raw, 4);
        if !len.is_multiple_of(4) || len >= 0x100_0000 {
            return Err(ImageError::SegmentLength.into());
        }
        let end = data_offset
            .checked_add(len)
            .filter(|end| *end <= hash_end)
            .ok_or(ImageError::SegmentBounds)?;
        let load_end = load.checked_add(len).ok_or(ImageError::LoadAddress)?;

        if index == 0 {
            if len < APP_DESC_SIZE as u32 {
                return Err(ImageError::AppDescriptor.into());
            }
            read(data_offset, &mut buffer).map_err(ImageReadError::Read)?;
            if u32_at(&buffer, 0) != APP_DESC_MAGIC {
                return Err(ImageError::AppDescriptor.into());
            }
            if u32_at(&buffer, 4) != 0 {
                return Err(ImageError::UnsupportedSecureVersion.into());
            }
            let min_block_revision = u16_at(&buffer, 176);
            // Unlike the chip minimum, IDF treats 0xffff as unset here.
            let min_block_revision = if min_block_revision == u16::MAX {
                0
            } else {
                min_block_revision
            };
            if !revision_allowed(
                compatibility.efuse_block_revision,
                min_block_revision,
                u16_at(&buffer, 178),
            ) {
                return Err(ImageError::EfuseBlockRevision.into());
            }
            page_size = match buffer[180] {
                // Legacy descriptors use the bootloader's default 64 KiB.
                0 | 16 => 65536,
                15 => 32768,
                14 => 16384,
                13 => 8192,
                _ => return Err(ImageError::UnsupportedMmuPageSize.into()),
            };
        }
        let mapped = (0x4200_0000..0x4200_0000 + page_size * 256).contains(&load);
        if index == 0 && !mapped {
            return Err(ImageError::AppDescriptor.into());
        }
        // Padding (address 0) is included in both checksum and hash, not loaded.
        if load != 0 && len != 0 {
            let valid = if mapped {
                load_end <= 0x4200_0000 + page_size * 256
            } else {
                (load >= 0x4080_0000 && load_end <= ram_end)
                    || (load >= 0x5000_0000 && load_end <= rtc_end)
            };
            if !valid || !load.is_multiple_of(4) {
                return Err(ImageError::LoadAddress.into());
            }
            if mapped
                && (load % page_size != (partition_offset + data_offset) % page_size
                    || partition_offset + end > page_size * 256)
            {
                return Err(ImageError::MappingAlignment.into());
            }
            for previous in &segments[..index] {
                if previous.start == 0 || previous.start == previous.end {
                    continue;
                }
                if load < previous.end && previous.start < load_end {
                    return Err(ImageError::OverlappingSegments.into());
                }
                // Disjoint byte ranges sharing an MMU page must agree on its
                // physical mapping, not just on their within-page alignment.
                if mapped
                    && previous.mapped
                    && load / page_size <= (previous.end - 1) / page_size
                    && previous.start / page_size <= (load_end - 1) / page_size
                    && (load as i64 - data_offset as i64)
                        != (previous.start as i64 - previous.data_offset as i64)
                {
                    return Err(ImageError::MappingAlignment.into());
                }
            }
            let executable_start = load + if index == 0 { APP_DESC_SIZE as u32 } else { 0 };
            entry_found |= header.entry_addr >= executable_start
                && header.entry_addr < load_end
                && header.entry_addr % 2 == 0;
        }
        segments[index] = Segment {
            start: load,
            end: load_end,
            data_offset,
            mapped,
        };
        cursor = data_offset;
        while cursor < end {
            let take = (end - cursor).min(buffer.len() as u32) as usize;
            read(cursor, &mut buffer[..take]).map_err(ImageReadError::Read)?;
            for byte in &buffer[..take] {
                checksum ^= byte;
            }
            hasher.update(&buffer[..take]);
            cursor += take as u32;
        }
    }
    if !entry_found {
        return Err(ImageError::EntryAddress.into());
    }
    // Checksum is the last byte of the next 16-byte block, including a whole
    // block when the last segment already ends on a 16-byte boundary.
    let digest_offset = cursor.checked_add(16).ok_or(ImageError::ImageLength)? & !15;
    if digest_offset != hash_end {
        return Err(ImageError::ImageLength.into());
    }
    let trailer_len = (digest_offset - cursor) as usize;
    read(cursor, &mut buffer[..trailer_len]).map_err(ImageReadError::Read)?;
    if buffer[trailer_len - 1] != checksum {
        return Err(ImageError::Checksum.into());
    }
    hasher.update(&buffer[..trailer_len]);
    let digest = hasher.finalize();
    let mut stored = [0; DIGEST_SIZE];
    read(digest_offset, &mut stored).map_err(ImageReadError::Read)?;
    if digest != stored {
        return Err(ImageError::Hash.into());
    }
    Ok(digest)
}

#[cfg(test)]
#[path = "esp_image_tests.rs"]
pub(crate) mod tests;
