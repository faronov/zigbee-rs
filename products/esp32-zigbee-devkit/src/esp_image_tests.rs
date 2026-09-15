use super::*;
use crate::layout::OTA_1_OFFSET;
use crate::sha256::sha256;

pub(crate) fn compatibility() -> ImageCompatibility {
    ImageCompatibility {
        chip_id: EXPECTED_CHIP_ID,
        chip_revision: 0,
        efuse_block_revision: 0,
    }
}

/// A complete unsigned application-format fixture: mapped 256-byte EspAppDesc,
/// mapped RISC-V code (JAL x0,0 / NOPs), segment headers, checksum padding, hash.
/// Not a header plus arbitrary bytes masquerading as a successful image.
/// Actual espflash-generated firmware is tested separately with `ESP_OTA_TEST_IMAGE`.
pub(crate) fn application(chip_id: u16, code_len: usize) -> Vec<u8> {
    let code_len = (code_len.max(4) + 3) & !3;
    let code_offset = HEADER_SIZE + SEGMENT_HEADER_SIZE + APP_DESC_SIZE + SEGMENT_HEADER_SIZE;
    let code_addr = 0x4200_0000 + code_offset as u32;
    let mut image = vec![0; HEADER_SIZE];
    image[0] = IMAGE_MAGIC;
    image[1] = 2;
    image[4..8].copy_from_slice(&code_addr.to_le_bytes());
    image[12..14].copy_from_slice(&chip_id.to_le_bytes());
    image[17..19].copy_from_slice(&u16::MAX.to_le_bytes());
    image[23] = 1;
    image.extend_from_slice(&0x4200_0020u32.to_le_bytes());
    image.extend_from_slice(&(APP_DESC_SIZE as u32).to_le_bytes());
    let mut descriptor = [0; APP_DESC_SIZE];
    descriptor[..4].copy_from_slice(&APP_DESC_MAGIC.to_le_bytes());
    descriptor[178..180].copy_from_slice(&u16::MAX.to_le_bytes());
    descriptor[180] = 16;
    image.extend_from_slice(&descriptor);
    image.extend_from_slice(&code_addr.to_le_bytes());
    image.extend_from_slice(&(code_len as u32).to_le_bytes());
    image.extend_from_slice(&0x0000_006fu32.to_le_bytes()); // JAL x0,0
    for _ in 1..code_len / 4 {
        image.extend_from_slice(&0x0000_0013u32.to_le_bytes()); // NOP
    }
    finish(image)
}

fn finish(mut image: Vec<u8>) -> Vec<u8> {
    let mut cursor = HEADER_SIZE;
    let mut checksum = 0xEF;
    for _ in 0..image[1] {
        let len = u32_at(&image, cursor + 4) as usize;
        cursor += SEGMENT_HEADER_SIZE;
        for byte in &image[cursor..cursor + len] {
            checksum ^= byte;
        }
        cursor += len;
    }
    image.truncate(cursor);
    let end = (image.len() + 16) & !15;
    image.resize(end, 0);
    image[end - 1] = checksum;
    let digest = sha256(&image);
    image.extend_from_slice(&digest);
    image
}

fn append_segment(mut image: Vec<u8>, address: u32, data: &[u8]) -> Vec<u8> {
    let mut cursor = HEADER_SIZE;
    for _ in 0..image[1] {
        cursor += SEGMENT_HEADER_SIZE + u32_at(&image, cursor + 4) as usize;
    }
    image.truncate(cursor);
    image[1] += 1;
    image.extend_from_slice(&address.to_le_bytes());
    image.extend_from_slice(&(data.len() as u32).to_le_bytes());
    image.extend_from_slice(data);
    finish(image)
}

pub(crate) fn rehash(image: &mut [u8]) {
    let end = image.len() - DIGEST_SIZE;
    let digest = sha256(&image[..end]);
    image[end..].copy_from_slice(&digest);
}

fn validate(image: &[u8]) -> Result<[u8; DIGEST_SIZE], ImageReadError<()>> {
    verify_image(
        image.len() as u32,
        OTA_1_OFFSET,
        compatibility(),
        |offset, out| {
            // Panics if the verifier ever reads beyond the declared image boundary.
            out.copy_from_slice(&image[offset as usize..offset as usize + out.len()]);
            Ok(())
        },
    )
}

fn rejects(mut image: Vec<u8>, expected: ImageError) {
    // Most regressions have a VALID attacker-recomputable hash. Merely checking
    // the digest cannot reject these images.
    rehash(&mut image);
    assert_eq!(validate(&image), Err(ImageReadError::Image(expected)));
}

#[test]
fn complete_application_and_all_checksum_padding_lengths() {
    for len in [4, 8, 12, 16, 512, 4096, 4099] {
        let image = application(EXPECTED_CHIP_ID, len);
        assert_eq!(
            validate(&image).unwrap(),
            image[image.len() - DIGEST_SIZE..]
        );
    }
}

#[test]
fn accepts_idf_padding_zero_length_segments_and_sixteen_segments() {
    let mut image = append_segment(application(EXPECTED_CHIP_ID, 4), 0, &[0xFE, 0, 0, 0]);
    image = append_segment(image, 0x4080_0000, &[0x13, 0, 0, 0]);
    image = append_segment(image, 0x5000_0000, &[0x13, 0, 0, 0]);
    for _ in image[1]..MAX_SEGMENTS as u8 {
        image = append_segment(image, 0, &[]);
    }
    assert_eq!(image[1], 16);
    validate(&image).unwrap();
}

#[test]
fn rejects_overlapping_loaded_segments_and_accepts_adjacent_ones() {
    let image = append_segment(
        application(EXPECTED_CHIP_ID, 4),
        0x4080_0000,
        &[0x13, 0, 0, 0],
    );
    let adjacent = append_segment(image.clone(), 0x4080_0004, &[0x13, 0, 0, 0]);
    validate(&adjacent).unwrap();
    rejects(
        append_segment(image, 0x4080_0000, &[0x13, 0, 0, 0]),
        ImageError::OverlappingSegments,
    );
}

#[test]
fn rejects_original_56_byte_repro_and_impossible_segment_counts() {
    let mut repro = vec![0; HEADER_SIZE + DIGEST_SIZE];
    repro[0] = IMAGE_MAGIC;
    repro[1] = 17;
    repro[12..14].copy_from_slice(&EXPECTED_CHIP_ID.to_le_bytes());
    repro[23] = 1;
    rejects(repro, ImageError::TooSmall);
    for count in [0, 17, 255] {
        let mut image = application(EXPECTED_CHIP_ID, 4);
        image[1] = count;
        rejects(
            image,
            if count == 0 {
                ImageError::NoSegments
            } else {
                ImageError::TooManySegments
            },
        );
    }
}

#[test]
fn rejects_missing_segments_overflow_lengths_and_truncation() {
    let mut missing = application(EXPECTED_CHIP_ID, 4);
    missing[1] = 16;
    assert!(validate(&missing).is_err());
    for len in [u32::MAX, 0xffff_fffc, 0x100_0000, 3] {
        let mut image = application(EXPECTED_CHIP_ID, 4);
        image[28..32].copy_from_slice(&len.to_le_bytes());
        rejects(image, ImageError::SegmentLength);
    }
    let mut overrun = application(EXPECTED_CHIP_ID, 4);
    overrun[28..32].copy_from_slice(&0x10000u32.to_le_bytes());
    rejects(overrun, ImageError::SegmentBounds);
    let image = application(EXPECTED_CHIP_ID, 4);
    for end in 0..image.len() {
        assert!(
            validate(&image[..end]).is_err(),
            "accepted truncation at {end}"
        );
    }
}

#[test]
fn rejects_bad_checksum_even_with_a_valid_hash() {
    let mut image = application(EXPECTED_CHIP_ID, 4);
    let pos = image.len() - DIGEST_SIZE - 1;
    image[pos] ^= 1;
    rejects(image, ImageError::Checksum);
}

#[test]
fn rejects_bad_digest_missing_hash_and_trailing_data() {
    let mut image = application(EXPECTED_CHIP_ID, 4);
    *image.last_mut().unwrap() ^= 1;
    assert_eq!(
        validate(&image),
        Err(ImageReadError::Image(ImageError::Hash))
    );
    for flag in [0, 2, 255] {
        let mut image = application(EXPECTED_CHIP_ID, 4);
        image[23] = flag;
        rejects(image, ImageError::NoAppendedHash);
    }
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image.extend_from_slice(&[0; 32]);
    rejects(image, ImageError::ImageLength);
}

#[test]
fn rejects_incompatible_chip_and_revision_requirements() {
    let foreign = if EXPECTED_CHIP_ID == CHIP_ID_ESP32C6 {
        CHIP_ID_ESP32H2
    } else {
        CHIP_ID_ESP32C6
    };
    rejects(
        application(foreign, 4),
        ImageError::ChipMismatch { found: foreign },
    );
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[15] = 1;
    rejects(image, ImageError::ChipRevision);
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[15..17].copy_from_slice(&u16::MAX.to_le_bytes());
    rejects(image, ImageError::ChipRevision);
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[32 + 176] = 1;
    rejects(image, ImageError::EfuseBlockRevision);
    assert!(revision_allowed(99, 0, 99));
    assert!(!revision_allowed(100, 0, 99));
    assert!(revision_allowed(200, 0, 0));
    assert!(revision_allowed(200, 0, u16::MAX));
}

#[test]
fn rejects_missing_descriptor_bad_page_size_and_secure_version() {
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[32] ^= 1;
    rejects(image, ImageError::AppDescriptor);
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[28..32].copy_from_slice(&4u32.to_le_bytes());
    rejects(image, ImageError::AppDescriptor);
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[32 + 180] = 32;
    rejects(image, ImageError::UnsupportedMmuPageSize);
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[36] = 1;
    rejects(image, ImageError::UnsupportedSecureVersion);
}

#[test]
fn rejects_bad_load_entry_mapping_and_overlapping_segments() {
    for address in [0x6000_0000u32, 0xffff_fffc, 0x4087_fffc] {
        let mut image = application(EXPECTED_CHIP_ID, 16);
        image[288..292].copy_from_slice(&address.to_le_bytes());
        rejects(image, ImageError::LoadAddress);
    }
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[4..8].copy_from_slice(&0x4000_0000u32.to_le_bytes());
    rejects(image, ImageError::EntryAddress);
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[24..28].copy_from_slice(&0x4200_0024u32.to_le_bytes());
    rejects(image, ImageError::MappingAlignment);
    let mut image = application(EXPECTED_CHIP_ID, 4);
    image[288..292].copy_from_slice(&0x4200_0020u32.to_le_bytes());
    // Alignment is checked before overlap for mapped segments.
    rejects(image, ImageError::MappingAlignment);
}

#[test]
fn flash_read_errors_are_not_hidden() {
    assert_eq!(
        verify_image(512, OTA_1_OFFSET, compatibility(), |_, _| Err(
            "flash failed"
        )),
        Err(ImageReadError::Read("flash failed"))
    );
}

/// Generate with partitions/test_esp_app_image.py --write-corpus DIR, then set
/// ESP_OTA_PREFLIGHT_CORPUS=DIR and run this test explicitly with --ignored.
#[test]
#[ignore = "requires the shared Python preflight corpus, including real generated images"]
fn python_preflight_corpus() {
    let root = std::env::var("ESP_OTA_PREFLIGHT_CORPUS").expect("set ESP_OTA_PREFLIGHT_CORPUS");
    let chip = if EXPECTED_CHIP_ID == CHIP_ID_ESP32C6 {
        "esp32c6"
    } else {
        "esp32h2"
    };
    let directory = std::path::Path::new(&root).join(chip);
    let manifest =
        std::fs::read_to_string(directory.join("manifest.tsv")).expect("read corpus manifest");
    let mut count = 0;
    for line in manifest.lines() {
        let columns: Vec<_> = line.split('\t').collect();
        assert_eq!(columns.len(), 3);
        let image = std::fs::read(directory.join(columns[0])).expect("read corpus image");
        let accepted = match columns[1] {
            "1" => true,
            "0" => false,
            _ => panic!("invalid expected result"),
        };
        let offset: u32 = columns[2].parse().expect("partition offset");
        let result = verify_image(
            image.len() as u32,
            offset,
            compatibility(),
            |offset, out| {
                out.copy_from_slice(&image[offset as usize..offset as usize + out.len()]);
                Ok::<_, ()>(())
            },
        );
        assert_eq!(result.is_ok(), accepted, "{}: {result:?}", columns[0]);
        count += 1;
    }
    assert!(count > 0, "empty corpus");
    println!("{chip}: {count} Python/Rust preflight results agree");
}
