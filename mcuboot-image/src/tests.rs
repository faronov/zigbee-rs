//! Host tests against imgtool-produced vectors on a NOR flash model with
//! power-loss injection. Regenerate the vectors with
//! `tests/vectors/generate.sh`.

use embedded_storage::nor_flash::{
    ErrorType, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};

use crate::boot::{BootError, BootEvent, Layout, Update, boot};
use crate::image::{ImageError, ImageHeader, ImageVersion, TrustedKey, validate};
use crate::trailer::{BOOT_MAGIC, MagicState, magic_address, read_magic};

const TEST_KEY: [u8; 64] = *include_bytes!("../tests/vectors/test-key.xy");
const TI_KEY: [u8; 64] = *include_bytes!("../tests/vectors/ti-key.xy");
const V1: &[u8] = include_bytes!("../tests/vectors/test-v1.bin");
const V2: &[u8] = include_bytes!("../tests/vectors/test-v2.bin");
const V2_PAD: &[u8] = include_bytes!("../tests/vectors/test-v2-pad.bin");
const V1_PROTECTED: &[u8] = include_bytes!("../tests/vectors/test-v1-protected.bin");
const TI_V2: &[u8] = include_bytes!("../tests/vectors/ti-v2.bin");

const SECTOR: u32 = 0x800;
const SLOT: u32 = 0x4000;
const LAYOUT: Layout = Layout {
    primary: 0x0800,
    secondary: 0x0800 + SLOT,
    slot_size: SLOT,
    sector_size: SECTOR,
};
const FLASH_LEN: usize = 0x0800 + 2 * SLOT as usize;

fn key() -> TrustedKey {
    TrustedKey::new(TEST_KEY)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MockError {
    PowerLoss,
    Range,
}

impl NorFlashError for MockError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::PowerLoss => NorFlashErrorKind::Other,
            Self::Range => NorFlashErrorKind::OutOfBounds,
        }
    }
}

/// NOR flash model: erase sets 0xFF, programming can only clear bits.
/// `budget` counts erase/write operations; the operation that exhausts it
/// is applied halfway and then fails, as a power loss would.
pub(crate) struct MockFlash {
    pub data: Vec<u8>,
    pub budget: Option<usize>,
    pub operations: usize,
}

impl MockFlash {
    pub fn new() -> Self {
        Self {
            data: vec![0xFF; FLASH_LEN],
            budget: None,
            operations: 0,
        }
    }

    pub fn load(&mut self, address: u32, bytes: &[u8]) {
        self.data[address as usize..address as usize + bytes.len()].copy_from_slice(bytes);
    }

    fn spend(&mut self) -> bool {
        self.operations += 1;
        match &mut self.budget {
            Some(0) => true,
            Some(left) => {
                *left -= 1;
                *left == 0
            }
            None => false,
        }
    }
}

impl ErrorType for MockFlash {
    type Error = MockError;
}

impl ReadNorFlash for MockFlash {
    const READ_SIZE: usize = 1;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), MockError> {
        let start = offset as usize;
        let source = self
            .data
            .get(start..start + bytes.len())
            .ok_or(MockError::Range)?;
        bytes.copy_from_slice(source);
        Ok(())
    }

    fn capacity(&self) -> usize {
        self.data.len()
    }
}

impl NorFlash for MockFlash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = SECTOR as usize;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), MockError> {
        assert!(
            from.is_multiple_of(SECTOR) && to.is_multiple_of(SECTOR) && from < to,
            "erase {from:#x}..{to:#x}"
        );
        assert!(to as usize <= self.data.len());
        for sector in (from..to).step_by(SECTOR as usize) {
            let fail = self.spend();
            let end = if fail {
                sector + SECTOR / 2
            } else {
                sector + SECTOR
            };
            self.data[sector as usize..end as usize].fill(0xFF);
            if fail {
                return Err(MockError::PowerLoss);
            }
        }
        Ok(())
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), MockError> {
        assert!(
            offset.is_multiple_of(4) && bytes.len().is_multiple_of(4),
            "unaligned write {offset:#x}"
        );
        let fail = self.spend();
        let len = if fail { bytes.len() / 2 } else { bytes.len() };
        for (index, byte) in bytes[..len].iter().enumerate() {
            self.data[offset as usize + index] &= *byte;
        }
        if fail {
            Err(MockError::PowerLoss)
        } else {
            Ok(())
        }
    }
}

fn request(flash: &mut MockFlash) {
    flash.load(magic_address(LAYOUT.secondary, SLOT), &BOOT_MAGIC);
}

fn slot_bytes(flash: &MockFlash, slot: u32, len: usize) -> &[u8] {
    &flash.data[slot as usize..slot as usize + len]
}

fn version_of(image: &[u8]) -> ImageVersion {
    let mut header = [0u8; 32];
    header.copy_from_slice(&image[..32]);
    ImageHeader::parse(&header).unwrap().version
}

#[test]
fn imgtool_vectors_validate_with_their_signing_key() {
    for (image, key_xy, version) in [
        (V1, TEST_KEY, ImageVersion::new(1, 0, 0, 0)),
        (V2, TEST_KEY, ImageVersion::new(2, 0, 0, 0)),
        (V2_PAD, TEST_KEY, ImageVersion::new(2, 0, 0, 7)),
        (V1_PROTECTED, TEST_KEY, ImageVersion::new(1, 2, 3, 4)),
        (TI_V2, TI_KEY, ImageVersion::new(2, 0, 0, 0)),
    ] {
        let mut flash = MockFlash::new();
        flash.load(LAYOUT.primary, image);
        let valid = validate(
            &mut flash,
            LAYOUT.primary,
            LAYOUT.max_image_len(),
            &TrustedKey::new(key_xy),
        )
        .unwrap();
        assert_eq!(valid.header.version, version);
        assert_eq!(valid.header.hdr_size, 0x100);
        assert_eq!(valid.header.img_size, 3000);
        assert!(valid.end as usize <= image.len());
    }
}

#[test]
fn ti_sample_key_hash_matches_the_koptserg_bootloader() {
    // KEYHASH TLV observed in koptserg's released flower OTA images.
    let hash = TrustedKey::new(TI_KEY).key_hash();
    assert_eq!(&hash[..2], &[0xE3, 0x04]);
    assert_eq!(&hash[30..], &[0x38, 0xB6]);
}

#[test]
fn wrong_key_and_tampering_are_rejected() {
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V2);
    assert_eq!(
        validate(
            &mut flash,
            LAYOUT.primary,
            LAYOUT.max_image_len(),
            &TrustedKey::new(TI_KEY)
        ),
        Err(ImageError::BadSignature)
    );

    // Any flipped payload or header byte breaks the hash.
    for offset in [0x14usize, 0x100, 0x500, 0x100 + 2999] {
        let mut flash = MockFlash::new();
        flash.load(LAYOUT.primary, V2);
        flash.data[LAYOUT.primary as usize + offset] ^= 0x01;
        let result = validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &key());
        assert!(
            matches!(
                result,
                Err(ImageError::HashMismatch | ImageError::Malformed)
            ),
            "offset {offset:#x}: {result:?}"
        );
    }

    // A flipped signature byte keeps the hash but fails the signature.
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V2);
    let last = LAYOUT.primary as usize + V2.len() - 1;
    flash.data[last] ^= 0x01;
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &key()),
        Err(ImageError::BadSignature)
    );

    // Truncation and an oversized declared length.
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, &V2[..V2.len() - 40]);
    assert!(validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &key()).is_err());
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V2);
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, 0x400, &key()),
        Err(ImageError::Malformed)
    );
    let mut empty = MockFlash::new();
    assert_eq!(
        validate(&mut empty, LAYOUT.primary, LAYOUT.max_image_len(), &key()),
        Err(ImageError::NoImage)
    );
}

#[test]
fn image_flags_are_not_supported() {
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V2);
    flash.data[LAYOUT.primary as usize + 16] = 0x04; // IMAGE_F_ENCRYPTED_AES128
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &key()),
        Err(ImageError::Unsupported)
    );
}

#[test]
fn no_request_boots_the_primary_untouched() {
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V1);
    flash.load(LAYOUT.secondary, V2); // staged but not requested
    let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
    assert_eq!(bootable.update, Update::None);
    assert_eq!(bootable.vector_table, LAYOUT.primary + 0x100);
    assert_eq!(bootable.image.header.version, version_of(V1));
    assert_eq!(slot_bytes(&flash, LAYOUT.secondary, V2.len()), V2);
}

#[test]
fn upgrade_installs_and_drops_the_request() {
    for staged in [V2, V2_PAD] {
        let mut flash = MockFlash::new();
        flash.load(LAYOUT.primary, V1);
        flash.load(LAYOUT.secondary, staged);
        request(&mut flash);
        let mut copying = false;
        let bootable = boot(&mut flash, &LAYOUT, &key(), |event| {
            copying |= matches!(event, BootEvent::Copying { .. });
        })
        .unwrap();
        assert!(copying);
        assert_eq!(bootable.update, Update::Installed);
        assert_eq!(bootable.image.header.version, version_of(staged));
        assert_eq!(
            slot_bytes(&flash, LAYOUT.primary, V2.len()),
            &staged[..V2.len()]
        );
        assert_eq!(
            read_magic(&mut flash, LAYOUT.secondary, SLOT).unwrap(),
            MagicState::Erased
        );
        assert!(
            ImageHeader::read(&mut flash, LAYOUT.secondary)
                .unwrap()
                .is_none()
        );

        // The next boot has nothing to do.
        let operations = flash.operations;
        let again = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
        assert_eq!(again.update, Update::None);
        assert_eq!(flash.operations, operations);
    }
}

#[test]
fn upgrade_into_an_empty_primary_skips_the_downgrade_check() {
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.secondary, V1);
    request(&mut flash);
    let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
    assert_eq!(bootable.update, Update::Installed);
}

#[test]
fn downgrade_is_refused_and_equal_versions_are_reinstalled() {
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V2);
    flash.load(LAYOUT.secondary, V1);
    request(&mut flash);
    let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
    assert_eq!(bootable.update, Update::Downgrade);
    assert_eq!(slot_bytes(&flash, LAYOUT.primary, V2.len()), V2);
    assert_eq!(
        read_magic(&mut flash, LAYOUT.secondary, SLOT).unwrap(),
        MagicState::Erased
    );

    // 2.0.0+7 vs 2.0.0: the build number is ignored, like MCUboot.
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V2);
    flash.load(LAYOUT.secondary, V2_PAD);
    let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
    assert_eq!(bootable.update, Update::Installed);
    assert_eq!(bootable.image.header.version.build, 7);
}

#[test]
fn invalid_requests_are_dropped_without_touching_the_primary() {
    for (staged, expected) in [
        (TI_V2, ImageError::BadSignature),
        (&V2[..2000], ImageError::Malformed),
    ] {
        let mut flash = MockFlash::new();
        flash.load(LAYOUT.primary, V1);
        flash.load(LAYOUT.secondary, staged);
        request(&mut flash);
        let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
        assert_eq!(bootable.update, Update::Rejected(expected));
        assert_eq!(slot_bytes(&flash, LAYOUT.primary, V1.len()), V1);
        assert_eq!(
            read_magic(&mut flash, LAYOUT.secondary, SLOT).unwrap(),
            MagicState::Erased
        );
    }
    // Request magic without any image.
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V1);
    request(&mut flash);
    let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
    assert_eq!(bootable.update, Update::Rejected(ImageError::NoImage));
}

#[test]
fn damaged_magic_is_not_a_request() {
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, V1);
    flash.load(LAYOUT.secondary, V2);
    request(&mut flash);
    flash.data[magic_address(LAYOUT.secondary, SLOT) as usize + 15] = 0x00;
    let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
    assert_eq!(bootable.update, Update::None);
    assert_eq!(bootable.image.header.version, version_of(V1));
}

#[test]
fn nothing_bootable_is_reported() {
    let mut flash = MockFlash::new();
    assert_eq!(
        boot(&mut flash, &LAYOUT, &key(), |_| {}),
        Err(BootError::NoBootableImage {
            primary: ImageError::NoImage,
            update: Update::None
        })
    );
    let mut flash = MockFlash::new();
    flash.load(LAYOUT.primary, TI_V2);
    assert_eq!(
        boot(&mut flash, &LAYOUT, &key(), |_| {}),
        Err(BootError::NoBootableImage {
            primary: ImageError::BadSignature,
            update: Update::None
        })
    );
}

/// Inject a power loss at every erase/write of an upgrade, reboot, and
/// require the final state to be the fully installed new image.
#[test]
fn upgrade_survives_power_loss_at_every_flash_operation() {
    for (old, staged) in [(Some(V1), V2), (Some(V1), V2_PAD), (None, V2)] {
        let fresh = || {
            let mut flash = MockFlash::new();
            if let Some(old) = old {
                flash.load(LAYOUT.primary, old);
            }
            flash.load(LAYOUT.secondary, staged);
            request(&mut flash);
            flash
        };
        let mut reference = fresh();
        boot(&mut reference, &LAYOUT, &key(), |_| {}).unwrap();
        let total = reference.operations;
        assert!(total > 3);

        for cut in 1..=total {
            let mut flash = fresh();
            flash.budget = Some(cut);
            let first = boot(&mut flash, &LAYOUT, &key(), |_| {});
            assert_eq!(first, Err(BootError::Flash), "cut {cut} of {total}");
            // Power loss may strike repeatedly during recovery as well.
            flash.budget = Some(cut.div_ceil(2));
            let _ = boot(&mut flash, &LAYOUT, &key(), |_| {});
            flash.budget = None;
            let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {})
                .unwrap_or_else(|error| panic!("cut {cut} of {total}: {error:?}"));
            assert_eq!(
                bootable.image.header.version,
                version_of(staged),
                "cut {cut}"
            );
            assert_eq!(
                slot_bytes(&flash, LAYOUT.primary, V2.len()),
                &staged[..V2.len()],
                "cut {cut}"
            );
            assert_ne!(
                read_magic(&mut flash, LAYOUT.secondary, SLOT).unwrap(),
                MagicState::Good
            );
        }
    }
}

#[test]
fn rejected_request_survives_power_loss_while_dropping() {
    let fresh = || {
        let mut flash = MockFlash::new();
        flash.load(LAYOUT.primary, V2);
        flash.load(LAYOUT.secondary, V1);
        request(&mut flash);
        flash
    };
    let mut reference = fresh();
    boot(&mut reference, &LAYOUT, &key(), |_| {}).unwrap();
    for cut in 1..=reference.operations {
        let mut flash = fresh();
        flash.budget = Some(cut);
        assert_eq!(
            boot(&mut flash, &LAYOUT, &key(), |_| {}),
            Err(BootError::Flash)
        );
        flash.budget = None;
        let bootable = boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap();
        assert_eq!(bootable.image.header.version, version_of(V2));
        assert_eq!(slot_bytes(&flash, LAYOUT.primary, V2.len()), V2);
    }
}

#[cfg(feature = "firmware-writer")]
mod writer {
    use super::*;
    use crate::writer::{SlotPolicy, SlotWriter};
    use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

    fn policy(key_xy: [u8; 64]) -> SlotPolicy {
        SlotPolicy {
            key: TrustedKey::new(key_xy),
            header_size: 0x100,
        }
    }

    fn download(writer: &mut SlotWriter<MockFlash>, image: &[u8], block: usize) {
        writer.erase_slot().unwrap();
        let mut offset = 0;
        for chunk in image.chunks(block) {
            writer.write_block(offset, chunk).unwrap();
            offset += chunk.len() as u32;
        }
    }

    fn dirty_flash(running: &[u8]) -> MockFlash {
        let mut flash = MockFlash::new();
        flash.load(LAYOUT.primary, running);
        // Stale data and a stale request in the secondary slot.
        flash.load(LAYOUT.secondary, &[0x5A; SLOT as usize]);
        request(&mut flash);
        flash
    }

    #[test]
    fn staged_image_installs_after_activation() {
        for (staged, block) in [(V2, 48), (V2_PAD, 64), (V2, 1), (V2_PAD, 223)] {
            let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
            assert_eq!(writer.slot_size(), SLOT);
            download(&mut writer, staged, block);
            assert_eq!(writer.running_version(), Some(version_of(V1)));
            writer.verify(staged.len() as u32, None).unwrap();
            // No request until activation, even for a padded image.
            assert_eq!(
                read_magic(writer.flash_mut(), LAYOUT.secondary, SLOT).unwrap(),
                MagicState::Erased
            );
            let mut flash = core::mem::replace(writer.flash_mut(), MockFlash::new());
            assert_eq!(
                boot(&mut flash, &LAYOUT, &key(), |_| {}).unwrap().update,
                Update::None
            );
            *writer.flash_mut() = flash;
            writer.activate().unwrap();
            writer.activate().unwrap(); // idempotent
            let flash = writer.flash_mut();
            let bootable = boot(flash, &LAYOUT, &key(), |_| {}).unwrap();
            assert_eq!(bootable.update, Update::Installed);
            assert_eq!(
                slot_bytes(flash, LAYOUT.primary, V2.len()),
                &staged[..V2.len()]
            );
        }
    }

    #[test]
    fn verification_failures_block_activation() {
        // Wrong key.
        let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TI_KEY));
        download(&mut writer, V2, 64);
        assert_eq!(
            writer.verify(V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(writer.activate(), Err(FirmwareError::ActivateFailed));

        // Size mismatch.
        let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
        download(&mut writer, V2, 64);
        assert_eq!(
            writer.verify(V2.len() as u32 + 1, None),
            Err(FirmwareError::VerifyFailed)
        );

        // Downgrade below the running image.
        let mut writer = SlotWriter::new(dirty_flash(V2), LAYOUT, policy(TEST_KEY));
        download(&mut writer, V1, 64);
        assert_eq!(
            writer.verify(V1.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );

        // Unexpected header size.
        let mut writer = SlotWriter::new(
            dirty_flash(V1),
            LAYOUT,
            SlotPolicy {
                key: TrustedKey::new(TEST_KEY),
                header_size: 0x200,
            },
        );
        download(&mut writer, V2, 64);
        assert_eq!(
            writer.verify(V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );

        // Corrupted block.
        let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
        let mut corrupt = V2.to_vec();
        corrupt[0x400] ^= 0xFF;
        download(&mut writer, &corrupt, 64);
        assert_eq!(
            writer.verify(V2.len() as u32, None),
            Err(FirmwareError::VerifyFailed)
        );
        assert_eq!(writer.activate(), Err(FirmwareError::ActivateFailed));
    }

    #[test]
    fn out_of_order_and_oversized_writes_fail() {
        let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
        writer.erase_slot().unwrap();
        assert_eq!(
            writer.write_block(4, &[0; 4]),
            Err(FirmwareError::OutOfRange)
        );
        writer.write_block(0, &[0; 4]).unwrap();
        assert_eq!(
            writer.write_block(0, &[0; 4]),
            Err(FirmwareError::OutOfRange)
        );
        assert_eq!(
            writer.write_block(4, &vec![0; SLOT as usize]),
            Err(FirmwareError::ImageTooLarge)
        );
    }

    #[test]
    fn abort_and_restart_leave_no_request() {
        let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
        download(&mut writer, V2_PAD, 64);
        writer.verify(V2_PAD.len() as u32, None).unwrap();
        writer.abort().unwrap();
        assert_eq!(writer.activate(), Err(FirmwareError::ActivateFailed));
        assert_eq!(
            read_magic(writer.flash_mut(), LAYOUT.secondary, SLOT).unwrap(),
            MagicState::Erased
        );
        // Restart from scratch after an abort works.
        download(&mut writer, V2, 100);
        writer.verify(V2.len() as u32, None).unwrap();
        writer.activate().unwrap();
        let bootable = boot(writer.flash_mut(), &LAYOUT, &key(), |_| {}).unwrap();
        assert_eq!(bootable.update, Update::Installed);
    }

    #[test]
    fn abort_without_staging_does_not_touch_flash() {
        let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
        let before = writer.flash_mut().operations;
        writer.abort().unwrap();
        writer.abort().unwrap();
        assert_eq!(writer.flash_mut().operations, before);
        // Once staging started, abort erases the trailer sector once.
        writer.erase_slot().unwrap();
        let staged = writer.flash_mut().operations;
        writer.abort().unwrap();
        writer.abort().unwrap();
        assert_eq!(writer.flash_mut().operations, staged + 1);
    }

    #[test]
    fn power_loss_during_download_never_requests_an_upgrade() {
        let mut reference = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
        download(&mut reference, V2_PAD, 64);
        reference.verify(V2_PAD.len() as u32, None).unwrap();
        let total = reference.flash_mut().operations;
        for cut in 1..=total {
            let mut writer = SlotWriter::new(dirty_flash(V1), LAYOUT, policy(TEST_KEY));
            writer.flash_mut().budget = Some(cut);
            let failed = (|| {
                writer.erase_slot()?;
                let mut offset = 0;
                for chunk in V2_PAD.chunks(64) {
                    writer.write_block(offset, chunk)?;
                    offset += chunk.len() as u32;
                }
                writer.verify(V2_PAD.len() as u32, None)
            })();
            assert!(failed.is_err(), "cut {cut}");
            let flash = writer.flash_mut();
            flash.budget = None;
            let bootable = boot(flash, &LAYOUT, &key(), |_| {}).unwrap();
            // The stale request was erased first, or the stale garbage is
            // rejected; the running image is never replaced.
            assert_ne!(bootable.update, Update::Installed, "cut {cut}");
            assert_eq!(bootable.image.header.version, version_of(V1));
        }
    }
}
