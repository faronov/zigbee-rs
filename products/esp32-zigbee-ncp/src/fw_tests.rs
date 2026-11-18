use super::*;
use crate::layout::{FLASH_SIZE, ota_slot_offset};
use crate::test_support::*;
use esp32_zigbee_devkit_product::otadata::{OtaSelectEntry, STATE_ABORTED, STATE_NEW};
use esp32_zigbee_devkit_product::sha256::sha256;
use ncp_coordinator_app::platform::slot_state;

const NEWER: u32 = 0x0103_0000;
const OLDER: u32 = 0x0101_0000;

fn engine(flash: &MockFlash, dev_build: bool) -> FwUpdate<MockFlash> {
    FwUpdate::new(flash.clone(), policy(dev_build))
        .map_err(|(_, error)| error)
        .unwrap()
}

fn request(file: &[u8], version: u32, flags: u8) -> FwImage {
    FwImage {
        image_size: file.len() as u32,
        format: FORMAT_ESP_APP,
        sha256: sha256(file),
        version,
        flags,
    }
}

/// A signed 1.3.0 envelope around a ~20 KiB ESP image.
fn signed_newer() -> (Vec<u8>, Vec<u8>) {
    let image = esp_app(20_000, "1.3.0");
    let file = envelope(&image, (1, 3, 0), &signing_key(1));
    (image, file)
}

fn send(fw: &mut FwUpdate<MockFlash>, file: &[u8], chunk: usize) -> Result<u32, PlatformError> {
    let mut offset = 0u32;
    for part in file.chunks(chunk) {
        offset = fw.write(offset, part)?;
    }
    Ok(offset)
}

fn stage(fw: &mut FwUpdate<MockFlash>, file: &[u8], version: u32) -> Result<u8, PlatformError> {
    fw.begin(&request(file, version, 0))?;
    send(fw, file, MAX_CHUNK as usize)?;
    fw.finish()
}

fn header_invalidated(flash: &MockFlash, slot: u8) -> bool {
    flash.bytes(ota_slot_offset(slot), SECTOR_SIZE as usize) == vec![0xFF; SECTOR_SIZE as usize]
}

#[test]
fn versions_parse_like_cargo_and_the_app_descriptor() {
    assert_eq!(parse_version(b"1.2.3"), 0x0102_0003);
    assert_eq!(parse_version(b"0.1.0\0\0\0"), 0x0001_0000);
    assert_eq!(parse_version(b"2.10.300-dev"), 0x020A_012C);
    assert_eq!(parse_version(b"3"), 0x0300_0000);
    assert_eq!(parse_version(b"256.0.0"), 0);
    assert_eq!(parse_version(b"1.2.70000"), 0);
    assert_eq!(parse_version(b"v1.2.3"), 0);
    assert_eq!(parse_version(b""), 0);
    assert_eq!(version_word(1, 2, 3), 0x0102_0003);
}

#[test]
fn a_signed_envelope_is_staged_verified_and_activated_as_a_trial() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (image, file) = signed_newer();
    let begin = fw.begin(&request(&file, NEWER, 0)).unwrap();
    assert_eq!(
        begin,
        FwBegin {
            max_chunk: 1024,
            slot: 1
        }
    );
    // Ragged chunks: words, the 32-byte header and the TLVs are split.
    assert_eq!(send(&mut fw, &file, 1001), Ok(file.len() as u32));
    assert_eq!(fw.finish(), Ok(0));
    // Only the ESP image lands in the slot, at its start.
    assert_eq!(flash.bytes(ota_slot_offset(1), image.len()), image);
    assert_eq!(fw.activate(), Ok(()));
    let data = flash.otadata();
    assert_eq!(data.active_slot(), Some(1));
    assert_eq!(data.active_entry().unwrap().1.state, STATE_NEW);
    // The running slot (ota_0) was never written.
    let mut slots = [FwSlot {
        slot: 9,
        state: 9,
        version: 9,
        sha256: [9; 32],
    }; 2];
    assert_eq!(fw.slots(false, &mut slots), 2);
    assert_eq!(slots[1].state, slot_state::STAGED);
    assert_eq!(slots[1].version, NEWER);
    assert_eq!(slots[1].sha256[..], image[image.len() - 32..]);
    assert_eq!(slots[0].state, slot_state::RUNNING);
    assert_eq!(slots[0].version, RUNNING);
}

#[test]
fn writes_are_sequential() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    assert_eq!(fw.write(0, &file[..100]), Ok(100));
    // A gap.
    assert_eq!(
        fw.write(101, &file[101..200]),
        Err(PlatformError::InvalidParameter)
    );
    // Past the declared size, or bigger than max_chunk.
    let size = file.len() as u32;
    assert_eq!(
        fw.write(100, &vec![0; size as usize]),
        Err(PlatformError::InvalidParameter)
    );
    assert_eq!(
        fw.write(100, &file[100..100 + 1025]),
        Err(PlatformError::InvalidParameter)
    );
    // Nothing was lost: the transfer continues at 100.
    assert_eq!(fw.write(100, &file[100..1100]), Ok(1100));
    let mut offset = 1100;
    for part in file[1100..].chunks(1024) {
        offset = fw.write(offset, part).unwrap();
    }
    assert_eq!(fw.finish(), Ok(0));
}

#[test]
fn identical_rewrites_succeed_without_writing() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (image, file) = signed_newer();
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    // 1 + 3 + ... odd sizes leave sub-word tails in RAM.
    let mut offset = 0;
    for part in file[..5003].chunks(997) {
        offset = fw.write(offset, part).unwrap();
    }
    assert_eq!(offset, 5003);
    flash.take_log();
    // Header (RAM), programmed image bytes (flash), the tail word (RAM),
    // and a retry of the whole last chunk.
    for (at, len) in [
        (0, 32),
        (0, 1000),
        (40, 900),
        (4000, 1003),
        (5000, 3),
        (4006, 997),
    ] {
        assert_eq!(
            fw.write(at, &file[at as usize..at as usize + len]),
            Ok(5003),
            "rewrite at {at}"
        );
    }
    assert_eq!(flash.take_log(), (vec![], vec![]));
    // A retry overlapping next_offset appends the rest.
    assert_eq!(fw.write(4500, &file[4500..5500]), Ok(5500));
    // The trailer (TLVs, RAM) can be retried as well.
    let mut offset = 5500;
    for part in file[5500..].chunks(1000) {
        offset = fw.write(offset, part).unwrap();
    }
    let tlvs = 32 + image.len();
    flash.take_log();
    assert_eq!(
        fw.write(tlvs as u32 - 10, &file[tlvs - 10..]),
        Ok(file.len() as u32)
    );
    assert_eq!(flash.take_log(), (vec![], vec![]));
    assert_eq!(fw.finish(), Ok(0));
    assert_eq!(flash.bytes(ota_slot_offset(1), image.len()), image);
}

#[test]
fn different_bytes_below_next_offset_conflict() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    let mut offset = 0;
    for part in file[..3001].chunks(1000) {
        offset = fw.write(offset, part).unwrap();
    }
    for at in [3usize, 100, 2999, 3000] {
        let mut changed = file[at..at + 1].to_vec();
        changed[0] ^= 0x40;
        assert_eq!(
            fw.write(at as u32, &changed),
            Err(PlatformError::Conflict),
            "byte {at}"
        );
    }
    // An overlap whose stored part differs is refused as a whole.
    let mut overlap = file[2990..3100].to_vec();
    overlap[0] ^= 1;
    assert_eq!(fw.write(2990, &overlap), Err(PlatformError::Conflict));
    // The transaction is intact.
    for part in file[3001..].chunks(1000) {
        offset = fw.write(offset, part).unwrap();
    }
    assert_eq!(fw.finish(), Ok(0));
}

#[test]
fn each_write_programs_one_chunk_and_erases_at_most_one_sector() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    // FW_BEGIN erases nothing.
    assert_eq!(flash.take_log().0, vec![]);
    let mut offset = 0;
    let mut erased = Vec::new();
    for part in file.chunks(MAX_CHUNK as usize) {
        offset = fw.write(offset, part).unwrap();
        let (erases, writes) = flash.take_log();
        assert!(erases.len() <= 1, "{erases:?}");
        let programmed: usize = writes.iter().map(|(_, len)| len).sum();
        assert!(programmed <= MAX_CHUNK as usize + 4);
        erased.extend(erases);
    }
    // Sectors are erased in order, once each, right before use.
    let expected: Vec<u32> = (0..fw.sectors_erased())
        .map(|sector| ota_slot_offset(1) + sector * SECTOR_SIZE)
        .collect();
    assert_eq!(erased, expected);
    assert_eq!(fw.finish(), Ok(0));
}

#[test]
fn too_large_images_are_refused() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, true);
    let mut image = request(&[0; 64], NEWER, 0);
    image.image_size = MAX_FILE_SIZE + 1;
    assert_eq!(fw.begin(&image), Err(PlatformError::TooLarge));

    // A plain image declared larger than the slot.
    let plain = esp_app(1000, "1.3.0");
    let mut image = request(&plain, NEWER, 0);
    image.image_size = OTA_SLOT_SIZE + 4;
    fw.begin(&image).unwrap();
    assert_eq!(fw.write(0, &plain[..64]), Err(PlatformError::TooLarge));
    assert_eq!(
        fw.write(64, &plain[64..128]),
        Err(PlatformError::InvalidState)
    );

    // An envelope whose ESP image exceeds the slot.
    let (_, mut file) = signed_newer();
    file[12..16].copy_from_slice(&(OTA_SLOT_SIZE + 4).to_le_bytes());
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    assert_eq!(fw.write(0, &file[..32]), Err(PlatformError::TooLarge));

    // TLVs larger than the RAM trailer.
    let (image_bytes, mut file) = signed_newer();
    file.extend_from_slice(&[0u8; TRAILER_CAPACITY]);
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    assert_eq!(fw.write(0, &file[..64]), Err(PlatformError::TooLarge));
    assert!(image_bytes.len() < OTA_SLOT_SIZE as usize);

    // Other formats and tiny images.
    let mut image = request(&plain, NEWER, 0);
    image.format = 2;
    assert_eq!(fw.begin(&image), Err(PlatformError::InvalidParameter));
    image.format = FORMAT_ESP_APP;
    image.image_size = 31;
    assert_eq!(fw.begin(&image), Err(PlatformError::InvalidParameter));
}

#[test]
fn a_wrong_file_hash_is_refused_and_the_slot_invalidated() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    let mut image = request(&file, NEWER, 0);
    image.sha256[0] ^= 1;
    fw.begin(&image).unwrap();
    send(&mut fw, &file, 1024).unwrap();
    assert_eq!(fw.finish(), Err(PlatformError::VerifyFailed));
    assert!(header_invalidated(&flash, 1));
    assert_eq!(fw.write(0, &file[..10]), Err(PlatformError::InvalidState));
    assert_eq!(fw.activate(), Err(PlatformError::InvalidState));
    assert_eq!(flash.otadata().active_slot(), None);
}

#[test]
fn a_corrupt_esp_image_is_refused_even_with_a_matching_file_hash() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, true);
    let mut plain = esp_app(5000, "1.3.0");
    plain[2000] ^= 0x10; // breaks the checksum and the appended SHA-256
    assert_eq!(
        stage(&mut fw, &plain, NEWER),
        Err(PlatformError::VerifyFailed)
    );
    assert!(header_invalidated(&flash, 1));
    // Neither an ESP image nor an envelope.
    let mut junk = esp_app(5000, "1.3.0");
    junk[0] = 0x00;
    fw.begin(&request(&junk, NEWER, 0)).unwrap();
    assert_eq!(fw.write(0, &junk[..32]), Err(PlatformError::VerifyFailed));
}

#[test]
fn an_incomplete_transfer_is_a_length_failure_that_can_still_complete() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    let half = file.len() / 2;
    let mut offset = 0;
    for part in file[..half].chunks(1024) {
        offset = fw.write(offset, part).unwrap();
    }
    assert_eq!(fw.finish(), Err(PlatformError::VerifyFailed));
    for part in file[half..].chunks(1024) {
        offset = fw.write(offset, part).unwrap();
    }
    assert_eq!(fw.finish(), Ok(0));
    // A retried FW_FINISH (lost response) reports success again.
    assert_eq!(fw.finish(), Ok(0));
}

#[test]
fn unsigned_images_need_a_development_build() {
    let plain = esp_app(5000, "1.3.0");
    let flash = MockFlash::new();
    let mut release = engine(&flash, false);
    assert_eq!(
        stage(&mut release, &plain, NEWER),
        Err(PlatformError::AccessDenied)
    );
    assert!(header_invalidated(&flash, 1));

    let flash = MockFlash::new();
    let mut dev = engine(&flash, true);
    assert_eq!(stage(&mut dev, &plain, NEWER), Ok(0));
    assert_eq!(dev.activate(), Ok(()));
    assert_eq!(flash.otadata().active_slot(), Some(1));
}

#[test]
fn envelopes_from_untrusted_keys_are_refused() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, true);
    let file = envelope(&esp_app(5000, "1.3.0"), (1, 3, 0), &signing_key(2));
    assert_eq!(
        stage(&mut fw, &file, NEWER),
        Err(PlatformError::AccessDenied)
    );
    assert!(header_invalidated(&flash, 1));
    // A tampered TLV area (hash TLV) is a verification failure.
    let (_, mut file) = signed_newer();
    let at = file.len() - 120;
    file[at] ^= 1;
    assert!(matches!(
        stage(&mut fw, &file, NEWER),
        Err(PlatformError::VerifyFailed | PlatformError::AccessDenied)
    ));
}

#[test]
fn downgrades_need_the_flag_and_a_development_build() {
    let older_signed = envelope(&esp_app(5000, "1.1.0"), (1, 1, 0), &signing_key(1));
    let older_plain = esp_app(5000, "1.1.0");

    // Release: refused at FW_BEGIN, with or without the flag.
    let flash = MockFlash::new();
    let mut release = engine(&flash, false);
    for flags in [0, FLAG_ALLOW_DOWNGRADE] {
        assert_eq!(
            release.begin(&request(&older_signed, OLDER, flags)),
            Err(PlatformError::AccessDenied)
        );
    }
    // Lying about the version does not help: the envelope's version must
    // match FW_BEGIN.version.
    assert_eq!(
        stage(&mut release, &older_signed, NEWER),
        Err(PlatformError::VerifyFailed)
    );

    // Development build: only with the flag.
    let flash = MockFlash::new();
    let mut dev = engine(&flash, true);
    assert_eq!(
        dev.begin(&request(&older_plain, OLDER, 0)),
        Err(PlatformError::AccessDenied)
    );
    dev.begin(&request(&older_signed, OLDER, FLAG_ALLOW_DOWNGRADE))
        .unwrap();
    send(&mut dev, &older_signed, 1024).unwrap();
    assert_eq!(dev.finish(), Ok(0));
    dev.begin(&request(&older_plain, OLDER, FLAG_ALLOW_DOWNGRADE))
        .unwrap();
    send(&mut dev, &older_plain, 1024).unwrap();
    assert_eq!(dev.finish(), Ok(0));
    // Same version (reinstall) is not a downgrade.
    let same = envelope(&esp_app(5000, "1.2.0"), (1, 2, 0), &signing_key(1));
    let mut release = engine(&MockFlash::new(), false);
    assert_eq!(stage(&mut release, &same, RUNNING), Ok(0));
}

#[test]
fn abort_discards_and_invalidates_only_what_this_transfer_wrote() {
    let flash = MockFlash::new();
    // An older valid image sits in ota_1.
    let old = esp_app(3000, "1.1.0");
    flash.put(ota_slot_offset(1), &old);
    let mut fw = engine(&flash, false);
    // Abort while idle, or after FW_BEGIN only: nothing is erased.
    assert_eq!(fw.abort(), Ok(()));
    let (_, file) = signed_newer();
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    assert_eq!(fw.abort(), Ok(()));
    assert_eq!(flash.take_log().0, vec![]);
    let mut slots = [FwSlot {
        slot: 0,
        state: 0,
        version: 0,
        sha256: [0; 32],
    }; 2];
    fw.slots(false, &mut slots);
    assert_eq!(slots[1].state, slot_state::INACTIVE);
    assert_eq!(slots[1].version, OLDER);
    assert_eq!(slots[1].sha256[..], old[old.len() - 32..]);
    // Abort mid-transfer: the slot header is gone.
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    fw.write(0, &file[..2000]).unwrap_err(); // > max_chunk
    fw.write(0, &file[..1000]).unwrap();
    assert_eq!(fw.abort(), Ok(()));
    assert!(header_invalidated(&flash, 1));
    assert_eq!(
        fw.write(1000, &file[1000..1010]),
        Err(PlatformError::InvalidState)
    );
    fw.slots(false, &mut slots);
    assert_eq!(slots[1].state, slot_state::EMPTY);
    // Abort after FW_FINISH as well.
    assert_eq!(stage(&mut fw, &file, NEWER), Ok(0));
    assert_eq!(fw.abort(), Ok(()));
    assert!(header_invalidated(&flash, 1));
    assert_eq!(fw.activate(), Err(PlatformError::InvalidState));
}

#[test]
fn abort_after_activation_reselects_the_running_slot() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    assert_eq!(stage(&mut fw, &file, NEWER), Ok(0));
    fw.activate().unwrap();
    assert_eq!(flash.otadata().active_slot(), Some(1));
    assert_eq!(fw.abort(), Ok(()));
    let data = flash.otadata();
    assert_eq!(data.active_slot(), Some(0));
    assert_eq!(data.active_entry().unwrap().1.state, STATE_VALID);
    assert!(header_invalidated(&flash, 1));
    // The next boot of ota_0 has nothing to verify.
    let mut boot = flash.clone();
    assert_eq!(check_boot(&mut boot), Ok(BootCheck::Confirmed));
}

#[test]
fn a_flash_failure_while_writing_is_a_storage_error_and_ends_the_transfer() {
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    fw.begin(&request(&file, NEWER, 0)).unwrap();
    fw.write(0, &file[..1000]).unwrap();
    flash.fail_write_at.set(Some(ota_slot_offset(1) + 968));
    assert_eq!(
        fw.write(1000, &file[1000..2000]),
        Err(PlatformError::Storage)
    );
    assert_eq!(
        fw.write(1000, &file[1000..2000]),
        Err(PlatformError::InvalidState)
    );
    assert!(header_invalidated(&flash, 1));
}

#[test]
fn trial_boot_confirm_and_rollback_transitions() {
    // Stage and activate slot 1 while running ota_0 (otadata erased: the
    // factory state).
    let flash = MockFlash::new();
    let mut fw = engine(&flash, false);
    let (_, file) = signed_newer();
    stage(&mut fw, &file, NEWER).unwrap();
    fw.activate().unwrap();

    // Reboot into ota_1: first boot marks the attempt (trial).
    let mut boot = flash.clone();
    boot.set_running(1);
    assert_eq!(
        check_boot(&mut boot),
        Ok(BootCheck::PendingVerify { slot: 1 })
    );
    // No staging over the rollback target while on trial.
    let mut trial = engine(&flash, false);
    assert_eq!(
        trial.begin(&request(&file, NEWER, 0)),
        Err(PlatformError::InvalidState)
    );

    // Confirmed: later boots are normal, and staging works again.
    let snapshot = flash.data.borrow().clone();
    assert_eq!(confirm_boot(&mut boot), Ok(true));
    assert_eq!(confirm_boot(&mut boot), Ok(false));
    assert_eq!(check_boot(&mut boot), Ok(BootCheck::Confirmed));
    assert_eq!(flash.otadata().active_entry().unwrap().1.state, STATE_VALID);
    assert!(trial.begin(&request(&file, NEWER, 0)).is_ok());

    // Not confirmed, second boot: the bootloader-side rollback.
    *flash.data.borrow_mut() = snapshot.clone();
    assert_eq!(check_boot(&mut boot), Ok(BootCheck::RolledBack { slot: 0 }));
    let data = flash.otadata();
    assert_eq!(data.active_slot(), Some(0));
    let abandoned = data
        .entries
        .iter()
        .find(|entry| entry.seq != data.max_seq().unwrap())
        .unwrap();
    assert_eq!(abandoned.state, STATE_ABORTED);
    boot.set_running(0);
    assert_eq!(check_boot(&mut boot), Ok(BootCheck::Confirmed));

    // Not confirmed within the window: the application rolls back.
    *flash.data.borrow_mut() = snapshot;
    boot.set_running(1);
    assert_eq!(roll_back(&mut boot), Ok(Some(0)));
    assert_eq!(flash.otadata().active_slot(), Some(0));
    assert_eq!(roll_back(&mut boot), Ok(None));
    boot.set_running(0);
    assert_eq!(check_boot(&mut boot), Ok(BootCheck::Confirmed));
}

#[test]
fn a_bootloader_fallback_is_recorded() {
    // otadata prefers ota_1 (NEW) but the bootloader started ota_0.
    let flash = MockFlash::new();
    let entry = OtaSelectEntry::new(2, STATE_NEW).encode();
    flash.put(otadata_sector_offset(0), &entry);
    let mut boot = flash.clone();
    assert_eq!(
        check_boot(&mut boot),
        Ok(BootCheck::FallbackRecorded { slot: 0 })
    );
    assert_eq!(flash.otadata().active_slot(), Some(0));
}

#[test]
fn a_foreign_partition_table_or_missing_evidence_disables_fw() {
    let flash = MockFlash::new();
    flash.put(PARTITION_TABLE_OFFSET + 32 + 8, &[0, 0, 0x1F, 0]);
    assert!(FwUpdate::new(flash.clone(), policy(false)).is_err());
    let mut boot = flash.clone();
    assert_eq!(
        check_boot(&mut boot),
        Err(BootCheckError::Flash(FirmwareError::HardwareError))
    );

    let flash = MockFlash::new();
    flash.running.set(Err(RunningImageError::Unavailable));
    assert!(matches!(
        FwUpdate::new(flash.clone(), policy(false)),
        Err((
            _,
            BootCheckError::RunningImage(RunningImageError::Unavailable)
        ))
    ));
}

#[test]
fn staging_from_ota_1_targets_ota_0() {
    let flash = MockFlash::new();
    flash.set_running(1);
    let mut fw = engine(&flash, false);
    let (image, file) = signed_newer();
    let begin = fw.begin(&request(&file, NEWER, 0)).unwrap();
    assert_eq!(begin.slot, 0);
    send(&mut fw, &file, 512).unwrap();
    assert_eq!(fw.finish(), Ok(0));
    assert_eq!(flash.bytes(ota_slot_offset(0), image.len()), image);
    assert!(FLASH_SIZE >= ota_slot_offset(1) + OTA_SLOT_SIZE);
}
