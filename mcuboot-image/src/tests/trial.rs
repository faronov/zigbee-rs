//! Phase 2 behaviour: multiple trusted keys, Ed25519, downgrade checks only
//! against a valid primary, trial boots with confirm, and swap-move with
//! revert. Every flash-changing path is replayed with a power loss at each
//! erase and write.

use super::*;
use crate::boot::{Bootable, Mode, Trial, TrialState, confirm, trial_state};

const KEY_B: [u8; 64] = *include_bytes!("../../tests/vectors/test-key-b.xy");
/// 2.0.0 signed with key B.
const B_V2: &[u8] = include_bytes!("../../tests/vectors/test-b-v2.bin");
/// 2.0.0 signed with the Ed25519 test key.
const ED_V2: &[u8] = include_bytes!("../../tests/vectors/test-ed-v2.bin");
#[cfg(feature = "ed25519")]
const ED_KEY: [u8; 32] = *include_bytes!("../../tests/vectors/test-ed-key.pub");

static BOTH: [TrustedKey; 2] = [TrustedKey::new(KEY_B), TrustedKey::new(TEST_KEY)];
const TRIAL: BootConfig<'static> = BootConfig {
    trial_boots: 3,
    ..CONFIG
};

fn copy_of(flash: &MockFlash) -> MockFlash {
    let mut copy = MockFlash::new();
    copy.data.clone_from(&flash.data);
    copy
}

fn with(primary: Option<&[u8]>, staged: Option<&[u8]>) -> MockFlash {
    let mut flash = MockFlash::new();
    if let Some(image) = primary {
        flash.load(LAYOUT.primary, image);
    }
    if let Some(image) = staged {
        flash.load(LAYOUT.secondary, image);
        request(&mut flash);
    }
    flash
}

fn boot_ok(flash: &mut MockFlash, config: &BootConfig<'_>) -> Bootable {
    boot(flash, config, |_| {}).unwrap()
}

/// Cut power at every erase/write of one boot from `start`, let the
/// recovery boot be cut as well, then boot without cuts and `check` the
/// result. (A cut while counting a trial boot is not reported: accounting
/// never blocks a boot.)
fn replay_cuts(
    start: &MockFlash,
    config: &BootConfig<'_>,
    mut check: impl FnMut(&mut MockFlash, Bootable, usize),
) {
    let mut reference = copy_of(start);
    boot_ok(&mut reference, config);
    let total = reference.operations;
    assert!(total > 0);
    for cut in 1..=total {
        let mut flash = copy_of(start);
        flash.budget = Some(cut);
        let _ = boot(&mut flash, config, |_| {});
        flash.budget = Some(cut.div_ceil(2));
        let _ = boot(&mut flash, config, |_| {});
        flash.budget = None;
        let bootable = boot(&mut flash, config, |_| {})
            .unwrap_or_else(|error| panic!("cut {cut} of {total}: {error:?}"));
        check(&mut flash, bootable, cut);
    }
}

// ── keys ────────────────────────────────────────────────────────────────

#[test]
fn any_trusted_key_validates_and_the_keyhash_selects_it() {
    for image in [V2, B_V2] {
        let mut flash = with(Some(image), None);
        let valid = validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &BOTH).unwrap();
        assert_eq!(valid.header.version, ImageVersion::new(2, 0, 0, 0));
    }
    let mut flash = with(Some(B_V2), None);
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &KEYS),
        Err(ImageError::BadSignature)
    );
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &[]),
        Err(ImageError::BadSignature)
    );
}

/// Rotation: a loader trusting {A, B} installs a B-signed update over an
/// A-signed image; a loader trusting only A refuses it.
#[test]
fn rotation_installs_an_image_signed_with_the_second_key() {
    let both = BootConfig::overwrite(LAYOUT, &BOTH);
    let mut flash = with(Some(V1), Some(B_V2));
    let bootable = boot_ok(&mut flash, &both);
    assert_eq!(bootable.update, Update::Installed);
    assert_eq!(slot_bytes(&flash, LAYOUT.primary, B_V2.len()), B_V2);

    let mut flash = with(Some(V1), Some(B_V2));
    let bootable = boot_ok(&mut flash, &CONFIG);
    assert_eq!(bootable.update, Update::Rejected(ImageError::BadSignature));
    assert_eq!(slot_bytes(&flash, LAYOUT.primary, V1.len()), V1);
}

#[test]
fn key_set_caps_and_filters() {
    use crate::image::{KeySet, MAX_TRUSTED_KEYS};
    static MANY: [TrustedKey; 5] = [TrustedKey::new([1; 64]); 5];
    assert_eq!(KeySet::new(&MANY).len(), MAX_TRUSTED_KEYS);
    assert_eq!(KeySet::new(&MANY).iter().count(), MAX_TRUSTED_KEYS);
    assert!(KeySet::new(&[]).is_empty());
    let mut set = KeySet::new(&BOTH);
    set.retain(|key| key.id() == TrustedKey::new(TEST_KEY).id());
    assert!(set.iter().eq(&BOTH[1..]));
    set.retain(|_| false);
    assert!(set.is_empty());
}

#[cfg(feature = "ed25519")]
#[test]
fn ed25519_images_validate_with_an_ed25519_key_only() {
    let ed = TrustedKey::ed25519(ED_KEY);
    let mut flash = with(Some(ED_V2), None);
    let valid = validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &[ed]).unwrap();
    assert_eq!(valid.header.version, ImageVersion::new(2, 0, 0, 0));
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &KEYS),
        Err(ImageError::BadSignature)
    );
    // A mixed set validates both algorithms.
    let mixed = [TrustedKey::new(TEST_KEY), ed];
    let mut flash = with(Some(V2), None);
    assert!(validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &mixed).is_ok());
    // A damaged signature fails.
    let mut flash = with(Some(ED_V2), None);
    let last = LAYOUT.primary as usize + ED_V2.len() - 1;
    flash.data[last] ^= 1;
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &[ed]),
        Err(ImageError::BadSignature)
    );
    // Upgrade through the engine.
    let config = BootConfig::overwrite(LAYOUT, &mixed);
    let mut flash = with(Some(V1), Some(ED_V2));
    assert_eq!(boot_ok(&mut flash, &config).update, Update::Installed);
}

#[cfg(not(feature = "ed25519"))]
#[test]
fn ed25519_images_are_rejected_without_the_feature() {
    let mut flash = with(Some(ED_V2), None);
    assert_eq!(
        validate(&mut flash, LAYOUT.primary, LAYOUT.max_image_len(), &KEYS),
        Err(ImageError::BadSignature)
    );
}

// ── downgrade rule only against a valid primary ─────────────────────────

/// A primary whose header claims a higher version but that does not
/// validate (a torn erase sets header bits) never blocks a valid update.
#[test]
fn invalid_primary_never_blocks_a_valid_update() {
    // Version 255.x claimed by a damaged V1 header.
    let mut torn = V1.to_vec();
    torn[20] = 0xFF;
    // V2 with a damaged payload: version 2 is real but the image is not.
    let mut corrupt = V2.to_vec();
    corrupt[0x400] ^= 0x01;
    for primary in [&torn[..], &corrupt[..]] {
        let mut flash = with(Some(primary), Some(V1));
        let bootable = boot_ok(&mut flash, &CONFIG);
        assert_eq!(bootable.update, Update::Installed);
        assert_eq!(bootable.image.header.version, version_of(V1));
        assert_eq!(slot_bytes(&flash, LAYOUT.primary, V1.len()), V1);
    }
    // A valid higher primary still refuses the downgrade.
    let mut flash = with(Some(V2), Some(V1));
    assert_eq!(boot_ok(&mut flash, &CONFIG).update, Update::Downgrade);
}

/// The reported brick: power lost while erasing a primary leaves a header
/// that reads as a higher version. Every cut during an install from such a
/// state ends with the update installed.
#[test]
fn torn_primary_header_survives_power_loss_and_installs() {
    let mut torn = V2.to_vec();
    torn[20] = 0xFF;
    torn[0x400] = 0xFF;
    let start = with(Some(&torn), Some(V1));
    for config in [CONFIG, TRIAL] {
        replay_cuts(&start, &config, |flash, bootable, cut| {
            assert_eq!(bootable.image.header.version, version_of(V1), "cut {cut}");
            assert_eq!(slot_bytes(flash, LAYOUT.primary, V1.len()), V1, "cut {cut}");
        });
    }
}

// ── trial boots and confirm (overwrite) ─────────────────────────────────

#[test]
fn installed_image_is_on_trial_until_confirmed() {
    let mut flash = with(Some(V1), Some(V2));
    let bootable = boot_ok(&mut flash, &TRIAL);
    assert_eq!(bootable.update, Update::Installed);
    assert_eq!(bootable.trial, Trial::Testing { boot: 1 });
    assert_eq!(
        trial_state(&mut flash, &LAYOUT, Mode::Overwrite),
        Ok(TrialState::Testing { attempts: 1 })
    );
    assert_eq!(
        boot_ok(&mut flash, &TRIAL).trial,
        Trial::Testing { boot: 2 }
    );
    assert_eq!(
        confirm(&mut flash, &LAYOUT, Mode::Overwrite),
        Ok(TrialState::Testing { attempts: 2 })
    );
    // Confirmed images cost no flash writes, and confirming again neither.
    let before = flash.operations;
    for _ in 0..5 {
        assert_eq!(boot_ok(&mut flash, &TRIAL).trial, Trial::Confirmed);
        assert_eq!(
            confirm(&mut flash, &LAYOUT, Mode::Overwrite),
            Ok(TrialState::Confirmed)
        );
    }
    assert_eq!(flash.operations, before);
    assert_eq!(
        boot_ok(&mut flash, &TRIAL).image.header.version,
        version_of(V2)
    );
}

#[test]
fn unconfirmed_overwrite_install_keeps_booting_without_writes() {
    let mut flash = with(Some(V1), Some(V2));
    for boot_number in 1..=3 {
        assert_eq!(
            boot_ok(&mut flash, &TRIAL).trial,
            Trial::Testing { boot: boot_number }
        );
    }
    let before = flash.operations;
    for _ in 0..4 {
        let bootable = boot_ok(&mut flash, &TRIAL);
        assert_eq!(bootable.trial, Trial::Unconfirmed);
        assert_eq!(bootable.image.header.version, version_of(V2));
    }
    assert_eq!(flash.operations, before);
    // A late confirm still sticks.
    confirm(&mut flash, &LAYOUT, Mode::Overwrite).unwrap();
    assert_eq!(boot_ok(&mut flash, &TRIAL).trial, Trial::Confirmed);
}

#[test]
fn factory_and_untracked_images_are_permanent() {
    // No install record: factory image, or trial accounting disabled.
    let mut flash = with(Some(V1), None);
    let before = flash.operations;
    assert_eq!(boot_ok(&mut flash, &TRIAL).trial, Trial::Permanent);
    assert_eq!(
        confirm(&mut flash, &LAYOUT, Mode::Overwrite),
        Ok(TrialState::Permanent)
    );
    assert_eq!(flash.operations, before);

    let mut flash = with(Some(V1), Some(V2));
    let bootable = boot_ok(&mut flash, &CONFIG);
    assert_eq!(bootable.trial, Trial::Permanent);
    let trailer = LAYOUT.trailer_sector(LAYOUT.primary) as usize;
    assert!(
        flash.data[trailer..trailer + SECTOR as usize]
            .iter()
            .all(|byte| *byte == 0xFF)
    );
    // A loader with trial boots later treats it as permanent too.
    assert_eq!(boot_ok(&mut flash, &TRIAL).trial, Trial::Permanent);
}

#[test]
fn padded_image_installs_on_trial() {
    // imgtool --pad puts its trailer in the last 128 bytes of the slot,
    // which the trial log leaves alone; only the image is copied.
    let mut flash = with(Some(V1), None);
    flash.load(LAYOUT.secondary, V2_PAD);
    let bootable = boot_ok(&mut flash, &TRIAL);
    assert_eq!(bootable.update, Update::Installed);
    assert_eq!(bootable.trial, Trial::Testing { boot: 1 });
    let end = (LAYOUT.primary + SLOT) as usize;
    assert!(flash.data[end - 128..end].iter().all(|byte| *byte == 0xFF));
}

#[test]
fn trial_install_survives_power_loss_at_every_operation() {
    let start = with(Some(V1), Some(V2));
    replay_cuts(&start, &TRIAL, |flash, bootable, cut| {
        assert_eq!(bootable.image.header.version, version_of(V2), "cut {cut}");
        assert!(
            matches!(bootable.trial, Trial::Testing { boot } if boot <= 3),
            "cut {cut}: {:?}",
            bootable.trial
        );
        assert_ne!(
            read_magic(flash, LAYOUT.secondary, SLOT).unwrap(),
            MagicState::Good
        );
        confirm(flash, &LAYOUT, Mode::Overwrite).unwrap();
        assert_eq!(boot_ok(flash, &TRIAL).trial, Trial::Confirmed, "cut {cut}");
    });
}

#[test]
fn confirm_survives_power_loss() {
    let mut start = with(Some(V1), Some(V2));
    boot_ok(&mut start, &TRIAL);
    let mut reference = copy_of(&start);
    confirm(&mut reference, &LAYOUT, Mode::Overwrite).unwrap();
    assert_eq!(reference.operations, 1);
    let mut flash = copy_of(&start);
    flash.budget = Some(1);
    assert!(confirm(&mut flash, &LAYOUT, Mode::Overwrite).is_err());
    flash.budget = None;
    // The torn record counts as a boot, never as a confirmation.
    let bootable = boot_ok(&mut flash, &TRIAL);
    assert_eq!(bootable.image.header.version, version_of(V2));
    assert_eq!(bootable.trial, Trial::Testing { boot: 3 });
    confirm(&mut flash, &LAYOUT, Mode::Overwrite).unwrap();
    assert_eq!(boot_ok(&mut flash, &TRIAL).trial, Trial::Confirmed);
}

// ── swap-move ───────────────────────────────────────────────────────────

#[cfg(feature = "swap-move")]
mod swap {
    use super::*;

    const STATUS: u32 = LAYOUT.secondary + SLOT;
    const SWAP: BootConfig<'static> = BootConfig {
        layout: LAYOUT,
        keys: &KEYS,
        trial_boots: 3,
        mode: Mode::SwapMove { status: STATUS },
    };
    const MODE: Mode = SWAP.mode;

    fn installed(flash: &mut MockFlash) {
        assert_eq!(slot_bytes(flash, LAYOUT.primary, V2.len()), V2);
        assert_eq!(slot_bytes(flash, LAYOUT.secondary, V1.len()), V1);
        assert_ne!(
            read_magic(flash, LAYOUT.secondary, SLOT).unwrap(),
            MagicState::Good
        );
    }

    #[test]
    fn install_keeps_the_previous_image_and_confirm_keeps_the_new() {
        let mut flash = with(Some(V1), Some(V2));
        let bootable = boot_ok(&mut flash, &SWAP);
        assert_eq!(bootable.update, Update::Installed);
        assert_eq!(bootable.trial, Trial::Testing { boot: 1 });
        installed(&mut flash);
        assert_eq!(
            confirm(&mut flash, &LAYOUT, MODE),
            Ok(TrialState::Testing { attempts: 1 })
        );
        let before = flash.operations;
        for _ in 0..5 {
            let bootable = boot_ok(&mut flash, &SWAP);
            assert_eq!(bootable.trial, Trial::Confirmed);
            assert_eq!(bootable.update, Update::None);
        }
        assert_eq!(flash.operations, before);
        installed(&mut flash);
    }

    #[test]
    fn unconfirmed_install_reverts_after_its_trial_boots() {
        let mut flash = with(Some(V1), Some(V2));
        for boot_number in 1..=3 {
            assert_eq!(
                boot_ok(&mut flash, &SWAP).trial,
                Trial::Testing { boot: boot_number }
            );
        }
        let bootable = boot_ok(&mut flash, &SWAP);
        assert_eq!(bootable.update, Update::Reverted);
        assert_eq!(bootable.trial, Trial::Permanent);
        assert_eq!(bootable.image.header.version, version_of(V1));
        assert_eq!(slot_bytes(&flash, LAYOUT.primary, V1.len()), V1);
        assert_eq!(
            trial_state(&mut flash, &LAYOUT, MODE),
            Ok(TrialState::Permanent)
        );
        let before = flash.operations;
        for _ in 0..3 {
            let bootable = boot_ok(&mut flash, &SWAP);
            assert_eq!(bootable.update, Update::None);
            assert_eq!(bootable.image.header.version, version_of(V1));
        }
        assert_eq!(flash.operations, before);
    }

    #[test]
    fn broken_install_reverts_immediately() {
        let mut flash = with(Some(V1), Some(V2));
        boot_ok(&mut flash, &SWAP);
        flash.data[LAYOUT.primary as usize + 0x300] ^= 0x01;
        let bootable = boot_ok(&mut flash, &SWAP);
        assert_eq!(bootable.update, Update::Reverted);
        assert_eq!(bootable.image.header.version, version_of(V1));
    }

    #[test]
    fn without_a_previous_image_an_unconfirmed_install_stays() {
        let mut flash = with(None, Some(V2));
        for _ in 0..3 {
            boot_ok(&mut flash, &SWAP);
        }
        let bootable = boot_ok(&mut flash, &SWAP);
        assert_eq!(bootable.trial, Trial::Unconfirmed);
        assert_eq!(bootable.image.header.version, version_of(V2));
    }

    #[test]
    fn swap_applies_the_same_downgrade_rule() {
        let mut flash = with(Some(V2), Some(V1));
        assert_eq!(boot_ok(&mut flash, &SWAP).update, Update::Downgrade);
        assert_eq!(slot_bytes(&flash, LAYOUT.primary, V2.len()), V2);
        let mut torn = V2.to_vec();
        torn[0x400] ^= 1;
        let mut flash = with(Some(&torn), Some(V1));
        assert_eq!(boot_ok(&mut flash, &SWAP).update, Update::Installed);
    }

    #[test]
    fn inconsistent_status_area_is_a_config_error() {
        for status in [LAYOUT.secondary, LAYOUT.primary - SECTOR, STATUS + 4] {
            let config = BootConfig {
                mode: Mode::SwapMove { status },
                ..SWAP
            };
            let mut flash = with(Some(V1), Some(V2));
            assert_eq!(boot(&mut flash, &config, |_| {}), Err(BootError::Config));
            assert_eq!(flash.operations, 0);
        }
    }

    #[test]
    fn install_survives_power_loss_at_every_operation() {
        let start = with(Some(V1), Some(V2));
        replay_cuts(&start, &SWAP, |flash, bootable, cut| {
            assert_eq!(bootable.image.header.version, version_of(V2), "cut {cut}");
            installed(flash);
        });
    }

    #[test]
    fn revert_survives_power_loss_at_every_operation() {
        let mut start = with(Some(V1), Some(V2));
        for _ in 0..3 {
            boot_ok(&mut start, &SWAP);
        }
        replay_cuts(&start, &SWAP, |flash, bootable, cut| {
            assert_eq!(bootable.image.header.version, version_of(V1), "cut {cut}");
            assert_eq!(bootable.trial, Trial::Permanent, "cut {cut}");
            assert_eq!(slot_bytes(flash, LAYOUT.primary, V1.len()), V1, "cut {cut}");
            assert_eq!(
                slot_bytes(flash, LAYOUT.secondary, V2.len()),
                V2,
                "cut {cut}"
            );
        });
    }

    /// A second update after a confirmed one exchanges again, through the
    /// other status sector.
    #[test]
    fn consecutive_updates_alternate_status_sectors() {
        let mut flash = with(Some(V1), Some(V2));
        boot_ok(&mut flash, &SWAP);
        confirm(&mut flash, &LAYOUT, MODE).unwrap();
        // Stage V2_PAD (2.0.0+7: an equal release is reinstalled).
        flash.data[LAYOUT.secondary as usize..(LAYOUT.secondary + SLOT) as usize].fill(0xFF);
        flash.load(LAYOUT.secondary, V2_PAD);
        let bootable = boot_ok(&mut flash, &SWAP);
        assert_eq!(bootable.update, Update::Installed);
        assert_eq!(bootable.image.header.version.build, 7);
        assert_eq!(bootable.trial, Trial::Testing { boot: 1 });
        assert_eq!(slot_bytes(&flash, LAYOUT.secondary, V2.len()), V2);
    }

    #[cfg(feature = "firmware-writer")]
    #[test]
    fn writer_keeps_the_revert_image_until_confirmed() {
        use crate::image::KeySet;
        use crate::writer::{SlotPolicy, SlotWriter};
        use zigbee_runtime::firmware_writer::FirmwareWriter;

        let mut flash = with(Some(V1), Some(V2));
        boot_ok(&mut flash, &SWAP);
        let policy = SlotPolicy {
            keys: KeySet::new(&KEYS),
            header_size: 0x100,
            mode: MODE,
        };
        let mut writer = SlotWriter::new(flash, LAYOUT, policy);
        assert!(writer.erase_slot().is_err());
        assert_eq!(
            writer.trial_state(),
            Some(TrialState::Testing { attempts: 1 })
        );
        writer.confirm_image().unwrap();
        assert_eq!(writer.trial_state(), Some(TrialState::Confirmed));
        assert!(writer.erase_slot().is_ok());
    }
}
