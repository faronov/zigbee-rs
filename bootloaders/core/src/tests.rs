//! Boot sequencing on the simulated flash, with a power cut injected at
//! every erase and program of an install and of a dropped request, in each
//! [`Cut`] mode, always through the slot-guarding [`BootFlash`].
//!
//! The images are the imgtool vectors of `mcuboot-image` (0x4000-byte
//! slots, 0x100-byte header, `--align 4`).

use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use mcuboot_image::trailer::{BOOT_MAGIC, MagicState, magic_address, read_magic};

use crate::entry::{check_fixed_entry, check_vector_table};
use crate::flash::{BootFlash, BootFlashError};
use crate::sim::{Cut, SimFlash};
use crate::{
    BootError, BootEvent, Bootable, ImageError, Layout, Platform, TrustedKey, Update, image_end,
    launch,
};

const KEY: [u8; 64] = *include_bytes!("../../../mcuboot-image/tests/vectors/test-key.xy");
const V1: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v1.bin");
const V2: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v2.bin");
const V2_PAD: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v2-pad.bin");
const TI_V2: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/ti-v2.bin");

const SECTOR: u32 = 0x800;
const SLOT: u32 = 0x4000;
const HEADER: u32 = 0x100;

/// ```text
/// 0x0000..0x0800  loader + LoaderInfo   (protected)
/// 0x0800..0x4800  primary slot
/// 0x4800..0x8800  secondary slot
/// 0x8800..0x9000  persistence journal   (protected)
/// ```
const LAYOUT: Layout = Layout {
    primary: 0x0800,
    secondary: 0x0800 + SLOT,
    slot_size: SLOT,
    sector_size: SECTOR,
};
const LOADER: (u32, u32) = (0, 0x0800);
const NV: (u32, u32) = (0x8800, 0x9000);
const FLASH_LEN: usize = 0x9000;

type Flash = SimFlash<{ SECTOR as usize }, 4>;

fn key() -> TrustedKey {
    TrustedKey::new(KEY)
}

/// A device with a recognisable pattern in both protected regions, `primary`
/// installed, and `secondary` staged (with the request magic if `request`).
fn device(primary: &[u8], secondary: Option<&[u8]>, request: bool) -> Flash {
    let mut flash = Flash::new(FLASH_LEN).strict();
    let pattern: Vec<u8> = (0..FLASH_LEN).map(|index| (index * 13 + 5) as u8).collect();
    flash.load(LOADER.0, &pattern[LOADER.0 as usize..LOADER.1 as usize]);
    flash.load(NV.0, &pattern[NV.0 as usize..NV.1 as usize]);
    flash.load(LAYOUT.primary, primary);
    if let Some(image) = secondary {
        flash.load(LAYOUT.secondary, image);
    }
    if request {
        flash.load(magic_address(LAYOUT.secondary, SLOT), &BOOT_MAGIC);
    }
    flash
}

fn assert_protected_intact(flash: &Flash) {
    let pattern = |index: usize| (index * 13 + 5) as u8;
    for (start, end) in [LOADER, NV] {
        for (offset, byte) in flash
            .range(start, (end - start) as usize)
            .iter()
            .enumerate()
        {
            let index = start as usize + offset;
            assert_eq!(*byte, pattern(index), "protected byte {index:#x} changed");
        }
    }
}

/// Records the calls the core makes; starts RISC-V style fixed entries.
#[derive(Default)]
struct Recorder {
    events: Vec<BootEvent>,
    kicks: usize,
    outcome: Option<Result<Bootable, BootError>>,
    prepared: Option<Bootable>,
    refuse: bool,
}

impl Platform for Recorder {
    fn progress(&mut self, event: BootEvent) {
        self.events.push(event);
    }

    fn kick_watchdog(&mut self) {
        self.kicks += 1;
    }

    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {
        assert!(self.outcome.is_none(), "decided twice");
        self.outcome = Some(*outcome);
    }

    fn prepare(&mut self, layout: &Layout, bootable: &Bootable) -> Option<u32> {
        self.prepared = Some(*bootable);
        if self.refuse {
            return None;
        }
        let entry = bootable.vector_table;
        check_fixed_entry(layout.primary, HEADER, entry, image_end(layout, bootable)).ok()?;
        Some(entry)
    }

    unsafe fn start(&mut self, _entry: u32) -> ! {
        unreachable!("host tests use launch")
    }

    fn fail(&mut self) -> ! {
        unreachable!("host tests use launch")
    }
}

/// One boot through [`BootFlash`].
fn boot_once(flash: &mut Flash) -> (Option<u32>, Recorder) {
    let mut platform = Recorder::default();
    let mut guarded = BootFlash::new(
        core::mem::replace(flash, Flash::new(SECTOR as usize)),
        &LAYOUT,
    );
    let entry = launch(&mut guarded, &LAYOUT, &key(), &mut platform);
    *flash = guarded.into_inner();
    (entry, platform)
}

fn primary_version(flash: &Flash) -> (u8, u8, u16, u32) {
    let header =
        mcuboot_image::ImageHeader::parse(flash.range(LAYOUT.primary, 32).try_into().unwrap())
            .expect("primary header");
    let version = header.version;
    (
        version.major,
        version.minor,
        version.revision,
        version.build,
    )
}

fn request_state(flash: &mut Flash) -> MagicState {
    read_magic(flash, LAYOUT.secondary, SLOT).unwrap()
}

#[test]
fn no_request_starts_the_primary() {
    let mut flash = device(V1, None, false);
    let (entry, platform) = boot_once(&mut flash);
    assert_eq!(entry, Some(LAYOUT.primary + HEADER));
    let bootable = platform.outcome.unwrap().unwrap();
    assert_eq!(bootable.update, Update::None);
    assert_eq!(platform.prepared, Some(bootable));
    assert_eq!(platform.events, [BootEvent::Validating]);
    assert_eq!(platform.kicks, platform.events.len());
    assert_eq!(flash.operations(), 0);
}

#[test]
fn request_installs_and_starts_the_new_image() {
    for image in [V2, V2_PAD] {
        let mut flash = device(V1, Some(image), image != V2_PAD);
        assert_eq!(request_state(&mut flash), MagicState::Good);
        let (entry, platform) = boot_once(&mut flash);
        assert_eq!(entry, Some(LAYOUT.primary + HEADER));
        assert_eq!(platform.outcome.unwrap().unwrap().update, Update::Installed);
        assert_eq!(primary_version(&flash).0, 2);
        assert_ne!(request_state(&mut flash), MagicState::Good);
        assert!(
            platform
                .events
                .iter()
                .any(|event| matches!(event, BootEvent::Copying { .. }))
        );
        assert_eq!(platform.kicks, platform.events.len());
        assert_protected_intact(&flash);
    }
}

#[test]
fn rejected_and_downgrade_requests_keep_the_primary() {
    // Signed by another key.
    let mut flash = device(V1, Some(TI_V2), true);
    let (entry, platform) = boot_once(&mut flash);
    assert!(entry.is_some());
    assert_eq!(
        platform.outcome.unwrap().unwrap().update,
        Update::Rejected(ImageError::BadSignature)
    );
    assert_eq!(primary_version(&flash).0, 1);
    assert_eq!(request_state(&mut flash), MagicState::Erased);

    let mut flash = device(V2, Some(V1), true);
    let (entry, platform) = boot_once(&mut flash);
    assert!(entry.is_some());
    assert_eq!(platform.outcome.unwrap().unwrap().update, Update::Downgrade);
    assert_eq!(primary_version(&flash).0, 2);
    assert_protected_intact(&flash);
}

#[test]
fn refused_entry_and_nothing_bootable_start_nothing() {
    let mut flash = device(V1, None, false);
    let mut platform = Recorder {
        refuse: true,
        ..Recorder::default()
    };
    assert_eq!(launch(&mut flash, &LAYOUT, &key(), &mut platform), None);
    assert!(platform.prepared.is_some());

    let mut flash = device(&[], None, false);
    let (entry, platform) = boot_once(&mut flash);
    assert_eq!(entry, None);
    assert!(platform.prepared.is_none());
    assert_eq!(
        platform.outcome,
        Some(Err(BootError::NoBootableImage {
            primary: ImageError::NoImage,
            update: Update::None,
        }))
    );
}

#[test]
fn cortex_m_shell_refuses_a_non_vector_table() {
    // The vectors' payload is not a vector table: a Cortex-M shell's check
    // must refuse it although the signature is valid.
    let mut flash = device(V1, None, false);
    let mut word = [0u8; 4];
    let vector_table = LAYOUT.primary + HEADER;
    flash.read(vector_table, &mut word).unwrap();
    let initial_sp = u32::from_le_bytes(word);
    flash.read(vector_table + 4, &mut word).unwrap();
    let reset = u32::from_le_bytes(word);
    assert!(
        check_vector_table(
            initial_sp,
            reset,
            vector_table,
            vector_table + 3000,
            (0x2000_0000, 0x2000_9000)
        )
        .is_err()
    );
}

#[test]
fn boot_flash_refuses_everything_outside_the_slots() {
    let mut flash = BootFlash::new(device(V1, None, false), &LAYOUT);
    for (from, to) in [
        (0x0, 0x800),
        (0x0, 0x1000),
        (0x8800, 0x9000),
        (0x8000, 0x9000),
    ] {
        assert_eq!(flash.erase(from, to), Err(BootFlashError::Protected));
    }
    assert_eq!(flash.erase(0x1000, 0x800), Err(BootFlashError::Protected));
    assert_eq!(flash.write(0x7FC, &[0; 8]), Err(BootFlashError::Protected));
    assert_eq!(flash.write(0x87FC, &[0; 8]), Err(BootFlashError::Protected));
    assert_eq!(
        flash.write(u32::MAX - 3, &[0; 4]),
        Err(BootFlashError::Protected)
    );
    assert_eq!(flash.erase(0x8000, 0x8800), Ok(()));
    assert_eq!(flash.write(0x87FC, &[0; 4]), Ok(()));
    let flash = flash.into_inner();
    assert_protected_intact(&flash);
}

/// Replay `setup` with a cut at every operation of its first boot, in every
/// mode, then reboot until something starts. `expect` checks the final
/// state. Returns the number of operations of the uninterrupted boot.
fn replay_with_cuts(setup: impl Fn() -> Flash, expect: impl Fn(&mut Flash, &Recorder)) -> usize {
    let mut clean = setup();
    let (entry, platform) = boot_once(&mut clean);
    assert!(entry.is_some());
    expect(&mut clean, &platform);
    let operations = clean.operations();
    assert!(operations > 0);

    for cut in Cut::ALL {
        for index in 0..operations {
            let mut flash = setup();
            flash.cut_at(index, cut);
            let _ = boot_once(&mut flash);
            assert!(flash.lost_power(), "{cut:?} at {index}: cut not reached");
            flash.power_cycle();
            assert_protected_intact(&flash);

            // The resumed boot must finish the job; a second boot must be a
            // plain boot of the same image.
            let (entry, platform) = boot_once(&mut flash);
            assert!(
                entry.is_some(),
                "{cut:?} at {index}: {:?}",
                platform.outcome
            );
            expect(&mut flash, &platform);
            let (again, platform) = boot_once(&mut flash);
            assert_eq!(again, entry, "{cut:?} at {index}");
            assert_eq!(platform.outcome.unwrap().unwrap().update, Update::None);
            assert_protected_intact(&flash);
        }
    }
    operations
}

#[test]
fn install_survives_a_power_cut_at_every_operation() {
    for (image, request) in [(V2, true), (V2_PAD, false)] {
        let operations = replay_with_cuts(
            || device(V1, Some(image), request),
            |flash, _| {
                assert_eq!(primary_version(flash).0, 2);
                assert_ne!(request_state(flash), MagicState::Good);
            },
        );
        // Erase primary, the copy writes, the two drop erases.
        assert!(operations >= 4, "{operations}");
    }
}

#[test]
fn dropping_a_rejected_request_survives_a_power_cut_at_every_operation() {
    for (primary, secondary) in [(V1, TI_V2), (V2, V1)] {
        replay_with_cuts(
            || device(primary, Some(secondary), true),
            |flash, _| {
                assert_eq!(
                    primary_version(flash),
                    primary_version(&device(primary, None, false))
                );
                assert_ne!(request_state(flash), MagicState::Good);
            },
        );
    }
}

/// KNOWN HAZARD, kept as a characterisation test so that fixing it is a
/// deliberate change (see docs/book/src/advanced/bootloader.md, "Open
/// decisions").
///
/// A primary whose header magic survives an interrupted erase while its
/// version bytes read higher (erase only sets bits) makes the engine apply
/// the downgrade rule against an image that no longer validates: the
/// request is dropped and nothing is bootable. MCUboot's overwrite-only
/// mode compares against the primary header the same way. Uniform
/// prefix-erase cuts (the replays above) never produce this state.
#[test]
fn known_hazard_torn_primary_header_reads_as_a_downgrade() {
    let mut torn = V1.to_vec();
    torn[20] = 0xFF; // major
    torn[0x200..0x400].fill(0xFF); // partly erased payload
    let mut flash = device(&torn, Some(V2), true);
    let (entry, platform) = boot_once(&mut flash);
    assert_eq!(entry, None);
    assert_eq!(
        platform.outcome,
        Some(Err(BootError::NoBootableImage {
            primary: ImageError::HashMismatch,
            update: Update::Downgrade,
        }))
    );
    assert_eq!(request_state(&mut flash), MagicState::Erased);
}
