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
    BootConfig, BootError, BootEvent, Bootable, ERASE_STEP, ImageError, Layout, Platform, Trial,
    TrustedKey, Update, image_end, launch,
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
/// 0x9000..0xA000  swap-move status      (swap tests only)
/// ```
const LAYOUT: Layout = Layout {
    primary: 0x0800,
    secondary: 0x0800 + SLOT,
    slot_size: SLOT,
    sector_size: SECTOR,
};
const LOADER: (u32, u32) = (0, 0x0800);
const NV: (u32, u32) = (0x8800, 0x9000);
const FLASH_LEN: usize = 0xA000;

type Flash = SimFlash<{ SECTOR as usize }, 4>;

static KEYS: [TrustedKey; 1] = [TrustedKey::new(KEY)];
const CONFIG: BootConfig<'static> = BootConfig::overwrite(LAYOUT, &KEYS);
const TRIAL: BootConfig<'static> = BootConfig {
    trial_boots: 3,
    ..CONFIG
};

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
        watchdog::kick();
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
    boot_with(flash, &CONFIG)
}

fn boot_with(flash: &mut Flash, config: &BootConfig<'_>) -> (Option<u32>, Recorder) {
    let mut platform = Recorder::default();
    let mut guarded = BootFlash::for_config(
        core::mem::replace(flash, Flash::new(SECTOR as usize)),
        config,
    );
    let entry = launch(&mut guarded, config, &mut platform);
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
    assert_eq!(platform.events, [BootEvent::Validating, BootEvent::Busy]);
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
    assert_eq!(launch(&mut flash, &CONFIG, &mut platform), None);
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
    replay_config(&CONFIG, setup, expect, true)
}

/// [`replay_with_cuts`] under `config`. `settles` asks the boot after the
/// recovery to be a plain boot (no trial boot accounting in between).
fn replay_config(
    config: &BootConfig<'_>,
    setup: impl Fn() -> Flash,
    expect: impl Fn(&mut Flash, &Recorder),
    settles: bool,
) -> usize {
    let boot_once = |flash: &mut Flash| boot_with(flash, config);
    let mut clean = setup();
    let before = clean.operations();
    let (entry, platform) = boot_once(&mut clean);
    assert!(entry.is_some());
    expect(&mut clean, &platform);
    let operations = clean.operations() - before;
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
            if settles {
                let (again, platform) = boot_once(&mut flash);
                assert_eq!(again, entry, "{cut:?} at {index}");
                assert_eq!(platform.outcome.unwrap().unwrap().update, Update::None);
            }
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

/// The hazard Phase 1 pinned, now fixed: a primary whose header magic
/// survives an interrupted erase while its version bytes read higher
/// (erase only sets bits) no longer blocks the update, because the
/// downgrade rule only applies against a primary that validates fully.
#[test]
fn torn_primary_header_never_blocks_a_valid_update() {
    let mut torn = V1.to_vec();
    torn[20] = 0xFF; // major
    torn[0x200..0x400].fill(0xFF); // partly erased payload
    let mut flash = device(&torn, Some(V2), true);
    let (entry, platform) = boot_once(&mut flash);
    assert_eq!(entry, Some(LAYOUT.primary + HEADER));
    assert_eq!(platform.outcome.unwrap().unwrap().update, Update::Installed);
    assert_eq!(primary_version(&flash).0, 2);
    assert_eq!(request_state(&mut flash), MagicState::Erased);
}

/// No brick from any cut in any mode while installing over a torn primary
/// whose header claims a higher version, with and without trial boots.
#[test]
fn torn_primary_install_survives_a_power_cut_at_every_operation() {
    let mut torn = V2.to_vec();
    torn[20] = 0xFF;
    torn[0x200..0x400].fill(0xFF);
    for config in [CONFIG, TRIAL] {
        replay_config(
            &config,
            || device(&torn, Some(V1), true),
            |flash, _| {
                assert_eq!(primary_version(flash).0, 1);
                assert_ne!(request_state(flash), MagicState::Good);
            },
            config.trial_boots == 0,
        );
    }
}

#[test]
fn trial_install_survives_a_power_cut_at_every_operation() {
    replay_config(
        &TRIAL,
        || device(V1, Some(V2), true),
        |flash, platform| {
            assert_eq!(primary_version(flash).0, 2);
            assert_ne!(request_state(flash), MagicState::Good);
            let bootable = platform.outcome.unwrap().unwrap();
            assert!(
                matches!(bootable.trial, Trial::Testing { boot } if boot <= 3),
                "{:?}",
                bootable.trial
            );
        },
        false,
    );
}

/// Trial boots end in `Unconfirmed` (overwrite cannot revert) and the
/// image keeps starting; a confirmed image is never on trial again.
#[test]
fn trial_boots_then_unconfirmed_or_confirmed() {
    use mcuboot_image::{Mode, TrialState, confirm};
    let mut flash = device(V1, Some(V2), true);
    for boot in 1..=3 {
        let (entry, platform) = boot_with(&mut flash, &TRIAL);
        assert!(entry.is_some());
        assert_eq!(
            platform.outcome.unwrap().unwrap().trial,
            Trial::Testing { boot }
        );
    }
    let (entry, platform) = boot_with(&mut flash, &TRIAL);
    assert!(entry.is_some());
    assert_eq!(platform.outcome.unwrap().unwrap().trial, Trial::Unconfirmed);
    assert_eq!(
        confirm(&mut flash, &LAYOUT, Mode::Overwrite),
        Ok(TrialState::Testing { attempts: 3 })
    );
    let operations = flash.operations();
    let (_, platform) = boot_with(&mut flash, &TRIAL);
    assert_eq!(platform.outcome.unwrap().unwrap().trial, Trial::Confirmed);
    assert_eq!(flash.operations(), operations);
    assert_protected_intact(&flash);
}

/// Flash work (bytes read, programmed or erased) between two watchdog
/// kicks, metered per test thread.
mod watchdog {
    use std::cell::Cell;

    use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

    thread_local! {
        static SINCE: Cell<(usize, usize)> = const { Cell::new((0, 0)) };
        static WORST: Cell<(usize, usize)> = const { Cell::new((0, 0)) };
    }

    pub fn kick() {
        SINCE.set((0, 0));
    }

    /// Worst `(read + programmed, erased)` bytes between two kicks since the
    /// last call.
    pub fn take_worst() -> (usize, usize) {
        kick();
        WORST.replace((0, 0))
    }

    fn add(io: usize, erased: usize) {
        let (a, b) = SINCE.get();
        let since = (a + io, b + erased);
        SINCE.set(since);
        let (w, e) = WORST.get();
        WORST.set((w.max(since.0), e.max(since.1)));
    }

    pub struct Metered<F>(pub F);

    impl<F: ErrorType> ErrorType for Metered<F> {
        type Error = F::Error;
    }

    impl<F: ReadNorFlash> ReadNorFlash for Metered<F> {
        const READ_SIZE: usize = F::READ_SIZE;
        fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), F::Error> {
            add(bytes.len(), 0);
            self.0.read(offset, bytes)
        }
        fn capacity(&self) -> usize {
            self.0.capacity()
        }
    }

    impl<F: NorFlash> NorFlash for Metered<F> {
        const WRITE_SIZE: usize = F::WRITE_SIZE;
        const ERASE_SIZE: usize = F::ERASE_SIZE;
        fn erase(&mut self, from: u32, to: u32) -> Result<(), F::Error> {
            add(0, (to - from) as usize);
            self.0.erase(from, to)
        }
        fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), F::Error> {
            add(bytes.len(), 0);
            self.0.write(offset, bytes)
        }
    }
}

/// Boot like [`boot_with`], returning the worst flash work between two
/// watchdog kicks.
fn boot_metered(flash: &mut Flash, config: &BootConfig<'_>) -> (Option<u32>, (usize, usize)) {
    let mut platform = Recorder::default();
    let mut guarded = BootFlash::for_config(
        watchdog::Metered(core::mem::replace(flash, Flash::new(SECTOR as usize))),
        config,
    );
    watchdog::take_worst();
    let entry = launch(&mut guarded, config, &mut platform);
    let worst = watchdog::take_worst();
    *flash = guarded.into_inner().0;
    (entry, worst)
}

/// The engine kicks before every full validation, so no stretch between two
/// kicks reads more than one image (plus small trailer and log reads), even
/// on paths that validate both slots or the primary twice (downgrade check,
/// install verify, trial accounting).
#[test]
fn watchdog_never_waits_longer_than_one_validation() {
    // One validation of the largest image reads it once; allow the small
    // magic, header and status-log reads around it (a second validation
    // without a kick would double the figure).
    let bound = V2.len() + 128;
    let cases: [(&str, Flash, &BootConfig<'_>); 5] = [
        ("plain", device(V1, None, false), &CONFIG),
        ("install", device(V1, Some(V2), true), &TRIAL),
        ("downgrade", device(V2, Some(V1), true), &CONFIG),
        ("rejected", device(V1, Some(TI_V2), true), &CONFIG),
        ("padded", device(V1, Some(V2_PAD), false), &CONFIG),
    ];
    for (name, mut flash, config) in cases {
        let (entry, (io, erased)) = boot_metered(&mut flash, config);
        assert!(entry.is_some(), "{name}");
        assert!(
            io <= bound,
            "{name}: {io} bytes between kicks (bound {bound})"
        );
        assert!(
            erased <= ERASE_STEP.max(SECTOR) as usize,
            "{name}: {erased} bytes erased between kicks"
        );
    }
}

#[cfg(feature = "swap-move")]
mod swap {
    use super::*;
    use crate::Mode;
    use mcuboot_image::{TrialState, confirm};

    const STATUS: u32 = 0x9000;
    const SWAP: BootConfig<'static> = BootConfig {
        layout: LAYOUT,
        keys: &KEYS,
        trial_boots: 3,
        mode: Mode::SwapMove { status: STATUS },
    };

    #[test]
    fn boot_flash_admits_the_status_sectors_only_in_swap_mode() {
        let mut flash = BootFlash::for_config(device(V1, None, false), &SWAP);
        assert_eq!(flash.erase(STATUS, STATUS + 2 * SECTOR), Ok(()));
        assert_eq!(flash.write(STATUS, &[0; 4]), Ok(()));
        assert_eq!(
            flash.erase(STATUS - SECTOR, STATUS),
            Err(BootFlashError::Protected)
        );
        let mut flash = BootFlash::for_config(flash.into_inner(), &CONFIG);
        assert_eq!(
            flash.erase(STATUS, STATUS + SECTOR),
            Err(BootFlashError::Protected)
        );
        assert_protected_intact(&flash.into_inner());
    }

    #[test]
    fn install_survives_a_power_cut_at_every_operation() {
        replay_config(
            &SWAP,
            || device(V1, Some(V2), true),
            |flash, platform| {
                assert_eq!(primary_version(flash).0, 2);
                assert_ne!(request_state(flash), MagicState::Good);
                let bootable = platform.outcome.unwrap().unwrap();
                assert!(matches!(bootable.trial, Trial::Testing { .. }));
                assert_eq!(flash.range(LAYOUT.secondary, V1.len()), V1);
            },
            false,
        );
    }

    #[test]
    fn revert_survives_a_power_cut_at_every_operation() {
        let unconfirmed = || {
            let mut flash = device(V1, Some(V2), true);
            for _ in 0..3 {
                boot_with(&mut flash, &SWAP);
            }
            flash
        };
        replay_config(
            &SWAP,
            unconfirmed,
            |flash, platform| {
                assert_eq!(primary_version(flash).0, 1);
                let bootable = platform.outcome.unwrap().unwrap();
                assert_eq!(bootable.trial, Trial::Permanent);
                assert_eq!(flash.range(LAYOUT.secondary, V2.len()), V2);
            },
            true,
        );
    }

    #[test]
    fn swap_watchdog_never_waits_longer_than_one_validation() {
        let mut flash = device(V1, Some(V2), true);
        let (entry, (io, erased)) = boot_metered(&mut flash, &SWAP);
        assert!(entry.is_some());
        assert!(io <= V2.len() + 128, "install: {io}");
        assert!(erased <= 2 * SECTOR as usize, "install: {erased}");
        // Expire the trial: the last boot validates both slots and reverts.
        for _ in 0..2 {
            boot_with(&mut flash, &SWAP);
        }
        let (entry, (io, erased)) = boot_metered(&mut flash, &SWAP);
        assert!(entry.is_some());
        assert_eq!(primary_version(&flash).0, 1);
        assert!(io <= V2.len() + 128, "revert: {io}");
        assert!(erased <= 2 * SECTOR as usize, "revert: {erased}");
    }

    #[test]
    fn confirmed_swap_install_is_kept() {
        let mut flash = device(V1, Some(V2), true);
        boot_with(&mut flash, &SWAP);
        assert_eq!(
            confirm(&mut flash, &LAYOUT, SWAP.mode),
            Ok(TrialState::Testing { attempts: 1 })
        );
        for _ in 0..5 {
            let (entry, platform) = boot_with(&mut flash, &SWAP);
            assert!(entry.is_some());
            assert_eq!(platform.outcome.unwrap().unwrap().trial, Trial::Confirmed);
        }
        assert_eq!(primary_version(&flash).0, 2);
        assert_protected_intact(&flash);
    }
}
