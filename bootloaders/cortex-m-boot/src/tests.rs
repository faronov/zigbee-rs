//! Entry checks through the flash driver, and the whole decision on the
//! simulated flash with the imgtool vectors of `mcuboot-image`
//! (0x4000-byte slots, 0x100-byte header).

use zrs_boot_core::embedded_storage::nor_flash::NorFlash;
use zrs_boot_core::sim::SimFlash;

use super::*;

const KEY: [u8; 64] = *include_bytes!("../../../mcuboot-image/tests/vectors/test-key.xy");
const V1: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-v1.bin");
/// 0x200-byte header, vector table `[0x2001_0000, 0xA41]` for primary 0x800.
const CM_V1: &[u8] = include_bytes!("../../../mcuboot-image/tests/vectors/test-cm-v1.bin");
/// `img_size` of `CM_V1` (vector table and pattern after the 0x200 header).
const CM_V1_IMG_SIZE: u32 = 0x200;

const SECTOR: u32 = 0x800;
const SLOT: u32 = 0x4000;
const LAYOUT: Layout = Layout {
    primary: 0x0800,
    secondary: 0x0800 + SLOT,
    slot_size: SLOT,
    sector_size: SECTOR,
};
static KEYS: [TrustedKey; 1] = [TrustedKey::new(KEY)];
const RAM: (u32, u32) = (0x2000_0000, 0x2001_0000);

type Flash = SimFlash<{ SECTOR as usize }, 4>;

fn config(xip_base: u32, header_size: u32) -> Config {
    Config {
        boot: BootConfig::overwrite(LAYOUT, &KEYS),
        header_size,
        ram: RAM,
        xip_base,
        vtor: xip_base == 0,
    }
}

fn vectors(flash: &mut Flash, physical: u32, sp: u32, reset: u32) {
    let mut words = [0u8; 8];
    words[..4].copy_from_slice(&sp.to_le_bytes());
    words[4..].copy_from_slice(&reset.to_le_bytes());
    flash.write(physical, &words).unwrap();
}

#[derive(Default)]
struct Recorder {
    events: usize,
    kicks: usize,
    decided: Option<bool>,
    refused: Option<EntryError>,
}

impl Hooks for Recorder {
    fn progress(&mut self, _event: BootEvent) {
        self.events += 1;
    }
    fn kick_watchdog(&mut self) {
        self.kicks += 1;
    }
    fn decided(&mut self, outcome: &Result<Bootable, BootError>) {
        self.decided = Some(outcome.is_ok());
    }
    fn refused(&mut self, error: EntryError) {
        self.refused = Some(error);
    }
    fn fail(&mut self) -> ! {
        panic!("fail is the caller's")
    }
}

#[test]
fn entry_reads_vectors_through_flash_and_maps_xip() {
    let xip = 0x1100_0000;
    let config = config(xip, 0x100);
    let mut flash = Flash::new(0x10000);
    let table = LAYOUT.primary + 0x100;
    vectors(&mut flash, table, RAM.1, xip + table + 0x41);
    assert_eq!(
        entry(&mut flash, &config, 0x100, 0x200),
        Ok(Entry {
            vector_table: xip + table,
            initial_sp: RAM.1,
            reset: xip + table + 0x41,
            image_end: xip + table + 0x200,
        })
    );
    // An application linked for the unmapped (physical) address is refused.
    let mut flash = Flash::new(0x10000);
    vectors(&mut flash, table, RAM.1, table + 0x41);
    assert_eq!(
        entry(&mut flash, &config, 0x100, 0x200),
        Err(EntryError::Vector(VectorError::ResetVector))
    );
}

#[test]
fn entry_refuses_other_header_size_and_bad_vectors() {
    let config = config(0, 0x200);
    let mut flash = Flash::new(0x10000);
    let table = LAYOUT.primary + 0x200;
    vectors(&mut flash, table, RAM.1, table + 0x101);
    assert!(entry(&mut flash, &config, 0x200, 0x400).is_ok());
    assert_eq!(
        entry(&mut flash, &config, 0x100, 0x400),
        Err(EntryError::HeaderSize)
    );
    let mut flash = Flash::new(0x10000);
    vectors(&mut flash, table, RAM.1 + 8, table + 0x101);
    assert_eq!(
        entry(&mut flash, &config, 0x200, 0x400),
        Err(EntryError::Vector(VectorError::StackPointer))
    );
    let mut flash = Flash::new(0x10000);
    vectors(&mut flash, table, RAM.1, table + 0x400 + 1);
    assert_eq!(
        entry(&mut flash, &config, 0x200, 0x400),
        Err(EntryError::Vector(VectorError::ResetVector))
    );
    // Even (ARM-state) reset handler.
    let mut flash = Flash::new(0x10000);
    vectors(&mut flash, table, RAM.1, table + 0x100);
    assert!(entry(&mut flash, &config, 0x200, 0x400).is_err());
}

#[test]
fn decide_starts_a_signed_cortex_m_image() {
    let mut flash = Flash::new(0xA000);
    flash.load(LAYOUT.primary, CM_V1);
    let mut hooks = Recorder::default();
    let config = config(0, 0x200);
    let mut guarded = BootFlash::for_config(&mut flash, &config.boot);
    assert_eq!(
        decide(&mut guarded, &config, &mut hooks),
        Some(Entry {
            vector_table: 0xA00,
            initial_sp: 0x2001_0000,
            reset: 0xA41,
            image_end: 0xA00 + CM_V1_IMG_SIZE,
        })
    );
    assert_eq!((hooks.decided, hooks.refused), (Some(true), None));
}

#[test]
fn decide_installs_then_starts() {
    let mut flash = Flash::new(0xA000);
    flash.load(LAYOUT.secondary, CM_V1);
    flash.load(
        zrs_boot_core::mcuboot_image::trailer::magic_address(LAYOUT.secondary, SLOT),
        &zrs_boot_core::mcuboot_image::trailer::BOOT_MAGIC,
    );
    let mut hooks = Recorder::default();
    let config = config(0, 0x200);
    let mut guarded = BootFlash::for_config(&mut flash, &config.boot);
    assert!(decide(&mut guarded, &config, &mut hooks).is_some());
    assert_eq!(flash.range(LAYOUT.primary, CM_V1.len()), CM_V1);
}

#[test]
fn decide_refuses_image_linked_for_another_slot() {
    // Same image with the slots exchanged: signed and valid, but its reset
    // handler lies outside the image as installed.
    let layout = Layout {
        primary: 0x0800 + SLOT,
        secondary: 0x0800,
        ..LAYOUT
    };
    let config = Config {
        boot: BootConfig::overwrite(layout, &KEYS),
        ..config(0, 0x200)
    };
    let mut flash = Flash::new(0xA000);
    flash.load(layout.primary, CM_V1);
    let mut hooks = Recorder::default();
    let mut guarded = BootFlash::for_config(&mut flash, &config.boot);
    assert_eq!(decide(&mut guarded, &config, &mut hooks), None);
    assert_eq!(
        hooks.refused,
        Some(EntryError::Vector(VectorError::ResetVector))
    );
}

#[test]
fn decide_validates_then_refuses_wrong_header_size() {
    // V1 is a valid, signed image with a 0x100-byte header; a layout linked
    // for 0x200 must not start it.
    let mut flash = Flash::new(0xA000);
    flash.load(LAYOUT.primary, V1);
    let mut hooks = Recorder::default();
    let mut guarded = BootFlash::for_config(&mut flash, &config(0, 0x200).boot);
    assert_eq!(decide(&mut guarded, &config(0, 0x200), &mut hooks), None);
    assert_eq!(hooks.decided, Some(true));
    assert_eq!(hooks.refused, Some(EntryError::HeaderSize));
    assert!(hooks.kicks >= hooks.events && hooks.events > 0);
}

#[test]
fn decide_refuses_payload_that_is_not_a_vector_table() {
    // The vector payload is a byte pattern, not a vector table.
    let mut flash = Flash::new(0xA000);
    flash.load(LAYOUT.primary, V1);
    let mut hooks = Recorder::default();
    let config = config(0, 0x100);
    let mut guarded = BootFlash::for_config(&mut flash, &config.boot);
    assert_eq!(decide(&mut guarded, &config, &mut hooks), None);
    assert!(matches!(hooks.refused, Some(EntryError::Vector(_))));
}

#[test]
fn decide_without_image_reports_and_returns_none() {
    let mut flash = Flash::new(0xA000);
    let mut hooks = Recorder::default();
    let config = config(0, 0x100);
    let mut guarded = BootFlash::for_config(&mut flash, &config.boot);
    assert_eq!(decide(&mut guarded, &config, &mut hooks), None);
    assert_eq!(hooks.decided, Some(false));
    assert_eq!(hooks.refused, None);
}
