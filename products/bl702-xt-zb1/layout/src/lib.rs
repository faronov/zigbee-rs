//! Flash map of the BL702 XT-ZB1 product, shared by the application, its
//! bootloader and the image tooling (`tools/bl702_ota.py`).
//!
//! Physical offsets into the 1 MiB on-module SPI flash (4 KiB sectors):
//!
//! ```text
//! 0x00000..0x01000  BL702 ROM boot header (BFNP) for the loader, img_start 0x2000
//! 0x01000..0x02000  unused (boot-header reserved area, left as built)
//! 0x02000..0x10000  bootloader, runs at XIP 0x23000000
//!                   0x0FFC0..0x0FFE0  LoaderKeys record (extra trusted
//!                                     key ids, erased with one key)
//!                   0x0FFE0..0x10000  LoaderInfo record (XIP 0x2300DFE0)
//! 0x10000..0x87000  primary slot: 0x100 MCUboot header, application at
//!                   0x10100 (XIP 0x2300E100), TLVs; the last sector holds
//!                   the trial log (0x86000..0x86F80, application confirm
//!                   writes) and the trailer
//! 0x87000..0xFE000  secondary slot: OTA download area
//! 0xFE000..0x100000 Zigbee security-state journal (two 4 KiB sectors)
//! ```
//!
//! The boot ROM maps the XIP window linearly from the boot header's
//! `img_start`: physical `0x2000` appears at `0x23000000`, so the primary
//! slot's application appears at `0x2300E100`. Only the loader is covered by
//! the ROM's boot-header hash; the application is authenticated by the
//! loader (SHA-256 + ECDSA P-256 over the `imgtool` TLVs).

#![no_std]

use mcuboot_image::Layout;

include!(concat!(env!("OUT_DIR"), "/keys.rs"));

pub const FLASH_SIZE: u32 = 0x10_0000;
/// Serial-flash erase sector.
pub const SECTOR_SIZE: u32 = 0x1000;

/// Base of the BL702 flash XIP window.
pub const XIP_BASE: u32 = 0x2300_0000;
/// Physical offset the boot header's `img_start` names; it appears at
/// [`XIP_BASE`].
pub const LOADER_START: u32 = 0x2000;
pub const LOADER_END: u32 = PRIMARY_SLOT;
/// Largest loader image the region holds before the loader records.
pub const LOADER_MAX_LEN: u32 = LOADER_BLOCK_ADDR - LOADER_START;
/// [`mcuboot_image::LoaderInfo`] (32 bytes) at the end of the loader region,
/// where every loader version keeps it.
pub const LOADER_INFO_ADDR: u32 = LOADER_END - 32;
/// The 64-byte [`mcuboot_image::loader_block`]: the `LoaderKeys` record
/// (erased with a single key) right before the `LoaderInfo` record.
pub const LOADER_BLOCK_ADDR: u32 = LOADER_INFO_ADDR - 32;
/// Version stamped into the loader's `LoaderInfo`: 1 was the single-key
/// loader without trial accounting, 2 adds trial boots and up to three
/// trusted keys.
pub const LOADER_VERSION: u32 = 2;

pub const PRIMARY_SLOT: u32 = 0x1_0000;
pub const SECONDARY_SLOT: u32 = 0x8_7000;
pub const SLOT_SIZE: u32 = 0x7_7000;

/// Zigbee security-state journal partition (product storage).
pub const NV_START: u32 = 0xF_E000;
pub const NV_SIZE: u32 = 0x2000;

/// `imgtool sign --header-size`.
pub const IMAGE_HEADER_SIZE: u32 = 0x100;
/// Physical offset of the application entry (`_start`).
pub const APP_START: u32 = PRIMARY_SLOT + IMAGE_HEADER_SIZE;
/// Room kept after the application for the TLV area (SHA-256, key hash and
/// ECDSA signature need about 152 bytes).
pub const TLV_RESERVE: u32 = 0x300;
/// Largest linked application (`FLASH` length in the product `memory.x`).
pub const APP_MAX_LEN: u32 = MCUBOOT.max_image_len() - IMAGE_HEADER_SIZE - TLV_RESERVE;

pub const MCUBOOT: Layout = Layout {
    primary: PRIMARY_SLOT,
    secondary: SECONDARY_SLOT,
    slot_size: SLOT_SIZE,
    sector_size: SECTOR_SIZE,
};

/// XIP address at which physical flash offset `physical` executes. Valid for
/// offsets at or above [`LOADER_START`].
pub const fn xip_address(physical: u32) -> u32 {
    XIP_BASE + (physical - LOADER_START)
}

/// Application entry in the XIP window (`ORIGIN` of the product `FLASH`).
pub const APP_XIP_START: u32 = xip_address(APP_START);
/// XIP address of the LoaderInfo record.
pub const LOADER_INFO_XIP: u32 = xip_address(LOADER_INFO_ADDR);
/// XIP address of the loader record block.
pub const LOADER_BLOCK_XIP: u32 = xip_address(LOADER_BLOCK_ADDR);

/// The loader's boot configuration: overwrite install, the build's trusted
/// keys and trial-boot budget.
pub const BOOT_CONFIG: mcuboot_image::BootConfig<'static> = mcuboot_image::BootConfig {
    trial_boots: BOOT_TRIAL_BOOTS,
    ..mcuboot_image::BootConfig::overwrite(MCUBOOT, &TRUSTED_KEYS)
};

const _: () = {
    assert!(LOADER_START.is_multiple_of(SECTOR_SIZE));
    assert!(LOADER_END == PRIMARY_SLOT);
    assert!(PRIMARY_SLOT + SLOT_SIZE == SECONDARY_SLOT);
    assert!(SECONDARY_SLOT + SLOT_SIZE == NV_START);
    assert!(NV_START + NV_SIZE == FLASH_SIZE);
    assert!(NV_SIZE == 2 * SECTOR_SIZE);
    assert!(PRIMARY_SLOT.is_multiple_of(SECTOR_SIZE));
    assert!(SECONDARY_SLOT.is_multiple_of(SECTOR_SIZE));
    assert!(SLOT_SIZE.is_multiple_of(SECTOR_SIZE));
    // The loader region is at most 56 KiB.
    assert!(LOADER_END - LOADER_START <= 56 * 1024);
    assert!(APP_XIP_START == 0x2300_E100);
    assert!(LOADER_INFO_XIP == 0x2300_DFE0);
    assert!(LOADER_BLOCK_XIP == 0x2300_DFC0);
    assert!(TRUSTED_KEYS.len() <= mcuboot_image::MAX_TRUSTED_KEYS);
    assert!(APP_MAX_LEN == 0x7_5C00);
};

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::format;
    use std::string::String;

    #[test]
    fn layout_matches_the_documented_map() {
        assert_eq!((PRIMARY_SLOT, SLOT_SIZE), (0x10000, 0x77000));
        assert_eq!(SECONDARY_SLOT, 0x87000);
        assert_eq!(SECONDARY_SLOT + SLOT_SIZE, 0xFE000);
        assert_eq!(MCUBOOT.max_image_len(), 0x76000);
        assert_eq!(APP_START + APP_MAX_LEN + TLV_RESERVE, 0x86000);
        assert_eq!(MCUBOOT.trailer_sector(SECONDARY_SLOT), 0xFD000);
        assert_eq!(xip_address(LOADER_START), XIP_BASE);
        assert_eq!(xip_address(PRIMARY_SLOT), 0x2300_E000);
    }

    #[test]
    fn default_key_is_a_valid_point() {
        assert_ne!(OWN_KEY_XY, [0; 64]);
        assert!(mcuboot_image::p256::VerifyingKey::from_xy(&OWN_KEY_XY).is_some());
        assert_eq!(TRUSTED_KEYS[0], mcuboot_image::TrustedKey::new(OWN_KEY_XY));
    }

    // The build script refuses anything else; this catches a script edit.
    const _: () = assert!(BOOT_WATCHDOG_MS <= 63_999);

    #[test]
    fn trial_log_stays_clear_of_the_image_and_the_magic() {
        let (start, end) = mcuboot_image::trial_log_area(&MCUBOOT);
        assert_eq!((start, end), (0x86000, 0x86F80));
        assert!(start >= PRIMARY_SLOT + MCUBOOT.max_image_len());
        assert!(end <= PRIMARY_SLOT + SLOT_SIZE);
    }

    fn squash(script: &str) -> String {
        script.chars().filter(|c| !c.is_whitespace()).collect()
    }

    fn expect(script: &str, needle: String) {
        assert!(script.contains(&needle), "missing {needle}");
    }

    /// The linker scripts restate the map; keep them in sync.
    #[test]
    fn linker_scripts_match() {
        let app = squash(include_str!("../../link/memory.x"));
        expect(
            &app,
            format!("FLASH:ORIGIN=0x{APP_XIP_START:08X},LENGTH=0x{APP_MAX_LEN:X}"),
        );
        let loader = squash(include_str!("../../bootloader/memory.x"));
        expect(
            &loader,
            format!(
                "FLASH:ORIGIN=0x{XIP_BASE:08X},LENGTH=0x{:X}",
                LOADER_MAX_LEN
            ),
        );
        expect(
            &loader,
            format!("LOADER_INFO:ORIGIN=0x{LOADER_BLOCK_XIP:08X},LENGTH=64"),
        );
    }

    /// The release tool restates the map; keep it in sync.
    #[test]
    fn release_tool_matches() {
        let tool = include_str!("../../../../tools/bl702_ota.py");
        for (name, value) in [
            ("LOADER_START", LOADER_START),
            ("LOADER_INFO_ADDR", LOADER_INFO_ADDR),
            ("LOADER_BLOCK_ADDR", LOADER_BLOCK_ADDR),
            ("PRIMARY_SLOT", PRIMARY_SLOT),
            ("SECONDARY_SLOT", SECONDARY_SLOT),
            ("SLOT_SIZE", SLOT_SIZE),
            ("SECTOR_SIZE", SECTOR_SIZE),
            ("NV_START", NV_START),
            ("IMAGE_HEADER_SIZE", IMAGE_HEADER_SIZE),
            ("XIP_BASE", XIP_BASE),
        ] {
            expect(tool, format!("\n{name} = 0x{value:X}\n"));
        }
    }
}
