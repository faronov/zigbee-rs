//! The stock SONOFF Dongle-M firmware GBL against this crate's parser,
//! writer and Gecko start-up check.
//!
//! The stock image is not redistributed here. Point
//! `EFR32_GBL_STOCK_DONGLE_M` at a copy of
//! `donglem_mg24_zigbee_stable_1.0.0_115200_7.4.5.gbl` (= SONOFF's
//! `zigbee_ncp_uart_hw_donglem_v7.4.5.gbl`; iHost-Open-Source-Project
//! `hassio-ihost-sonoff-dongle-flasher` at commit `136ab2fe`,
//! `firmware-build/`); CI downloads it. Without the variable the test
//! returns early. A file with another SHA-256 fails.

use efr32_gbl::{GeckoTarget, gecko_boot_check, parse, program_image, write};
use sha2::{Digest, Sha256};

const STOCK_SHA256: &str = "44b81c1ecf81032b98d625388ae44c10d7c8a6abd1dcea10b2e2d934d7a311fa";

#[test]
fn stock_dongle_m_gbl_round_trips_and_passes_the_start_up_check() {
    let Some(path) = std::env::var_os("EFR32_GBL_STOCK_DONGLE_M") else {
        eprintln!("EFR32_GBL_STOCK_DONGLE_M not set: stock GBL test skipped");
        return;
    };
    let stock = std::fs::read(&path).unwrap();
    let digest: String = Sha256::digest(&stock)
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect();
    assert_eq!(digest, STOCK_SHA256, "not the pinned stock GBL");

    let parsed = parse(&stock).unwrap();
    // GBL v3, plain: header, application, one erase-and-program, end.
    assert_eq!(&stock[8..12], &0x0300_0000u32.to_le_bytes());
    assert_eq!(parsed.gbl_type, 0);
    assert!(!parsed.signed && !parsed.encrypted && !parsed.updates_bootloader);
    assert_eq!(parsed.records.len(), 4);
    assert_eq!(parsed.programs, vec![(0x0800_6000, 0x3_D754)]);
    let app = parsed.application.unwrap();
    assert_eq!((app.app_type, app.version), (1, 1));

    // This crate's writer reproduces the file byte for byte from its
    // application record and program data: the structure our NCP GBLs use
    // (plus the metadata record) is the one SONOFF's bootloader accepts.
    let (base, image) = program_image(&parsed).unwrap();
    assert_eq!(write(&app, base, &image, None), stock);

    // The bootloader's start-up checks pass on the stock application.
    let boot = gecko_boot_check(&image, base, GeckoTarget::MG24_F1536).unwrap();
    assert_eq!(boot.properties.address, 0x0803_C148);
    assert_eq!(boot.properties.struct_version, 0x0201);
    assert_eq!(boot.properties.app, app);
}
