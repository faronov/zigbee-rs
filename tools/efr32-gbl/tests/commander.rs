//! Byte-for-byte comparison with Simplicity Commander 1v22p1b1957.
//!
//! The fixtures were made from a synthetic 512-byte application (vector
//! table, `ApplicationProperties_t` at 0x0800_6100 referenced by vector
//! word 13, a branch-to-self) with
//! `commander convert app13.bin --address 0x08006000` and
//! `commander gbl create golden-plain.gbl --app app13.s37`
//! (`--metadata meta.json` for golden-meta.gbl; `odd.bin` is the same
//! image plus 3 bytes for golden-odd.gbl).

use efr32_gbl::{
    GeckoTarget, find_app_properties, gecko_boot_check, parse, prepare_app_image, program_image,
    write, write_erase,
};

const BASE: u32 = 0x0800_6000;

fn fixture(name: &str) -> Vec<u8> {
    std::fs::read(format!(
        "{}/tests/fixtures/{name}",
        env!("CARGO_MANIFEST_DIR")
    ))
    .unwrap()
}

#[test]
fn plain_gbl_matches_commander() {
    let image = fixture("app13.bin");
    let props = find_app_properties(&image, BASE).unwrap();
    assert_eq!(props.address, 0x0800_6100);
    assert_eq!(
        write(&props.app, BASE, &image, None),
        fixture("golden-plain.gbl")
    );
}

#[test]
fn metadata_gbl_matches_commander() {
    let image = fixture("app13.bin");
    let props = find_app_properties(&image, BASE).unwrap();
    let metadata = fixture("meta.json");
    assert_eq!(
        write(&props.app, BASE, &image, Some(&metadata)),
        fixture("golden-meta.gbl")
    );
}

#[test]
fn unaligned_image_is_not_padded_like_commander() {
    let image = fixture("odd.bin");
    let props = find_app_properties(&image, BASE).unwrap();
    assert_eq!(
        write(&props.app, BASE, &image, None),
        fixture("golden-odd.gbl")
    );
}

#[test]
fn parse_round_trip_and_crc_check() {
    let gbl = fixture("golden-meta.gbl");
    let parsed = parse(&gbl).unwrap();
    assert_eq!(parsed.gbl_type, 0);
    assert!(!parsed.signed && !parsed.encrypted);
    assert_eq!(parsed.programs, vec![(BASE, 512)]);
    assert_eq!(parsed.metadata.as_deref(), Some(&fixture("meta.json")[..]));
    assert_eq!(parsed.application.unwrap().version, 0x0001_0000);
    let mut broken = gbl.clone();
    broken[40] ^= 1;
    assert!(matches!(
        parse(&broken),
        Err(efr32_gbl::Error::BadCrc { .. })
    ));
    // Trailing padding is not part of the record chain; a cut CRC is.
    assert!(parse(&gbl[..gbl.len() - 3]).is_ok());
    assert!(parse(&gbl[..gbl.len() - 5]).is_err());
}

#[test]
fn word13_is_patched_and_conflicts_are_refused() {
    let mut image = fixture("app13.bin");
    image[52..56].copy_from_slice(&[0; 4]);
    let props = prepare_app_image(&mut image, BASE, 0x2000_0000..0x2004_0000).unwrap();
    assert_eq!(image, fixture("app13.bin"));
    assert_eq!(props.signature_type, 0);
    image[52..56].copy_from_slice(&0x0800_7000u32.to_le_bytes());
    assert!(prepare_app_image(&mut image, BASE, 0x2000_0000..0x2004_0000).is_err());
    let mut bad_sp = fixture("app13.bin");
    bad_sp[0..4].copy_from_slice(&0x1000_0000u32.to_le_bytes());
    assert!(prepare_app_image(&mut bad_sp, BASE, 0x2000_0000..0x2004_0000).is_err());
}

#[test]
fn gecko_start_up_checks() {
    let target = GeckoTarget::MG24_F1536;
    let image = fixture("app13.bin");
    let boot = gecko_boot_check(&image, BASE, target).unwrap();
    assert_eq!(
        (boot.initial_sp, boot.reset_vector, boot.properties.address),
        (0x2004_0000, 0x0800_6181, 0x0800_6100)
    );
    // The program data of the GBL is the same image.
    let parsed = parse(&fixture("golden-plain.gbl")).unwrap();
    assert_eq!(program_image(&parsed), Some((BASE, image.clone())));

    let patched = |at: usize, word: u32| {
        let mut bad = image.clone();
        bad[at..at + 4].copy_from_slice(&word.to_le_bytes());
        gecko_boot_check(&bad, BASE, target)
    };
    // SP past the end of SRAM, reset vector in the bootloader or ARM mode,
    // word 13 not set (or inside the first 64 bytes), signed image.
    assert!(patched(0, 0x2004_0004).is_err());
    assert!(patched(4, 0x0800_0101).is_err());
    assert!(patched(4, 0x0800_6180).is_err());
    assert!(patched(52, 0).is_err());
    assert!(patched(52, 0x0800_6020).is_err());
    assert!(patched(0x100 + 20, 1).is_err());
    // Struct major 2 would be refused by every GSDK 3.x/4.x bootloader.
    assert!(patched(0x100 + 16, 0x0000_0202).is_err());
    // Linked elsewhere than the application space.
    assert!(gecko_boot_check(&image, 0x0800_4000, target).is_err());
}

#[test]
fn nv_clear_gbl_erases_only_its_window() {
    let gbl = write_erase(0x0815_8000, 0x0818_0000, 8192).unwrap();
    let parsed = parse(&gbl).unwrap();
    assert_eq!(parsed.gbl_type, 0);
    assert!(parsed.application.is_none() && parsed.metadata.is_none());
    assert!(!parsed.signed && !parsed.encrypted && !parsed.updates_bootloader);
    assert_eq!(parsed.programs, vec![(0x0815_8000, 0x2_8000)]);
    let (base, image) = program_image(&parsed).unwrap();
    assert_eq!(base, 0x0815_8000);
    assert!(image.iter().all(|b| *b == 0xFF));
    // Unaligned or empty ranges are refused.
    assert!(write_erase(0x0815_8000, 0x0815_8001, 8192).is_err());
    assert!(write_erase(0x0815_9000, 0x0818_0000, 8192).is_err());
    assert!(write_erase(0x0818_0000, 0x0818_0000, 8192).is_err());
}
