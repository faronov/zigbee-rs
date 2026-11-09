use super::*;

const DK: &str = "
name = nrf52840-dk
flash = 0x0..0x100000
sector = 0x1000
loader = 0x0..0x4000
primary = 0x4000..0x81000
secondary = 0x81000..0xFE000
journal = 0xFE000..0x100000   # persistence
ram = 0x20000000..0x20040000
header = 0x200
vector-align = 0x200
loader-version = 1
trial-boots = 3
env-prefix = NRF52840
key-file = keys/own.xy.hex
";

const CM4: ChipLimits = ChipLimits {
    watchdog_max_ms: 0,
    vtor: true,
};
/// Cortex-M0 without VTOR (PHY62x2).
const CM0: ChipLimits = ChipLimits {
    watchdog_max_ms: 0,
    vtor: false,
};

fn dk() -> BootLayout {
    BootLayout::parse(DK, Path::new("/product/boot")).unwrap()
}

fn problems(layout: &BootLayout, limits: &ChipLimits) -> String {
    layout.check(limits).unwrap_err().join("\n")
}

#[test]
fn parses_and_derives() {
    let layout = dk();
    assert_eq!(layout.check(&CM4), Ok(()));
    assert_eq!(layout.slot_size(), 0x7D000);
    assert_eq!(layout.max_image_len(), 0x7C000);
    assert_eq!(layout.app_start(), 0x4200);
    assert_eq!(layout.app_max_len(), 0x7C000 - 0x200 - 0x300);
    assert_eq!(layout.loader_block_addr(), 0x3FC0);
    assert_eq!(layout.loader_info_addr(), 0x3FE0);
    assert_eq!(
        layout.key_file,
        Some(PathBuf::from("/product/boot/keys/own.xy.hex"))
    );
    assert_eq!(layout.mode, Mode::Overwrite);
    assert_eq!(layout.trial_boots, 3);
}

#[test]
fn reports_unknown_keys_missing_values_and_bad_ranges() {
    let error = BootLayout::parse("name = x\nfoo = 1\n", Path::new(".")).unwrap_err();
    assert!(
        error.contains("line 2") && error.contains("unknown key `foo`"),
        "{error}"
    );
    let error = BootLayout::parse("name = x\n", Path::new(".")).unwrap_err();
    assert!(error.contains("missing `flash`"), "{error}");
    let error = BootLayout::parse("flash = 0x10..0x10\n", Path::new(".")).unwrap_err();
    assert!(error.contains("empty"), "{error}");
    let error = BootLayout::parse("flash = 0..1\nflash = 0..2\n", Path::new(".")).unwrap_err();
    assert!(error.contains("given twice"), "{error}");
}

#[test]
fn refuses_overlap_and_outside_flash() {
    let mut layout = dk();
    layout.journal = Region {
        start: 0xFD000,
        end: 0x101000,
    };
    let text = problems(&layout, &CM4);
    assert!(
        text.contains("journal") && text.contains("outside flash"),
        "{text}"
    );
    assert!(text.contains("overlaps"), "{text}");
}

#[test]
fn refuses_protected_data_between_slots() {
    let mut layout = dk();
    layout.secondary = Region {
        start: 0x82000,
        end: 0xFF000,
    };
    layout.journal = Region {
        start: 0x81000,
        end: 0x82000,
    };
    layout.flash.end = 0x100000;
    let text = problems(&layout, &CM4);
    assert!(
        text.contains("journal") && text.contains("between the slots"),
        "{text}"
    );

    let mut layout = dk();
    layout.secondary = Region {
        start: 0x83000,
        end: 0x100000,
    };
    layout.journal = Region {
        start: 0x81000,
        end: 0x83000,
    };
    layout.reserved.push(Reserved {
        name: "uf2".into(),
        region: Region {
            start: 0x81000,
            end: 0x82000,
        },
    });
    let text = problems(&layout, &CM4);
    assert!(
        text.contains("uf2") && text.contains("between the slots"),
        "{text}"
    );
}

#[test]
fn refuses_unequal_or_misaligned_slots() {
    let mut layout = dk();
    layout.secondary.end = 0xFD000;
    assert!(problems(&layout, &CM4).contains("slots differ"));
    let mut layout = dk();
    layout.primary.start = 0x4800;
    layout.loader.end = 0x4800;
    let text = problems(&layout, &CM4);
    assert!(
        text.contains("primary") && text.contains("not sector aligned"),
        "{text}"
    );
}

#[test]
fn vector_table_alignment_only_with_vtor() {
    let mut layout = dk();
    layout.header = 0x100;
    let text = problems(&layout, &CM4);
    assert!(text.contains("not 0x200-aligned for VTOR"), "{text}");
    let m0 = ChipLimits {
        watchdog_max_ms: 0,
        vtor: false,
    };
    assert_eq!(layout.check(&m0), Ok(()));
}

#[test]
fn xip_base_counts_for_vector_alignment() {
    let mut layout = dk();
    layout.xip_base = 0x100;
    assert!(problems(&layout, &CM4).contains("VTOR"));
}

#[test]
fn watchdog_bounded_by_chip() {
    let mut layout = dk();
    layout.watchdog_ms = 1000;
    assert!(problems(&layout, &CM4).contains("has no watchdog"));
    let limits = ChipLimits {
        watchdog_max_ms: 500,
        vtor: true,
    };
    assert!(problems(&layout, &limits).contains("exceeds"));
    layout.watchdog_ms = 500;
    assert_eq!(layout.check(&limits), Ok(()));
}

#[test]
fn swap_status_two_sectors_outside_slots() {
    let mut layout = dk();
    layout.secondary = Region {
        start: 0x81000,
        end: 0xFE000,
    };
    layout.mode = Mode::SwapMove {
        status: Region {
            start: 0xFE000,
            end: 0x100000,
        },
    };
    let text = problems(&layout, &CM4);
    assert!(
        text.contains("journal") && text.contains("overlaps swap status"),
        "{text}"
    );
    let mut layout = BootLayout::parse(
        &(DK.replace("primary = 0x4000..0x81000", "primary = 0x6000..0x82000")
            .replace(
                "secondary = 0x81000..0xFE000",
                "secondary = 0x82000..0xFE000",
            )
            + "mode = swap-move 0x4000..0x6000\n"),
        Path::new("."),
    )
    .unwrap();
    assert_eq!(layout.check(&CM4), Ok(()), "{:?}", layout.check(&CM4));
    layout.mode = Mode::SwapMove {
        status: Region {
            start: 0x4000,
            end: 0x5000,
        },
    };
    assert!(problems(&layout, &CM4).contains("exactly two sectors"));
}

#[test]
fn app_max_caps_application() {
    let mut layout = dk();
    layout.app_max = Some(130_816);
    assert_eq!(layout.app_max_len(), 130_816);
    layout.app_max = Some(0x10_0000);
    assert_eq!(layout.app_max_len(), 0x7C000 - 0x500);
}

#[test]
fn env_overrides_policy() {
    let mut layout = dk();
    let vars = layout
        .resolve(|var| match var {
            "NRF52840_BOOT_TRIAL_BOOTS" => Some("5".into()),
            "NRF52840_BOOT_WATCHDOG_MS" => Some("0x10".into()),
            _ => None,
        })
        .unwrap();
    assert_eq!(
        vars,
        ["NRF52840_BOOT_TRIAL_BOOTS", "NRF52840_BOOT_WATCHDOG_MS"]
    );
    assert_eq!((layout.trial_boots, layout.watchdog_ms), (5, 16));
    assert!(dk().resolve(|_| Some("300".into())).is_err());
}

const KEY_A: &str = "11111111111111111111111111111111111111111111111111111111111111112222222222222222222222222222222222222222222222222222222222222222";
const KEY_B: &str = "33333333333333333333333333333333333333333333333333333333333333334444444444444444444444444444444444444444444444444444444444444444";

#[test]
fn keys_from_file_then_env() {
    let layout = dk();
    let read = |path: &Path| {
        assert_eq!(path, Path::new("/product/boot/keys/own.xy.hex"));
        Ok(format!("{KEY_A}\n"))
    };
    let Keys { keys, vars, files } = layout.keys(|_| None, read).unwrap();
    assert_eq!(keys.len(), 1);
    assert_eq!(keys[0][0], 0x11);
    assert_eq!(keys[0][63], 0x22);
    assert_eq!(vars.len(), 3);
    assert_eq!(files, [PathBuf::from("/product/boot/keys/own.xy.hex")]);

    let env = |var: &str| match var {
        "NRF52840_OTA_KEY_XY" => Some(KEY_B.to_string()),
        "NRF52840_OTA_KEY3_XY" => Some("/k/a.hex".to_string()),
        _ => None,
    };
    let read = |path: &Path| {
        assert_eq!(path, Path::new("/k/a.hex"));
        Ok(KEY_A.to_string())
    };
    let Keys { keys, files, .. } = layout.keys(env, read).unwrap();
    assert_eq!((keys[0][0], keys[1][0]), (0x33, 0x11));
    assert_eq!(files, [PathBuf::from("/k/a.hex")]);
}

#[test]
fn keys_refuse_duplicates_garbage_and_absence() {
    let mut layout = dk();
    let env = |var: &str| (var != "NRF52840_OTA_KEY3_XY").then(|| KEY_A.to_string());
    let error = layout.keys(env, |_| unreachable!()).unwrap_err();
    assert!(error.contains("repeats"), "{error}");
    let error = layout.keys(|_| None, |_| Ok("abc".into())).unwrap_err();
    assert!(error.contains("64 hex-encoded bytes"), "{error}");
    layout.key_file = None;
    let error = layout.keys(|_| None, |_| unreachable!()).unwrap_err();
    assert!(error.contains("NRF52840_OTA_KEY_XY"), "{error}");
}

#[test]
fn emits_rust_and_linker_scripts() {
    let layout = dk();
    let rust = layout.rust(&[[7; 64]], "mcuboot_image");
    for needle in [
        "pub const PRIMARY_SLOT: u32 = 0x4000;",
        "pub const SECONDARY_SLOT: u32 = 0x81000;",
        "pub const SLOT_SIZE: u32 = 0x7D000;",
        "pub const APP_START: u32 = 0x4200;",
        "pub const LOADER_MAX_LEN: u32 = 0x3FC0;",
        "pub const BOOT_TRIAL_BOOTS: u8 = 3;",
        "pub const TRUSTED_KEYS: [mcuboot_image::TrustedKey; 1]",
        "mode: mcuboot_image::Mode::Overwrite",
    ] {
        assert!(rust.contains(needle), "missing {needle} in\n{rust}");
    }
    let loader = layout.loader_memory_x();
    assert!(
        loader.contains("FLASH       : ORIGIN = 0x00000000, LENGTH = 0x3FC0"),
        "{loader}"
    );
    assert!(
        loader.contains("LOADER_INFO : ORIGIN = 0x00003FC0, LENGTH = 64"),
        "{loader}"
    );
    assert!(
        loader.contains("RAM         : ORIGIN = 0x20000000, LENGTH = 0x40000"),
        "{loader}"
    );
    assert!(loader.contains("exceeds its 16320-byte region"), "{loader}");
    let app = layout.app_memory_x();
    assert!(
        app.contains("FLASH : ORIGIN = 0x00004200, LENGTH = 0x7BB00"),
        "{app}"
    );
    assert!(
        app.contains("_security_partition_start = 0x000FE000;"),
        "{app}"
    );
}

#[test]
fn swap_mode_emits_status() {
    let mut layout = dk();
    layout.mode = Mode::SwapMove {
        status: Region {
            start: 0x1000,
            end: 0x3000,
        },
    };
    assert!(
        layout
            .rust(&[[7; 64]], "m")
            .contains("m::Mode::SwapMove { status: 0x1000 }")
    );
}

/// Every committed product layout parses and passes its chip's checks.
#[test]
fn committed_layouts_are_valid() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let layouts = [
        ("products/nrf52840-sensor/boot/dk.layout", CM4),
        ("products/nrf52840-sensor/boot/mdk-uf2.layout", CM4),
        ("products/nrf52840-sensor/boot/pca10059-uf2.layout", CM4),
        ("products/nrf52840-sensor/boot/promicro-uf2.layout", CM4),
        ("products/efr32mg22-sensor/boot/devkit.layout", CM4),
        ("products/efr32mg21-sensor/boot/devkit.layout", CM4),
        ("products/ha-connect-zbt1/boot/zbt1.layout", CM4),
        ("products/pb03f-kit/boot/pb03f.layout", CM0),
    ];
    for (path, limits) in layouts {
        let path = root.join(path);
        let text = std::fs::read_to_string(&path).unwrap();
        let layout = BootLayout::parse(&text, path.parent().unwrap())
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        layout
            .check(&limits)
            .unwrap_or_else(|problems| panic!("{}: {problems:#?}", path.display()));
        let Keys { keys, .. } = layout
            .keys(
                |_| None,
                |path| std::fs::read_to_string(path).map_err(|e| e.to_string()),
            )
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        assert!(!keys.is_empty());
    }
}

#[test]
fn app_verify_parses_and_loads_keys() {
    let base = Path::new("/p/boot");
    let config = AppVerify::parse(
        "name = esp\nenv-prefix = ESP_X  # comment\nkey-file = ../k.hex\n",
        base,
    )
    .unwrap();
    assert_eq!(
        config.key_file.as_deref(),
        Some(Path::new("/p/boot/../k.hex"))
    );
    let read = |path: &Path| {
        assert_eq!(path, Path::new("/p/boot/../k.hex"));
        Ok(KEY_A.to_string())
    };
    let keys = config.keys(|_| None, read).unwrap();
    assert_eq!(keys.keys.len(), 1);
    assert_eq!(
        keys.vars,
        ["ESP_X_OTA_KEY_XY", "ESP_X_OTA_KEY2_XY", "ESP_X_OTA_KEY3_XY"]
    );
    let env = |var: &str| (var == "ESP_X_OTA_KEY2_XY").then(|| KEY_B.to_string());
    assert_eq!(config.keys(env, read).unwrap().keys.len(), 2);
    let rust = config.rust(&keys.keys, "mcuboot_image");
    assert!(rust.contains("pub const TRUSTED_KEYS: [mcuboot_image::TrustedKey; 1]"));

    for (text, error) in [
        ("env-prefix = A\n", "missing `name`"),
        ("name = a\n", "missing `env-prefix`"),
        ("name = a\nenv-prefix = a-b\n", "env-prefix"),
        (
            "name = a\nenv-prefix = A\nflash = 0..1\n",
            "unknown key `flash`",
        ),
        ("name = a\nname = b\n", "given twice"),
    ] {
        let problem = AppVerify::parse(text, base).unwrap_err();
        assert!(problem.contains(error), "{text:?}: {problem}");
    }
    let keyless = AppVerify::parse("name = a\nenv-prefix = A\n", base).unwrap();
    assert!(
        keyless
            .keys(|_| None, read)
            .unwrap_err()
            .contains("no trusted key")
    );
}

#[test]
fn committed_app_verify_configs_load() {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    for path in [
        "products/esp32-zigbee-devkit/boot/app-verify.conf",
        "products/tlsr8258-tb04/boot/app-verify.conf",
    ] {
        let path = root.join(path);
        let text = std::fs::read_to_string(&path).unwrap();
        let config = AppVerify::parse(&text, path.parent().unwrap()).unwrap();
        let read = |path: &Path| std::fs::read_to_string(path).map_err(|e| e.to_string());
        let keys = config.keys(|_| None, read).unwrap();
        assert_eq!(keys.keys.len(), 1, "{}", path.display());
    }
}
