use std::{env, fs, path::PathBuf};

fn enabled(name: &str) -> bool {
    env::var_os(name).is_some()
}

fn main() {
    // (feature, direct link map, zigbee-rs loader layout)
    let selections = [
        (
            enabled("CARGO_FEATURE_BOARD_PROMICRO"),
            "uf2-promicro.x",
            "promicro-uf2.layout",
        ),
        (
            enabled("CARGO_FEATURE_BOARD_MDK"),
            "uf2-mdk.x",
            "mdk-uf2.layout",
        ),
        (
            enabled("CARGO_FEATURE_BOARD_NRF_DONGLE"),
            "uf2-pca10059.x",
            "pca10059-uf2.layout",
        ),
        (
            enabled("CARGO_FEATURE_BOARD_NRF_DK"),
            "uf2-dk.x",
            "dk.layout",
        ),
    ];
    let selected: Vec<_> = selections
        .into_iter()
        .filter_map(|(enabled, script, layout)| enabled.then_some((script, layout)))
        .collect();
    assert_eq!(
        selected.len(),
        1,
        "select exactly one nRF52840 UF2 board feature"
    );

    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let product_link = manifest.join("../../products/nrf52840-sensor/link");
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("memory.x");
    if enabled("CARGO_FEATURE_ZRS_BOOT") {
        // The primary slot of the loader layout; the security journal stays
        // where the direct map has it (checked below).
        let build = zrs_boot_layout::Build::load(
            format!("../../products/nrf52840-sensor/boot/{}", selected[0].1),
            zrs_boot_layout::ChipLimits {
                watchdog_max_ms: 0,
                vtor: true,
            },
        );
        let direct =
            fs::read_to_string(product_link.join(selected[0].0)).expect("read the direct link map");
        let journal = journal_origin(&direct);
        assert_eq!(
            (build.layout.journal.start, build.layout.journal.len()),
            (journal, 8 * 1024),
            "{}: the loader layout must keep the security journal of {}",
            selected[0].1,
            selected[0].0
        );
        let mut script = build.layout.app_memory_x();
        script.push_str(
            "ASSERT(_stack_start - __euninit >= 16K,\n  \
             \"nRF52840 static data must leave at least 16 KiB for the stack\");\n",
        );
        fs::write(&output, script).expect("write loader-slot linker map");
    } else {
        let script = product_link.join(selected[0].0);
        fs::copy(&script, &output).expect("copy selected product linker map");
    }

    println!(
        "cargo:rustc-link-search={}",
        output.parent().unwrap().display()
    );
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
    for script in ["uf2-promicro.x", "uf2-mdk.x", "uf2-pca10059.x", "uf2-dk.x"] {
        println!(
            "cargo:rerun-if-changed={}",
            product_link.join(script).display()
        );
    }
}

/// `ORIGIN` of `SECURITY_JOURNAL` in a direct link map.
fn journal_origin(script: &str) -> u32 {
    let line = script
        .lines()
        .find(|line| line.trim_start().starts_with("SECURITY_JOURNAL"))
        .expect("direct link map without SECURITY_JOURNAL");
    let origin = line
        .split("ORIGIN = 0x")
        .nth(1)
        .and_then(|rest| rest.split(',').next())
        .expect("SECURITY_JOURNAL without a hex ORIGIN");
    u32::from_str_radix(origin.trim(), 16).expect("SECURITY_JOURNAL ORIGIN is not hex")
}
