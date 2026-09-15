use std::{env, fs, path::PathBuf};

fn enabled(name: &str) -> bool {
    env::var_os(name).is_some()
}

fn main() {
    let selections = [
        (enabled("CARGO_FEATURE_BOARD_PROMICRO"), "uf2-promicro.x"),
        (enabled("CARGO_FEATURE_BOARD_MDK"), "uf2-mdk.x"),
        (enabled("CARGO_FEATURE_BOARD_NRF_DONGLE"), "uf2-pca10059.x"),
        (enabled("CARGO_FEATURE_BOARD_NRF_DK"), "uf2-dk.x"),
    ];
    let selected: Vec<_> = selections
        .into_iter()
        .filter_map(|(enabled, script)| enabled.then_some(script))
        .collect();
    assert_eq!(
        selected.len(),
        1,
        "select exactly one nRF52840 UF2 board feature"
    );

    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let product_link = manifest.join("../../products/nrf52840-sensor/link");
    let script = product_link.join(selected[0]);
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("memory.x");
    fs::copy(&script, &output).expect("copy selected product linker map");

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
