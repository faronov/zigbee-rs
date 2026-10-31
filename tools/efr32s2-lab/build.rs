//! Select the board memory map and the chip's interrupt table.

use std::{env, path::PathBuf};

fn main() {
    let wstk = env::var_os("CARGO_FEATURE_WSTK").is_some();
    let zbt1 = env::var_os("CARGO_FEATURE_ZBT1").is_some();
    let (board, chip) = match (wstk, zbt1) {
        (true, false) => ("wstk-brd4183a", "efr32mg22"),
        (false, true) => ("zbt1", "efr32mg21"),
        _ => panic!("enable exactly one of the `wstk` or `zbt1` features"),
    };
    let dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    println!(
        "cargo:rustc-link-search={}",
        dir.join("link").join(board).display()
    );
    println!(
        "cargo:rustc-link-search={}",
        dir.join("../../efr32s2-hal/link").join(chip).display()
    );
    println!("cargo:rerun-if-changed=link");
    println!("cargo:rerun-if-changed=build.rs");
}
