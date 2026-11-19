//! Copy the NCP product memory map (`products/cc26x2-ncp/link/memory-<chip>.x`)
//! to `OUT_DIR/memory.x`.

use std::{env, fs, path::PathBuf};

fn main() {
    let chip = match env::var_os("CARGO_FEATURE_CC2652P7_DONGLE").is_some() {
        true => "cc2652p7",
        false => panic!("enable the board variant: cc2652p7-dongle"),
    };
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let source = manifest
        .join("../../products/cc26x2-ncp/link")
        .join(format!("memory-{chip}.x"));
    println!("cargo:rerun-if-changed={}", source.display());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy(&source, out.join("memory.x")).expect("copy the product memory map");
    println!("cargo:rustc-link-search={}", out.display());
}
