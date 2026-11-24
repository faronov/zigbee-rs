//! Copy the NCP product memory map (`products/efr32mg24/link/memory.x`)
//! to `OUT_DIR/memory.x`.

use std::{env, fs, path::PathBuf};

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let source = manifest.join("../../products/efr32mg24/link/memory.x");
    println!("cargo:rerun-if-changed={}", source.display());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy(&source, out.join("memory.x")).expect("copy the product memory map");
    println!("cargo:rustc-link-search={}", out.display());
}
