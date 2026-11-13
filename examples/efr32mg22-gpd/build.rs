//! Copies the product's linker memory map
//! (`products/efr32mg22-gpd/link/memory.x`) into `OUT_DIR`.

use std::{env, fs, path::PathBuf};

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let map = manifest.join("../../products/efr32mg22-gpd/link/memory.x");
    println!("cargo:rerun-if-changed={}", map.display());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy(&map, out.join("memory.x")).expect("copy the product link map");
    println!("cargo:rustc-link-search={}", out.display());
}
