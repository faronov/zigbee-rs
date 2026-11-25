//! Copy the NCP product memory map of the selected flash size
//! (`products/efr32mg21-ncp/link/memory-{768k,512k}.x`) to
//! `OUT_DIR/memory.x`.
//!
//! `EFR32MG21_NCP_MEASURE=1` links the measurement-only map
//! (`memory-measure.x`: 256 KiB of "RAM", never flashable) so a static RAM
//! that exceeds the 64 KiB part can still be measured.

use std::{env, fs, path::PathBuf};

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rerun-if-env-changed=EFR32MG21_NCP_MEASURE");
    let measure = env::var_os("EFR32MG21_NCP_MEASURE").is_some_and(|value| value == "1");
    let map = if measure {
        println!("cargo:warning=EFR32MG21_NCP_MEASURE=1: measurement-only image, never flash it");
        "memory-measure.x"
    } else if env::var_os("CARGO_FEATURE_FLASH_512K").is_some() {
        "memory-512k.x"
    } else {
        "memory-768k.x"
    };
    let source = manifest.join("../../products/efr32mg21-ncp/link").join(map);
    println!("cargo:rerun-if-changed={}", source.display());
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy(&source, out.join("memory.x")).expect("copy the product memory map");
    println!("cargo:rustc-link-search={}", out.display());
}
