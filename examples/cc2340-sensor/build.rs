//! build.rs for cc2340-sensor
//!
//! Imports the linker script for the CC2340 firmware.
//!
//! `zigbee-mac/build.rs` consumes `CC2340_SDK_DIR` and translates TI's PBE,
//! MCE, and RFE firmware arrays into Rust build-time data. No TI host library
//! or Zigbee platform shim is linked.

use std::env;
use std::fs;
use std::path::PathBuf;

fn main() {
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let local_script = manifest_dir.join("memory.x");
    let product_script = manifest_dir.join("../../products/cc2340-sensor/link/memory.x");

    let local = fs::read(&local_script).expect("read example memory.x");
    let product = fs::read(&product_script).expect("read product memory.x");
    assert_eq!(
        local, product,
        "example and product CC2340 memory.x files diverged"
    );

    let output_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    fs::copy(&local_script, output_dir.join("memory.x")).expect("copy memory.x to OUT_DIR");

    println!("cargo:rustc-link-search={}", output_dir.display());
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed={}", local_script.display());
    println!("cargo:rerun-if-changed={}", product_script.display());
}
