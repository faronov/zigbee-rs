//! Build script for TLSR8258 Zigbee sensor — pure Rust, no vendor SDK.
//!
//! Uses rust-lld for linking with a custom linker script (memory.x).
//! No Telink SDK, no C compiler, no external toolchain required.

fn main() {
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());

    // Copy memory.x for the linker
    if std::path::Path::new("memory.x").exists() {
        let _ = std::fs::copy("memory.x", out_dir.join("memory.x"));
        println!("cargo:rustc-link-search={}", out_dir.display());
        println!("cargo:rerun-if-changed=memory.x");
    }

    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=--noinhibit-exec");
    println!("cargo:rustc-link-arg=--gc-sections");
}
