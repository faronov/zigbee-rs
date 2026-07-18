//! Build script for TLSR8258 Zigbee sensor — pure Rust, no vendor SDK.
//!
//! Uses rust-lld for linking with a custom linker script (memory.x).
//! No Telink SDK and no C compilation; tc32 builds still require the modern-tc32 toolchain.

fn main() {
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let production_runtime = (std::env::var_os("CARGO_FEATURE_RUNTIME_SENSOR").is_some()
        || std::env::var_os("CARGO_FEATURE_RUNTIME_ROUTER").is_some())
        && std::env::var_os("CARGO_FEATURE_LAB").is_none();
    let linker_script = if production_runtime {
        "memory-runtime.x"
    } else {
        "memory.x"
    };

    if std::path::Path::new(linker_script).exists() {
        let _ = std::fs::copy(linker_script, out_dir.join("memory.x"));
        println!("cargo:rustc-link-search={}", out_dir.display());
    }
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-runtime.x");

    println!(
        "cargo:rustc-link-arg=-T{}",
        out_dir.join("memory.x").display()
    );
    println!("cargo:rustc-link-arg=--gc-sections");
}
