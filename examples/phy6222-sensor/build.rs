//! build.rs for phy6222-sensor
//!
//! Installs the PHY62x2 ROM-aware linker scripts.

fn main() {
    let out_dir = std::env::var("OUT_DIR").unwrap();
    for script in ["device.x", "memory.x", "phy6222.x"] {
        std::fs::copy(script, format!("{out_dir}/{script}"))
            .unwrap_or_else(|_| panic!("failed to copy {script}"));
        println!("cargo:rerun-if-changed={script}");
    }
    println!("cargo:rustc-link-search={}", out_dir);
    println!("cargo:rustc-link-arg=-Tphy6222.x");
}
