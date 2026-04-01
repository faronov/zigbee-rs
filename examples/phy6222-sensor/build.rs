//! build.rs for phy6222-sensor
//!
//! Provides the `device.x` interrupt vector definitions to cortex-m-rt
//! and sets up the linker script.

fn main() {
    // Copy device.x to OUT_DIR so cortex-m-rt can INCLUDE it
    let out_dir = std::env::var("OUT_DIR").unwrap();
    std::fs::copy("device.x", format!("{}/device.x", out_dir))
        .expect("failed to copy device.x");
    println!("cargo:rustc-link-search={}", out_dir);

    // Memory layout
    println!("cargo:rustc-link-arg=-Tlink.x");

    // Rebuild if device.x changes
    println!("cargo:rerun-if-changed=device.x");
    println!("cargo:rerun-if-changed=memory.x");
}
