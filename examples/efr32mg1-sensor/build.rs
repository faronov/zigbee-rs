//! build.rs for efr32mg1-sensor
//!
//! Pure-Rust EFR32 build: provide device.x to cortex-m-rt and let the Rust
//! MAC/radio driver own the hardware path. No RAIL/GSDK build steps here.

fn main() {
    let out_dir = std::env::var("OUT_DIR").unwrap();

    // Copy device.x
    std::fs::copy("device.x", format!("{}/device.x", out_dir)).expect("failed to copy device.x");
    println!("cargo:rustc-link-search={}", out_dir);
    println!("cargo:rustc-link-arg=-Tlink.x");

    // Rebuild triggers
    println!("cargo:rerun-if-changed=device.x");
    println!("cargo:rerun-if-changed=memory.x");
}
