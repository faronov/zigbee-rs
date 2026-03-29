//! build.rs for phy6222-sensor
//!
//! No vendor libraries needed — PHY6222 uses a pure-Rust radio driver.

fn main() {
    // Memory layout
    println!("cargo:rustc-link-arg=-Tlink.x");
}
