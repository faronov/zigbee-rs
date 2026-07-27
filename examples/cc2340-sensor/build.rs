//! build.rs for cc2340-sensor
//!
//! Imports the linker script for the CC2340 firmware.
//!
//! `zigbee-mac/build.rs` consumes `CC2340_SDK_DIR` and translates TI's PBE,
//! MCE, and RFE firmware arrays into Rust build-time data. No TI host library
//! or Zigbee platform shim is linked.

fn main() {
    println!("cargo:rustc-link-arg=-Tlink.x");
}
