//! Build-time firmware version.
//!
//! `ESP32_OTA_VERSION` (decimal or `0x`-prefixed) becomes the Zigbee OTA file
//! version this build advertises; it defaults to `1`. The packaging tool
//! (`tools/create-ota.py`) sets it to the version it stamps into the OTA
//! container, so a device can never report a version its image does not have.

use std::{env, fs, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-env-changed=ESP32_OTA_VERSION");

    let raw = env::var("ESP32_OTA_VERSION").unwrap_or_else(|_| "1".to_owned());
    let version = if let Some(hex) = raw.strip_prefix("0x").or_else(|| raw.strip_prefix("0X")) {
        u32::from_str_radix(hex, 16)
    } else {
        raw.parse()
    }
    .expect("ESP32_OTA_VERSION must be a decimal or 0x-prefixed u32");
    assert!(version != u32::MAX, "0xFFFFFFFF is reserved by Zigbee OTA");

    let generated = format!(
        "pub const FIRMWARE_VERSION: u32 = {version};\n\
         pub const FIRMWARE_VERSION_STR: &str = \"{version}\";\n"
    );
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by Cargo"));
    fs::write(out.join("firmware_version.rs"), generated)
        .expect("write generated firmware version");
}
