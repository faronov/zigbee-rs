//! Generates the firmware version constants (`OUT_DIR/version.rs`).
//!
//! The release version is the package version `MM.mm.pp`; its Zigbee OTA
//! file version is `0x25MMmmpp` (as on the flower CC2340 product) and its
//! MCUboot image version is `MM.mm.pp+0` (set by `tools/bl702_ota.py`).
//!
//! TEST ONLY: `BL702_OTA_VERSION_OVERRIDE=MM.mm.pp` builds an otherwise
//! identical image with another version, e.g. a "v2" to exercise an OTA
//! upgrade on the bench without editing `Cargo.toml`. Never publish such a
//! build: the next real release could carry the same version.

use std::{env, fs, path::PathBuf};

fn component(text: &str, what: &str) -> u8 {
    text.parse()
        .unwrap_or_else(|_| panic!("{what}: version components must be 0..=255, got {text:?}"))
}

fn main() {
    println!("cargo:rerun-if-env-changed=BL702_OTA_VERSION_OVERRIDE");
    let package = env::var("CARGO_PKG_VERSION").expect("CARGO_PKG_VERSION is set by Cargo");
    let (version, overridden) = match env::var("BL702_OTA_VERSION_OVERRIDE") {
        Ok(value) if !value.is_empty() => (value, true),
        _ => (package, false),
    };
    let parts: Vec<&str> = version.split('.').collect();
    assert!(
        parts.len() == 3,
        "firmware version must be MAJOR.MINOR.PATCH, got {version:?}"
    );
    let source = if overridden {
        "BL702_OTA_VERSION_OVERRIDE"
    } else {
        "CARGO_PKG_VERSION"
    };
    let [major, minor, patch] = [0, 1, 2].map(|index| component(parts[index], source));
    let generated = format!(
        "/// Release version `MAJOR.MINOR.PATCH` (Basic SW build ID).\n\
         pub const SW_BUILD: &str = \"{major}.{minor}.{patch}\";\n\
         pub const VERSION: (u8, u8, u8) = ({major}, {minor}, {patch});\n\
         /// `true` for a TEST-ONLY `BL702_OTA_VERSION_OVERRIDE` build.\n\
         pub const VERSION_OVERRIDDEN: bool = {overridden};\n"
    );
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by Cargo"));
    fs::write(out.join("version.rs"), generated).expect("write generated version");
}
