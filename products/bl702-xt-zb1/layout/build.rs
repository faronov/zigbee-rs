//! Embed the bootloader trust anchors as raw P-256 `X || Y` constants and
//! the trial-boot budget.
//!
//! `../keys/own-ec-p256.xy.hex` is the public half of the default signing
//! key (the flower project's own key, whose private half lives outside every
//! repository in `~/.flower-keys/flower-own-ec-p256.pem`). Set
//! `BL702_OTA_KEY_XY` to another 64-byte hex string, or to the path of a
//! file holding one, to build for a different key; CI does this with an
//! ephemeral key and `tools/bl702_ota.py` does it for the key it signs with.
//! The private key is never read here and never committed.
//!
//! Key rotation: `BL702_OTA_KEY2_XY` and `BL702_OTA_KEY3_XY` (same forms)
//! add further trusted keys. The first key stays the one the loader's
//! `LoaderInfo` names; the others go into the `LoaderKeys` record before it.
//!
//! `BL702_BOOT_TRIAL_BOOTS` (0..=255, default 3) is how many boots an
//! installed image may take before the loader reports it unconfirmed;
//! `0` disables trial accounting.

use std::env;
use std::fs;
use std::path::PathBuf;

fn parse_xy(origin: &str, text: &str) -> [u8; 64] {
    let digits: String = text.chars().filter(|c| !c.is_whitespace()).collect();
    assert_eq!(
        digits.len(),
        128,
        "{origin} must hold 64 hex-encoded bytes (P-256 X || Y)"
    );
    let mut xy = [0u8; 64];
    for (index, byte) in xy.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&digits[2 * index..2 * index + 2], 16)
            .unwrap_or_else(|_| panic!("{origin} is not hex"));
    }
    xy
}

/// The key named by `var` (a hex string or a file path), if set.
fn env_key(var: &str) -> Option<[u8; 64]> {
    println!("cargo:rerun-if-env-changed={var}");
    let value = env::var(var).ok()?;
    if value.trim().len() == 128 && !value.contains(['/', '\\', '.']) {
        return Some(parse_xy(var, &value));
    }
    println!("cargo:rerun-if-changed={value}");
    let text =
        fs::read_to_string(&value).unwrap_or_else(|error| panic!("read {var}={value}: {error}"));
    Some(parse_xy(&value, &text))
}

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let default = manifest.join("../keys/own-ec-p256.xy.hex");
    println!("cargo:rerun-if-changed={}", default.display());

    let xy = env_key("BL702_OTA_KEY_XY").unwrap_or_else(|| {
        let text = fs::read_to_string(&default)
            .unwrap_or_else(|error| panic!("read {}: {error}", default.display()));
        parse_xy(&default.display().to_string(), &text)
    });
    let mut keys = vec![xy];
    for var in ["BL702_OTA_KEY2_XY", "BL702_OTA_KEY3_XY"] {
        if let Some(key) = env_key(var) {
            assert!(!keys.contains(&key), "{var} repeats a trusted key");
            keys.push(key);
        }
    }

    println!("cargo:rerun-if-env-changed=BL702_BOOT_TRIAL_BOOTS");
    let trial_boots: u8 = match env::var("BL702_BOOT_TRIAL_BOOTS") {
        Ok(value) => value
            .trim()
            .parse()
            .unwrap_or_else(|_| panic!("BL702_BOOT_TRIAL_BOOTS={value} is not 0..=255")),
        Err(_) => 3,
    };

    let mut text = format!(
        "/// Public half (`X || Y`) of the primary key the bootloader trusts.\n\
         pub const OWN_KEY_XY: [u8; 64] = {xy:?};\n\
         /// Every key the bootloader trusts, [`OWN_KEY_XY`] first.\n\
         pub const TRUSTED_KEYS: [mcuboot_image::TrustedKey; {}] = [\n",
        keys.len()
    );
    for key in &keys {
        text.push_str(&format!("    mcuboot_image::TrustedKey::new({key:?}),\n"));
    }
    text.push_str(&format!(
        "];\n\
         /// Boots an installed image may take before it counts as unconfirmed.\n\
         pub const BOOT_TRIAL_BOOTS: u8 = {trial_boots};\n"
    ));
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("keys.rs");
    fs::write(&out, text).expect("write keys.rs");
}
