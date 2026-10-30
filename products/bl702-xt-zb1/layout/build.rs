//! Embed the bootloader trust anchor as a raw P-256 `X || Y` constant.
//!
//! `../keys/own-ec-p256.xy.hex` is the public half of the default signing
//! key (the flower project's own key, whose private half lives outside every
//! repository in `~/.flower-keys/flower-own-ec-p256.pem`). Set
//! `BL702_OTA_KEY_XY` to another 64-byte hex string, or to the path of a
//! file holding one, to build for a different key; CI does this with an
//! ephemeral key and `tools/bl702_ota.py` does it for the key it signs with.
//! The private key is never read here and never committed.

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

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let default = manifest.join("../keys/own-ec-p256.xy.hex");
    println!("cargo:rerun-if-env-changed=BL702_OTA_KEY_XY");
    println!("cargo:rerun-if-changed={}", default.display());

    let (origin, text) = match env::var("BL702_OTA_KEY_XY") {
        Ok(value) if value.trim().len() == 128 && !value.contains(['/', '\\', '.']) => {
            ("BL702_OTA_KEY_XY".to_owned(), value)
        }
        Ok(path) => {
            println!("cargo:rerun-if-changed={path}");
            let text = fs::read_to_string(&path)
                .unwrap_or_else(|error| panic!("read BL702_OTA_KEY_XY={path}: {error}"));
            (path, text)
        }
        Err(_) => {
            let text = fs::read_to_string(&default)
                .unwrap_or_else(|error| panic!("read {}: {error}", default.display()));
            (default.display().to_string(), text)
        }
    };
    let xy = parse_xy(&origin, &text);

    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("keys.rs");
    fs::write(
        &out,
        format!(
            "/// Public half (`X || Y`) of the key the bootloader trusts.\n\
             pub const OWN_KEY_XY: [u8; 64] = {xy:?};\n"
        ),
    )
    .expect("write keys.rs");
}
