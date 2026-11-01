//! Router-only layout guard for the `phy6222-router` bin.
//!
//! `.cargo/config.toml` links every bin with the product's `phy6222.x`. The
//! router probe additionally reserves its child/APS journals
//! (`products/phy62x2-evk/link/router-tables-*.x`, variant-specific); the
//! sensor and End Device link arguments are unchanged.

fn main() {
    let script = if std::env::var_os("CARGO_FEATURE_PHY6252").is_some() {
        "router-tables-phy6252.x"
    } else {
        "router-tables-phy6222.x"
    };
    println!("cargo:rustc-link-arg-bin=phy6222-router=-T{script}");
    println!("cargo:rerun-if-changed=../../products/phy62x2-evk/link/{script}");
}
