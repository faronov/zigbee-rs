//! Router-only layout guard for the `pb03f-router` bin.
//!
//! `.cargo/config.toml` links every bin with the product's `pb03f.x`. The
//! router probe additionally reserves its child/APS journals
//! (`products/pb03f-kit/link/router-tables.x`); the other bins' link
//! arguments are unchanged.

fn main() {
    println!("cargo:rustc-link-arg-bin=pb03f-router=-Trouter-tables.x");
    println!("cargo:rerun-if-changed=../../products/pb03f-kit/link/router-tables.x");
}
