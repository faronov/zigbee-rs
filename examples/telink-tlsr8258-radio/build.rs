//! Build script for the standalone TLSR8258 raw-radio bring-up crate.
//!
//! Pure Rust, no vendor SDK, no C compilation. Uses rust-lld with a custom
//! linker script (memory.x). Mirrors examples/telink-tlsr8258-sensor/build.rs.

fn main() {
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-env-changed=TARGET");

    // `target_arch = "tc32"` is a real value for the custom tc32-45/tc32-43
    // toolchains' built-in `tc32-unknown-none-elf` target, but the host
    // rustc's `check-cfg` doesn't know about it (it only ships with the
    // architectures the host compiler itself supports), which would
    // otherwise produce a spurious `unexpected_cfgs` warning on every
    // `target_arch = "tc32"` gate when running host-side `cargo test`.
    println!("cargo:rustc-check-cfg=cfg(target_arch, values(\"tc32\"))");

    // Host-side `cargo test`/`cargo check` builds this same build script for
    // the host target (e.g. x86_64-apple-darwin). memory.x is a tc32/rust-lld
    // linker script and must never be handed to the host linker, so only
    // apply it when the actual compilation target is tc32.
    let target = std::env::var("TARGET").unwrap_or_default();
    if target != "tc32-unknown-none-elf" {
        return;
    }

    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    if std::path::Path::new("memory.x").exists() {
        let _ = std::fs::copy("memory.x", out_dir.join("memory.x"));
        println!("cargo:rustc-link-search={}", out_dir.display());
    }

    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=--gc-sections");
}
