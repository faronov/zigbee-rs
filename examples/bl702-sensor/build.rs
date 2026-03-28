use std::env;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());

    // Memory layout for the BL702
    println!("cargo:rustc-link-search={}", out_dir.display());
    std::fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    println!("cargo:rerun-if-changed=memory.x");

    // Linker script
    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=-Tlink.x");

    // ── Vendor library linking ──────────────────────────────────
    // Skip vendor libs when `stubs` feature is active (CI mode).
    // The stubs feature provides no-op implementations of all FFI symbols.
    let use_stubs = env::var("CARGO_FEATURE_STUBS").is_ok();
    if use_stubs {
        return;
    }

    // Priority order:
    // 1. BL_IOT_SDK_DIR env var (full SDK path, auto-derives lib paths)
    // 2. LMAC154_LIB_DIR + BL702_RF_LIB_DIR env vars (explicit paths)
    // 3. vendor_libs/ directory (ABI-patched copies, for local builds)
    //
    // The vendor .a files are compiled with rv32imfc/ilp32f (float ABI).
    // Since Rust targets riscv32imac/ilp32 (soft-float), the .a files
    // must have their ELF float-ABI flag stripped before linking.
    // Use: python3 scripts/strip_float_abi.py <input.a> <output.a>

    let mut linked = false;

    if let Ok(sdk_dir) = env::var("BL_IOT_SDK_DIR") {
        let lmac_dir = format!("{}/components/network/lmac154/lib", sdk_dir);
        let rf_dir = format!("{}/components/platform/soc/bl702/bl702_rf/lib", sdk_dir);
        if std::path::Path::new(&lmac_dir).exists() {
            println!("cargo:rustc-link-search=native={}", lmac_dir);
            println!("cargo:rustc-link-lib=static=lmac154");
            linked = true;
        }
        if std::path::Path::new(&rf_dir).exists() {
            println!("cargo:rustc-link-search=native={}", rf_dir);
            println!("cargo:rustc-link-lib=static=bl702_rf");
        }
    }

    if !linked {
        if let Ok(lib_dir) = env::var("LMAC154_LIB_DIR") {
            println!("cargo:rustc-link-search=native={}", lib_dir);
            println!("cargo:rustc-link-lib=static=lmac154");
            linked = true;
        }
        if let Ok(rf_dir) = env::var("BL702_RF_LIB_DIR") {
            println!("cargo:rustc-link-search=native={}", rf_dir);
            println!("cargo:rustc-link-lib=static=bl702_rf");
        }
    }

    // Fallback: check for ABI-patched vendor libs in vendor_libs/ directory
    if !linked {
        let vendor_dir = manifest_dir.join("vendor_libs");
        if vendor_dir.join("liblmac154.a").exists() {
            println!("cargo:rustc-link-search=native={}", vendor_dir.display());
            println!("cargo:rustc-link-lib=static=lmac154");
            println!("cargo:rustc-link-lib=static=bl702_rf");
        }
    }

    println!("cargo:rerun-if-env-changed=LMAC154_LIB_DIR");
    println!("cargo:rerun-if-env-changed=BL702_RF_LIB_DIR");
    println!("cargo:rerun-if-env-changed=BL_IOT_SDK_DIR");
}
