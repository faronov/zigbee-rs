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

    let target = env::var("TARGET").unwrap();
    if target != "riscv32imafc-unknown-none-elf" {
        panic!(
            "Bouffalo's BL702 radio archives use the ilp32f hard-float ABI; \
             build with --target riscv32imafc-unknown-none-elf. Do not strip \
             the ELF float-ABI flag."
        );
    }

    // Priority order:
    // 1. BL_IOT_SDK_DIR env var (full SDK path, auto-derives lib paths)
    // 2. LMAC154_LIB_DIR + BL702_RF_LIB_DIR env vars (explicit paths)
    // 3. vendor_libs/ directory (local copies)
    //
    // The vendor .a files are compiled with rv32imfc/ilp32f. The Rust target
    // must use the same ABI; changing archive metadata does not make a
    // soft-float caller ABI-compatible with hard-float code.

    let mut linked_lmac = false;
    let mut linked_rf = false;

    if let Ok(sdk_dir) = env::var("BL_IOT_SDK_DIR") {
        let lmac_dir = format!("{}/components/network/lmac154/lib", sdk_dir);
        let rf_dir = format!("{}/components/platform/soc/bl702/bl702_rf/lib", sdk_dir);
        if std::path::Path::new(&lmac_dir).exists() {
            println!("cargo:rustc-link-search=native={}", lmac_dir);
            println!("cargo:rustc-link-lib=static=lmac154");
            linked_lmac = true;
        }
        if std::path::Path::new(&rf_dir).exists() {
            println!("cargo:rustc-link-search=native={}", rf_dir);
            println!("cargo:rustc-link-lib=static=bl702_rf");
            linked_rf = true;
        }
    }

    if !linked_lmac {
        if let Ok(lib_dir) = env::var("LMAC154_LIB_DIR") {
            println!("cargo:rustc-link-search=native={}", lib_dir);
            println!("cargo:rustc-link-lib=static=lmac154");
            linked_lmac = true;
        }
    }
    if !linked_rf {
        if let Ok(rf_dir) = env::var("BL702_RF_LIB_DIR") {
            println!("cargo:rustc-link-search=native={}", rf_dir);
            println!("cargo:rustc-link-lib=static=bl702_rf");
            linked_rf = true;
        }
    }

    // Fallback: check for vendor libs in vendor_libs/ directory.
    if !linked_lmac || !linked_rf {
        let vendor_dir = manifest_dir.join("vendor_libs");
        if !linked_lmac && vendor_dir.join("liblmac154.a").exists() {
            println!("cargo:rustc-link-search=native={}", vendor_dir.display());
            println!("cargo:rustc-link-lib=static=lmac154");
            linked_lmac = true;
        }
        if !linked_rf && vendor_dir.join("libbl702_rf.a").exists() {
            println!("cargo:rustc-link-search=native={}", vendor_dir.display());
            println!("cargo:rustc-link-lib=static=bl702_rf");
            linked_rf = true;
        }
    }

    if !linked_lmac || !linked_rf {
        panic!(
            "BL702 vendor build requires both liblmac154.a and libbl702_rf.a; \
             set BL_IOT_SDK_DIR, set both explicit library directories, or \
             use --features stubs for the compile-only check"
        );
    }

    println!("cargo:rerun-if-env-changed=LMAC154_LIB_DIR");
    println!("cargo:rerun-if-env-changed=BL702_RF_LIB_DIR");
    println!("cargo:rerun-if-env-changed=BL_IOT_SDK_DIR");
}
