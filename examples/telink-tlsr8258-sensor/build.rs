//! Build script for TLSR8258 Zigbee sensor.
//!
//! Supports two build modes:
//!
//! 1. **CI/stub mode** (`--features stubs` with `thumbv6m-none-eabi`):
//!    Links memory.x and emits linker script args. Stubs provide FFI symbols.
//!
//! 2. **Real TC32 mode** (`tc32-unknown-none-elf` with modern-tc32 toolchain):
//!    Compiles Telink SDK startup/drivers with `clang --target=tc32` and links
//!    `libdrivers_8258.a`. Requires:
//!    - TC32 Rust toolchain: <https://github.com/modern-tc32/examples_rust>
//!    - Telink SDK: `TC32_SDK_DIR=/path/to/tl_zigbee_sdk`
//!    - TC32 LLVM: `TC32_TOOLCHAIN=/path/to/tc32-stage1`

fn main() {
    let target = std::env::var("TARGET").unwrap_or_default();
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());

    // Copy memory.x for the linker
    if std::path::Path::new("memory.x").exists() {
        let _ = std::fs::copy("memory.x", out_dir.join("memory.x"));
        println!("cargo:rustc-link-search={}", out_dir.display());
        println!("cargo:rerun-if-changed=memory.x");
    }

    if target == "tc32-unknown-none-elf" {
        build_tc32_sdk();
    } else {
        // CI stub mode — just emit linker args for thumbv6m stand-in
        println!("cargo:rustc-link-arg=-Tlink.x");
        println!("cargo:rustc-link-arg=--noinhibit-exec");
    }

    // Link Telink driver library when SDK path is provided (either mode)
    if let Ok(sdk_dir) = std::env::var("TELINK_SDK_DIR") {
        let lib_path = format!("{}/platform/lib", sdk_dir);
        println!("cargo:rustc-link-search=native={}", lib_path);
        println!("cargo:rustc-link-lib=static=drivers_8258");
    }
    println!("cargo:rerun-if-env-changed=TELINK_SDK_DIR");
    println!("cargo:rerun-if-env-changed=TC32_SDK_DIR");
    println!("cargo:rerun-if-env-changed=TC32_TOOLCHAIN");
    println!("cargo:rerun-if-env-changed=TC32_LLVM_BIN");
}

/// Compile Telink SDK C sources with `clang --target=tc32` for real TC32 builds.
fn build_tc32_sdk() {
    use std::env;
    use std::process::Command;

    let sdk_dir = env::var("TC32_SDK_DIR")
        .or_else(|_| env::var("TELINK_SDK_DIR"))
        .unwrap_or_else(|_| {
            println!("cargo:warning=TC32_SDK_DIR not set — skipping SDK compilation");
            return String::new();
        });
    if sdk_dir.is_empty() { return; }

    let llvm_bin = env::var("TC32_LLVM_BIN")
        .unwrap_or_else(|_| {
            let tc = env::var("TC32_TOOLCHAIN").unwrap_or_default();
            format!("{}/llvm/bin", tc)
        });
    let clang = format!("{}/clang", llvm_bin);

    let manifest_dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    let out_dir = env::var("OUT_DIR").unwrap();
    let obj_dir = format!("{}/objects", out_dir);
    let _ = std::fs::create_dir_all(&obj_dir);

    let sources = [
        format!("{}/platform/chip_8258/flash.c", sdk_dir),
        format!("{}/proj/drivers/drv_hw.c", sdk_dir),
        format!("{}/proj/drivers/drv_gpio.c", sdk_dir),
        format!("{}/proj/drivers/drv_timer.c", sdk_dir),
    ];

    let flags = [
        "--target=tc32", "-c", "-DMCU_CORE_8258=1", "-O2",
        "-ffunction-sections", "-fdata-sections", "-fpack-struct",
        "-fshort-enums", "-ffreestanding", "-nostdlib",
    ];

    let includes = [
        format!("-I{}/include", manifest_dir),
        format!("-I{}/proj", sdk_dir),
        format!("-I{}/platform", sdk_dir),
        format!("-I{}/platform/chip_8258", sdk_dir),
    ];

    for src in &sources {
        if !std::path::Path::new(src).exists() { continue; }
        let fname = std::path::Path::new(src).file_name().unwrap().to_string_lossy();
        let obj = format!("{}/{}.o", obj_dir, fname);
        let status = Command::new(&clang)
            .args(&flags)
            .args(&includes)
            .arg("-o").arg(&obj)
            .arg(src)
            .status();
        match status {
            Ok(s) if s.success() => {
                println!("cargo:rustc-link-arg={}", obj);
            }
            _ => {
                println!("cargo:warning=Failed to compile {}", src);
            }
        }
    }

    // Link vendor libs
    let drivers = format!("{}/platform/lib/libdrivers_8258.a", sdk_dir);
    let soft_fp = format!("{}/platform/tc32/libsoft-fp.a", sdk_dir);
    println!("cargo:rustc-link-arg=--gc-sections");
    if std::path::Path::new(&drivers).exists() {
        println!("cargo:rustc-link-arg=--start-group");
        println!("cargo:rustc-link-arg={}", drivers);
        if std::path::Path::new(&soft_fp).exists() {
            println!("cargo:rustc-link-arg={}", soft_fp);
        }
        println!("cargo:rustc-link-arg=--end-group");
    }
}
