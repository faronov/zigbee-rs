//! build.rs for efr32mg1-sensor
//!
//! Compiles the RAIL FFI C shim, links librail_efr32xg1, and provides
//! device.x interrupt vector definitions to cortex-m-rt.

fn main() {
    let out_dir = std::env::var("OUT_DIR").unwrap();

    // Copy device.x
    std::fs::copy("device.x", format!("{}/device.x", out_dir))
        .expect("failed to copy device.x");
    println!("cargo:rustc-link-search={}", out_dir);
    println!("cargo:rustc-link-arg=-Tlink.x");

    // ── RAIL FFI: compile C shim and link RAIL library ──
    let gsdk = "/tmp/gsdk";
    let rail = format!("{}/platform/radio/rail_lib", gsdk);
    let ffi_dir = "../../zigbee-mac/src/efr32/rail_ffi";

    // Compile rail_shim.c
    cc::Build::new()
        .file(format!("{}/rail_shim.c", ffi_dir))
        .include(ffi_dir)
        .include(format!("{}/common", rail))
        .include(format!("{}/chip/efr32/efr32xg1x", rail))
        .include(format!("{}/protocol/ieee802154", rail))
        .define("EFR32MG1P232F256GM48", None)
        .flag("-mcpu=cortex-m4")
        .flag("-mthumb")
        .flag("-mfloat-abi=softfp")
        .flag("-mfpu=fpv4-sp-d16")
        .opt_level_str("s")
        .warnings(false)
        .compile("rail_shim");

    // Link RAIL library
    println!("cargo:rustc-link-search={}/autogen/librail_release", rail);
    println!("cargo:rustc-link-lib=static=rail_efr32xg1_gcc_release");

    // Rebuild triggers
    println!("cargo:rerun-if-changed=device.x");
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed={}/rail_shim.c", ffi_dir);
    println!("cargo:rerun-if-changed={}/em_device.h", ffi_dir);
}
