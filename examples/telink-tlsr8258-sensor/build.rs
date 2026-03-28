// build.rs — Telink TLSR8258 sensor build script
//
// When TELINK_SDK_DIR is set, links the Telink driver library.
// Without it, cargo check still works (CI verification).

fn main() {
    // Copy memory.x to OUT_DIR so the linker can find it
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    std::fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rustc-link-arg=-Tlink.x");

    // Link Telink driver library when SDK path is provided
    if let Ok(sdk_dir) = std::env::var("TELINK_SDK_DIR") {
        let lib_path = format!("{}/platform/lib", sdk_dir);
        println!("cargo:rustc-link-search=native={}", lib_path);
        println!("cargo:rustc-link-lib=static=drivers_8258");
    }
    println!("cargo:rerun-if-env-changed=TELINK_SDK_DIR");
}
