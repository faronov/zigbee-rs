use std::env;
use std::path::PathBuf;

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // Memory layout for the BL702
    println!("cargo:rustc-link-search={}", out_dir.display());
    std::fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    println!("cargo:rerun-if-changed=memory.x");

    // Linker script
    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=-Tlink.x");

    // Link the Bouffalo lmac154 static library (downloaded in CI or placed manually)
    if let Ok(lib_dir) = env::var("LMAC154_LIB_DIR") {
        println!("cargo:rustc-link-search=native={}", lib_dir);
        println!("cargo:rustc-link-lib=static=lmac154");
    }
}
