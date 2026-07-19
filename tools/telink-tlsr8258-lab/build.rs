fn main() {
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    let linker_script = if std::env::var_os("CARGO_FEATURE_DIAG_PM").is_some() {
        "memory-pm.x"
    } else {
        "memory.x"
    };

    std::fs::copy(linker_script, out_dir.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-changed=memory-pm.x");
    println!(
        "cargo:rustc-link-arg=-T{}",
        out_dir.join("memory.x").display()
    );
    println!("cargo:rustc-link-arg=--gc-sections");
}
