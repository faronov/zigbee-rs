fn main() {
    let out_dir = std::path::PathBuf::from(std::env::var("OUT_DIR").unwrap());
    std::fs::copy("memory.x", out_dir.join("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!(
        "cargo:rustc-link-arg=-T{}",
        out_dir.join("memory.x").display()
    );
    println!("cargo:rustc-link-arg=--gc-sections");
}
