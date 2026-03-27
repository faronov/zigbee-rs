use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Board-specific memory layout:
    // - ProMicro: SoftDevice S140 present → app at 0x26000, RAM at 0x20002000
    // - MDK / PCA10059: no SoftDevice → app at 0x1000, full RAM
    // - DK: no bootloader at all → app at 0x0000, full everything
    let memory_x = if cfg!(feature = "board-promicro") {
        r#"MEMORY
{
  FLASH : ORIGIN = 0x00026000, LENGTH = 808K
  RAM   : ORIGIN = 0x20002000, LENGTH = 248K
}
"#
    } else if cfg!(feature = "board-nrf-dk") {
        r#"MEMORY
{
  FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
"#
    } else {
        r#"MEMORY
{
  FLASH : ORIGIN = 0x00001000, LENGTH = 1020K
  RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
"#
    };
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memory_x.as_bytes())
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:rustc-link-arg-bins=--noinhibit-exec");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
