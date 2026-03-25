fn main() {
    println!("cargo:rustc-link-arg-bins=--noinhibit-exec");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
