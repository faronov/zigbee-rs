use std::{env, path::PathBuf};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let memory = "../../products/bl702-xt-zb1/link/memory.x";
    std::fs::copy(memory, out_dir.join("memory.x")).unwrap();

    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rustc-link-arg=-Tmemory.x");
    println!("cargo:rustc-link-arg=-Tlink.x");
    println!("cargo:rerun-if-changed={memory}");

    // LAB ONLY soak image: a deterministic, non-erased padding blob that
    // makes the image about as large as the sensor, so the loader's copy
    // takes as long as a real install.
    if env::var_os("CARGO_FEATURE_OTA_SOAK").is_some() {
        const PAD_LEN: usize = 0x34000;
        let mut state = 0x2545_F491_u32;
        let pad: Vec<u8> = (0..PAD_LEN)
            .map(|_| {
                state ^= state << 13;
                state ^= state >> 17;
                state ^= state << 5;
                state as u8
            })
            .collect();
        std::fs::write(out_dir.join("soak-pad.bin"), pad).unwrap();
    }
}
