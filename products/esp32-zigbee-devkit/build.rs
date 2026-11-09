//! Feature `signed-ota`: trusted envelope keys from `boot/app-verify.conf`
//! (overridable with `ESP32_DEVKIT_OTA_KEY{,2,3}_XY`).

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    #[cfg(feature = "signed-ota")]
    zrs_boot_layout::AppVerify::build("boot/app-verify.conf", "mcuboot_image");
}
