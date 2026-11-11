//! For feature `zrs-boot` only: the loader layout constants
//! (`OUT_DIR/boot_layout.rs`) and the release version (`OUT_DIR/version.rs`)
//! from `boot/zbt1.layout`.
//!
//! The release version is the package version `MM.mm.pp`, or for a
//! TEST-ONLY bench build `ZBT1_OTA_VERSION_OVERRIDE` (the layout's
//! `env-prefix`); see `zrs_boot_layout::ReleaseVersion`. The direct-linked
//! role images keep their fixed `SW_BUILD` and do not read the layout.

const LAYOUT: &str = "boot/zbt1.layout";

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
    if std::env::var_os("CARGO_FEATURE_ZRS_BOOT").is_none() {
        return;
    }
    let build = zrs_boot_layout::Build::load(
        LAYOUT,
        zrs_boot_layout::ChipLimits {
            // WDOG0 on the ULFRCO (`efr32s2_hal::watchdog::MAX_PERIOD_MS`),
            // as in bootloaders/efr32s2-boot/build.rs.
            watchdog_max_ms: 238_313,
            vtor: true,
        },
    );
    build.write_version();
    build.write_rust("mcuboot_image");
}
