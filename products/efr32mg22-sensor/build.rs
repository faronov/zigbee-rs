//! Generates the release version (`OUT_DIR/version.rs`) and, for feature
//! `zrs-boot`, the loader layout constants (`OUT_DIR/boot_layout.rs`) from
//! `boot/devkit.layout`.
//!
//! The release version is the package version `MM.mm.pp`, or for a
//! TEST-ONLY bench build `EFR32MG22_OTA_VERSION_OVERRIDE` (the layout's
//! `env-prefix`); see `zrs_boot_layout::ReleaseVersion`.

const LAYOUT: &str = "boot/devkit.layout";

fn main() {
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
    if std::env::var_os("CARGO_FEATURE_ZRS_BOOT").is_some() {
        build.write_rust("mcuboot_image");
    }
}
