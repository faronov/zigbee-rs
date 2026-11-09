//! Builds the selected deployment's product layout into the loader:
//! constants and keys (`boot_layout.rs`), `memory.x`, and the chip's
//! `device.x` search path.

use std::env;

use zrs_boot_layout::{Build, ChipLimits};

/// (feature, product layout, chip link directory)
const DEPLOYMENTS: [(&str, &str, &str); 3] = [
    (
        "efr32mg22-devkit",
        "../../products/efr32mg22-sensor/boot/devkit.layout",
        "efr32mg22",
    ),
    (
        "efr32mg21-devkit",
        "../../products/efr32mg21-sensor/boot/devkit.layout",
        "efr32mg21",
    ),
    (
        "ha-connect-zbt1",
        "../../products/ha-connect-zbt1/boot/zbt1.layout",
        "efr32mg21",
    ),
];

fn main() {
    let selected: Vec<_> = DEPLOYMENTS
        .into_iter()
        .filter(|(feature, ..)| {
            let var = format!("CARGO_FEATURE_{}", feature.to_uppercase().replace('-', "_"));
            env::var_os(var).is_some()
        })
        .collect();
    let [(feature, layout, chip)] = selected[..] else {
        let names: Vec<_> = DEPLOYMENTS.iter().map(|(name, ..)| *name).collect();
        let selected: Vec<_> = selected.iter().map(|(name, ..)| *name).collect();
        panic!(
            "efr32s2-boot: select exactly one deployment feature of {names:?} \
             (selected {selected:?})"
        );
    };
    let build = Build::load(
        layout,
        ChipLimits {
            // efr32s2-hal has no WDOG driver yet.
            watchdog_max_ms: 0,
            vtor: true,
        },
    );
    if env::var_os("CARGO_FEATURE_GECKO_RECOVERY").is_some()
        && !build
            .layout
            .reserved
            .iter()
            .any(|r| r.region.start == 0 && r.region.end <= build.layout.loader.start)
    {
        panic!(
            "efr32s2-boot ({feature}): `gecko-recovery` needs a chained layout with the Gecko \
             Bootloader reserved below the loader"
        );
    }
    build.write_rust("zrs_boot_cortex_m::mcuboot_image");
    build.write_loader_memory_x();
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={manifest}/../../efr32s2-hal/link/{chip}");
    println!("cargo:rerun-if-changed=../../efr32s2-hal/link/{chip}/device.x");
}
