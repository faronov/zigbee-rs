//! Builds the selected deployment's layout (`../boot/<name>.layout`) into
//! the loader: constants and keys (`boot_layout.rs`) and `memory.x`.

use zrs_boot_layout::{Build, ChipLimits};

const DEPLOYMENTS: [&str; 4] = ["dk", "mdk-uf2", "pca10059-uf2", "promicro-uf2"];

fn main() {
    let selected: Vec<&str> = DEPLOYMENTS
        .into_iter()
        .filter(|name| {
            let feature = format!("CARGO_FEATURE_{}", name.to_uppercase().replace('-', "_"));
            std::env::var_os(feature).is_some()
        })
        .collect();
    let [deployment] = selected[..] else {
        panic!(
            "nrf52840-sensor-boot: select exactly one deployment feature of {DEPLOYMENTS:?} \
             (selected {selected:?})"
        );
    };
    let build = Build::load(
        format!("../boot/{deployment}.layout"),
        ChipLimits {
            // No loader watchdog: the nRF52 WDT cannot be stopped again.
            watchdog_max_ms: 0,
            vtor: true,
        },
    );
    build.write_rust("zrs_boot_cortex_m::mcuboot_image");
    build.write_loader_memory_x();
}
