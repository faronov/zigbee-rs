//! Builds the selected deployment's product layout into the loader:
//! constants and keys (`boot_layout.rs`), `memory.x` (included by
//! `link/phy62x2-loader.x`) and the link-script search path.

use std::env;

use zrs_boot_layout::{Build, ChipLimits};

/// (feature, product layout)
const DEPLOYMENTS: [(&str, &str); 1] =
    [("pb03f-kit", "../../products/pb03f-kit/boot/pb03f.layout")];

/// Flash offsets the PHY62xx boot ROM reads (`tools/phy62x2-image`): the
/// application table sector and the start of the SRAM load data.
const ROM_BOOT_TABLE: u32 = 0x2000;
const ROM_LOAD_DATA: u32 = 0x5000;

fn main() {
    let selected: Vec<_> = DEPLOYMENTS
        .into_iter()
        .filter(|(feature, _)| {
            let var = format!("CARGO_FEATURE_{}", feature.to_uppercase().replace('-', "_"));
            env::var_os(var).is_some()
        })
        .collect();
    let [(feature, layout)] = selected[..] else {
        let names: Vec<_> = DEPLOYMENTS.iter().map(|(name, _)| *name).collect();
        panic!("phy62x2-boot: select exactly one deployment feature of {names:?}");
    };
    let build = Build::load(
        layout,
        ChipLimits {
            // phy6222-hal has no loader watchdog support.
            watchdog_max_ms: 0,
            // Cortex-M0: no VTOR; the ROM dispatches through a jump table.
            vtor: false,
        },
    );
    // The ROM-boot table and the SRAM load data are written with the loader
    // and must be protected from the slots and the journal.
    let protected = |start: u32| {
        build
            .layout
            .reserved
            .iter()
            .any(|r| r.region.start <= start && start < r.region.end)
    };
    if !protected(ROM_BOOT_TABLE) || !protected(ROM_LOAD_DATA) {
        panic!(
            "phy62x2-boot ({feature}): the layout must reserve the ROM-boot table \
             ({ROM_BOOT_TABLE:#x}) and the SRAM load data ({ROM_LOAD_DATA:#x})"
        );
    }
    if build.layout.loader.start <= ROM_LOAD_DATA {
        panic!(
            "phy62x2-boot ({feature}): the loader's XIP code must start above the SRAM \
             load data at {ROM_LOAD_DATA:#x}"
        );
    }
    build.write_rust("zrs_boot_cortex_m::mcuboot_image");
    build.write_loader_memory_x();
    let manifest = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search={manifest}/link");
    println!("cargo:rerun-if-changed=link/phy62x2-loader.x");
}
