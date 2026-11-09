//! Selects the linker memory map: the product's direct map
//! (`products/efr32mg21-sensor/link/memory.x`) by default, or with feature
//! `zrs-boot` the primary slot of the zigbee-rs loader layout
//! (`products/efr32mg21-sensor/boot/devkit.layout`) — the image the loader in
//! `bootloaders/efr32s2-boot` (deployment `efr32mg21-devkit`) verifies and
//! starts. Both keep the security journal where the direct map has it.

use std::{env, fs, path::PathBuf};

const PRODUCT: &str = "efr32mg21-sensor";
const LAYOUT: &str = "devkit.layout";

fn main() {
    let manifest = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let product_link = manifest.join("../../products").join(PRODUCT).join("link");
    let direct_path = product_link.join("memory.x");
    println!("cargo:rerun-if-changed={}", direct_path.display());
    let direct = fs::read_to_string(&direct_path).expect("read the product link map");
    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("memory.x");

    if env::var_os("CARGO_FEATURE_ZRS_BOOT").is_some() {
        let build = zrs_boot_layout::Build::load(
            format!("../../products/{PRODUCT}/boot/{LAYOUT}"),
            zrs_boot_layout::ChipLimits {
                // WDOG0 on the ULFRCO (`efr32s2_hal::watchdog::MAX_PERIOD_MS`);
                // the loader stops it before the hand-over.
                watchdog_max_ms: 238_313,
                vtor: true,
            },
        );
        let layout = &build.layout;
        let (journal, journal_len) = region(&direct, "PERSISTENCE");
        assert_eq!(
            (layout.journal.start, layout.journal.len()),
            (journal, journal_len),
            "{LAYOUT}: the loader layout must keep the security journal of link/memory.x"
        );
        // The router role journals its child and APS tables into
        // ROUTER_TABLES: a layout that hands that range to the slots cannot
        // carry the router role.
        let (tables, tables_len) = region(&direct, "ROUTER_TABLES");
        let tables_reserved = layout
            .reserved
            .iter()
            .any(|r| r.region.start == tables && r.region.len() == tables_len);
        assert!(
            tables_reserved || env::var_os("CARGO_FEATURE_ROUTER").is_none(),
            "{LAYOUT} reclaims the router tables ({tables:#X}, {tables_len} B) for the \
             slots: feature `router` cannot be combined with `zrs-boot`"
        );
        let mut script = layout.app_memory_x();
        // Keep the product's stack-floor assertions.
        for statement in stack_asserts(&direct) {
            script.push_str(&statement);
            script.push('\n');
        }
        fs::write(&output, script).expect("write loader-slot linker map");
    } else {
        fs::write(&output, &direct).expect("copy the product link map");
    }
    println!(
        "cargo:rustc-link-search={}",
        output.parent().unwrap().display()
    );
}

/// `(ORIGIN, LENGTH)` of `name` in the `MEMORY` block of `script`.
fn region(script: &str, name: &str) -> (u32, u32) {
    let line = script
        .lines()
        .map(str::trim)
        .find(|line| {
            line.strip_prefix(name)
                .is_some_and(|rest| rest.trim_start().starts_with(':'))
        })
        .unwrap_or_else(|| panic!("link/memory.x has no {name} region"));
    let field = |key: &str| {
        let value = line
            .split(key)
            .nth(1)
            .and_then(|rest| rest.trim_start().strip_prefix('='))
            .and_then(|rest| rest.split(',').next())
            .unwrap_or_else(|| panic!("{name} has no {key}"))
            .trim();
        if let Some(hex) = value.strip_prefix("0x") {
            u32::from_str_radix(hex, 16).expect("hex value")
        } else if let Some(kib) = value.strip_suffix('K') {
            kib.parse::<u32>().expect("KiB value") * 1024
        } else {
            value.parse().expect("decimal value")
        }
    };
    (field("ORIGIN"), field("LENGTH"))
}

/// The `ASSERT(...);` statements of `script` that mention `_stack_start`.
fn stack_asserts(script: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = script;
    while let Some(start) = rest.find("ASSERT(") {
        let tail = &rest[start..];
        let end = tail.find(");").expect("unterminated ASSERT") + 2;
        if tail[..end].contains("_stack_start") {
            out.push(tail[..end].to_string());
        }
        rest = &tail[end..];
    }
    out
}
