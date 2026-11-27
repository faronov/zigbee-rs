//! `efr32-gbl`: package an EFR32 Series 2 application ELF for the Gecko
//! bootloader, or inspect a GBL file.
//!
//! ```text
//! efr32-gbl app --elf <ELF> --out <PREFIX> [--app-start 0x08006000]
//!               [--app-end 0x08158000] [--ram-end 0x20040000]
//!               [--metadata <FILE>]
//!     writes <PREFIX>.bin (raw image, vector word 13 -> properties, ZRHP
//!            build ID stored when the image has a build record),
//!            <PREFIX>.hex (same image, Intel HEX for J-Link) and
//!            <PREFIX>.gbl (plain GBL3 for the Gecko UART/XMODEM bootloader;
//!            with the ZRHP metadata record for NCP images)
//! efr32-gbl clear --out <GBL> [--from 0x08158000] [--to 0x08180000]
//!     writes a GBL that only erases [from, to) (default: the
//!     products/efr32mg24 NV journal window), e.g. before going back to a
//!     stock EmberZNet image; no application record
//! efr32-gbl parse <GBL>
//!     verifies the CRC and lists the records, the application data, the
//!     program ranges and any signature/encryption tags; an application
//!     GBL for the EFR32MG24 F1536 application space (0x08006000) must also
//!     pass the Gecko bootloader's start-up checks (`gecko_boot_check`)
//! ```
//!
//! Defaults match `products/efr32mg24` (EFR32MG24 F1536 above the
//! 24 KiB Gecko bootloader).

use std::path::PathBuf;
use std::process::ExitCode;

use efr32_gbl::{
    Error, GeckoTarget, ZrhpBuild, elf_flash_image, gecko_boot_check, intel_hex, parse,
    prepare_app_image, prepare_zrhp_build, program_image, write, write_erase,
};

fn number(text: &str) -> Result<u32, String> {
    let parsed = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(&hex.replace('_', ""), 16),
        None => text.replace('_', "").parse(),
    };
    parsed.map_err(|_| format!("not a number: {text}"))
}

fn app(args: &[String]) -> Result<(), String> {
    let mut elf = None;
    let mut out = None;
    let mut app_start = 0x0800_6000u32;
    let mut app_end = 0x0815_8000u32;
    let mut ram_end = 0x2004_0000u32;
    let mut metadata = None;
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        let mut value = || iter.next().cloned().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--elf" => elf = Some(PathBuf::from(value()?)),
            "--out" => out = Some(value()?),
            "--app-start" => app_start = number(&value()?)?,
            "--app-end" => app_end = number(&value()?)?,
            "--ram-end" => ram_end = number(&value()?)?,
            "--metadata" => metadata = Some(PathBuf::from(value()?)),
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let elf = elf.ok_or("--elf is required")?;
    let out = out.ok_or("--out is required")?;
    let bytes = std::fs::read(&elf).map_err(|e| format!("{}: {e}", elf.display()))?;
    // Everything in main flash below the user-data page counts, so a
    // stray segment outside the application window is reported, not lost.
    let (base, mut image) =
        elf_flash_image(&bytes, 0x0000_0000..0x0FE0_0000).map_err(|e| e.to_string())?;
    if base != app_start {
        return Err(Error::Layout(format!(
            "image starts at {base:#010x}, expected the application start {app_start:#010x}"
        ))
        .to_string());
    }
    let end = base as u64 + image.len() as u64;
    if end > app_end as u64 {
        return Err(Error::Layout(format!(
            "image ends at {end:#x}, past the application end {app_end:#010x}"
        ))
        .to_string());
    }
    let props =
        prepare_app_image(&mut image, base, 0x2000_0000..ram_end).map_err(|e| e.to_string())?;
    if props.signature_type != 0 {
        return Err(format!(
            "ApplicationProperties_t declares signature type {}, but only unsigned GBLs are written",
            props.signature_type
        ));
    }
    let zrhp = prepare_zrhp_build(&mut image, base).map_err(|e| e.to_string())?;
    let metadata = match (metadata, &zrhp) {
        (Some(_), Some(_)) => {
            return Err(
                "the image has a ZRHP build record; --metadata would replace its \
                        ZRHP metadata"
                    .into(),
            );
        }
        (Some(path), None) => {
            Some(std::fs::read(&path).map_err(|e| format!("{}: {e}", path.display()))?)
        }
        (None, Some(build)) => Some(build.metadata()),
        (None, None) => None,
    };
    let gbl = write(&props.app, base, &image, metadata.as_deref());
    parse(&gbl).map_err(|e| format!("self-check of the written GBL failed: {e}"))?;
    gecko_boot_check(
        &image,
        base,
        GeckoTarget {
            app_space_start: app_start,
            sram: (0x2000_0000, ram_end),
            // Stricter than the bootloader (end of flash): the application window.
            flash_end: app_end,
        },
    )
    .map_err(|e| e.to_string())?;
    for (suffix, data) in [
        ("bin", image.clone()),
        ("hex", intel_hex(&image, base).into_bytes()),
        ("gbl", gbl.clone()),
    ] {
        let path = format!("{out}.{suffix}");
        std::fs::write(&path, data).map_err(|e| format!("{path}: {e}"))?;
    }
    println!(
        "image {:#010x}..{:#010x} ({} B), properties at {:#010x}, app version {:#010x}, \
         product ID {}, GBL {} B (plain, unsigned)",
        base,
        end,
        image.len(),
        props.address,
        props.app.version,
        props
            .app
            .product_id
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect::<String>(),
        gbl.len()
    );
    if let Some(build) = zrhp {
        println!(
            "ZRHP build ID {} (record at {:#010x}), firmware {}",
            hex(&build.build_id),
            build.address,
            String::from_utf8_lossy(&build.fw_version)
        );
    }
    Ok(())
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn clear(args: &[String]) -> Result<(), String> {
    let mut out = None;
    let mut from = 0x0815_8000u32;
    let mut to = 0x0818_0000u32;
    let mut iter = args.iter();
    while let Some(arg) = iter.next() {
        let mut value = || iter.next().cloned().ok_or(format!("{arg} needs a value"));
        match arg.as_str() {
            "--out" => out = Some(value()?),
            "--from" => from = number(&value()?)?,
            "--to" => to = number(&value()?)?,
            other => return Err(format!("unknown argument {other}")),
        }
    }
    let out = out.ok_or("--out is required")?;
    // Never the 24 KiB Gecko bootloader.
    if from < 0x0800_6000 {
        return Err(format!("{from:#010x} is inside the Gecko bootloader"));
    }
    let gbl = write_erase(from, to, 8192).map_err(|e| e.to_string())?;
    parse(&gbl).map_err(|e| format!("self-check of the written GBL failed: {e}"))?;
    std::fs::write(&out, &gbl).map_err(|e| format!("{out}: {e}"))?;
    println!(
        "{out}: erases {from:#010x}..{to:#010x} ({} B), GBL {} B (plain, no application record)",
        to - from,
        gbl.len()
    );
    Ok(())
}

fn describe(path: &str) -> Result<(), String> {
    let data = std::fs::read(path).map_err(|e| format!("{path}: {e}"))?;
    let parsed = parse(&data).map_err(|e| e.to_string())?;
    println!(
        "{path}: {} B, CRC {:#010x} ok, type {:#010x}",
        data.len(),
        parsed.crc,
        parsed.gbl_type
    );
    for record in &parsed.records {
        println!(
            "  {:#010x} at {:#08x}: {} B",
            record.tag,
            record.offset,
            record.body.len()
        );
    }
    if let Some(app) = parsed.application {
        println!(
            "  application: type {:#x}, version {:#010x}, capabilities {:#x}, product {}",
            app.app_type,
            app.version,
            app.capabilities,
            app.product_id
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect::<String>()
        );
    }
    for (address, length) in &parsed.programs {
        println!(
            "  program {:#010x}..{:#010x}",
            address,
            *address as u64 + *length as u64
        );
    }
    if let Some(metadata) = &parsed.metadata {
        match ZrhpBuild::from_metadata(metadata) {
            Some((build_id, fw)) => println!(
                "  metadata: ZRHP v1 build ID {} firmware {}",
                hex(&build_id),
                String::from_utf8_lossy(&fw)
            ),
            None => println!("  metadata: {}", String::from_utf8_lossy(metadata)),
        }
    }
    println!(
        "  signed: {}, encrypted: {}, updates bootloader/SE: {}",
        parsed.signed, parsed.encrypted, parsed.updates_bootloader
    );
    // An application GBL for an EFR32MG24 F1536 above a 24 KiB Gecko
    // bootloader (the products/efr32mg24 and stock Dongle-M layout) must
    // pass the bootloader's start-up checks.
    if parsed.application.is_some() && !parsed.encrypted {
        let target = GeckoTarget::MG24_F1536;
        match program_image(&parsed) {
            Some((base, image)) if base == target.app_space_start => {
                let boot = gecko_boot_check(&image, base, target).map_err(|e| e.to_string())?;
                println!(
                    "  Gecko start-up check (MG24 F1536, app space {:#010x}): SP {:#010x}, reset \
                     {:#010x}, properties at {:#010x} v{:#06x}, signature NONE: ok",
                    target.app_space_start,
                    boot.initial_sp,
                    boot.reset_vector,
                    boot.properties.address,
                    boot.properties.struct_version
                );
            }
            Some((base, _)) => println!(
                "  Gecko start-up check skipped: program starts at {base:#010x}, not the MG24 \
                 F1536 application space"
            ),
            None => println!("  Gecko start-up check skipped: compressed or no program records"),
        }
    }
    Ok(())
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let result = match args.first().map(String::as_str) {
        Some("app") => app(&args[1..]),
        Some("clear") => clear(&args[1..]),
        Some("parse") if args.len() == 2 => describe(&args[1]),
        _ => Err(
            "usage: efr32-gbl app --elf <ELF> --out <PREFIX> [...] | efr32-gbl clear --out <GBL> \
             [--from A --to B] | efr32-gbl parse <GBL>"
                .into(),
        ),
    };
    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("efr32-gbl: {error}");
            ExitCode::FAILURE
        }
    }
}
