use std::env;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

const FIRMWARE_OUTPUT_FILE: &str = "cc2340_firmware.rs";
const PHY_CONFIG_OUTPUT_FILE: &str = "cc2340_phy_config.rs";
const IEEE_PHY_SETTING_COUNT: usize = 256;
const TX_POWER_ENTRY_COUNT: usize = 14;

#[derive(Clone, Copy, PartialEq, Eq)]
enum RegisterWidth {
    U16,
    U32,
}

struct RegisterBlock {
    name: &'static str,
    base: u32,
    header: &'static str,
    width: RegisterWidth,
}

const REGISTER_BLOCKS: &[RegisterBlock] = &[
    RegisterBlock {
        name: "LRFDMDM",
        base: 0x4008_2000,
        header: "hw_lrfdmdm.h",
        width: RegisterWidth::U32,
    },
    RegisterBlock {
        name: "LRFDPBE",
        base: 0x4008_1000,
        header: "hw_lrfdpbe.h",
        width: RegisterWidth::U32,
    },
    RegisterBlock {
        name: "LRFDRFE",
        base: 0x4008_3000,
        header: "hw_lrfdrfe.h",
        width: RegisterWidth::U32,
    },
    RegisterBlock {
        name: "PBE_IEEE_RAM",
        base: 0x4009_2000,
        header: "pbe_ieee_ram_regs.h",
        width: RegisterWidth::U16,
    },
    RegisterBlock {
        name: "RFE_COMMON_RAM",
        base: 0x4009_6000,
        header: "rfe_common_ram_regs.h",
        width: RegisterWidth::U16,
    },
];

struct PhyFieldSetting {
    block: String,
    register: String,
    field: String,
    value: u32,
}

struct RegisterWrite {
    address: u32,
    value: u32,
    mask: u32,
    width: RegisterWidth,
}

struct TxPowerEntry {
    dbm: i8,
    raw: u32,
}

fn main() {
    println!("cargo:rerun-if-env-changed=CC2340_SDK_DIR");

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by Cargo"));
    let firmware_output = out_dir.join(FIRMWARE_OUTPUT_FILE);
    let phy_config_output = out_dir.join(PHY_CONFIG_OUTPUT_FILE);

    if env::var_os("CARGO_FEATURE_CC2340").is_none() {
        write_unavailable_firmware(&firmware_output);
        write_unavailable_phy_config(&phy_config_output);
        return;
    }

    let Some(sdk_dir) = env::var_os("CC2340_SDK_DIR").map(PathBuf::from) else {
        write_unavailable_firmware(&firmware_output);
        write_unavailable_phy_config(&phy_config_output);
        return;
    };

    import_firmware(&sdk_dir, &firmware_output);
    import_phy_config(&sdk_dir, &phy_config_output);
}

fn import_firmware(sdk_dir: &Path, output: &Path) {
    let patches = sdk_dir.join("source/ti/devices/cc23x0r5/rf_patches");
    let images = [
        (
            "PBE_IMAGE",
            patches.join("lrf_pbe_binary_ieee_cc23x0r5.c"),
            "LRF_PBE_binary_ieee",
        ),
        (
            "MCE_IMAGE",
            patches.join("lrf_mce_binary_ieee_cc23x0r5.c"),
            "LRF_MCE_binary_ieee",
        ),
        (
            "RFE_IMAGE",
            patches.join("lrf_rfe_binary_ieee_cc23x0r5.c"),
            "LRF_RFE_binary_ieee",
        ),
    ];

    let mut generated = String::new();
    generated.push_str("// Generated from TI SimpleLink Low Power F3 SDK radio firmware arrays.\n");
    generated.push_str("pub(crate) const FIRMWARE_AVAILABLE: bool = true;\n");
    generated.push_str(
        "pub(crate) const FIRMWARE_SOURCE: &str = \
         \"TI SimpleLink Low Power F3 SDK cc23x0r5/rf_patches\";\n",
    );

    for (rust_name, path, c_symbol) in images {
        println!("cargo:rerun-if-changed={}", path.display());
        let words = parse_image(&path, c_symbol).unwrap_or_else(|error| {
            panic!(
                "failed to import CC2340 firmware image {}: {error}",
                path.display()
            )
        });
        write_image(&mut generated, rust_name, &words);
    }

    fs::write(output, generated).expect("write generated CC2340 firmware");
}

fn import_phy_config(sdk_dir: &Path, output: &Path) {
    let device_dir = sdk_dir.join("source/ti/devices/cc23x0r5");
    let phy_path = sdk_dir.join(
        "source/ti/devices/radioconfig/.meta/config/rcl/\
         ieee_802_15_4_oqpsk_250k_cc23xx.json",
    );
    let board_path =
        sdk_dir.join("source/ti/devices/radioconfig/.meta/config/rcl_common/boards_cc2340r5.json");
    let include_dir = device_dir.join("inc");

    println!("cargo:rerun-if-changed={}", phy_path.display());
    println!("cargo:rerun-if-changed={}", board_path.display());
    for block in REGISTER_BLOCKS {
        println!(
            "cargo:rerun-if-changed={}",
            include_dir.join(block.header).display()
        );
    }

    if !phy_path.is_file()
        || !board_path.is_file()
        || !REGISTER_BLOCKS
            .iter()
            .all(|block| include_dir.join(block.header).is_file())
    {
        println!(
            "cargo:warning=CC2340 IEEE PHY metadata is unavailable; \
             use a current SimpleLink Low Power F3 SDK"
        );
        write_unavailable_phy_config(output);
        return;
    }

    let settings = parse_phy_settings(&phy_path).unwrap_or_else(|error| {
        panic!(
            "failed to parse CC2340 IEEE PHY settings {}: {error}",
            phy_path.display()
        )
    });
    if settings.len() != IEEE_PHY_SETTING_COUNT {
        panic!(
            "CC2340 IEEE PHY metadata contains {} field settings, expected \
             {IEEE_PHY_SETTING_COUNT}",
            settings.len()
        );
    }

    let writes = resolve_phy_settings(&include_dir, &settings)
        .unwrap_or_else(|error| panic!("failed to resolve CC2340 IEEE PHY settings: {error}"));
    let tx_power = parse_tx_power_table(&board_path, "LP_EM_CC2340R5").unwrap_or_else(|error| {
        panic!(
            "failed to parse CC2340 TX power table {}: {error}",
            board_path.display()
        )
    });
    if tx_power.len() != TX_POWER_ENTRY_COUNT {
        panic!(
            "CC2340 LaunchPad TX power table contains {} entries, expected \
             {TX_POWER_ENTRY_COUNT}",
            tx_power.len()
        );
    }
    write_phy_config(output, &writes, &tx_power);
}

fn write_unavailable_firmware(output: &Path) {
    fs::write(
        output,
        "\
pub(crate) const FIRMWARE_AVAILABLE: bool = false;
pub(crate) const FIRMWARE_SOURCE: &str = \"set CC2340_SDK_DIR to a TI SimpleLink Low Power F3 SDK\";
pub(crate) static PBE_IMAGE: &[u32] = &[];
pub(crate) static MCE_IMAGE: &[u32] = &[];
pub(crate) static RFE_IMAGE: &[u32] = &[];
",
    )
    .expect("write unavailable CC2340 firmware marker");
}

fn write_unavailable_phy_config(output: &Path) {
    fs::write(
        output,
        "\
pub(crate) const PHY_CONFIG_AVAILABLE: bool = false;
pub(crate) const PHY_CONFIG_SOURCE: &str = \"set CC2340_SDK_DIR to a current TI SimpleLink Low Power F3 SDK\";
pub(crate) static IEEE_802154_PHY_WRITES: &[RegisterWrite] = &[];
pub(crate) static TX_POWER_TABLE: &[TxPowerEntry] = &[];
",
    )
    .expect("write unavailable CC2340 PHY config marker");
}

fn parse_image(path: &Path, symbol: &str) -> Result<Vec<u32>, String> {
    let source = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let declaration = source
        .find(symbol)
        .ok_or_else(|| format!("symbol {symbol} not found"))?;
    let body_start = source[declaration..]
        .find('{')
        .map(|offset| declaration + offset + 1)
        .ok_or_else(|| format!("opening brace for {symbol} not found"))?;
    let body_end = source[body_start..]
        .find('}')
        .map(|offset| body_start + offset)
        .ok_or_else(|| format!("closing brace for {symbol} not found"))?;

    let mut words = Vec::new();
    for token in source[body_start..body_end]
        .split(|character: char| character == ',' || character.is_ascii_whitespace())
    {
        let token = token.trim_end_matches(['U', 'u', 'L', 'l']);
        let Some(hex) = token
            .strip_prefix("0x")
            .or_else(|| token.strip_prefix("0X"))
        else {
            continue;
        };
        words.push(
            u32::from_str_radix(hex, 16)
                .map_err(|error| format!("invalid word {token}: {error}"))?,
        );
    }

    let (&declared_len, payload) = words
        .split_first()
        .ok_or_else(|| format!("{symbol} contains no words"))?;
    if declared_len as usize != payload.len() {
        return Err(format!(
            "{symbol} declares {declared_len} payload words but contains {}",
            payload.len()
        ));
    }
    if payload.len() > 0x1000 / size_of::<u32>() {
        return Err(format!(
            "{symbol} payload is {} words, larger than the 4 KiB TOPsm RAM",
            payload.len()
        ));
    }

    Ok(payload.to_vec())
}

fn write_image(generated: &mut String, name: &str, words: &[u32]) {
    writeln!(generated, "pub(crate) static {name}: &[u32] = &[").unwrap();
    for chunk in words.chunks(8) {
        generated.push_str("    ");
        for word in chunk {
            write!(generated, "0x{word:08X}, ").unwrap();
        }
        generated.push('\n');
    }
    generated.push_str("];\n");
}

fn parse_phy_settings(path: &Path) -> Result<Vec<PhyFieldSetting>, String> {
    let source = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let mut settings = Vec::new();
    let mut in_register_settings = false;
    let mut pending_path: Option<String> = None;

    for line in source.lines() {
        let line = line.trim();
        if line.starts_with("\"rcl_register_field_settings\"") {
            in_register_settings = true;
            continue;
        }
        if line.starts_with("\"rcl_struct_rf_design_deps\"") {
            break;
        }
        if !in_register_settings {
            continue;
        }

        if let Some(path) = json_object_key(line).filter(|path| path.matches('.').count() == 2) {
            pending_path = Some(path.to_owned());
            continue;
        }

        let Some(path) = pending_path.take() else {
            continue;
        };
        if !line.starts_with("\"value\"") {
            pending_path = Some(path);
            continue;
        }

        let value = json_string_value(line)
            .ok_or_else(|| format!("missing numeric value for PHY field {path}"))
            .and_then(parse_number)?;
        let mut parts = path.split('.');
        let block = parts.next().unwrap().to_owned();
        let register = parts.next().unwrap().to_owned();
        let field = parts.next().unwrap().to_owned();
        if parts.next().is_some() {
            return Err(format!("invalid PHY field path {path}"));
        }

        settings.push(PhyFieldSetting {
            block,
            register,
            field,
            value,
        });
    }

    if pending_path.is_some() {
        return Err("unterminated PHY field setting".to_owned());
    }
    Ok(settings)
}

fn json_object_key(line: &str) -> Option<&str> {
    let rest = line.strip_prefix('"')?;
    let end = rest.find('"')?;
    line.ends_with('{').then_some(&rest[..end])
}

fn json_string_value(line: &str) -> Option<&str> {
    let (_, rest) = line.split_once(':')?;
    let rest = rest.trim();
    let rest = rest.strip_prefix('"')?;
    let end = rest.find('"')?;
    Some(&rest[..end])
}

fn parse_tx_power_table(path: &Path, board: &str) -> Result<Vec<TxPowerEntry>, String> {
    let source = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let board_object =
        json_object(&source, board).ok_or_else(|| format!("board {board} not found"))?;
    let mut entries = Vec::new();
    let mut pending_dbm = None;

    for line in board_object.lines() {
        let line = line.trim();
        if line.starts_with("\"tx_power\"") {
            let dbm = json_string_value(line)
                .ok_or_else(|| "invalid tx_power value".to_owned())?
                .parse::<i8>()
                .map_err(|error| format!("invalid tx_power value: {error}"))?;
            pending_dbm = Some(dbm);
        } else if line.starts_with("\"value\"") {
            let Some(dbm) = pending_dbm.take() else {
                continue;
            };
            let raw = json_string_value(line)
                .ok_or_else(|| format!("missing PA value for {dbm} dBm"))
                .and_then(parse_number)?;
            entries.push(TxPowerEntry { dbm, raw });
        }
    }

    if pending_dbm.is_some() {
        return Err("unterminated TX power entry".to_owned());
    }
    entries.sort_by_key(|entry| entry.dbm);
    Ok(entries)
}

fn json_object<'a>(source: &'a str, key: &str) -> Option<&'a str> {
    let marker = format!("\"{key}\"");
    let key_start = source.find(&marker)?;
    let object_start = source[key_start + marker.len()..].find('{')? + key_start + marker.len();
    let mut depth = 0usize;
    let mut in_string = false;
    let mut escaped = false;

    for (offset, character) in source[object_start..].char_indices() {
        if in_string {
            if escaped {
                escaped = false;
            } else if character == '\\' {
                escaped = true;
            } else if character == '"' {
                in_string = false;
            }
            continue;
        }

        match character {
            '"' => in_string = true,
            '{' => depth += 1,
            '}' => {
                depth -= 1;
                if depth == 0 {
                    return Some(&source[object_start..=object_start + offset]);
                }
            }
            _ => {}
        }
    }

    None
}

fn resolve_phy_settings(
    include_dir: &Path,
    settings: &[PhyFieldSetting],
) -> Result<Vec<RegisterWrite>, String> {
    let mut block_defines = Vec::new();
    for block in REGISTER_BLOCKS {
        block_defines.push((block.name, parse_defines(&include_dir.join(block.header))?));
    }

    let mut writes: Vec<RegisterWrite> = Vec::new();
    for setting in settings {
        let block = REGISTER_BLOCKS
            .iter()
            .find(|block| block.name == setting.block)
            .ok_or_else(|| format!("unsupported register block {}", setting.block))?;
        let defines = block_defines
            .iter()
            .find_map(|(name, defines)| (*name == block.name).then_some(defines))
            .unwrap();

        let offset_name = format!("{}_O_{}", setting.block, setting.register);
        let field_name = format!("{}_{}_{}", setting.block, setting.register, setting.field);
        let mask_name = format!("{field_name}_M");
        let shift_name = format!("{field_name}_S");

        let offset = lookup_define(defines, &offset_name)?;
        let mask = lookup_define(defines, &mask_name)?;
        let shift = lookup_define(defines, &shift_name)?;
        let encoded = setting.value.wrapping_shl(shift) & mask;
        let unmasked = setting.value.wrapping_shl(shift) & !mask;
        if unmasked != 0 {
            return Err(format!(
                "value 0x{:X} does not fit {}",
                setting.value, field_name
            ));
        }

        let address = block.base + offset;
        if let Some(write) = writes
            .iter_mut()
            .find(|write| write.address == address && write.width == block.width)
        {
            if write.mask & mask != 0 {
                return Err(format!("overlapping field masks for {field_name}"));
            }
            write.value |= encoded;
            write.mask |= mask;
        } else {
            writes.push(RegisterWrite {
                address,
                value: encoded,
                mask,
                width: block.width,
            });
        }
    }

    Ok(writes)
}

fn parse_defines(path: &Path) -> Result<Vec<(String, u32)>, String> {
    let source = fs::read_to_string(path).map_err(|error| error.to_string())?;
    let mut defines = Vec::new();

    for line in source.lines() {
        let Some(rest) = line.trim().strip_prefix("#define ") else {
            continue;
        };
        let mut parts = rest.split_ascii_whitespace();
        let Some(name) = parts.next() else {
            continue;
        };
        let Some(value) = parts.next() else {
            continue;
        };
        if let Ok(value) = parse_number(value) {
            defines.push((name.to_owned(), value));
        }
    }

    Ok(defines)
}

fn lookup_define(defines: &[(String, u32)], name: &str) -> Result<u32, String> {
    defines
        .iter()
        .find_map(|(candidate, value)| (candidate == name).then_some(*value))
        .ok_or_else(|| format!("macro {name} not found"))
}

fn parse_number(value: &str) -> Result<u32, String> {
    let value = value
        .trim()
        .trim_matches(|character| character == '(' || character == ')')
        .trim_end_matches(['U', 'u', 'L', 'l']);

    if let Some(value) = value.strip_prefix('-') {
        let magnitude = parse_unsigned_number(value)?;
        Ok(0u32.wrapping_sub(magnitude))
    } else {
        parse_unsigned_number(value)
    }
}

fn parse_unsigned_number(value: &str) -> Result<u32, String> {
    if let Some(hex) = value
        .strip_prefix("0x")
        .or_else(|| value.strip_prefix("0X"))
    {
        u32::from_str_radix(hex, 16).map_err(|error| format!("invalid number {value}: {error}"))
    } else {
        value
            .parse()
            .map_err(|error| format!("invalid number {value}: {error}"))
    }
}

fn write_phy_config(output: &Path, writes: &[RegisterWrite], tx_power: &[TxPowerEntry]) {
    let mut generated = String::new();
    generated.push_str("// Generated from TI's CC23xx IEEE 802.15.4 RCL PHY metadata.\n");
    generated.push_str("pub(crate) const PHY_CONFIG_AVAILABLE: bool = true;\n");
    generated.push_str(
        "pub(crate) const PHY_CONFIG_SOURCE: &str = \
         \"TI ieee_802_15_4_oqpsk_250k_cc23xx.json\";\n",
    );
    writeln!(
        generated,
        "pub(crate) static IEEE_802154_PHY_WRITES: &[RegisterWrite] = &["
    )
    .unwrap();

    for write in writes {
        let width = match write.width {
            RegisterWidth::U16 => "RegisterWidth::U16",
            RegisterWidth::U32 => "RegisterWidth::U32",
        };
        writeln!(
            generated,
            "    RegisterWrite {{ address: 0x{:08X}, value: 0x{:08X}, width: {width} }},",
            write.address, write.value
        )
        .unwrap();
    }
    generated.push_str("];\n");
    generated.push_str("pub(crate) static TX_POWER_TABLE: &[TxPowerEntry] = &[\n");
    for entry in tx_power {
        writeln!(
            generated,
            "    TxPowerEntry {{ dbm: {}, raw: 0x{:08X} }},",
            entry.dbm, entry.raw
        )
        .unwrap();
    }
    generated.push_str("];\n");

    fs::write(output, generated).expect("write generated CC2340 PHY config");
}
