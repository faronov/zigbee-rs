//! Build-time flash layout of a zigbee-rs loader and its application.
//!
//! Every product that uses the common bootloader describes its flash map in
//! one small text file (by convention `boot/<deployment>.layout` in the
//! product). The loader binary's build script and the application's build
//! script both read that file through this crate, so the loader region, the
//! slots, the persistence journal and every protected region are written
//! down exactly once:
//!
//! * [`BootLayout::parse`] reads the file, [`BootLayout::check`] refuses any
//!   map that could brick a device or destroy network state (overlaps, a
//!   protected region between the slots, misaligned slots, a vector table
//!   the core cannot point `VTOR` at, a loader region too small for its
//!   records, ...);
//! * [`BootLayout::rust`] emits the constants, trusted keys and
//!   `BootConfig` for `include!`;
//! * [`BootLayout::loader_memory_x`] and [`BootLayout::app_memory_x`] emit
//!   the linker `MEMORY` regions with post-link assertions.
//!
//! Trusted keys and the two loader policy numbers come from the
//! environment, named after the file's `env-prefix` (see
//! [`BootLayout::resolve`]); private keys are never read.
//!
//! # File format
//!
//! One `key = value` per line, `#` starts a comment. Numbers are decimal or
//! `0x` hex, ranges are `start..end` (end exclusive) and refer to physical
//! flash offsets.
//!
//! ```text
//! name           = nrf52840-dk          # shown in messages
//! flash          = 0x0..0x100000        # the flash device
//! sector         = 0x1000               # largest erase unit of the slots
//! loader         = 0x0..0x4000          # loader; its last 64 bytes hold the loader block
//! primary        = 0x4000..0x81000
//! secondary      = 0x81000..0xFE000
//! journal        = 0xFE000..0x100000    # product persistence (never written by the loader)
//! reserved       = 0xF4000..0xFE000 uf2-bootloader   # repeatable, with a label
//! ram            = 0x20000000..0x20040000
//! xip-base       = 0                    # memory address of flash offset 0 (default 0)
//! header         = 0x200                # imgtool --header-size
//! vector-align   = 0x200                # VTOR alignment of the application (default 0x100)
//! tlv-reserve    = 0x300                # room for the TLVs after the application
//! app-max        = 130816               # optional cap on the linked application
//! loader-version = 1
//! trial-boots    = 3                    # default; <PREFIX>_BOOT_TRIAL_BOOTS overrides
//! watchdog-ms    = 0                    # default; <PREFIX>_BOOT_WATCHDOG_MS overrides
//! mode           = overwrite            # or: swap-move 0x7C000..0x7E000
//! env-prefix     = NRF52840
//! key-file       = ../../../bootloaders/keys/own-ec-p256.xy.hex  # relative to this file
//! ```

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

/// Size of the loader block (`LoaderKeys` + `LoaderInfo`) at the end of the
/// loader region.
pub const LOADER_BLOCK_LEN: u32 = 64;
/// Most keys a loader trusts (`mcuboot_image::MAX_TRUSTED_KEYS`).
pub const MAX_TRUSTED_KEYS: usize = 3;

/// `[start, end)` in physical flash offsets (or memory addresses for RAM).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    pub start: u32,
    pub end: u32,
}

impl Region {
    pub const fn len(&self) -> u32 {
        self.end - self.start
    }

    pub const fn is_empty(&self) -> bool {
        self.end <= self.start
    }

    pub const fn overlaps(&self, other: &Region) -> bool {
        self.start < other.end && other.start < self.end
    }

    pub const fn contains(&self, other: &Region) -> bool {
        other.start >= self.start && other.end <= self.end
    }
}

/// A region the loader must never touch, with its label.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Reserved {
    pub name: String,
    pub region: Region,
}

/// Install strategy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Overwrite,
    /// Swap-move with revert; `status` is two sectors outside both slots.
    SwapMove {
        status: Region,
    },
}

/// A product's loader and slot map. See the crate documentation for the
/// file format.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BootLayout {
    pub name: String,
    pub flash: Region,
    pub sector: u32,
    pub loader: Region,
    pub primary: Region,
    pub secondary: Region,
    pub journal: Region,
    pub reserved: Vec<Reserved>,
    pub ram: Region,
    pub xip_base: u32,
    pub header: u32,
    pub vector_align: u32,
    pub tlv_reserve: u32,
    pub app_max: Option<u32>,
    pub loader_version: u32,
    pub trial_boots: u8,
    pub watchdog_ms: u32,
    pub mode: Mode,
    pub env_prefix: String,
    /// Default primary key file (`X || Y` hex), resolved against the
    /// layout file's directory.
    pub key_file: Option<PathBuf>,
}

/// Trusted keys and what was consulted to find them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Keys {
    /// Raw P-256 `X || Y`, first the one `LoaderInfo` names.
    pub keys: Vec<[u8; 64]>,
    /// Environment variables read (for `cargo:rerun-if-env-changed`).
    pub vars: Vec<String>,
    /// Files read (for `cargo:rerun-if-changed`).
    pub files: Vec<PathBuf>,
}

/// Chip limits the product's build script passes to [`BootLayout::check`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChipLimits {
    /// Longest loader watchdog period the chip shell can program; `0`
    /// means the shell has no loader watchdog.
    pub watchdog_max_ms: u32,
    /// Whether the chip shell sets `VTOR` (Cortex-M0 cores have none: the
    /// application then has no alignment requirement from the loader).
    pub vtor: bool,
}

fn parse_number(text: &str) -> Result<u32, String> {
    let text = text.trim().replace('_', "");
    let parsed = match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u32::from_str_radix(hex, 16),
        None => text.parse(),
    };
    parsed.map_err(|_| format!("`{text}` is not a number"))
}

fn parse_region(text: &str) -> Result<Region, String> {
    let (start, end) = text
        .split_once("..")
        .ok_or_else(|| format!("`{text}` is not a `start..end` range"))?;
    let region = Region {
        start: parse_number(start)?,
        end: parse_number(end)?,
    };
    if region.is_empty() {
        return Err(format!("range `{text}` is empty"));
    }
    Ok(region)
}

impl BootLayout {
    /// Parse the layout text. `base` resolves a relative `key-file`.
    pub fn parse(text: &str, base: &Path) -> Result<Self, String> {
        let mut name = None;
        let mut flash = None;
        let mut sector = None;
        let mut loader = None;
        let mut primary = None;
        let mut secondary = None;
        let mut journal = None;
        let mut ram = None;
        let mut reserved = Vec::new();
        let mut xip_base = 0;
        let mut header = 0x100;
        let mut vector_align = 0x100;
        let mut tlv_reserve = 0x300;
        let mut app_max = None;
        let mut loader_version = None;
        let mut trial_boots = 0u8;
        let mut watchdog_ms = 0;
        let mut mode = Mode::Overwrite;
        let mut env_prefix = None;
        let mut key_file = None;

        for (number, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let at = |error: String| format!("line {}: {error}", number + 1);
            let (key, value) = line
                .split_once('=')
                .ok_or_else(|| at(format!("`{line}` is not `key = value`")))?;
            let (key, value) = (key.trim(), value.trim());
            let set = |slot: &mut Option<Region>| -> Result<(), String> {
                if slot.is_some() {
                    return Err(at(format!("`{key}` given twice")));
                }
                *slot = Some(parse_region(value).map_err(at)?);
                Ok(())
            };
            match key {
                "name" => name = Some(value.to_string()),
                "flash" => set(&mut flash)?,
                "loader" => set(&mut loader)?,
                "primary" => set(&mut primary)?,
                "secondary" => set(&mut secondary)?,
                "journal" => set(&mut journal)?,
                "ram" => set(&mut ram)?,
                "reserved" => {
                    let (range, label) = value
                        .split_once(char::is_whitespace)
                        .ok_or_else(|| at("`reserved` needs a range and a label".to_string()))?;
                    reserved.push(Reserved {
                        name: label.trim().to_string(),
                        region: parse_region(range).map_err(at)?,
                    });
                }
                "sector" => sector = Some(parse_number(value).map_err(at)?),
                "xip-base" => xip_base = parse_number(value).map_err(at)?,
                "header" => header = parse_number(value).map_err(at)?,
                "vector-align" => vector_align = parse_number(value).map_err(at)?,
                "tlv-reserve" => tlv_reserve = parse_number(value).map_err(at)?,
                "app-max" => app_max = Some(parse_number(value).map_err(at)?),
                "loader-version" => loader_version = Some(parse_number(value).map_err(at)?),
                "trial-boots" => {
                    trial_boots = u8::try_from(parse_number(value).map_err(at)?)
                        .map_err(|_| at("`trial-boots` must be 0..=255".to_string()))?
                }
                "watchdog-ms" => watchdog_ms = parse_number(value).map_err(at)?,
                "mode" => {
                    mode = match value.split_once(char::is_whitespace) {
                        None if value == "overwrite" => Mode::Overwrite,
                        Some(("swap-move", range)) => Mode::SwapMove {
                            status: parse_region(range.trim()).map_err(at)?,
                        },
                        _ => {
                            return Err(at(format!(
                                "mode `{value}` is not `overwrite` or `swap-move <range>`"
                            )));
                        }
                    }
                }
                "env-prefix" => env_prefix = Some(value.to_string()),
                "key-file" => key_file = Some(base.join(value)),
                _ => return Err(at(format!("unknown key `{key}`"))),
            }
        }

        let missing = |what: &str| format!("missing `{what}`");
        Ok(Self {
            name: name.ok_or_else(|| missing("name"))?,
            flash: flash.ok_or_else(|| missing("flash"))?,
            sector: sector.ok_or_else(|| missing("sector"))?,
            loader: loader.ok_or_else(|| missing("loader"))?,
            primary: primary.ok_or_else(|| missing("primary"))?,
            secondary: secondary.ok_or_else(|| missing("secondary"))?,
            journal: journal.ok_or_else(|| missing("journal"))?,
            reserved,
            ram: ram.ok_or_else(|| missing("ram"))?,
            xip_base,
            header,
            vector_align,
            tlv_reserve,
            app_max,
            loader_version: loader_version.ok_or_else(|| missing("loader-version"))?,
            trial_boots,
            watchdog_ms,
            mode,
            env_prefix: env_prefix.ok_or_else(|| missing("env-prefix"))?,
            key_file,
        })
    }

    /// Slot size (both slots are equal).
    pub const fn slot_size(&self) -> u32 {
        self.primary.len()
    }

    /// Longest image (header through TLVs): the slot's last sector is the
    /// trailer.
    pub const fn max_image_len(&self) -> u32 {
        self.slot_size() - self.sector
    }

    /// The loader code region (without the loader block).
    pub const fn loader_code(&self) -> Region {
        Region {
            start: self.loader.start,
            end: self.loader.end - LOADER_BLOCK_LEN,
        }
    }

    pub const fn loader_block_addr(&self) -> u32 {
        self.loader.end - LOADER_BLOCK_LEN
    }

    pub const fn loader_info_addr(&self) -> u32 {
        self.loader.end - 32
    }

    /// Physical offset of the application's vector table / entry.
    pub const fn app_start(&self) -> u32 {
        self.primary.start + self.header
    }

    /// Memory address of physical flash offset `physical`.
    pub const fn mapped(&self, physical: u32) -> u32 {
        self.xip_base + physical
    }

    /// Longest linked application: the image minus header and TLV room,
    /// capped by `app-max`.
    pub fn app_max_len(&self) -> u32 {
        let fits = self.max_image_len() - self.header - self.tlv_reserve;
        self.app_max.map_or(fits, |cap| cap.min(fits))
    }

    /// The span [`BootFlash`](../zrs_boot_core/flash/struct.BootFlash.html)
    /// lets the loader write: from the lower to the end of the upper slot.
    pub fn writable_span(&self) -> Region {
        Region {
            start: self.primary.start.min(self.secondary.start),
            end: self.primary.end.max(self.secondary.end),
        }
    }

    /// Apply the environment: `<PREFIX>_BOOT_TRIAL_BOOTS` and
    /// `<PREFIX>_BOOT_WATCHDOG_MS` override the file's defaults. Returns the
    /// variable names consulted (for `cargo:rerun-if-env-changed`).
    pub fn resolve(&mut self, env: impl Fn(&str) -> Option<String>) -> Result<Vec<String>, String> {
        let trial = format!("{}_BOOT_TRIAL_BOOTS", self.env_prefix);
        let watchdog = format!("{}_BOOT_WATCHDOG_MS", self.env_prefix);
        if let Some(value) = env(&trial) {
            self.trial_boots = value
                .trim()
                .parse()
                .map_err(|_| format!("{trial}={value} is not 0..=255"))?;
        }
        if let Some(value) = env(&watchdog) {
            self.watchdog_ms =
                parse_number(&value).map_err(|error| format!("{watchdog}: {error}"))?;
        }
        Ok(vec![trial, watchdog])
    }

    /// Check the map. Every problem is reported, not just the first.
    pub fn check(&self, limits: &ChipLimits) -> Result<(), Vec<String>> {
        let mut problems = Vec::new();
        let mut fail = |problem: String| problems.push(problem);
        let sector = self.sector;

        if sector == 0 || !sector.is_power_of_two() {
            fail(format!("sector {sector:#x} is not a power of two"));
        }
        let mut regions = vec![
            ("loader", self.loader),
            ("primary", self.primary),
            ("secondary", self.secondary),
            ("journal", self.journal),
        ];
        if let Mode::SwapMove { status } = self.mode {
            regions.push(("swap status", status));
            if status.len() != 2 * sector {
                fail(format!(
                    "swap status {status:x?} must be exactly two sectors"
                ));
            }
        }
        for reserved in &self.reserved {
            regions.push((reserved.name.as_str(), reserved.region));
        }
        for (name, region) in &regions {
            if !self.flash.contains(region) {
                fail(format!(
                    "{name} {region:x?} is outside flash {:x?}",
                    self.flash
                ));
            }
        }
        for (index, (name, region)) in regions.iter().enumerate() {
            for (other, other_region) in &regions[index + 1..] {
                if region.overlaps(other_region) {
                    fail(format!(
                        "{name} {region:x?} overlaps {other} {other_region:x?}"
                    ));
                }
            }
        }
        for (name, region) in [
            ("primary", self.primary),
            ("secondary", self.secondary),
            ("journal", self.journal),
        ] {
            if sector != 0 && (region.start % sector != 0 || region.end % sector != 0) {
                fail(format!("{name} {region:x?} is not sector aligned"));
            }
        }
        if let Mode::SwapMove { status } = self.mode
            && sector != 0
            && status.start % sector != 0
        {
            fail(format!("swap status {status:x?} is not sector aligned"));
        }
        if self.primary.len() != self.secondary.len() {
            fail(format!(
                "slots differ in size: primary {:#x}, secondary {:#x}",
                self.primary.len(),
                self.secondary.len()
            ));
        }
        if self.primary.len() < 2 * sector {
            fail("a slot needs at least two sectors (image and trailer)".to_string());
        }
        // BootFlash admits the whole span between the slots.
        let span = self.writable_span();
        for (name, region) in &regions {
            if !matches!(*name, "primary" | "secondary" | "swap status") && span.overlaps(region) {
                fail(format!(
                    "{name} {region:x?} lies between the slots ({span:x?}), which the loader may write"
                ));
            }
        }
        if self.loader.len() <= LOADER_BLOCK_LEN || !self.loader.end.is_multiple_of(32) {
            fail(format!(
                "loader {:x?} cannot hold code and the 64-byte loader block at its end",
                self.loader
            ));
        }
        if self.header < 32 || !self.header.is_multiple_of(4) {
            fail(format!(
                "header {:#x} is not a valid imgtool header size",
                self.header
            ));
        }
        if limits.vtor {
            if !self.vector_align.is_power_of_two() || self.vector_align < 128 {
                fail(format!(
                    "vector-align {:#x} is not a VTOR alignment",
                    self.vector_align
                ));
            } else if !self
                .mapped(self.app_start())
                .is_multiple_of(self.vector_align)
            {
                fail(format!(
                    "application vector table {:#x} (primary + header) is not {:#x}-aligned for VTOR",
                    self.mapped(self.app_start()),
                    self.vector_align
                ));
            }
        }
        if self.max_image_len() <= self.header + self.tlv_reserve {
            fail("slots leave no room for an application".to_string());
        }
        if !self.ram.end.is_multiple_of(8) {
            fail(format!("ram end {:#x} is not 8-byte aligned", self.ram.end));
        }
        if limits.watchdog_max_ms == 0 && self.watchdog_ms != 0 {
            fail(format!(
                "{}: this chip's loader has no watchdog; watchdog-ms must be 0",
                self.name
            ));
        } else if self.watchdog_ms > limits.watchdog_max_ms {
            fail(format!(
                "watchdog-ms {} exceeds the chip's {} ms",
                self.watchdog_ms, limits.watchdog_max_ms
            ));
        }
        if problems.is_empty() {
            Ok(())
        } else {
            Err(problems)
        }
    }

    /// The trusted keys (see [`Keys`]): `<PREFIX>_OTA_KEY_XY` (or the layout's
    /// `key-file`), then the optional `<PREFIX>_OTA_KEY2_XY` and
    /// `<PREFIX>_OTA_KEY3_XY`. Each variable holds 128 hex digits (`X || Y`)
    /// or the path of a file that does. Returns the keys and every variable
    /// and file consulted, for `cargo:rerun-if-*`.
    pub fn keys(
        &self,
        env: impl Fn(&str) -> Option<String>,
        read: impl Fn(&Path) -> Result<String, String>,
    ) -> Result<Keys, String> {
        load_keys(
            &self.name,
            &self.env_prefix,
            self.key_file.as_deref(),
            env,
            read,
        )
    }

    /// Rust constants for `include!`. `mcuboot` is the path through which
    /// the consumer reaches the `mcuboot-image` crate (for example
    /// `zrs_boot_cortex_m::mcuboot_image`).
    pub fn rust(&self, keys: &[[u8; 64]], mcuboot: &str) -> String {
        let mut text = String::new();
        let _ = writeln!(
            text,
            "// Generated by zrs-boot-layout from the `{}` layout.",
            self.name
        );
        let mut constant = |name: &str, doc: &str, value: u32| {
            let _ = writeln!(text, "/// {doc}\npub const {name}: u32 = {value:#X};");
        };
        constant(
            "FLASH_START",
            "Start of the flash device (physical offset).",
            self.flash.start,
        );
        constant("FLASH_END", "End of the flash device.", self.flash.end);
        constant(
            "SECTOR_SIZE",
            "Largest erase unit of the slots.",
            self.sector,
        );
        constant("LOADER_START", "Loader region start.", self.loader.start);
        constant(
            "LOADER_END",
            "Loader region end (the loader block ends here).",
            self.loader.end,
        );
        constant(
            "LOADER_MAX_LEN",
            "Largest loader image: the region minus the loader block.",
            self.loader_code().len(),
        );
        constant(
            "LOADER_BLOCK_ADDR",
            "The 64-byte loader block (LoaderKeys, LoaderInfo).",
            self.loader_block_addr(),
        );
        constant(
            "LOADER_INFO_ADDR",
            "The LoaderInfo record.",
            self.loader_info_addr(),
        );
        constant("PRIMARY_SLOT", "Primary slot start.", self.primary.start);
        constant(
            "SECONDARY_SLOT",
            "Secondary slot start.",
            self.secondary.start,
        );
        constant("SLOT_SIZE", "Size of each slot.", self.slot_size());
        constant(
            "JOURNAL_START",
            "Product persistence journal start.",
            self.journal.start,
        );
        constant(
            "JOURNAL_END",
            "Product persistence journal end.",
            self.journal.end,
        );
        constant("RAM_START", "SRAM start.", self.ram.start);
        constant(
            "RAM_END",
            "SRAM end (initial stack pointer bound).",
            self.ram.end,
        );
        constant(
            "XIP_BASE",
            "Memory address of physical flash offset 0.",
            self.xip_base,
        );
        constant("IMAGE_HEADER_SIZE", "imgtool --header-size.", self.header);
        constant(
            "VECTOR_ALIGN",
            "Alignment of the application vector table.",
            self.vector_align,
        );
        constant(
            "TLV_RESERVE",
            "Room for the TLVs after the application.",
            self.tlv_reserve,
        );
        constant(
            "APP_START",
            "Physical offset of the application vector table.",
            self.app_start(),
        );
        constant(
            "APP_XIP_START",
            "Memory address of the application vector table.",
            self.mapped(self.app_start()),
        );
        constant(
            "APP_MAX_LEN",
            "Longest linked application.",
            self.app_max_len(),
        );
        constant(
            "LOADER_VERSION",
            "Version stamped into LoaderInfo.",
            self.loader_version,
        );
        constant(
            "BOOT_WATCHDOG_MS",
            "Loader watchdog period (0 = off).",
            self.watchdog_ms,
        );
        let _ = writeln!(
            text,
            "/// Boots an installed image may take before it counts as unconfirmed.\n\
             pub const BOOT_TRIAL_BOOTS: u8 = {};",
            self.trial_boots
        );
        let _ = writeln!(
            text,
            "/// Layout name.\npub const LAYOUT_NAME: &str = {:?};",
            self.name
        );
        let _ = writeln!(
            text,
            "/// Regions the loader never writes besides the journal and itself.\n\
             pub const RESERVED: [(&str, u32, u32); {}] = [",
            self.reserved.len()
        );
        for reserved in &self.reserved {
            let _ = writeln!(
                text,
                "    ({:?}, {:#X}, {:#X}),",
                reserved.name, reserved.region.start, reserved.region.end
            );
        }
        let _ = writeln!(text, "];");
        let _ = writeln!(
            text,
            "/// The two slots.\npub const MCUBOOT: {mcuboot}::Layout = {mcuboot}::Layout {{\n    \
             primary: PRIMARY_SLOT,\n    secondary: SECONDARY_SLOT,\n    slot_size: SLOT_SIZE,\n    \
             sector_size: SECTOR_SIZE,\n}};"
        );
        let _ = writeln!(
            text,
            "/// Every key the loader trusts; the first is named by LoaderInfo.\n\
             pub const TRUSTED_KEYS: [{mcuboot}::TrustedKey; {}] = [",
            keys.len()
        );
        for key in keys {
            let _ = writeln!(text, "    {mcuboot}::TrustedKey::new({key:?}),");
        }
        let _ = writeln!(text, "];");
        let mode = match self.mode {
            Mode::Overwrite => format!("{mcuboot}::Mode::Overwrite"),
            Mode::SwapMove { status } => {
                format!(
                    "{mcuboot}::Mode::SwapMove {{ status: {:#X} }}",
                    status.start
                )
            }
        };
        let _ = writeln!(
            text,
            "/// The loader's boot configuration.\n\
             pub const BOOT_CONFIG: {mcuboot}::BootConfig<'static> = {mcuboot}::BootConfig {{\n    \
             layout: MCUBOOT,\n    keys: &TRUSTED_KEYS,\n    trial_boots: BOOT_TRIAL_BOOTS,\n    \
             mode: {mode},\n}};"
        );
        text
    }

    /// Linker `MEMORY` and assertions for a `cortex-m-rt` loader: `FLASH` is
    /// the loader code region, `LOADER_INFO` the loader block (fill it with
    /// a `.loader_info` section), `RAM` the SRAM.
    pub fn loader_memory_x(&self) -> String {
        let code = self.loader_code();
        format!(
            "/* Generated by zrs-boot-layout from the `{name}` layout. */\n\
             MEMORY\n{{\n  \
               FLASH       : ORIGIN = {flash:#010X}, LENGTH = {flash_len:#X}\n  \
               LOADER_INFO : ORIGIN = {block:#010X}, LENGTH = {block_len}\n  \
               RAM         : ORIGIN = {ram:#010X}, LENGTH = {ram_len:#X}\n\
             }}\n\n\
             SECTIONS\n{{\n  .loader_info : {{ KEEP(*(.loader_info)) }} > LOADER_INFO\n}}\n\
             INSERT AFTER .rodata;\n\n\
             ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(LOADER_INFO),\n  \
               \"zrs-boot ({name}): the loader exceeds its {code_len}-byte region\");\n\
             ASSERT(SIZEOF(.loader_info) == {block_len},\n  \
               \"zrs-boot ({name}): the loader block must be {block_len} bytes\");\n",
            name = self.name,
            flash = self.mapped(code.start),
            flash_len = code.len(),
            code_len = code.len(),
            block = self.mapped(self.loader_block_addr()),
            block_len = LOADER_BLOCK_LEN,
            ram = self.ram.start,
            ram_len = self.ram.len(),
        )
    }

    /// Linker `MEMORY` for the application under the loader: `FLASH` starts
    /// at the application vector table in the primary slot and is
    /// [`app_max_len`](Self::app_max_len) long. Also defines
    /// `_security_partition_start`/`_end` (the journal) and
    /// `_bootloader_start` (the loader region) for product assertions.
    pub fn app_memory_x(&self) -> String {
        format!(
            "/* Generated by zrs-boot-layout from the `{name}` layout. */\n\
             MEMORY\n{{\n  \
               FLASH : ORIGIN = {app:#010X}, LENGTH = {app_len:#X}\n  \
               RAM   : ORIGIN = {ram:#010X}, LENGTH = {ram_len:#X}\n\
             }}\n\n\
             _application_start = ORIGIN(FLASH);\n\
             _application_end = ORIGIN(FLASH) + LENGTH(FLASH);\n\
             _security_partition_start = {journal:#010X};\n\
             _security_partition_end = {journal_end:#010X};\n\
             _bootloader_start = {loader:#010X};\n\
             _boot_state_start = {loader_end:#010X};\n",
            name = self.name,
            app = self.mapped(self.app_start()),
            app_len = self.app_max_len(),
            ram = self.ram.start,
            ram_len = self.ram.len(),
            journal = self.journal.start,
            journal_end = self.journal.end,
            loader = self.loader.start,
            loader_end = self.loader.end,
        )
    }
}

fn parse_xy(origin: &str, text: &str) -> Result<[u8; 64], String> {
    let digits: String = text.chars().filter(|c| !c.is_whitespace()).collect();
    if digits.len() != 128 {
        return Err(format!(
            "{origin} must hold 64 hex-encoded bytes (P-256 X || Y)"
        ));
    }
    let mut xy = [0u8; 64];
    for (index, byte) in xy.iter_mut().enumerate() {
        *byte = u8::from_str_radix(&digits[2 * index..2 * index + 2], 16)
            .map_err(|_| format!("{origin} is not hex"))?;
    }
    Ok(xy)
}

/// The trusted keys of a layout or app-verify configuration named `name`:
/// `<prefix>_OTA_KEY_XY` (or `key_file`), then the optional
/// `<prefix>_OTA_KEY2_XY` and `<prefix>_OTA_KEY3_XY`.
fn load_keys(
    name: &str,
    prefix: &str,
    key_file: Option<&Path>,
    env: impl Fn(&str) -> Option<String>,
    read: impl Fn(&Path) -> Result<String, String>,
) -> Result<Keys, String> {
    let mut vars = Vec::new();
    let mut files = Vec::new();
    let load = |var: String,
                vars: &mut Vec<String>,
                files: &mut Vec<PathBuf>|
     -> Result<Option<[u8; 64]>, String> {
        let value = env(&var);
        vars.push(var.clone());
        let Some(value) = value else {
            return Ok(None);
        };
        let digits: String = value.chars().filter(|c| !c.is_whitespace()).collect();
        if digits.len() == 128 && digits.chars().all(|c| c.is_ascii_hexdigit()) {
            return parse_xy(&var, &digits).map(Some);
        }
        let path = PathBuf::from(value.trim());
        let text = read(&path).map_err(|error| format!("{var}={}: {error}", path.display()))?;
        files.push(path);
        parse_xy(&var, &text).map(Some)
    };

    let first = match load(format!("{prefix}_OTA_KEY_XY"), &mut vars, &mut files)? {
        Some(key) => key,
        None => {
            let path = key_file.ok_or_else(|| {
                format!(
                    "{}: no trusted key: set {prefix}_OTA_KEY_XY (128 hex digits or a file) or `key-file`",
                    name
                )
            })?;
            let text = read(path).map_err(|error| format!("{}: {error}", path.display()))?;
            files.push(path.to_path_buf());
            parse_xy(&path.display().to_string(), &text)?
        }
    };
    let mut keys = vec![first];
    for index in 2..=MAX_TRUSTED_KEYS {
        if let Some(key) = load(format!("{prefix}_OTA_KEY{index}_XY"), &mut vars, &mut files)? {
            if keys.contains(&key) {
                return Err(format!("{prefix}_OTA_KEY{index}_XY repeats a trusted key"));
            }
            keys.push(key);
        }
    }
    Ok(Keys { keys, vars, files })
}

/// What a product build script needs: the checked layout, its keys and the
/// generated texts. Prints the `cargo:` directives and panics with every
/// problem on an invalid layout, so a bad map fails the build.
pub struct Build {
    pub layout: BootLayout,
    pub keys: Vec<[u8; 64]>,
}

impl Build {
    /// Load `path` (relative to the calling crate's manifest directory) for
    /// a build script.
    pub fn load(path: impl AsRef<Path>, limits: ChipLimits) -> Self {
        let manifest = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").expect("build script"));
        let path = manifest.join(path);
        println!("cargo:rerun-if-changed={}", path.display());
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|error| panic!("read {}: {error}", path.display()));
        let base = path.parent().unwrap_or(Path::new("."));
        let env = |var: &str| std::env::var(var).ok();
        let mut layout = BootLayout::parse(&text, base)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        let vars = layout
            .resolve(env)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        for var in vars {
            println!("cargo:rerun-if-env-changed={var}");
        }
        if let Err(problems) = layout.check(&limits) {
            panic!(
                "{}: invalid boot layout:\n  {}",
                path.display(),
                problems.join("\n  ")
            );
        }
        let read = |path: &Path| std::fs::read_to_string(path).map_err(|error| error.to_string());
        let Keys { keys, vars, files } = layout
            .keys(env, read)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        for var in vars {
            println!("cargo:rerun-if-env-changed={var}");
        }
        for file in files {
            println!("cargo:rerun-if-changed={}", file.display());
        }
        Self { layout, keys }
    }

    /// Write `boot_layout.rs` (for `include!(concat!(env!("OUT_DIR"),
    /// "/boot_layout.rs"))`) into `OUT_DIR`.
    pub fn write_rust(&self, mcuboot: &str) {
        write_out("boot_layout.rs", &self.layout.rust(&self.keys, mcuboot));
    }

    /// Write the loader's `memory.x` into `OUT_DIR` and add it to the link
    /// search path.
    pub fn write_loader_memory_x(&self) {
        write_out("memory.x", &self.layout.loader_memory_x());
        link_search_out_dir();
    }

    /// Write the application's `memory.x` into `OUT_DIR` and add it to the
    /// link search path.
    pub fn write_app_memory_x(&self) {
        write_out("memory.x", &self.layout.app_memory_x());
        link_search_out_dir();
    }
}

fn out_dir() -> PathBuf {
    PathBuf::from(std::env::var_os("OUT_DIR").expect("build script"))
}

fn write_out(name: &str, text: &str) {
    let path = out_dir().join(name);
    std::fs::write(&path, text).unwrap_or_else(|error| panic!("write {}: {error}", path.display()));
}

fn link_search_out_dir() {
    println!("cargo:rustc-link-search={}", out_dir().display());
}

#[cfg(test)]
mod tests;

/// Trust configuration for app-side envelope verification on platforms whose
/// ROM or SDK boot chain stays (no zigbee-rs loader, so no flash layout):
///
/// ```text
/// name       = esp32-devkit
/// env-prefix = ESP32_DEVKIT      # <prefix>_OTA_KEY{,2,3}_XY override/add keys
/// key-file   = ../../../bootloaders/keys/own-ec-p256.xy.hex
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AppVerify {
    pub name: String,
    pub env_prefix: String,
    pub key_file: Option<PathBuf>,
}

impl AppVerify {
    /// Parse the configuration text. `base` resolves a relative `key-file`.
    pub fn parse(text: &str, base: &Path) -> Result<Self, String> {
        let (mut name, mut env_prefix, mut key_file) = (None, None, None);
        for (number, line) in text.lines().enumerate() {
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let at = |error: String| format!("line {}: {error}", number + 1);
            let (key, value) = line
                .split_once('=')
                .ok_or_else(|| at(format!("`{line}` is not `key = value`")))?;
            let (key, value) = (key.trim(), value.trim());
            let slot = match key {
                "name" => &mut name,
                "env-prefix" => &mut env_prefix,
                "key-file" => &mut key_file,
                _ => return Err(at(format!("unknown key `{key}`"))),
            };
            if slot.replace(value.to_string()).is_some() {
                return Err(at(format!("`{key}` given twice")));
            }
        }
        let name = name.ok_or("missing `name`")?;
        let env_prefix = env_prefix.ok_or_else(|| format!("{name}: missing `env-prefix`"))?;
        if env_prefix.is_empty()
            || !env_prefix
                .chars()
                .all(|c| c.is_ascii_uppercase() || c.is_ascii_digit() || c == '_')
        {
            return Err(format!("{name}: `env-prefix` must be A-Z, 0-9 and `_`"));
        }
        Ok(Self {
            name,
            env_prefix,
            key_file: key_file.map(|file| base.join(file)),
        })
    }

    /// The trusted keys, as [`BootLayout::keys`].
    pub fn keys(
        &self,
        env: impl Fn(&str) -> Option<String>,
        read: impl Fn(&Path) -> Result<String, String>,
    ) -> Result<Keys, String> {
        load_keys(
            &self.name,
            &self.env_prefix,
            self.key_file.as_deref(),
            env,
            read,
        )
    }

    /// Rust constants for `include!`: `TRUSTED_KEYS`. `mcuboot` is the path
    /// to the `mcuboot-image` crate.
    pub fn rust(&self, keys: &[[u8; 64]], mcuboot: &str) -> String {
        let mut text = format!(
            "// Generated by zrs-boot-layout from the `{}` app-verify configuration.\n\
             /// Keys whose envelope signatures this application accepts.\n\
             pub const TRUSTED_KEYS: [{mcuboot}::TrustedKey; {}] = [\n",
            self.name,
            keys.len()
        );
        for key in keys {
            let _ = writeln!(text, "    {mcuboot}::TrustedKey::new({key:?}),");
        }
        text.push_str("];\n");
        text
    }

    /// Build-script entry: load `path` (relative to the calling crate's
    /// manifest directory), print the `cargo:` directives and write
    /// `app_verify.rs` into `OUT_DIR`. Panics on any problem.
    pub fn build(path: impl AsRef<Path>, mcuboot: &str) {
        let manifest = PathBuf::from(std::env::var_os("CARGO_MANIFEST_DIR").expect("build script"));
        let path = manifest.join(path);
        println!("cargo:rerun-if-changed={}", path.display());
        let text = std::fs::read_to_string(&path)
            .unwrap_or_else(|error| panic!("read {}: {error}", path.display()));
        let config = Self::parse(&text, path.parent().unwrap_or(Path::new(".")))
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        let read = |path: &Path| std::fs::read_to_string(path).map_err(|error| error.to_string());
        let Keys { keys, vars, files } = config
            .keys(|var| std::env::var(var).ok(), read)
            .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
        for var in vars {
            println!("cargo:rerun-if-env-changed={var}");
        }
        for file in files {
            println!("cargo:rerun-if-changed={}", file.display());
        }
        write_out("app_verify.rs", &config.rust(&keys, mcuboot));
    }
}
