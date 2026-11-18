//! Flash layouts of the ESP32-H2 ZRHP NCP (spec 8.9).
//!
//! Two partition tables exist, one per module variant: 4 MB
//! (`partitions/esp32h2-ncp-4mb.csv`, default) and 2 MB
//! (`partitions/esp32h2-ncp-2mb.csv`, feature `flash-2mb`). An image is
//! built for one of them; at boot [`crate::flash_size`] compares the
//! fitted flash with [`ACTIVE.flash_end()`](Layout::flash_end) and refuses
//! to start the stack on a smaller part.
//!
//! `zbnv` holds every NCP journal at the same relative offsets in both
//! layouts ([`Nv`]). The tests parse both CSVs and prove the constants agree
//! with them, that no region overlaps another, and that every journal
//! window is sector-aligned and inside `zbnv`.

/// Flash sector (erase) size.
pub const SECTOR_SIZE: u32 = 4096;
const KIB: u32 = 1024;

/// One partition-table layout.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Layout {
    /// Flash size the table is made for.
    pub flash_size: u32,
    pub otadata_offset: u32,
    pub otadata_size: u32,
    pub ota_0_offset: u32,
    pub ota_1_offset: u32,
    pub ota_slot_size: u32,
    pub zbnv_offset: u32,
    pub zbnv_size: u32,
}

impl Layout {
    /// First byte after the last partition.
    pub const fn flash_end(&self) -> u32 {
        self.zbnv_offset + self.zbnv_size
    }

    /// Absolute address of a `zbnv`-relative offset.
    pub const fn nv(&self, relative: u32) -> u32 {
        self.zbnv_offset + relative
    }
}

pub const LAYOUT_4MB: Layout = Layout {
    flash_size: 4 * 1024 * KIB,
    otadata_offset: 0x9000,
    otadata_size: 0x2000,
    ota_0_offset: 0x1_0000,
    ota_1_offset: 0x1F_0000,
    ota_slot_size: 0x1E_0000,
    zbnv_offset: 0x3D_0000,
    zbnv_size: 192 * KIB,
};

pub const LAYOUT_2MB: Layout = Layout {
    flash_size: 2 * 1024 * KIB,
    otadata_offset: 0x9000,
    otadata_size: 0x2000,
    ota_0_offset: 0x1_0000,
    ota_1_offset: 0xF_0000,
    ota_slot_size: 0xE_0000,
    zbnv_offset: 0x1D_0000,
    zbnv_size: 192 * KIB,
};

/// The layout this image is built for.
#[cfg(not(feature = "flash-2mb"))]
pub const ACTIVE: Layout = LAYOUT_4MB;
#[cfg(feature = "flash-2mb")]
pub const ACTIVE: Layout = LAYOUT_2MB;

// ── Firmware update view of [`ACTIVE`] (spec 7.9) ──────────────────────
//
// The names below are the ones `products/esp32-zigbee-devkit/src/
// running_image.rs` expects from `crate::layout`; this crate compiles that
// file against the NCP layout (see `crate::fw_flash`).

/// Address of the ESP-IDF partition table.
pub const PARTITION_TABLE_OFFSET: u32 = 0x8000;
/// Flash program granularity of the staging writer.
pub const WORD_SIZE: u32 = 4;
/// Physical flash size the active partition table is made for.
pub const FLASH_SIZE: u32 = ACTIVE.flash_size;
/// Application slots the ESP-IDF bootloader alternates between.
pub const OTA_SLOT_COUNT: u8 = 2;
/// Size of each application slot of the active layout.
pub const OTA_SLOT_SIZE: u32 = ACTIVE.ota_slot_size;

/// Base address of application slot `slot` (0 = `ota_0`, else `ota_1`).
pub const fn ota_slot_offset(slot: u8) -> u32 {
    match slot {
        0 => ACTIVE.ota_0_offset,
        _ => ACTIVE.ota_1_offset,
    }
}

/// Base address of `otadata` sector `sector` (0 or 1).
pub const fn otadata_sector_offset(sector: u8) -> u32 {
    ACTIVE.otadata_offset + sector as u32 * SECTOR_SIZE
}

/// Whether firmware staging may erase or program `[address, address +
/// len)`: only `otadata` and the two application slots. The bootloader,
/// the partition table and every `zbnv` journal (including the identity
/// item) are outside, and this is checked on every flash operation of the
/// staging path rather than trusting the offset arithmetic above it.
pub const fn is_fw_writable(address: u32, len: u32) -> bool {
    let Some(end) = address.checked_add(len) else {
        return false;
    };
    if len == 0 {
        return false;
    }
    let windows = [
        (
            ACTIVE.otadata_offset,
            ACTIVE.otadata_offset + ACTIVE.otadata_size,
        ),
        (
            ACTIVE.ota_0_offset,
            ACTIVE.ota_0_offset + ACTIVE.ota_slot_size,
        ),
        (
            ACTIVE.ota_1_offset,
            ACTIVE.ota_1_offset + ACTIVE.ota_slot_size,
        ),
    ];
    let mut index = 0;
    while index < windows.len() {
        if address >= windows[index].0 && end <= windows[index].1 {
            return true;
        }
        index += 1;
    }
    false
}

/// `zbnv`-relative journal windows (136 KiB used, 56 KiB spare).
///
/// Each window holds two logical sectors (A at the window start, B at its
/// middle). Where today's runtime journal uses smaller sectors than the
/// window half (Trust Center devices, GP), the rest of the half is reserved
/// for the planned streamed snapshots and 1 KiB slots (spec 8.9).
pub struct Nv;

impl Nv {
    /// `SecurityStateJournal`: 2 × 16 KiB (NWK + global APS reservations,
    /// keys, APS replay floors).
    pub const SECURITY: (u32, u32) = (0, 32 * KIB);
    /// `SecurityStateJournal` logical sector.
    pub const SECURITY_SECTOR: usize = 16 * 1024;
    /// `TrustCenterDeviceJournal`: 2 × 32 KiB reserved; today's journal
    /// uses the first 4 KiB of each half.
    pub const TRUST_CENTER: (u32, u32) = (32 * KIB, 64 * KIB);
    /// `ChildTableJournal`: 2 × 4 KiB.
    pub const CHILDREN: (u32, u32) = (96 * KIB, 8 * KIB);
    /// GP journal (Proxy + Sink tables): 2 × 8 KiB.
    pub const GREEN_POWER: (u32, u32) = (104 * KIB, 16 * KIB);
    /// GP journal logical sector.
    pub const GREEN_POWER_SECTOR: usize = 8 * 1024;
    /// APS bindings, groups and application keys: 2 × 4 KiB.
    pub const APS_TABLES: (u32, u32) = (120 * KIB, 8 * KIB);
    /// Identity and settings (IEEE override, UART default): 2 × 4 KiB.
    pub const IDENTITY: (u32, u32) = (128 * KIB, 8 * KIB);
    /// Bytes of `zbnv` the journals use.
    pub const USED: u32 = 136 * KIB;

    pub const ALL: [(&'static str, (u32, u32)); 6] = [
        ("security", Self::SECURITY),
        ("trust-center", Self::TRUST_CENTER),
        ("children", Self::CHILDREN),
        ("green-power", Self::GREEN_POWER),
        ("aps-tables", Self::APS_TABLES),
        ("identity", Self::IDENTITY),
    ];
}

const fn check(layout: Layout) -> bool {
    let mut ok = layout.flash_end() <= layout.flash_size
        && layout.otadata_offset + layout.otadata_size <= layout.ota_0_offset
        && layout.ota_0_offset + layout.ota_slot_size <= layout.ota_1_offset
        && layout.ota_1_offset + layout.ota_slot_size <= layout.zbnv_offset
        && layout.ota_0_offset.is_multiple_of(0x1_0000)
        && layout.ota_1_offset.is_multiple_of(0x1_0000)
        && layout.zbnv_offset.is_multiple_of(SECTOR_SIZE)
        && Nv::USED <= layout.zbnv_size;
    let mut index = 0;
    let mut end = 0;
    while index < Nv::ALL.len() {
        let (start, size) = Nv::ALL[index].1;
        ok = ok
            && start == end
            && start.is_multiple_of(SECTOR_SIZE)
            && size.is_multiple_of(2 * SECTOR_SIZE)
            && start + size <= layout.zbnv_size;
        end = start + size;
        index += 1;
    }
    ok && end == Nv::USED
}

const _: () = assert!(check(LAYOUT_4MB));
const _: () = assert!(check(LAYOUT_2MB));
const _: () = assert!(Nv::SECURITY.1 as usize == 2 * Nv::SECURITY_SECTOR);
const _: () = assert!(Nv::GREEN_POWER.1 as usize == 2 * Nv::GREEN_POWER_SECTOR);

#[cfg(test)]
mod tests {
    use super::*;

    struct Row {
        name: String,
        offset: u32,
        size: u32,
    }

    fn parse_hex(text: &str) -> u32 {
        u32::from_str_radix(text.trim().trim_start_matches("0x"), 16).unwrap()
    }

    fn parse(csv: &str) -> Vec<Row> {
        csv.lines()
            .map(str::trim)
            .filter(|line| !line.is_empty() && !line.starts_with('#'))
            .map(|line| {
                let fields: Vec<&str> = line.split(',').map(str::trim).collect();
                Row {
                    name: fields[0].to_string(),
                    offset: parse_hex(fields[3]),
                    size: parse_hex(fields[4]),
                }
            })
            .collect()
    }

    fn agrees(layout: Layout, csv: &str) {
        let rows = parse(csv);
        let find = |name: &str| rows.iter().find(|row| row.name == name).unwrap();
        assert_eq!(
            (find("otadata").offset, find("otadata").size),
            (layout.otadata_offset, layout.otadata_size)
        );
        assert_eq!(
            (find("ota_0").offset, find("ota_0").size),
            (layout.ota_0_offset, layout.ota_slot_size)
        );
        assert_eq!(
            (find("ota_1").offset, find("ota_1").size),
            (layout.ota_1_offset, layout.ota_slot_size)
        );
        assert_eq!(
            (find("zbnv").offset, find("zbnv").size),
            (layout.zbnv_offset, layout.zbnv_size)
        );
        assert_eq!(rows.len(), 4);
        assert_eq!(
            rows.last().unwrap().offset + rows.last().unwrap().size,
            layout.flash_end()
        );
    }

    #[test]
    fn the_4mb_constants_match_the_csv() {
        agrees(
            LAYOUT_4MB,
            include_str!("../partitions/esp32h2-ncp-4mb.csv"),
        );
        assert_eq!(LAYOUT_4MB.flash_end(), LAYOUT_4MB.flash_size);
    }

    #[test]
    fn the_2mb_constants_match_the_csv() {
        agrees(
            LAYOUT_2MB,
            include_str!("../partitions/esp32h2-ncp-2mb.csv"),
        );
        assert_eq!(LAYOUT_2MB.flash_end(), LAYOUT_2MB.flash_size);
    }

    #[test]
    fn layouts_are_consistent() {
        assert!(check(LAYOUT_4MB));
        assert!(check(LAYOUT_2MB));
    }

    #[test]
    fn journals_are_contiguous_disjoint_and_leave_the_spare() {
        let mut end = 0;
        for (_, (start, size)) in Nv::ALL {
            assert_eq!(start, end);
            end = start + size;
        }
        assert_eq!(end, 136 * KIB);
        assert_eq!(LAYOUT_4MB.zbnv_size - end, 56 * KIB);
    }

    #[test]
    fn the_ncp_journals_never_touch_the_devkit_security_journal() {
        // products/esp32-zigbee-devkit keeps its security journal (and the
        // frame-counter reservation of a joined router/sensor) at
        // 0x3FE000..0x400000. On 4 MB parts that lies in the NCP's spare
        // `zbnv` tail, so moving a board devkit → NCP → devkit never
        // rewinds the devkit image's counters. The rest of the old `zbnv`
        // and ota_1 contents are foreign data to the NCP journals: NCP
        // provisioning erases the NCP `zbnv` once (README).
        const { assert!(LAYOUT_4MB.nv(Nv::USED) <= 0x3F_E000) };
        assert_eq!(LAYOUT_4MB.flash_end(), 0x40_0000);
    }

    #[test]
    fn staging_never_reaches_the_journals_the_identity_item_or_the_bootloader() {
        // Spec 7.9 "Protected regions".
        assert!(!is_fw_writable(0, SECTOR_SIZE));
        assert!(!is_fw_writable(PARTITION_TABLE_OFFSET, SECTOR_SIZE));
        assert!(!is_fw_writable(ACTIVE.zbnv_offset, SECTOR_SIZE));
        assert!(!is_fw_writable(ACTIVE.nv(Nv::IDENTITY.0), SECTOR_SIZE));
        assert!(!is_fw_writable(
            ACTIVE.nv(Nv::IDENTITY.0 + SECTOR_SIZE),
            SECTOR_SIZE
        ));
        // A range straddling the end of ota_1 into zbnv.
        assert!(!is_fw_writable(ACTIVE.zbnv_offset - 4, 8));
        assert!(!is_fw_writable(u32::MAX - 3, 8));
        assert!(!is_fw_writable(ota_slot_offset(0), 0));
        for slot in 0..OTA_SLOT_COUNT {
            let base = ota_slot_offset(slot);
            assert!(is_fw_writable(base, SECTOR_SIZE));
            assert!(is_fw_writable(
                base + OTA_SLOT_SIZE - SECTOR_SIZE,
                SECTOR_SIZE
            ));
        }
        assert!(is_fw_writable(otadata_sector_offset(0), SECTOR_SIZE));
        assert!(is_fw_writable(otadata_sector_offset(1), SECTOR_SIZE));
        assert!(!is_fw_writable(otadata_sector_offset(2), SECTOR_SIZE));
    }

    #[test]
    fn the_app_slots_hold_the_estimated_image() {
        // Spec 8.9: the NCP image is estimated at 650–700 KB.
        const { assert!(LAYOUT_2MB.ota_slot_size >= 700 * 1000) };
    }
}
