//! Flash layout of the 4 MiB ESP32-C6/H2 Zigbee devkit partition table.
//!
//! This is C6-only product policy: the ESP32-H2 build keeps the default
//! single-app partition table and has no OTA writer (see the crate docs).
//!
//! The single source of truth for the addresses is
//! `partitions/esp32-4mb-ota.csv`; the constants below mirror it and the unit
//! tests at the bottom of this file parse the CSV to prove they still agree.

// `esp32_zigbee_devkit::flash` (the board crate) defines the same physical
// chip constants, but it is `target_os = "none"`-gated so its ROM flash
// driver dependency does not leak onto the host; this module's unit tests
// (including the CSV round trip below) must run on the host, so the values
// are mirrored here instead of re-exported. They are physical facts of the
// flash chip fitted on every supported devkit and do not change per board
// revision.

/// Total flash size of the supported devkits.
pub const FLASH_SIZE: u32 = 0x0040_0000;

/// Flash sector (erase) size.
pub const SECTOR_SIZE: u32 = 4096;

/// Flash word (program) size.
pub const WORD_SIZE: u32 = 4;

/// Address of the ESP-IDF partition table.
pub const PARTITION_TABLE_OFFSET: u32 = 0x0000_8000;

/// Size of the ESP-IDF partition table binary.
pub const PARTITION_TABLE_SIZE: u32 = 0x0000_0C00;

/// Size of one ESP-IDF partition-table entry.
pub(crate) const PARTITION_ENTRY_SIZE: usize = 32;

#[derive(Clone, Copy)]
pub(crate) struct PartitionSpec {
    partition_type: u8,
    subtype: u8,
    offset: u32,
    size: u32,
    label: &'static [u8],
}

impl PartitionSpec {
    const fn new(
        partition_type: u8,
        subtype: u8,
        offset: u32,
        size: u32,
        label: &'static [u8],
    ) -> Self {
        Self {
            partition_type,
            subtype,
            offset,
            size,
            label,
        }
    }

    pub(crate) fn matches(self, entry: &[u8; PARTITION_ENTRY_SIZE]) -> bool {
        if entry[..2] != [0xAA, 0x50]
            || entry[2] != self.partition_type
            || entry[3] != self.subtype
            || entry[4..8] != self.offset.to_le_bytes()
            || entry[8..12] != self.size.to_le_bytes()
            || entry[28..32] != [0; 4]
            || self.label.len() > 16
        {
            return false;
        }

        let mut label = [0u8; 16];
        label[..self.label.len()].copy_from_slice(self.label);
        entry[12..28] == label
    }

    #[cfg(test)]
    pub(crate) fn encode(self) -> [u8; PARTITION_ENTRY_SIZE] {
        let mut entry = [0u8; PARTITION_ENTRY_SIZE];
        entry[..2].copy_from_slice(&[0xAA, 0x50]);
        entry[2] = self.partition_type;
        entry[3] = self.subtype;
        entry[4..8].copy_from_slice(&self.offset.to_le_bytes());
        entry[8..12].copy_from_slice(&self.size.to_le_bytes());
        entry[12..12 + self.label.len()].copy_from_slice(self.label);
        entry
    }
}

/// `otadata` — two redundant sectors of boot-slot selection entries.
pub const OTADATA_OFFSET: u32 = 0x0000_9000;
/// Size of `otadata` (two sectors).
pub const OTADATA_SIZE: u32 = 0x0000_2000;

/// `ota_0` — first application slot (where the pre-OTA firmware already runs).
pub const OTA_0_OFFSET: u32 = 0x0001_0000;

/// `ota_1` — second application slot.
pub const OTA_1_OFFSET: u32 = 0x0020_0000;

/// Size of each application slot.
pub const OTA_SLOT_SIZE: u32 = 0x001F_0000;

/// Number of application slots the bootloader alternates between.
pub const OTA_SLOT_COUNT: u8 = 2;

/// `zbnv` — data partition reserved for Zigbee persistence.
pub const ZBNV_OFFSET: u32 = 0x003F_0000;
/// Size of `zbnv`.
pub const ZBNV_SIZE: u32 = 0x0001_0000;

/// Start of the log-structured Zigbee NV storage.
///
/// Deliberately the *last* 8 KiB of `zbnv`: these are the exact addresses used
/// before the partition table existed, so introducing the table does not move
/// any joined-network state.
pub const NV_OFFSET: u32 = 0x003F_E000;

/// Size of the log-structured Zigbee NV storage (two pages).
pub const NV_SIZE: u32 = 0x0000_2000;

/// Exact entries required by the OTA writer and the ESP-IDF bootloader.
pub(crate) const EXPECTED_PARTITIONS: [PartitionSpec; 4] = [
    PartitionSpec::new(0x01, 0x00, OTADATA_OFFSET, OTADATA_SIZE, b"otadata"),
    PartitionSpec::new(0x00, 0x10, OTA_0_OFFSET, OTA_SLOT_SIZE, b"ota_0"),
    PartitionSpec::new(0x00, 0x11, OTA_1_OFFSET, OTA_SLOT_SIZE, b"ota_1"),
    PartitionSpec::new(0x01, 0x06, ZBNV_OFFSET, ZBNV_SIZE, b"zbnv"),
];

/// Offset of NV page A inside [`NV_OFFSET`].
pub const NV_PAGE_A: u32 = 0;
/// Offset of NV page B inside [`NV_OFFSET`].
pub const NV_PAGE_B: u32 = SECTOR_SIZE;

/// Base address of an application slot.
pub const fn ota_slot_offset(slot: u8) -> u32 {
    match slot {
        0 => OTA_0_OFFSET,
        _ => OTA_1_OFFSET,
    }
}

/// Base address of an `otadata` sector (0 or 1).
pub const fn otadata_sector_offset(sector: u8) -> u32 {
    OTADATA_OFFSET + (sector as u32) * SECTOR_SIZE
}

/// Whether `[address, address + len)` may be erased or programmed by the OTA
/// code.
///
/// Only the two application slots and `otadata` are writable. The bootloader,
/// the partition table and the Zigbee NV pages must never be touched by an
/// upgrade, so this is checked on every flash operation instead of trusting
/// the offset arithmetic above it.
pub fn is_ota_writable(address: u32, len: u32) -> bool {
    let Some(end) = address.checked_add(len) else {
        return false;
    };
    if len == 0 || end > FLASH_SIZE {
        return false;
    }
    let windows = [
        (OTADATA_OFFSET, OTADATA_OFFSET + OTADATA_SIZE),
        (OTA_0_OFFSET, OTA_0_OFFSET + OTA_SLOT_SIZE),
        (OTA_1_OFFSET, OTA_1_OFFSET + OTA_SLOT_SIZE),
    ];
    windows
        .iter()
        .any(|(start, stop)| address >= *start && end <= *stop)
}

// ── Layout invariants, checked at compile time ──────────────────────────────

const _: () = assert!(PARTITION_TABLE_OFFSET + PARTITION_TABLE_SIZE <= OTADATA_OFFSET);
const _: () = assert!(OTADATA_OFFSET.is_multiple_of(SECTOR_SIZE));
const _: () = assert!(OTADATA_SIZE == 2 * SECTOR_SIZE);
const _: () = assert!(OTADATA_OFFSET + OTADATA_SIZE <= OTA_0_OFFSET);
// App partitions must start on a 64 KiB boundary (MMU page size).
const _: () = assert!(OTA_0_OFFSET.is_multiple_of(0x1_0000));
const _: () = assert!(OTA_1_OFFSET.is_multiple_of(0x1_0000));
const _: () = assert!(OTA_SLOT_SIZE.is_multiple_of(SECTOR_SIZE));
const _: () = assert!(OTA_0_OFFSET + OTA_SLOT_SIZE <= OTA_1_OFFSET);
const _: () = assert!(OTA_1_OFFSET + OTA_SLOT_SIZE <= ZBNV_OFFSET);
const _: () = assert!(ZBNV_OFFSET + ZBNV_SIZE == FLASH_SIZE);
// The NV pages must live inside `zbnv`, at its very end.
const _: () = assert!(NV_OFFSET >= ZBNV_OFFSET);
const _: () = assert!(NV_OFFSET.is_multiple_of(SECTOR_SIZE));
const _: () = assert!(NV_SIZE == 2 * SECTOR_SIZE);
const _: () = assert!(NV_OFFSET + NV_SIZE == ZBNV_OFFSET + ZBNV_SIZE);
// Untouched pre-partition-table NV placement.
const _: () = assert!(NV_OFFSET == 0x003F_E000);

#[cfg(test)]
mod tests {
    use super::*;

    const CSV: &str = include_str!("../partitions/esp32-4mb-ota.csv");

    #[derive(Debug, PartialEq, Eq)]
    struct Row {
        name: String,
        kind: String,
        subtype: String,
        offset: u32,
        size: u32,
    }

    fn parse_csv() -> Vec<Row> {
        CSV.lines()
            .map(|line| line.split('#').next().unwrap_or("").trim())
            .filter(|line| !line.is_empty())
            .map(|line| {
                let cells: Vec<&str> = line.split(',').map(str::trim).collect();
                assert!(cells.len() >= 5, "malformed CSV row: {line}");
                let number = |cell: &str| {
                    let cell = cell.trim();
                    cell.strip_prefix("0x")
                        .map(|hex| u32::from_str_radix(hex, 16))
                        .unwrap_or_else(|| cell.parse())
                        .unwrap_or_else(|_| panic!("bad number: {cell}"))
                };
                Row {
                    name: cells[0].to_owned(),
                    kind: cells[1].to_owned(),
                    subtype: cells[2].to_owned(),
                    offset: number(cells[3]),
                    size: number(cells[4]),
                }
            })
            .collect()
    }

    #[test]
    fn constants_match_checked_in_partition_csv() {
        let rows = parse_csv();
        assert_eq!(rows.len(), 4, "unexpected partition count: {rows:?}");

        assert_eq!(rows[0].name, "otadata");
        assert_eq!(
            (rows[0].kind.as_str(), rows[0].subtype.as_str()),
            ("data", "ota")
        );
        assert_eq!(
            (rows[0].offset, rows[0].size),
            (OTADATA_OFFSET, OTADATA_SIZE)
        );

        assert_eq!(rows[1].name, "ota_0");
        assert_eq!(
            (rows[1].kind.as_str(), rows[1].subtype.as_str()),
            ("app", "ota_0")
        );
        assert_eq!(
            (rows[1].offset, rows[1].size),
            (OTA_0_OFFSET, OTA_SLOT_SIZE)
        );

        assert_eq!(rows[2].name, "ota_1");
        assert_eq!(
            (rows[2].kind.as_str(), rows[2].subtype.as_str()),
            ("app", "ota_1")
        );
        assert_eq!(
            (rows[2].offset, rows[2].size),
            (OTA_1_OFFSET, OTA_SLOT_SIZE)
        );

        assert_eq!(rows[3].name, "zbnv");
        assert_eq!(
            (rows[3].kind.as_str(), rows[3].subtype.as_str()),
            ("data", "undefined")
        );
        assert_eq!((rows[3].offset, rows[3].size), (ZBNV_OFFSET, ZBNV_SIZE));
    }

    #[test]
    fn binary_partition_entries_match_the_expected_layout() {
        for partition in EXPECTED_PARTITIONS {
            let entry = partition.encode();
            assert!(partition.matches(&entry));
        }

        let mut wrong = EXPECTED_PARTITIONS[1].encode();
        wrong[4] ^= 1;
        assert!(!EXPECTED_PARTITIONS[1].matches(&wrong));
    }

    #[test]
    fn partitions_do_not_overlap_and_fit_the_flash() {
        let rows = parse_csv();
        let mut cursor = PARTITION_TABLE_OFFSET + PARTITION_TABLE_SIZE;
        for row in &rows {
            assert!(
                row.offset >= cursor,
                "{} overlaps the previous partition",
                row.name
            );
            cursor = row.offset + row.size;
        }
        assert_eq!(cursor, FLASH_SIZE);
    }

    #[test]
    fn nv_pages_stay_at_their_pre_partition_table_addresses() {
        // The joined ZHA state currently lives at 0x3FE000..0x400000. Moving it
        // would silently unpair the device, so this is asserted explicitly.
        assert_eq!(NV_OFFSET, 0x003F_E000);
        assert_eq!(NV_OFFSET + NV_SIZE, 0x0040_0000);
        const { assert!(NV_OFFSET >= ZBNV_OFFSET) };
        const { assert!(NV_OFFSET + NV_SIZE <= ZBNV_OFFSET + ZBNV_SIZE) };
    }

    #[test]
    fn slot_helpers_agree_with_constants() {
        assert_eq!(ota_slot_offset(0), OTA_0_OFFSET);
        assert_eq!(ota_slot_offset(1), OTA_1_OFFSET);
        assert_eq!(otadata_sector_offset(0), OTADATA_OFFSET);
        assert_eq!(otadata_sector_offset(1), OTADATA_OFFSET + SECTOR_SIZE);
    }

    #[test]
    fn only_otadata_and_the_app_slots_are_writable() {
        assert!(is_ota_writable(OTADATA_OFFSET, OTADATA_SIZE));
        assert!(is_ota_writable(OTA_0_OFFSET, OTA_SLOT_SIZE));
        assert!(is_ota_writable(OTA_1_OFFSET, OTA_SLOT_SIZE));

        // Bootloader, partition table, NV pages and the gap around them.
        assert!(!is_ota_writable(0, SECTOR_SIZE));
        assert!(!is_ota_writable(
            PARTITION_TABLE_OFFSET,
            PARTITION_TABLE_SIZE
        ));
        assert!(!is_ota_writable(NV_OFFSET, NV_SIZE));
        assert!(!is_ota_writable(ZBNV_OFFSET, ZBNV_SIZE));

        // Ranges that start inside a window but run past its end.
        assert!(!is_ota_writable(OTADATA_OFFSET, OTADATA_SIZE + 4));
        assert!(!is_ota_writable(
            OTA_1_OFFSET + OTA_SLOT_SIZE - 4,
            SECTOR_SIZE
        ));
        assert!(!is_ota_writable(FLASH_SIZE - 4, 8));
        assert!(!is_ota_writable(OTA_0_OFFSET, 0));
        assert!(!is_ota_writable(u32::MAX, 4));
    }
}
