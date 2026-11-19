//! Fitted flash size and the layout-mismatch rule (spec 11.5, decision N4).
//!
//! The image is built for one partition table ([`crate::layout::ACTIVE`]).
//! At boot the NCP reads the flash chip's JEDEC ID, reports the size in
//! `SYS_GET_INFO.flash_size` and, if the part is smaller than the end of
//! its partition table (a 4 MB image on a 2 MB module), refuses to start
//! the Zigbee stack: it keeps answering `SYS_*`, raises `EVT_FATAL` (kind
//! [`LAYOUT_MISMATCH_FATAL_KIND`]) and waits for a corrected image.

use crate::layout::Layout;

/// `EVT_FATAL.kind` for "partition table larger than the fitted flash".
pub const LAYOUT_MISMATCH_FATAL_KIND: u8 = 1;

/// Size in bytes encoded by a JEDEC ID (`manufacturer << 16 | type << 8 |
/// capacity`), for the usual `2^capacity` codes of 128 KiB..256 MiB.
pub const fn jedec_capacity_bytes(jedec_id: u32) -> Option<u32> {
    let code = (jedec_id & 0xFF) as u8;
    if jedec_id == 0 || jedec_id == 0x00FF_FFFF || jedec_id == 0xFFFF_FFFF {
        return None;
    }
    if code >= 17 && code <= 28 {
        Some(1u32 << code)
    } else {
        None
    }
}

/// Whether the stack may run on the fitted flash.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashVerdict {
    /// The partition table fits.
    Fits { flash_size: u32 },
    /// The partition table ends beyond the fitted flash: refuse to start.
    TooSmall { flash_size: u32, layout_end: u32 },
    /// The ID could not be decoded: the size is unknown and reported as 0;
    /// the stack still starts (the ROM driver's own bounds apply).
    Unknown,
}

impl FlashVerdict {
    pub const fn from_jedec(jedec_id: u32, layout: &Layout) -> Self {
        match jedec_capacity_bytes(jedec_id) {
            None => Self::Unknown,
            Some(flash_size) if flash_size < layout.flash_end() => Self::TooSmall {
                flash_size,
                layout_end: layout.flash_end(),
            },
            Some(flash_size) => Self::Fits { flash_size },
        }
    }

    pub const fn stack_allowed(&self) -> bool {
        !matches!(self, Self::TooSmall { .. })
    }

    /// `SYS_GET_INFO.flash_size` (0 when unknown).
    pub const fn reported_size(&self) -> u32 {
        match self {
            Self::Fits { flash_size } | Self::TooSmall { flash_size, .. } => *flash_size,
            Self::Unknown => 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::layout::{LAYOUT_2MB, LAYOUT_4MB};

    // Capacity codes of parts used in ESP32-H2 modules.
    const ID_4MB: u32 = 0x0020_4016;
    const ID_2MB: u32 = 0x0020_4015;

    #[test]
    fn decodes_power_of_two_capacities() {
        assert_eq!(jedec_capacity_bytes(ID_4MB), Some(4 * 1024 * 1024));
        assert_eq!(jedec_capacity_bytes(ID_2MB), Some(2 * 1024 * 1024));
        assert_eq!(jedec_capacity_bytes(0), None);
        assert_eq!(jedec_capacity_bytes(0xFFFF_FFFF), None);
        assert_eq!(jedec_capacity_bytes(0x00C8_4010), None);
    }

    #[test]
    fn a_4mb_image_refuses_a_2mb_part() {
        let verdict = FlashVerdict::from_jedec(ID_2MB, &LAYOUT_4MB);
        assert_eq!(
            verdict,
            FlashVerdict::TooSmall {
                flash_size: 2 * 1024 * 1024,
                layout_end: 4 * 1024 * 1024
            }
        );
        assert!(!verdict.stack_allowed());
        assert_eq!(verdict.reported_size(), 2 * 1024 * 1024);
    }

    #[test]
    fn a_2mb_image_runs_on_either_part() {
        assert!(FlashVerdict::from_jedec(ID_2MB, &LAYOUT_2MB).stack_allowed());
        assert!(FlashVerdict::from_jedec(ID_4MB, &LAYOUT_2MB).stack_allowed());
        assert!(FlashVerdict::from_jedec(ID_4MB, &LAYOUT_4MB).stack_allowed());
    }

    #[test]
    fn an_unreadable_id_reports_zero_and_does_not_block() {
        let verdict = FlashVerdict::from_jedec(0xFFFF_FFFF, &LAYOUT_4MB);
        assert_eq!(verdict, FlashVerdict::Unknown);
        assert!(verdict.stack_allowed());
        assert_eq!(verdict.reported_size(), 0);
    }
}
