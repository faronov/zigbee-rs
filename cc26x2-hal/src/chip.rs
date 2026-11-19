//! Compile-time chip variant and the factory identity in FCFG1.
//!
//! FCFG1 fields (SDK `inc/hw_fcfg1.h`): `ICEPICK_DEVICE_ID.WAFER_ID[27:12]`
//! identifies the family (0xBB41 = CC13x2/CC26x2 rev E, 0xBB77 =
//! CC13x2x7/CC26x2x7; driverlib `chipinfo.c`), `USER_ID.PA[25]` the +20 dBm
//! PA, `USER_ID.CC13[23]` sub-1 GHz support and `USER_ID.PKG[18:16]` the
//! package. The primary IEEE EUI-64 is `MAC_15_4_0/1`, stored LSB first.

use crate::mmio::{self, FCFG1_BASE, FLASH_BASE};

/// Silicon family of the selected chip feature.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Family {
    /// CC13x2/CC26x2 rev E (`cc13x2_cc26x2`): 352 KiB flash, 80 KiB SRAM.
    X2,
    /// CC13x2x7/CC26x2x7 (`cc13x2x7_cc26x2x7`): 704 KiB flash, 144 KiB SRAM.
    X2x7,
}

#[cfg(feature = "family-x2")]
pub const FAMILY: Family = Family::X2;
#[cfg(feature = "family-x2x7")]
pub const FAMILY: Family = Family::X2x7;

/// ICEPICK wafer ID of [`FAMILY`].
pub const WAFER_ID: u16 = match FAMILY {
    Family::X2 => 0xBB41,
    Family::X2x7 => 0xBB77,
};

/// The selected chip has the +20 dBm high-power PA.
pub const HIGH_PA: bool = cfg!(feature = "high-pa");

/// Flash erase sector size.
pub const SECTOR_SIZE: usize = 8 * 1024;

/// Main flash size of [`FAMILY`].
pub const FLASH_SIZE: usize = match FAMILY {
    Family::X2 => 352 * 1024,
    Family::X2x7 => 704 * 1024,
};

/// System SRAM size of [`FAMILY`] (at `0x2000_0000`).
pub const SRAM_SIZE: usize = match FAMILY {
    Family::X2 => 80 * 1024,
    Family::X2x7 => 144 * 1024,
};

/// Part name used in logs and docs.
pub const PART: &str = if cfg!(feature = "cc2652p") {
    "CC2652P"
} else if cfg!(feature = "cc1352p") {
    "CC1352P"
} else if cfg!(feature = "cc2652rb") {
    "CC2652RB"
} else if cfg!(feature = "cc2652r") {
    "CC2652R"
} else if cfg!(feature = "cc2652p7") {
    "CC2652P7"
} else if cfg!(feature = "cc1352p7") {
    "CC1352P7"
} else {
    "CC2652R7"
};

/// Factory identity decoded from FCFG1 and the flash controller.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Identity {
    pub wafer_id: u16,
    /// ICEPICK `PG_REV` (3 on CC26x2 rev E).
    pub pg_rev: u8,
    /// `USER_ID.PA`: +20 dBm PA fitted.
    pub high_pa: bool,
    /// `USER_ID.CC13`: sub-1 GHz capable (CC1352).
    pub sub_ghz: bool,
    /// `USER_ID.PKG` (2 = 7x7 mm RGZ).
    pub package: u8,
    /// Flash size reported by `FLASH:FLASH_SIZE` (sectors x 8 KiB).
    pub flash_size: usize,
    /// Primary IEEE 802.15.4 EUI-64 in over-the-air (little-endian) order.
    pub eui64: [u8; 8],
}

impl Identity {
    /// Decode the raw factory words (host-testable).
    pub const fn decode(
        icepick_device_id: u32,
        user_id: u32,
        flash_sectors: u32,
        mac0: u32,
        mac1: u32,
    ) -> Self {
        let lo = mac0.to_le_bytes();
        let hi = mac1.to_le_bytes();
        Self {
            wafer_id: ((icepick_device_id >> 12) & 0xFFFF) as u16,
            pg_rev: (icepick_device_id >> 28) as u8,
            high_pa: user_id & (1 << 25) != 0,
            sub_ghz: user_id & (1 << 23) != 0,
            package: ((user_id >> 16) & 0x7) as u8,
            flash_size: (flash_sectors & 0xFF) as usize * SECTOR_SIZE,
            eui64: [lo[0], lo[1], lo[2], lo[3], hi[0], hi[1], hi[2], hi[3]],
        }
    }

    /// Read the running chip.
    pub fn read() -> Self {
        Self::decode(
            mmio::read(FCFG1_BASE + 0x318),
            mmio::read(FCFG1_BASE + 0x294),
            mmio::read(FLASH_BASE + 0x2C),
            mmio::read(FCFG1_BASE + 0x2F0),
            mmio::read(FCFG1_BASE + 0x2F4),
        )
    }

    /// `Ok` when the silicon matches the compiled chip feature: family,
    /// flash size and, for a `high-pa` build, the fitted PA. A build for the
    /// standard-PA part may run on a P part (it simply never selects the
    /// high-power PA).
    pub fn check_compiled(&self) -> Result<(), ChipMismatch> {
        if self.wafer_id != WAFER_ID {
            return Err(ChipMismatch::Family);
        }
        if self.flash_size != FLASH_SIZE {
            return Err(ChipMismatch::FlashSize);
        }
        if HIGH_PA && !self.high_pa {
            return Err(ChipMismatch::NoHighPa);
        }
        Ok(())
    }
}

/// The running silicon is not the chip this image was built for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ChipMismatch {
    Family,
    FlashSize,
    NoHighPa,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Words read over the ROM bootloader from the lab Sonoff ZBDongle-P
    /// (CC2652P, `~/.hwlab/artifacts/cc2652p-00124b0020cd1d7d-backup`).
    #[test]
    fn decodes_the_lab_cc2652p() {
        let id = Identity::decode(0x3BB4_102F, 0x3202_F000, 0x2C, 0x20CD_1D7D, 0x0012_4B00);
        assert_eq!(id.wafer_id, 0xBB41);
        assert_eq!(id.pg_rev, 3);
        assert!(id.high_pa);
        assert!(!id.sub_ghz);
        assert_eq!(id.package, 2);
        assert_eq!(id.flash_size, 352 * 1024);
        // 00:12:4B:00:20:CD:1D:7D, least significant byte first on air.
        assert_eq!(id.eui64, [0x7D, 0x1D, 0xCD, 0x20, 0x00, 0x4B, 0x12, 0x00]);
        #[cfg(all(feature = "family-x2", feature = "high-pa"))]
        assert_eq!(id.check_compiled(), Ok(()));
    }

    #[test]
    fn rejects_a_foreign_family() {
        let id = Identity::decode(0x3BB7_702F, 0x3202_F000, 0x58, 0, 0);
        #[cfg(feature = "family-x2")]
        assert_eq!(id.check_compiled(), Err(ChipMismatch::Family));
        #[cfg(feature = "family-x2x7")]
        assert_eq!(id.check_compiled(), Ok(()));
    }
}
