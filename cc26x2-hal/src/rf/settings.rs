//! IEEE 802.15.4 radio settings and TX power tables.
//!
//! Transcribed from SimpleLink Low Power F2 SDK 8.33.00.16
//! (`source/ti/devices/radioconfig/.meta/config`):
//!
//! - override words: `<device>_ieee_15_4_pg*/cmd_settings/setting_ieee_802_15_4.json`
//!   (`cc2652r_ieee_15_4_pg21`, `cc2652r7_ieee_15_4_pg10`,
//!   `cc2652p_ieee_15_4_pg21`, `cc2652p7_ieee_15_4_pg10`); the coexistence
//!   and HPOSC blocks of those templates are omitted (no coex wiring, no
//!   BAW oscillator on the supported parts);
//! - ANADIV words: `driverlib/rfc.c` `RFCAnaDivTxOverride(loDivider 0,
//!   frontEndMode 0 | 255)` = `0x0532_0703` (differential) / `0x01C2_0703`
//!   (+20 dBm PA);
//! - standard PA tables: `pa_tables/cc13x2_cc26x2_r3` `LAUNCHXL-CC1352P2`
//!   (identical to `CC2652REM-7ID`) and `pa_tables/cc13x2_cc26x2_r7`
//!   `LP-CC1352P7-2` / `CC26x2-R7EM-7ID`;
//! - high-power PA tables: `LAUNCHXL-CC1352P2_20dBm` / `LP-CC1352P7-2_20dBm`
//!   (`TxHighPa`, 14..20 dBm; identical values).
//!
//! The TI setting for every part uses the same CPE patch
//! (`rf_patch_cpe_ieee_802_15_4`) and no MCE/RFE patch.

use super::cmd::{END_OVERRIDE, hw_reg_override, tx_std_power_override, tx20_power_override};

/// One TX power table entry: `dbm` and the raw PA setting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PaEntry {
    pub dbm: i8,
    pub raw: u32,
}

const fn e(dbm: i8, raw: u32) -> PaEntry {
    PaEntry { dbm, raw }
}

/// Standard PA, CC13x2/CC26x2 rev E (highest first).
pub const STD_PA_X2: [PaEntry; 15] = [
    e(5, 0x7217),
    e(4, 0x4E63),
    e(3, 0x385D),
    e(2, 0x3259),
    e(1, 0x2856),
    e(0, 0x2853),
    e(-3, 0x1893),
    e(-5, 0x12D6),
    e(-6, 0x10D4),
    e(-9, 0x0AD0),
    e(-10, 0x0ACF),
    e(-12, 0x0ACC),
    e(-15, 0x06CA),
    e(-18, 0x06C8),
    e(-20, 0x04C6),
];

/// Standard PA, CC2652P7/CC1352P7 (`LP-CC1352P7-2`).
pub const STD_PA_P7: [PaEntry; 15] = [
    e(5, 0x762E),
    e(4, 0x8220),
    e(3, 0x486F),
    e(2, 0x3E66),
    e(1, 0x3261),
    e(0, 0x2C5D),
    e(-3, 0x1899),
    e(-5, 0x1695),
    e(-6, 0x1693),
    e(-9, 0x0CD4),
    e(-10, 0x0AD3),
    e(-12, 0x0AD0),
    e(-15, 0x06CD),
    e(-18, 0x04CA),
    e(-20, 0x04C8),
];

/// Standard PA, CC2652R7 (`CC26x2-R7EM-7ID`).
pub const STD_PA_R7: [PaEntry; 15] = [
    e(5, 0x701F),
    e(4, 0x3A17),
    e(3, 0x3A64),
    e(2, 0x325F),
    e(1, 0x2C5C),
    e(0, 0x2659),
    e(-3, 0x1697),
    e(-5, 0x1693),
    e(-6, 0x1292),
    e(-9, 0x0CD3),
    e(-10, 0x0AD1),
    e(-12, 0x0ACF),
    e(-15, 0x06CD),
    e(-18, 0x06CA),
    e(-20, 0x04C8),
];

/// +20 dBm PA (P parts, 2.4 GHz path), both families.
pub const HIGH_PA: [PaEntry; 7] = [
    e(20, 0x3F_75F5),
    e(19, 0x3F_61E2),
    e(18, 0x30_47E0),
    e(17, 0x1B_4FE5),
    e(16, 0x1B_39DE),
    e(15, 0x1B_2FDA),
    e(14, 0x1B_27D6),
];

/// Standard PA table of the compiled chip.
pub const STD_PA: &[PaEntry] = if cfg!(feature = "family-x2") {
    &STD_PA_X2
} else if cfg!(feature = "high-pa") {
    &STD_PA_P7
} else {
    &STD_PA_R7
};

/// Which amplifier a setting uses.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Pa {
    Standard,
    High,
}

/// Selected TX power.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TxPower {
    pub dbm: i8,
    pub pa: Pa,
    pub raw: u32,
}

/// Highest table setting not above `dbm` (the lowest setting when `dbm` is
/// below the table). The high-power PA is used only when `high_pa` and
/// `dbm >= 14`.
pub fn select(dbm: i8, high_pa: bool) -> TxPower {
    if high_pa && let Some(entry) = HIGH_PA.iter().find(|e| e.dbm <= dbm) {
        return TxPower {
            dbm: entry.dbm,
            pa: Pa::High,
            raw: entry.raw,
        };
    }
    let entry = STD_PA
        .iter()
        .find(|e| e.dbm <= dbm)
        .unwrap_or(&STD_PA[STD_PA.len() - 1]);
    TxPower {
        dbm: entry.dbm,
        pa: Pa::Standard,
        raw: entry.raw,
    }
}

/// Supported range in dBm.
pub fn range(high_pa: bool) -> (i8, i8) {
    let max = if high_pa {
        HIGH_PA[0].dbm
    } else {
        STD_PA[0].dbm
    };
    (STD_PA[STD_PA.len() - 1].dbm, max)
}

/// Base override list length (with room for the TX20 segment jump).
pub const BASE_LEN: usize = 4;
/// TX-path override list length.
pub const TX_LEN: usize = 7;

/// `pRegOverride`: LNA bias +15 and DC/DC IPEAK 3 without dither. Index 2
/// is the end marker, which becomes the TX20 segment jump while the high
/// PA is selected (`RF_attachOverrides`).
pub const fn base_overrides() -> [u32; BASE_LEN] {
    if cfg!(feature = "high-pa") {
        [0x000F_8883, 0x00F3_88D3, END_OVERRIDE, END_OVERRIDE]
    } else {
        [0x00F3_88D3, 0x000F_8883, END_OVERRIDE, END_OVERRIDE]
    }
}

/// Index of the base list terminator that `RF_attachOverrides` replaces.
pub const BASE_END_INDEX: usize = 2;

/// `pRegOverrideTxStd` for a P part; index 0 holds the TX power.
pub const fn tx_std_overrides(raw: u16) -> [u32; TX_LEN] {
    if cfg!(feature = "family-x2") {
        [
            tx_std_power_override(raw),
            0x0532_0703,
            0x0000_8783,
            0x0502_06C3,
            hw_reg_override(0x60A8, 0x0401),
            END_OVERRIDE,
            END_OVERRIDE,
        ]
    } else {
        [
            tx_std_power_override(raw),
            0x0532_0703,
            0x00F3_88D3,
            0x0000_8783,
            0x0502_06C3,
            hw_reg_override(0x60A8, 0x0401),
            END_OVERRIDE,
        ]
    }
}

/// `pRegOverrideTx20` for a P part; index 0 holds the TX power.
pub const fn tx20_overrides(raw: u32) -> [u32; TX_LEN] {
    let rtim = if cfg!(feature = "family-x2") {
        0x0003_8783
    } else {
        0x0003_0783
    };
    [
        tx20_power_override(raw),
        0x01C2_0703,
        rtim,
        0x0102_06C3,
        hw_reg_override(0x60A8, 0x0001),
        END_OVERRIDE,
        END_OVERRIDE,
    ]
}

/// `txPower` escape value: take the PA setting from the TX20 overrides.
pub const TX20_ENABLED: u16 = 0xFFFF;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selection_never_exceeds_the_request() {
        let p = select(20, true);
        assert_eq!((p.dbm, p.pa, p.raw), (20, Pa::High, 0x3F_75F5));
        let p = select(13, true);
        assert_eq!((p.dbm, p.pa), (5, Pa::Standard));
        let p = select(14, false);
        assert_eq!((p.dbm, p.pa), (5, Pa::Standard));
        let p = select(-1, false);
        assert_eq!(p.dbm, -3);
        let p = select(-40, true);
        assert_eq!(p.dbm, -20);
        assert_eq!(range(true), (-20, 20));
        assert_eq!(range(false), (-20, 5));
    }

    #[cfg(all(feature = "family-x2", feature = "high-pa"))]
    #[test]
    fn cc2652p_override_lists_match_the_ti_setting() {
        assert_eq!(
            base_overrides()[..3],
            [0x000F_8883, 0x00F3_88D3, END_OVERRIDE]
        );
        let std = tx_std_overrides(0x7217);
        assert_eq!(
            std[1..6],
            [
                0x0532_0703,
                0x0000_8783,
                0x0502_06C3,
                0x0401_60A8,
                END_OVERRIDE
            ]
        );
        let hi = tx20_overrides(0x3F_75F5);
        assert_eq!(hi[0], 0x002B | (0x3F_75F5 << 10));
        assert_eq!(
            hi[1..6],
            [
                0x01C2_0703,
                0x0003_8783,
                0x0102_06C3,
                0x0001_60A8,
                END_OVERRIDE
            ]
        );
        assert_eq!(STD_PA[0], e(5, 0x7217));
    }
}
