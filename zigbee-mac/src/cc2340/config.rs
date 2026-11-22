//! CC2340 IEEE 802.15.4 PHY settings imported at build time.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)] // Widths present only when the SDK-generated PHY table uses them.
pub(crate) enum RegisterWidth {
    U16,
    U32,
}

/// One register write of an imported RCL PHY, packed into two halfwords.
///
/// Every imported field setting lands in the LRFD register blocks
/// (`0x4008_1000..0x4008_4000`, 32-bit writes) or the PBE/RFE TOPsm RAM
/// blocks (`0x4009_2000`, `0x4009_6000`, 16-bit writes), and every value
/// fits 16 bits, so `address = BASE + 2 * code` and the width follows from
/// the address. `build.rs` refuses to generate a table that breaks either
/// rule, and a host test compares the decoded table with the resolved one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RegisterWrite {
    code: u16,
    value: u16,
}

impl RegisterWrite {
    const BASE: u32 = 0x4008_0000;
    /// First code of the 16-bit TOPsm RAM blocks (`0x4009_0000`).
    const U16_CODES: u16 = 0x8000;

    #[allow(dead_code)] // Only the SDK-generated tables construct writes.
    pub(crate) const fn packed(code: u16, value: u16) -> Self {
        Self { code, value }
    }

    pub(crate) const fn address(self) -> u32 {
        Self::BASE + 2 * self.code as u32
    }

    pub(crate) const fn value(self) -> u32 {
        self.value as u32
    }

    pub(crate) const fn width(self) -> RegisterWidth {
        if self.code >= Self::U16_CODES {
            RegisterWidth::U16
        } else {
            RegisterWidth::U32
        }
    }
}

/// One row of the SysConfig board PA table.
///
/// `raw` is the board JSON `value` verbatim, *not* the register value. The
/// SysConfig/RCL encoding (`convPaTableSettingToBinary` in
/// `srfs8_util_cc23xx.js`, `LRF_TxPowerTable_Entry` in RCL) is:
///
/// * bits 5:0 `ib`, 8:6 `gain`, 9 `mode`, 10 `noIfampRfLdoBypass`;
/// * bits 19:12 temperature coefficient for `ib`.
///
/// The RFE `SPARE5` PA shadow register takes the low 11 bits shifted up by
/// five (`reserved[4:0]`), see [`Self::rfe_value`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TxPowerEntry {
    pub dbm: i8,
    pub raw: u32,
}

/// RCL `LRF_TXPOWER_REFERENCE_TEMPERATURE`.
pub(crate) const TX_POWER_REFERENCE_TEMPERATURE_C: i32 = 25;
/// RCL `LRF_TXPOWER_TEMPERATURE_SCALING`.
const TX_POWER_TEMPERATURE_SCALING: i32 = 256;
/// RCL `RFE_PA0_IB_MIN_USED`: `ib = 0` turns the PA off.
const PA_IB_MIN_USED: i32 = 1;
/// `LRFDRFE_PA0_IB_MAX >> LRFDRFE_PA0_IB_S`.
const PA_IB_MAX: i32 = 0x3F;
const RFE_VALUE_IB_SHIFT: u32 = 5;
const RFE_VALUE_IB_MASK: u16 = 0x3F << RFE_VALUE_IB_SHIFT;

impl TxPowerEntry {
    /// The 16-bit `SPARE5` value at the reference temperature.
    pub(crate) const fn rfe_value(self) -> u16 {
        ((self.raw & 0x7FF) << RFE_VALUE_IB_SHIFT) as u16
    }

    pub(crate) const fn temperature_coefficient(self) -> u8 {
        ((self.raw >> 12) & 0xFF) as u8
    }

    /// RCL `LRF_programTemperatureCompensatedTxPower`: linear `ib`
    /// correction `ib += (T - 25) * tempCoeff / 256`, saturated to `1..=63`.
    pub(crate) fn compensated_rfe_value(self, temperature_c: i32) -> u16 {
        let value = self.rfe_value();
        let coefficient = i32::from(self.temperature_coefficient());
        if coefficient == 0 {
            return value;
        }
        let ib = i32::from((value & RFE_VALUE_IB_MASK) >> RFE_VALUE_IB_SHIFT);
        let ib = (ib
            + (temperature_c - TX_POWER_REFERENCE_TEMPERATURE_C) * coefficient
                / TX_POWER_TEMPERATURE_SCALING)
            .clamp(PA_IB_MIN_USED, PA_IB_MAX);
        (value & !RFE_VALUE_IB_MASK) | ((ib as u16) << RFE_VALUE_IB_SHIFT)
    }
}

include!(concat!(env!("OUT_DIR"), "/cc2340_phy_config.rs"));

pub(crate) fn ieee_802154_phy_writes() -> Option<&'static [RegisterWrite]> {
    PHY_CONFIG_AVAILABLE.then_some(IEEE_802154_PHY_WRITES)
}

pub(crate) fn source() -> &'static str {
    PHY_CONFIG_SOURCE
}

pub(crate) fn tx_power(requested_dbm: i8) -> Option<TxPowerEntry> {
    let first = *TX_POWER_TABLE.first()?;
    let last = *TX_POWER_TABLE.last()?;
    if requested_dbm < first.dbm || requested_dbm > last.dbm {
        return None;
    }

    TX_POWER_TABLE
        .iter()
        .copied()
        .take_while(|entry| entry.dbm <= requested_dbm)
        .last()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn decoded(writes: &[RegisterWrite]) -> impl Iterator<Item = (u32, u32, RegisterWidth)> + '_ {
        writes
            .iter()
            .map(|write| (write.address(), write.value(), write.width()))
    }

    /// The packed IEEE table decodes to exactly the writes `build.rs`
    /// resolved from TI's RCL metadata (same order).
    #[test]
    fn packed_ieee_phy_table_matches_resolved_writes() {
        assert_eq!(
            IEEE_802154_PHY_WRITES.len(),
            IEEE_802154_PHY_WRITES_RESOLVED.len()
        );
        assert!(
            decoded(IEEE_802154_PHY_WRITES).eq(IEEE_802154_PHY_WRITES_RESOLVED.iter().copied())
        );
        if PHY_CONFIG_AVAILABLE {
            // TI ieee_802_15_4_oqpsk_250k_cc23xx.json: 256 fields, 110 writes.
            assert_eq!(IEEE_802154_PHY_WRITES.len(), 110);
        }
    }

    #[test]
    fn packed_write_covers_both_register_spaces() {
        let rfe = RegisterWrite::packed(0x1858, 0xDF7F);
        assert_eq!(
            (rfe.address(), rfe.value(), rfe.width()),
            (0x4008_30B0, 0xDF7F, RegisterWidth::U32)
        );
        let pbe_ram = RegisterWrite::packed(0x9004, 0x0012);
        assert_eq!(pbe_ram.address(), 0x4009_2008);
        assert_eq!(pbe_ram.width(), RegisterWidth::U16);
        let rfe_ram = RegisterWrite::packed(0xB403, 0x0300);
        assert_eq!(rfe_ram.address(), 0x4009_6806);
        assert_eq!(rfe_ram.width(), RegisterWidth::U16);
        // The last 32-bit code and the first 16-bit code.
        assert_eq!(RegisterWrite::packed(0x7FFF, 0).address(), 0x4008_FFFE);
        assert_eq!(RegisterWrite::packed(0x7FFF, 0).width(), RegisterWidth::U32);
        assert_eq!(RegisterWrite::packed(0x8000, 0).address(), 0x4009_0000);
        assert_eq!(RegisterWrite::packed(0x8000, 0).width(), RegisterWidth::U16);
        assert_eq!(RegisterWrite::packed(0xFFFF, 0xFFFF).address(), 0x4009_FFFE);
        assert_eq!(RegisterWrite::packed(0xFFFF, 0xFFFF).value(), 0xFFFF);
    }
    #[test]
    fn selects_exact_or_next_lower_tx_power() {
        if !PHY_CONFIG_AVAILABLE {
            return;
        }

        assert_eq!(tx_power(0).map(|entry| entry.dbm), Some(0));
        assert_eq!(tx_power(-1).map(|entry| entry.dbm), Some(-4));
        assert_eq!(tx_power(8).map(|entry| entry.dbm), Some(8));
        assert_eq!(tx_power(9), None);
        assert_eq!(tx_power(-21), None);
    }

    /// Values from the LP_EM_CC2340R5 board table, checked against the
    /// SysConfig-generated `LRF_TxPowerTable_Entry` initialisers.
    #[test]
    fn encodes_pa_table_values_like_sysconfig() {
        let entry = |dbm, raw| TxPowerEntry { dbm, raw };

        // -20 dBm: ib 18, gain 0. The raw JSON value would program ib 0 (PA off).
        assert_eq!(entry(-20, 0x0000_0012).rfe_value(), 0x0240);
        assert_eq!(entry(-20, 0x0000_0012).temperature_coefficient(), 0);
        // 0 dBm: ib 21, gain 6, tempCoeff 40.
        assert_eq!(entry(0, 0x0002_8195).rfe_value(), 0x32A0);
        assert_eq!(entry(0, 0x0002_8195).temperature_coefficient(), 40);
        // +5 dBm: ib 27, gain 6, mode 1, tempCoeff 55.
        assert_eq!(entry(5, 0x0003_739B).rfe_value(), 0x7360);
        assert_eq!(entry(5, 0x0003_739B).temperature_coefficient(), 55);
        // +8 dBm: ib 63, gain 7, mode 1, tempCoeff 180.
        assert_eq!(entry(8, 0x000B_43FF).rfe_value(), 0x7FE0);
        assert_eq!(entry(8, 0x000B_43FF).temperature_coefficient(), 180);
    }

    #[test]
    fn compensates_pa_bias_for_temperature_like_rcl() {
        let plus5 = TxPowerEntry {
            dbm: 5,
            raw: 0x0003_739B,
        };
        // Reference temperature: unchanged.
        assert_eq!(plus5.compensated_rfe_value(25), plus5.rfe_value());
        // 85 C: ib 27 + 60 * 55 / 256 = 27 + 12 = 39.
        assert_eq!(
            plus5.compensated_rfe_value(85),
            (0x7360 & !0x07E0) | (39 << 5)
        );
        // -40 C: ib 27 - 65 * 55 / 256 = 27 - 13 = 14 (C truncation).
        assert_eq!(
            plus5.compensated_rfe_value(-40),
            (0x7360 & !0x07E0) | (14 << 5)
        );

        // Saturation at the top and never below 1.
        let plus8 = TxPowerEntry {
            dbm: 8,
            raw: 0x000B_43FF,
        };
        assert_eq!(plus8.compensated_rfe_value(85), 0x7FE0);
        // -40 C: 63 - 65 * 180 / 256 = 63 - 45 = 18.
        assert_eq!(
            plus8.compensated_rfe_value(-40),
            (0x7FE0 & !0x07E0) | (18 << 5)
        );
        let low_bias = TxPowerEntry {
            dbm: 0,
            raw: (0xFF << 12) | 2,
        };
        assert_eq!(low_bias.compensated_rfe_value(-40) & 0x07E0, 1 << 5);

        // No coefficient: no correction, even far from 25 C.
        let minus20 = TxPowerEntry {
            dbm: -20,
            raw: 0x0000_0012,
        };
        assert_eq!(minus20.compensated_rfe_value(85), 0x0240);
    }
}
