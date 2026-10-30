//! Product radio selection: PB-03F carries a PHY6252 in QFN32.

use zigbee_mac::phy6222::driver::{DEFAULT_FCS_MODE, PaTable, RadioConfig};

/// QFN32 PA table at 0 dBm (PA code 0x1F) with the 48 MHz DLL HCLK set up
/// by `phy6222_hal::time::init`.
pub fn radio_config() -> RadioConfig {
    RadioConfig {
        tx_power: 0,
        pa_table: PaTable::QFN32,
        hclk_mhz: 48,
        fcs: DEFAULT_FCS_MODE,
        ..RadioConfig::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn product_selects_qfn32_zero_dbm_at_48_mhz() {
        let config = radio_config();
        assert_eq!(config.pa_table, PaTable::QFN32);
        assert_eq!(config.tx_power, 0);
        assert_eq!(config.hclk_mhz, 48);
    }
}
