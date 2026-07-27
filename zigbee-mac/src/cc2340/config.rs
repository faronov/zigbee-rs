//! CC2340 IEEE 802.15.4 PHY settings imported at build time.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum RegisterWidth {
    U16,
    U32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct RegisterWrite {
    pub address: u32,
    pub value: u32,
    pub width: RegisterWidth,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct TxPowerEntry {
    pub dbm: i8,
    pub raw: u32,
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
}
