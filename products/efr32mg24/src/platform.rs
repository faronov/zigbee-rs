//! Product services of the EFR32MG24 ZRHP NCP behind
//! `ncp_coordinator_app::platform::NcpPlatform`: transmit power
//! (`NWK_SET_TX_POWER`, capability `TX_POWER`, range capacities 20/21).
//! Everything else keeps the trait's "unsupported" default.

use ncp_coordinator_app::platform::NcpPlatform;

use efr32s2_hal::radio::{TX_POWER_RANGE_DBM, tx_power_step_dbm};

/// Highest transmit power the NCP accepts: the fitted PA's
/// ([`crate::board::PA_MAX_DBM`]). No regulatory limit is applied here; the
/// platform guide gives the EU EIRP arithmetic per product (e.g. the SONOFF
/// Dongle Max), and the host chooses the power.
pub const TX_POWER_MAX_DBM: i8 = crate::board::PA_MAX_DBM;

/// The MG24 NCP's [`NcpPlatform`].
#[derive(Debug, Default, Clone, Copy)]
pub struct Mg24Platform;

impl NcpPlatform for Mg24Platform {
    /// The PA table entry a request selects ([`tx_power_step_dbm`]),
    /// capped at [`TX_POWER_MAX_DBM`]. `None` in a build without a
    /// validated PA table.
    fn tx_power_step(&self, dbm: i8) -> Option<i8> {
        let (min, max) = TX_POWER_RANGE_DBM;
        if min >= max {
            return None;
        }
        Some(tx_power_step_dbm(dbm.min(TX_POWER_MAX_DBM)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_power_range_is_the_pa_table_up_to_the_fitted_pa() {
        let platform = Mg24Platform;
        let (min, max) = TX_POWER_RANGE_DBM;
        assert_eq!(platform.tx_power_step(i8::MIN), Some(min));
        assert_eq!(
            platform.tx_power_step(i8::MAX),
            Some(max.min(TX_POWER_MAX_DBM))
        );
        // The product default is a step, so a host that reads it back and
        // sets it again changes nothing.
        assert_eq!(
            platform.tx_power_step(crate::TX_POWER_DBM),
            Some(crate::TX_POWER_DBM)
        );
        for dbm in i8::MIN..=i8::MAX {
            let step = platform.tx_power_step(dbm).unwrap();
            assert!(step <= dbm.max(min), "{dbm} -> {step}");
            assert_eq!(platform.tx_power_step(step), Some(step));
        }
    }

    #[test]
    #[cfg(any(feature = "sonoff-dongle-m", feature = "wstk-brd4187"))]
    fn hp_pa_boards_reach_20_dbm() {
        assert_eq!(Mg24Platform.tx_power_step(20), Some(20));
        assert_eq!(Mg24Platform.tx_power_step(-128), Some(-24));
    }

    #[test]
    #[cfg(any(feature = "ek2703a", feature = "rf-bm-mg24b1"))]
    fn mp_pa_boards_reach_10_dbm() {
        assert_eq!(Mg24Platform.tx_power_step(20), Some(10));
        assert_eq!(Mg24Platform.tx_power_step(-128), Some(-30));
    }
}
