//! Platform-independent RF front-end module (FEM: external PA/LNA) policy.
//!
//! A FEM places a power amplifier (PA) in the transmit path and a low-noise
//! amplifier (LNA) in the receive path between the radio and the antenna.
//! Every value the stack exchanges with a FEM-equipped PHY stays
//! **antenna-referred**, exactly as without a FEM:
//!
//! * `phyTransmitPower` (MAC PIB, MLME-SET, ZRHP `NWK_SET_TX_POWER`) is the
//!   requested output at the antenna port. The PHY programs the radio with
//!   `requested − tx_gain_db`, rounded **down** to a supported radio step,
//!   after capping the request at the product's regulatory
//!   [`FemParams::max_output_dbm`] ([`split_tx_power`]).
//! * Received levels (RSSI, energy detect, the level an LQI is derived from)
//!   have the LNA gain removed before they reach the stack
//!   ([`rx_level_at_antenna`], [`rssi_at_antenna`]).
//! * An energy CCA threshold expressed at the antenna is raised by the LNA
//!   gain before it is programmed into the radio
//!   ([`radio_ed_threshold`]).
//!
//! Pin control and its timing are platform mechanisms (`nrf_fem` for
//! nRF52); this module holds only the arithmetic and validation, so it is
//! host-tested and reusable by any backend.

/// Largest PA or LNA gain accepted by [`FemParams::validate`] (dB).
pub const MAX_FEM_GAIN_DB: u8 = 40;

/// Gains, timing and regulatory limits of one fitted FEM.
///
/// Board crates describe the fitted part (gains, settle times); the product
/// owns the regulatory [`Self::max_output_dbm`] and the
/// [`Self::default_output_dbm`] restored by MLME-RESET.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FemParams {
    /// PA gain (dB) between the radio output and the antenna port.
    pub tx_gain_db: u8,
    /// LNA gain (dB) between the antenna port and the radio input.
    pub rx_gain_db: u8,
    /// Minimum time (µs) from PA enable to the first transmitted symbol
    /// (Nordic `ctx-settle-time-us`).
    pub pa_settle_us: u8,
    /// Minimum time (µs) from LNA enable to the start of reception (Nordic
    /// `crx-settle-time-us`).
    pub lna_settle_us: u8,
    /// Regulatory cap on the antenna output (dBm, conducted). Requests above
    /// it are clamped; the PHY never programs a radio power whose
    /// antenna-referred output exceeds it.
    pub max_output_dbm: i8,
    /// `phyTransmitPower` (antenna-referred dBm) installed at MAC creation
    /// and restored by MLME-RESET with default PIB.
    pub default_output_dbm: i8,
}

/// Timing the platform's pin-control scheme can honour.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FemTimingLimits {
    /// Longest PA settle time the scheme provides before the first symbol.
    pub max_pa_settle_us: u8,
    /// Longest LNA settle time accepted (see the platform scheme for what a
    /// settle time above zero costs).
    pub max_lna_settle_us: u8,
}

/// Rejected FEM configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FemConfigError {
    /// A gain above [`MAX_FEM_GAIN_DB`].
    GainOutOfRange,
    /// Even the lowest radio power step plus the PA gain exceeds
    /// [`FemParams::max_output_dbm`]: the cap cannot be honoured.
    MaxOutputBelowMinimum,
    /// [`FemParams::default_output_dbm`] is above the cap or below the
    /// lowest achievable output.
    DefaultOutsideRange,
    /// The PA settle time is longer than the platform scheme provides.
    PaSettleTooLong,
    /// The LNA settle time is longer than the platform scheme accepts.
    LnaSettleTooLong,
    /// Platform resources (pins, channels) are invalid or shared.
    InvalidResources,
    /// A FEM is already attached to this radio.
    AlreadyActive,
}

impl FemParams {
    /// Check the parameters against the radio's ascending power `steps`
    /// (dBm) and the platform's timing scheme.
    pub fn validate(&self, steps: &[i8], timing: FemTimingLimits) -> Result<(), FemConfigError> {
        if self.tx_gain_db > MAX_FEM_GAIN_DB || self.rx_gain_db > MAX_FEM_GAIN_DB {
            return Err(FemConfigError::GainOutOfRange);
        }
        let (Some(&lowest), Some(_)) = (steps.first(), steps.last()) else {
            return Err(FemConfigError::InvalidResources);
        };
        let minimum = i16::from(lowest) + i16::from(self.tx_gain_db);
        if minimum > i16::from(self.max_output_dbm) {
            return Err(FemConfigError::MaxOutputBelowMinimum);
        }
        let default = i16::from(self.default_output_dbm);
        if default > i16::from(self.max_output_dbm) || default < minimum {
            return Err(FemConfigError::DefaultOutsideRange);
        }
        if self.pa_settle_us > timing.max_pa_settle_us {
            return Err(FemConfigError::PaSettleTooLong);
        }
        if self.lna_settle_us > timing.max_lna_settle_us {
            return Err(FemConfigError::LnaSettleTooLong);
        }
        Ok(())
    }

    /// Antenna-referred transmit power range `(min, max)` in dBm, for
    /// `PhyCapabilities`. Requires validated parameters.
    pub fn output_range(&self, steps: &[i8]) -> (i8, i8) {
        let gain = i16::from(self.tx_gain_db);
        let lowest = steps.first().map_or(0, |&s| i16::from(s)) + gain;
        let highest = steps.last().map_or(0, |&s| i16::from(s)) + gain;
        (
            saturate_i8(lowest),
            saturate_i8(highest.min(i16::from(self.max_output_dbm))),
        )
    }
}

/// Result of splitting a requested antenna power between radio and PA.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TxPowerSplit {
    /// `phyTransmitPower` as requested (antenna-referred dBm).
    pub requested_dbm: i8,
    /// Value programmed into the radio (one of its supported steps).
    pub radio_dbm: i8,
    /// Effective antenna output: `radio_dbm + tx_gain_db`.
    pub antenna_dbm: i8,
    /// The request was above [`FemParams::max_output_dbm`] and was capped.
    pub capped: bool,
}

/// Split a requested antenna output between the radio and the PA.
///
/// The request is capped at `max_output_dbm`; the radio power is the
/// highest step in `steps` (ascending) not above `capped − tx_gain_db`, so
/// the antenna output never exceeds the capped request — the same rule as
/// Nordic MPSL's power split. A request below the lowest achievable output
/// (`steps[0] + tx_gain_db`) selects the lowest step; [`FemParams::validate`]
/// guarantees that this is still within the cap.
pub fn split_tx_power(requested_dbm: i8, params: &FemParams, steps: &[i8]) -> TxPowerSplit {
    let capped = requested_dbm > params.max_output_dbm;
    let target = requested_dbm.min(params.max_output_dbm);
    let radio_target = i16::from(target) - i16::from(params.tx_gain_db);
    let radio_dbm = steps
        .iter()
        .rev()
        .copied()
        .find(|&step| i16::from(step) <= radio_target)
        .or_else(|| steps.first().copied())
        .unwrap_or(0);
    TxPowerSplit {
        requested_dbm,
        radio_dbm,
        antenna_dbm: saturate_i8(i16::from(radio_dbm) + i16::from(params.tx_gain_db)),
        capped,
    }
}

/// A received level in 1 dB hardware units (e.g. Nordic's ED/LQI byte,
/// `P[dBm] = ED_RSSIOFFS + value`) referred to the antenna: the LNA gain is
/// removed, saturating at the hardware floor (0).
pub const fn rx_level_at_antenna(radio_level: u8, rx_gain_db: u8) -> u8 {
    radio_level.saturating_sub(rx_gain_db)
}

/// A received signal strength (dBm) referred to the antenna.
pub const fn rssi_at_antenna(radio_dbm: i8, rx_gain_db: u8) -> i8 {
    let value = radio_dbm as i16 - rx_gain_db as i16;
    if value < i8::MIN as i16 {
        i8::MIN
    } else {
        value as i8
    }
}

/// An energy-detect CCA threshold (1 dB hardware units) chosen at the
/// antenna, converted to the value the radio compares against: the LNA
/// gain is added, saturating at the hardware ceiling.
pub const fn radio_ed_threshold(antenna_threshold: u8, rx_gain_db: u8) -> u8 {
    antenna_threshold.saturating_add(rx_gain_db)
}

fn saturate_i8(value: i16) -> i8 {
    value.clamp(i16::from(i8::MIN), i16::from(i8::MAX)) as i8
}

#[cfg(test)]
mod tests {
    use super::*;

    /// nRF52833/nRF52840 RADIO TXPOWER steps (dBm).
    const NRF: &[i8] = &[-40, -30, -20, -16, -12, -8, -4, 0, 2, 3, 4, 5, 6, 7, 8];
    const TIMING: FemTimingLimits = FemTimingLimits {
        max_pa_settle_us: 40,
        max_lna_settle_us: 16,
    };

    /// SKY66112-11 defaults (as used by AT2401C boards) with a +20 dBm cap.
    fn at2401c() -> FemParams {
        FemParams {
            tx_gain_db: 22,
            rx_gain_db: 5,
            pa_settle_us: 23,
            lna_settle_us: 5,
            max_output_dbm: 20,
            default_output_dbm: 20,
        }
    }

    #[test]
    fn split_rounds_down_and_never_exceeds_request() {
        let params = at2401c();
        // 20 - 22 = -2 -> -4 step -> 18 dBm at the antenna.
        assert_eq!(
            split_tx_power(20, &params, NRF),
            TxPowerSplit {
                requested_dbm: 20,
                radio_dbm: -4,
                antenna_dbm: 18,
                capped: false
            }
        );
        // Exact step: 22 - 22 = 0, but capped at 20 first.
        let split = split_tx_power(22, &params, NRF);
        assert!(split.capped);
        assert_eq!((split.radio_dbm, split.antenna_dbm), (-4, 18));
        // 0 dBm requested: -22 -> -30 step -> -8 dBm.
        let split = split_tx_power(0, &params, NRF);
        assert_eq!((split.radio_dbm, split.antenna_dbm), (-30, -8));
        for requested in i8::MIN..=i8::MAX {
            let split = split_tx_power(requested, &params, NRF);
            assert!(NRF.contains(&split.radio_dbm));
            assert!(split.antenna_dbm <= params.max_output_dbm);
            if requested >= -18 {
                assert!(split.antenna_dbm <= requested);
            }
        }
    }

    #[test]
    fn requests_below_the_floor_use_the_lowest_step() {
        let split = split_tx_power(-60, &at2401c(), NRF);
        assert_eq!((split.radio_dbm, split.antenna_dbm), (-40, -18));
        assert!(!split.capped);
    }

    #[test]
    fn high_cap_uses_full_radio_range() {
        let params = FemParams {
            max_output_dbm: 40,
            default_output_dbm: 0,
            ..at2401c()
        };
        let split = split_tx_power(127, &params, NRF);
        assert_eq!((split.radio_dbm, split.antenna_dbm), (8, 30));
        assert!(split.capped);
        assert_eq!(params.output_range(NRF), (-18, 30));
        assert_eq!(at2401c().output_range(NRF), (-18, 20));
    }

    #[test]
    fn zero_gain_is_identity_on_steps() {
        let params = FemParams {
            tx_gain_db: 0,
            rx_gain_db: 0,
            pa_settle_us: 0,
            lna_settle_us: 0,
            max_output_dbm: 8,
            default_output_dbm: 0,
        };
        for &step in NRF {
            assert_eq!(split_tx_power(step, &params, NRF).radio_dbm, step);
        }
    }

    #[test]
    fn validation_rejects_unsafe_configurations() {
        assert_eq!(at2401c().validate(NRF, TIMING), Ok(()));
        let reject = |params: FemParams| params.validate(NRF, TIMING).unwrap_err();
        assert_eq!(
            reject(FemParams {
                tx_gain_db: 41,
                ..at2401c()
            }),
            FemConfigError::GainOutOfRange
        );
        assert_eq!(
            reject(FemParams {
                rx_gain_db: 41,
                ..at2401c()
            }),
            FemConfigError::GainOutOfRange
        );
        // -40 + 22 = -18 > -20: the cap cannot be met at the lowest step.
        assert_eq!(
            reject(FemParams {
                max_output_dbm: -20,
                default_output_dbm: -20,
                ..at2401c()
            }),
            FemConfigError::MaxOutputBelowMinimum
        );
        assert_eq!(
            reject(FemParams {
                default_output_dbm: 21,
                ..at2401c()
            }),
            FemConfigError::DefaultOutsideRange
        );
        assert_eq!(
            reject(FemParams {
                default_output_dbm: -19,
                ..at2401c()
            }),
            FemConfigError::DefaultOutsideRange
        );
        assert_eq!(
            reject(FemParams {
                pa_settle_us: 41,
                ..at2401c()
            }),
            FemConfigError::PaSettleTooLong
        );
        assert_eq!(
            reject(FemParams {
                lna_settle_us: 17,
                ..at2401c()
            }),
            FemConfigError::LnaSettleTooLong
        );
        assert_eq!(
            at2401c().validate(&[], TIMING),
            Err(FemConfigError::InvalidResources)
        );
    }

    #[test]
    fn received_levels_are_antenna_referred() {
        assert_eq!(rx_level_at_antenna(40, 5), 35);
        assert_eq!(rx_level_at_antenna(3, 5), 0);
        assert_eq!(rssi_at_antenna(-60, 5), -65);
        assert_eq!(rssi_at_antenna(-126, 11), i8::MIN);
        assert_eq!(radio_ed_threshold(0x14, 5), 0x19);
        assert_eq!(radio_ed_threshold(250, 11), u8::MAX);
    }
}
