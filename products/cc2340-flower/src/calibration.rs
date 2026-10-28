//! Soil-probe calibration and the reference firmware's conversion math.
//!
//! The probe's detector voltage depends on the supply, so the reference
//! firmware interpolates between a supply-compensated "in air" (dry) and
//! "in water" (wet) level:
//!
//! ```text
//! vdds_mv = BATMON voltage + supply_compensation_mv
//! percent = map(avg_uV, air(vdds_mv), water(vdds_mv), 0, 100), clamped
//! ```
//!
//! Each board build has its own calibration (`BOARD_KOPTSERG` /
//! `BOARD_DIYZI` in `on_off_switch_ota_client.h`), selected here by the
//! `koptserg` (default) or `diyzi` cargo feature.

/// `offset_uv + slope_uv_per_mv * vdds_mv`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SupplyLine {
    pub slope_uv_per_mv: i64,
    pub offset_uv: i64,
}

impl SupplyLine {
    pub const fn at(self, vdds_mv: u16) -> i64 {
        self.slope_uv_per_mv * vdds_mv as i64 + self.offset_uv
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SoilCalibration {
    pub name: &'static str,
    /// Added to the BATMON reading (`BATTERY_MONITOR_COMPENSATION`).
    pub supply_compensation_mv: u16,
    /// Detector level with the probe in air (0 %).
    pub air: SupplyLine,
    /// Detector level with the probe in water (100 %).
    pub water: SupplyLine,
}

pub const KOPTSERG: SoilCalibration = SoilCalibration {
    name: "koptserg",
    supply_compensation_mv: 20,
    air: SupplyLine {
        slope_uv_per_mv: 148,
        offset_uv: 1_953_000,
    },
    water: SupplyLine {
        slope_uv_per_mv: 133,
        offset_uv: 800_000,
    },
};

pub const DIYZI: SoilCalibration = SoilCalibration {
    name: "diyzi",
    supply_compensation_mv: 10,
    air: SupplyLine {
        slope_uv_per_mv: 197,
        offset_uv: 1_600_000,
    },
    water: SupplyLine {
        slope_uv_per_mv: 148,
        offset_uv: 900_000,
    },
};

/// The calibration selected by the cargo features.
#[cfg(not(feature = "diyzi"))]
pub const ACTIVE: SoilCalibration = KOPTSERG;
#[cfg(feature = "diyzi")]
pub const ACTIVE: SoilCalibration = DIYZI;

/// Arduino-style linear map with the reference's output clamp
/// (`MIN(out_max, MAX(result, out_min))`). Integer division truncates
/// toward zero, like C `long`. A degenerate input range maps to `out_min`.
pub const fn map_clamped(x: i64, in_min: i64, in_max: i64, out_min: i64, out_max: i64) -> i64 {
    if in_max == in_min {
        return out_min;
    }
    let result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    if result < out_min {
        out_min
    } else if result > out_max {
        out_max
    } else {
        result
    }
}

impl SoilCalibration {
    /// Supply voltage used for compensation, from a raw BATMON reading.
    pub const fn compensated_supply_mv(self, batmon_mv: u16) -> u16 {
        batmon_mv.saturating_add(self.supply_compensation_mv)
    }

    /// Soil moisture in whole percent (0..=100) from the averaged detector
    /// voltage and the compensated supply.
    pub const fn percent(self, detector_uv: u32, supply_mv: u16) -> u8 {
        map_clamped(
            detector_uv as i64,
            self.air.at(supply_mv),
            self.water.at(supply_mv),
            0,
            100,
        ) as u8
    }

    /// ZCL Soil Moisture `MeasuredValue` (hundredths of a percent): the
    /// reference reports whole percent × 100.
    pub const fn zcl_value(self, detector_uv: u32, supply_mv: u16) -> u16 {
        self.percent(detector_uv, supply_mv) as u16 * 100
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_matches_the_reference_c_semantics() {
        assert_eq!(map_clamped(2_500, 2_000, 3_000, 0, 100), 50);
        // Truncation toward zero, as C `long` division.
        assert_eq!(map_clamped(2_019, 2_000, 3_000, 0, 100), 1);
        assert_eq!(map_clamped(1_000, 2_000, 3_000, 0, 100), 0);
        assert_eq!(map_clamped(4_000, 2_000, 3_000, 0, 100), 100);
        // Descending input range (air level above water level).
        assert_eq!(map_clamped(150, 200, 100, 0, 100), 50);
        assert_eq!(map_clamped(5, 7, 7, 0, 100), 0);
    }

    #[test]
    fn koptserg_calibration_at_3v() {
        let supply = KOPTSERG.compensated_supply_mv(2_980);
        assert_eq!(supply, 3_000);
        // air(3000) = 2_397_000 uV, water(3000) = 1_199_000 uV.
        assert_eq!(KOPTSERG.air.at(supply), 2_397_000);
        assert_eq!(KOPTSERG.water.at(supply), 1_199_000);
        assert_eq!(KOPTSERG.percent(2_397_000, supply), 0);
        assert_eq!(KOPTSERG.percent(2_600_000, supply), 0);
        assert_eq!(KOPTSERG.percent(1_199_000, supply), 100);
        assert_eq!(KOPTSERG.percent(900_000, supply), 100);
        // Midpoint 1_798_000 uV: (1798000-2397000)*100/(1199000-2397000) = 50.
        assert_eq!(KOPTSERG.percent(1_798_000, supply), 50);
        assert_eq!(KOPTSERG.zcl_value(1_798_000, supply), 5_000);
    }

    #[test]
    fn diyzi_calibration_at_2v5() {
        let supply = DIYZI.compensated_supply_mv(2_490);
        assert_eq!(supply, 2_500);
        // air(2500) = 2_092_500 uV, water(2500) = 1_270_000 uV.
        assert_eq!(DIYZI.air.at(supply), 2_092_500);
        assert_eq!(DIYZI.water.at(supply), 1_270_000);
        assert_eq!(DIYZI.percent(2_092_500, supply), 0);
        assert_eq!(DIYZI.percent(1_270_000, supply), 100);
        // (1681250-2092500)*100/(1270000-2092500) = 50.
        assert_eq!(DIYZI.percent(1_681_250, supply), 50);
        // Truncation: 25.0x % -> 25.
        assert_eq!(DIYZI.percent(1_886_800, supply), 25);
        assert_eq!(DIYZI.zcl_value(1_886_800, supply), 2_500);
    }

    #[test]
    fn supply_compensation_shifts_both_levels() {
        // Both levels fall with the supply, so the same detector voltage
        // maps to a different moisture: 41 % at 3.0 V, 36 % at 2.6 V.
        assert_eq!(KOPTSERG.percent(1_900_000, 3_000), 41);
        assert_eq!(KOPTSERG.percent(1_900_000, 2_600), 36);
    }

    #[test]
    fn feature_selects_the_active_calibration() {
        #[cfg(not(feature = "diyzi"))]
        assert_eq!(ACTIVE, KOPTSERG);
        #[cfg(feature = "diyzi")]
        assert_eq!(ACTIVE, DIYZI);
    }
}
