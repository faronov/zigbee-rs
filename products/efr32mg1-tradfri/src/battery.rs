//! Battery chemistry policy layered over the board's supply monitor.

use efr32mg1_tradfri::{SupplyError, SupplyMonitor, supply_monitor};

/// Voltage-to-capacity curves supported by the TRADFRI 2xAAA carrier.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BatteryCurve {
    /// Conservative 2xAAA alkaline curve from 1.8 V empty to 3.1 V full.
    TwoAaaAlkalineConservative,
    /// Piecewise 2xAAA NiMH curve used by the native reference firmware.
    TwoAaaNiMhReference,
}

/// The carrier documentation specifies two 1.5 V alkaline AAA cells, while
/// newer native firmware defaults to NiMH. Alkaline avoids over-reporting
/// capacity when chemistry was not configured.
pub const DEFAULT_BATTERY_CURVE: BatteryCurve = BatteryCurve::TwoAaaAlkalineConservative;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryReading {
    pub raw_adc: u16,
    pub millivolts: u16,
    /// ZCL BatteryVoltage, in 100 mV units.
    pub voltage_100mv: u8,
    /// ZCL BatteryPercentageRemaining, in half-percent units.
    pub percentage_remaining: u8,
}

pub struct BatteryMonitor {
    supply: SupplyMonitor,
    curve: BatteryCurve,
}

impl BatteryMonitor {
    /// Construct from an already-initialized supply monitor.
    ///
    /// Use this when the `SupplyMonitor` was obtained from a typed
    /// [`BoardResources`](efr32mg1_tradfri::resources::BoardResources) token.
    pub const fn from_supply(supply: SupplyMonitor, curve: BatteryCurve) -> Self {
        Self { supply, curve }
    }

    pub fn new(curve: BatteryCurve) -> Result<Self, SupplyError> {
        Ok(Self {
            supply: supply_monitor()?,
            curve,
        })
    }

    pub const fn curve(&self) -> BatteryCurve {
        self.curve
    }

    pub fn read(&mut self) -> Result<BatteryReading, SupplyError> {
        let reading = self.supply.read()?;
        Ok(BatteryReading {
            raw_adc: reading.raw_adc,
            millivolts: reading.millivolts,
            voltage_100mv: zcl_battery_voltage(reading.millivolts),
            percentage_remaining: battery_percentage(self.curve, reading.millivolts),
        })
    }
}

pub fn battery_monitor() -> Result<BatteryMonitor, SupplyError> {
    BatteryMonitor::new(DEFAULT_BATTERY_CURVE)
}

/// Convert millivolts to ZCL BatteryVoltage without producing reserved 0xFF.
pub const fn zcl_battery_voltage(millivolts: u16) -> u8 {
    let units = millivolts / 100;
    if units > 254 { 254 } else { units as u8 }
}

/// Convert supply voltage to ZCL half-percent battery capacity.
pub const fn battery_percentage(curve: BatteryCurve, millivolts: u16) -> u8 {
    match curve {
        BatteryCurve::TwoAaaAlkalineConservative => {
            const EMPTY_MV: u16 = 1_800;
            const FULL_MV: u16 = 3_100;
            if millivolts <= EMPTY_MV {
                0
            } else if millivolts >= FULL_MV {
                200
            } else {
                (((millivolts - EMPTY_MV) as u32 * 200) / (FULL_MV - EMPTY_MV) as u32) as u8
            }
        }
        BatteryCurve::TwoAaaNiMhReference => {
            let percent = if millivolts >= 2_700 {
                100
            } else if millivolts > 2_500 {
                80 + ((millivolts - 2_500) as u32 * 20) / 200
            } else if millivolts > 2_400 {
                50 + ((millivolts - 2_400) as u32 * 30) / 100
            } else if millivolts > 2_200 {
                10 + ((millivolts - 2_200) as u32 * 40) / 200
            } else if millivolts > 2_000 {
                ((millivolts - 2_000) as u32 * 10) / 200
            } else {
                0
            };
            (percent * 2) as u8
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{BatteryCurve, battery_percentage, zcl_battery_voltage};

    #[test]
    fn zcl_voltage_uses_100mv_units_and_reserves_unknown() {
        assert_eq!(zcl_battery_voltage(3_000), 30);
        assert_eq!(zcl_battery_voltage(3_099), 30);
        assert_eq!(zcl_battery_voltage(u16::MAX), 254);
    }

    #[test]
    fn conservative_alkaline_curve_clamps_and_interpolates() {
        let curve = BatteryCurve::TwoAaaAlkalineConservative;
        assert_eq!(battery_percentage(curve, 1_700), 0);
        assert_eq!(battery_percentage(curve, 1_800), 0);
        assert_eq!(battery_percentage(curve, 2_450), 100);
        assert_eq!(battery_percentage(curve, 3_100), 200);
        assert_eq!(battery_percentage(curve, 3_300), 200);
    }

    #[test]
    fn nimh_reference_curve_matches_piecewise_boundaries() {
        let curve = BatteryCurve::TwoAaaNiMhReference;
        assert_eq!(battery_percentage(curve, 2_000), 0);
        assert_eq!(battery_percentage(curve, 2_200), 20);
        assert_eq!(battery_percentage(curve, 2_300), 60);
        assert_eq!(battery_percentage(curve, 2_400), 100);
        assert_eq!(battery_percentage(curve, 2_450), 130);
        assert_eq!(battery_percentage(curve, 2_500), 160);
        assert_eq!(battery_percentage(curve, 2_600), 180);
        assert_eq!(battery_percentage(curve, 2_700), 200);
    }
}
