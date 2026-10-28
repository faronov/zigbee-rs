//! On-die battery and temperature monitor (PMUD "BATMON").
//!
//! Mirrors TI's `BatMonSupportLPF3_init`, `BatteryMonitor_getVoltage` and
//! `Temperature_getTemperature` (`source/ti/drivers/batterymonitor/` and
//! `source/ti/drivers/temperature/TemperatureLPF3.c`, SDK 8.40). The monitor
//! measures VDDS and the die temperature autonomously, including in standby;
//! reading it costs two register loads and no radio or CPU wake-ups.
//!
//! The temperature is the *die* temperature, not ambient air temperature.

/// `PMUD_BASE` (`hw_memmap.h`).
pub const PMUD_BASE: u32 = 0x4000_6000;
/// `PMUD.CTL`.
pub const PMUD_CTL: u32 = PMUD_BASE;
/// `PMUD.BAT`: VDDS, unsigned 3.8 fixed point volts in `[10:0]`.
pub const PMUD_BAT: u32 = PMUD_BASE + 0x28;
/// `PMUD.BATUPD`.
pub const PMUD_BATUPD: u32 = PMUD_BASE + 0x2C;
/// `PMUD.TEMP`: signed 9.2 fixed point degrees in `[16:6]`.
pub const PMUD_TEMP: u32 = PMUD_BASE + 0x30;
/// `PMUD.TEMPUPD`.
pub const PMUD_TEMPUPD: u32 = PMUD_BASE + 0x34;
/// `PMUD_CTL_CALC_EN | PMUD_CTL_MEAS_EN` with `HYST_EN` cleared; TI disables
/// the hardware hysteresis because of a temperature-accuracy erratum.
pub const CTL_ENABLE: u32 = 0x0000_0003;
/// `PMUD_BATUPD_STA` / `PMUD_TEMPUPD_STA`: a measurement has completed.
pub const UPD_STA: u32 = 0x0000_0001;

/// VDDS in millivolts from a raw `PMUD.BAT` value, rounded like TI.
pub const fn voltage_mv(bat: u32) -> u16 {
    (((bat & 0x7FF) * 1000 + (1 << 7)) >> 8) as u16
}

/// Uncorrected BATMON temperature in quarter degrees from raw `PMUD.TEMP`.
pub const fn temperature_quarter_c(temp: u32) -> i16 {
    let raw = ((temp >> 6) & 0x7FF) as i16;
    (raw << 5) >> 5
}

/// Die temperature in 0.01 °C from raw `PMUD.TEMP`, using TI's BATMON
/// linearity correction (`TemperatureLPF3_scaleToRealTemperature`:
/// `(raw * 1094172 - 7043721) / 2^22` on the quarter-degree value).
pub const fn temperature_centi_c(temp: u32) -> i16 {
    let scaled = (temperature_quarter_c(temp) as i64 * 1_094_172 - 7_043_721) * 100;
    let half = 1i64 << 21;
    let rounded = if scaled >= 0 {
        (scaled + half) >> 22
    } else {
        -((-scaled + half) >> 22)
    };
    rounded as i16
}

/// Shareable handle to the always-on BATMON. Reads are side-effect free,
/// so copies may be handed to independent sensor sources.
#[cfg(target_os = "none")]
#[derive(Debug, Clone, Copy)]
pub struct Batmon {
    _private: (),
}

#[cfg(target_os = "none")]
impl Batmon {
    /// Start continuous measurement. Called once from [`crate::take_core`].
    pub(crate) fn init() -> Self {
        // SAFETY: PMUD.CTL is a valid, always-on MMIO register.
        unsafe { core::ptr::write_volatile(PMUD_CTL as *mut u32, CTL_ENABLE) };
        Self { _private: () }
    }

    /// VDDS in millivolts, or `None` before the first measurement.
    pub fn voltage_mv(&self) -> Option<u16> {
        updated(PMUD_BATUPD).then(|| voltage_mv(read(PMUD_BAT)))
    }

    /// Corrected die temperature in 0.01 °C, or `None` before the first
    /// measurement.
    pub fn temperature_centi_c(&self) -> Option<i16> {
        updated(PMUD_TEMPUPD).then(|| temperature_centi_c(read(PMUD_TEMP)))
    }
}

#[cfg(target_os = "none")]
fn read(address: u32) -> u32 {
    // SAFETY: only called with the PMUD MMIO addresses above.
    unsafe { core::ptr::read_volatile(address as *const u32) }
}

#[cfg(target_os = "none")]
fn updated(address: u32) -> bool {
    read(address) & UPD_STA != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn voltage_decodes_3_8_fixed_point() {
        assert_eq!(voltage_mv(3 << 8), 3_000);
        assert_eq!(voltage_mv((3 << 8) | 0x4D), 3_301);
        assert_eq!(voltage_mv(0xFFFF_F000 | (2 << 8) | 0x80), 2_500);
    }

    #[test]
    fn temperature_sign_extends_and_applies_ti_correction() {
        assert_eq!(temperature_quarter_c(100 << 6), 100);
        assert_eq!(temperature_quarter_c(0x7FC << 6), -4);
        // TI: (100 * 1094172 - 7043721) / 2^22 = 24.41 °C.
        assert_eq!(temperature_centi_c(100 << 6), 2_441);
        assert_eq!(temperature_centi_c(0x7D8 << 6), -1_211);
    }
}
