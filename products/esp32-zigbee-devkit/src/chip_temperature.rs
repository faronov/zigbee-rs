//! ESP32-C6/H2 on-die temperature sensor (TSENS) range and calibration.
//!
//! Both chips share the same TSENS analog block. This module reproduces the
//! ESP-IDF v5.5 driver exactly (the references are behavioural sources, not
//! linked code):
//!
//! * `components/soc/{esp32c6,esp32h2}/temperature_sensor_periph.c` — the
//!   `{offset, reg_val, range_min, range_max, error_max}` range table. The
//!   default range (`s_tsens_idx = 2`) is DAC `15`, offset `0`,
//!   −10…80 °C, ±1 °C.
//! * `components/hal/{esp32c6,esp32h2}/include/hal/temperature_sensor_ll.h` —
//!   the range is the 4-bit `I2C_SARADC_TSENS_DAC` field (regi2c block
//!   `0x69`, register `0x06`, bits `3..0`), written read-modify-write.
//! * `components/esp_hw_support/sar_periph_ctrl_common.c`,
//!   `temperature_sensor_get_raw_value()` —
//!   `0.4386 * raw - 27.88 * offset - 20.52`.
//! * `components/efuse/{esp32c6,esp32h2}/esp_efuse_rtc_calib.c`,
//!   `esp_efuse_rtc_calib_get_tsens_val()` — the 9-bit `TEMP_CALIB` eFuse is
//!   sign-magnitude: bit 8 is the sign, bits `7..0` the magnitude in 0.1 °C.
//!   (ESP-IDF v5.3/v5.4 still returned 0 here for C6/H2, IDF-5236.)
//! * `components/esp_driver_tsens/src/temperature_sensor.c`,
//!   `parse_temp_sensor_raw_value()` — `celsius - delta_t / 10.0`.
//!
//! ESP-IDF truncates the uncorrected value to whole degrees before applying
//! the eFuse correction; this module keeps 0.01 °C resolution throughout.
//!
//! esp-hal 1.0's `TemperatureSensor` neither programs the range DAC nor
//! applies the eFuse correction, and hard-codes offset −1. Callers therefore
//! use [`configure`] to select the documented default range and read the
//! per-chip calibration, then convert with [`TsensCalibration::centi_celsius`].

/// `I2C_SARADC_TSENS_DAC` value for the −10…80 °C, ±1 °C range.
pub const DAC_MINUS_10_TO_80: u8 = 15;
/// Width mask of `I2C_SARADC_TSENS_DAC` (bits `3..0`).
pub const DAC_MASK: u8 = 0x0f;

/// DAC offset coefficient used by ESP-IDF for a programmed range DAC value.
///
/// Returns `None` for DAC values outside ESP-IDF's characterised table.
pub const fn dac_offset(dac: u8) -> Option<i8> {
    match dac & DAC_MASK {
        5 => Some(-2),
        7 => Some(-1),
        15 => Some(0),
        11 => Some(1),
        10 => Some(2),
        _ => None,
    }
}

/// Decode the 9-bit sign-magnitude `TEMP_CALIB` eFuse into 0.1 °C.
pub const fn efuse_delta_tenths(temp_calib: u16) -> i16 {
    let magnitude = (temp_calib & 0x00ff) as i16;
    if temp_calib & 0x0100 != 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// Per-boot TSENS conversion parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TsensCalibration {
    /// ESP-IDF DAC offset coefficient for the programmed range.
    pub dac_offset: i8,
    /// Per-chip eFuse correction in 0.1 °C (subtracted from the reading).
    pub efuse_delta_tenths: i16,
}

impl TsensCalibration {
    /// Build parameters from the read-back range DAC and raw eFuse field.
    pub const fn new(dac: u8, temp_calib: u16) -> Option<Self> {
        match dac_offset(dac) {
            Some(dac_offset) => Some(Self {
                dac_offset,
                efuse_delta_tenths: efuse_delta_tenths(temp_calib),
            }),
            None => None,
        }
    }

    /// Convert an 8-bit `APB_SARADC_TSENS_OUT` sample to 0.01 °C.
    pub const fn centi_celsius(&self, raw: u8) -> i16 {
        // Work in 1e-4 °C so every ESP-IDF coefficient is an exact integer:
        // 0.4386 -> 4386, 27.88 -> 278_800, 20.52 -> 205_200 and
        // delta/10 °C -> delta * 1000.
        let value = raw as i32 * 4_386
            - self.dac_offset as i32 * 278_800
            - 205_200
            - self.efuse_delta_tenths as i32 * 1_000;
        // |value| <= 1_676_030 + 255_000, so the quotient always fits in i16.
        (value / 100) as i16
    }
}

// ESP32-C5: esp-hal 1.1 has no TSENS driver for the C5 and this module's
// regi2c/eFuse access has not been ported to it, so C5 images get no die
// temperature source (the C5 sensor reports the measurement as unavailable).
#[cfg(all(target_os = "none", not(feature = "esp32c5")))]
mod hw {
    use esp_hal::efuse::{Efuse, TEMP_CALIB};
    use esp_hal::peripherals::{I2C_ANA_MST, MODEM_LPCON, PMU};

    use super::{DAC_MASK, DAC_MINUS_10_TO_80, TsensCalibration};

    const SAR_I2C_BLOCK: u8 = 0x69;
    const TSENS_DAC_REGISTER: u8 = 0x06;
    const REGI2C_BUSY_POLLS: u32 = 100_000;

    /// TSENS range configuration failure.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Error {
        /// The analog I2C master stayed busy.
        AnalogI2cBusy,
        /// The range DAC did not read back as a characterised value.
        UnexpectedDac(u8),
    }

    /// Select ESP-IDF's default −10…80 °C TSENS range and read the per-chip
    /// eFuse calibration.
    ///
    /// The caller must have enabled the APB SAR ADC and TSENS clocks. The
    /// range DAC is read back so a write the analog block did not accept is
    /// reported instead of silently converting with the wrong offset.
    pub fn configure() -> Result<TsensCalibration, Error> {
        enable_sar_regi2c();
        let current = regi2c_read(TSENS_DAC_REGISTER)?;
        regi2c_write(
            TSENS_DAC_REGISTER,
            (current & !DAC_MASK) | DAC_MINUS_10_TO_80,
        )?;
        let dac = regi2c_read(TSENS_DAC_REGISTER)? & DAC_MASK;
        let temp_calib = Efuse::read_field_le::<u16>(TEMP_CALIB);
        TsensCalibration::new(dac, temp_calib)
            .filter(|calibration| calibration.dac_offset == 0)
            .ok_or(Error::UnexpectedDac(dac))
    }

    /// `regi2c_ctrl_ll_i2c_sar_periph_enable()`: power the SAR/TSENS
    /// regi2c slave (and, on C6, release its reset).
    fn enable_sar_regi2c() {
        #[cfg(feature = "esp32c6")]
        PMU::regs()
            .rf_pwc()
            .modify(|_, w| w.perif_i2c_rstb().set_bit().xpd_perif_i2c().set_bit());
        #[cfg(feature = "esp32h2")]
        PMU::regs()
            .rf_pwc()
            .modify(|_, w| w.xpd_perif_i2c().set_bit());
    }

    fn regi2c_master() -> usize {
        MODEM_LPCON::regs()
            .clk_conf()
            .modify(|_, w| w.clk_i2c_mst_en().set_bit());

        let sar_uses_master_zero = I2C_ANA_MST::regs()
            .ana_conf2()
            .read()
            .sar_i2c_mst_sel()
            .bit_is_set();
        I2C_ANA_MST::regs().ana_conf1().write(|w| unsafe {
            w.bits(0x00ff_ffff);
            w.sar_i2c_rd().clear_bit()
        });

        if sar_uses_master_zero { 0 } else { 1 }
    }

    fn wait_regi2c(master: usize) -> Result<(), Error> {
        for _ in 0..REGI2C_BUSY_POLLS {
            if I2C_ANA_MST::regs()
                .i2c_ctrl(master)
                .read()
                .busy()
                .bit_is_clear()
            {
                return Ok(());
            }
            core::hint::spin_loop();
        }
        Err(Error::AnalogI2cBusy)
    }

    fn regi2c_read(register: u8) -> Result<u8, Error> {
        let master = regi2c_master();
        wait_regi2c(master)?;
        I2C_ANA_MST::regs().i2c_ctrl(master).write(|w| unsafe {
            w.slave_addr().bits(SAR_I2C_BLOCK);
            w.slave_reg_addr().bits(register)
        });
        wait_regi2c(master)?;
        Ok(I2C_ANA_MST::regs().i2c_ctrl(master).read().data().bits())
    }

    fn regi2c_write(register: u8, value: u8) -> Result<(), Error> {
        let master = regi2c_master();
        wait_regi2c(master)?;
        I2C_ANA_MST::regs().i2c_ctrl(master).write(|w| unsafe {
            w.slave_addr().bits(SAR_I2C_BLOCK);
            w.slave_reg_addr().bits(register);
            w.read_write().set_bit();
            w.data().bits(value)
        });
        wait_regi2c(master)
    }
}

#[cfg(all(target_os = "none", not(feature = "esp32c5")))]
pub use hw::{Error, configure};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn efuse_field_is_sign_magnitude_tenths() {
        assert_eq!(efuse_delta_tenths(0x000), 0);
        assert_eq!(efuse_delta_tenths(0x015), 21);
        assert_eq!(efuse_delta_tenths(0x115), -21);
        assert_eq!(efuse_delta_tenths(0x0ff), 255);
        assert_eq!(efuse_delta_tenths(0x1ff), -255);
        // Negative zero decodes to zero.
        assert_eq!(efuse_delta_tenths(0x100), 0);
        // Bits above the 9-bit field are ignored.
        assert_eq!(efuse_delta_tenths(0xfe15), 21);
    }

    #[test]
    fn dac_table_matches_esp_idf() {
        assert_eq!(dac_offset(5), Some(-2));
        assert_eq!(dac_offset(7), Some(-1));
        assert_eq!(dac_offset(15), Some(0));
        assert_eq!(dac_offset(11), Some(1));
        assert_eq!(dac_offset(10), Some(2));
        assert_eq!(dac_offset(0), None);
        assert_eq!(dac_offset(0xf0 | 15), Some(0));
    }

    #[test]
    fn conversion_matches_esp_idf_formula() {
        let uncalibrated = TsensCalibration::new(DAC_MINUS_10_TO_80, 0).unwrap();
        // 0.4386 * 100 - 20.52 = 23.34 °C
        assert_eq!(uncalibrated.centi_celsius(100), 2_334);
        // 0.4386 * 47 - 20.52 = 0.0942 °C
        assert_eq!(uncalibrated.centi_celsius(47), 9);
        // 0.4386 * 0 - 20.52
        assert_eq!(uncalibrated.centi_celsius(0), -2_052);

        // Positive eFuse delta lowers the reading by delta/10 °C.
        let plus = TsensCalibration::new(DAC_MINUS_10_TO_80, 0x015).unwrap();
        assert_eq!(plus.centi_celsius(100), 2_334 - 210);
        let minus = TsensCalibration::new(DAC_MINUS_10_TO_80, 0x115).unwrap();
        assert_eq!(minus.centi_celsius(100), 2_334 + 210);
    }

    #[test]
    fn esp_hal_offset_assumption_differs_by_dac_factor() {
        // esp-hal hard-codes offset -1 (DAC 7). Converting a DAC-15 sample
        // that way over-reports by exactly 27.88 °C.
        let assumed = TsensCalibration::new(7, 0).unwrap();
        let actual = TsensCalibration::new(15, 0).unwrap();
        assert_eq!(
            assumed.centi_celsius(100) - actual.centi_celsius(100),
            2_788
        );
    }

    #[test]
    fn extreme_inputs_do_not_overflow() {
        // Largest: DAC 5 (offset -2), raw 255, eFuse -25.5 °C.
        let max = TsensCalibration::new(5, 0x1ff).unwrap();
        assert_eq!(max.centi_celsius(255), 17_258);
        // Smallest: DAC 10 (offset 2), raw 0, eFuse +25.5 °C.
        let min = TsensCalibration::new(10, 0x0ff).unwrap();
        assert_eq!(min.centi_celsius(0), -10_178);
    }
}
