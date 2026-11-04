//! ESP32-H2 on-chip temperature sensor support.
//!
//! `esp-hal` 1.0 contains the generic TSENS driver, but its generated H2
//! metadata does not expose the peripheral. Keep the small H2-specific clock
//! sequence here until the HAL exposes it. Range selection and the per-chip
//! eFuse calibration are shared with the C6 in
//! [`esp32_zigbee_devkit_product::chip_temperature`].

use esp32_zigbee_devkit_product::chip_temperature;
use esp_hal::delay::Delay;
use esp_hal::peripherals::{APB_SARADC, SYSTEM};

pub use chip_temperature::Error;

pub struct H2TemperatureSensor;

impl H2TemperatureSensor {
    pub fn new() -> Result<Self, Error> {
        let system = SYSTEM::regs();

        system
            .saradc_conf()
            .modify(|_, w| w.saradc_reg_clk_en().set_bit());
        system
            .saradc_conf()
            .modify(|_, w| w.saradc_reg_rst_en().set_bit());
        system
            .saradc_conf()
            .modify(|_, w| w.saradc_reg_rst_en().clear_bit());

        system
            .tsens_clk_conf()
            .modify(|_, w| w.tsens_clk_en().set_bit());
        system
            .tsens_clk_conf()
            .modify(|_, w| w.tsens_rst_en().set_bit());
        system
            .tsens_clk_conf()
            .modify(|_, w| w.tsens_rst_en().clear_bit());
        system
            .tsens_clk_conf()
            .modify(|_, w| w.tsens_clk_sel().set_bit());

        chip_temperature::configure()?;
        Ok(Self)
    }

    /// Sample the die temperature in 0.01 °C.
    ///
    /// The −10…80 °C range is re-selected before every sample because the
    /// analog block may lose it across sleep; the read-back DAC offset and
    /// the per-chip eFuse delta are applied as ESP-IDF v5.5 does.
    pub fn read_centi_celsius(&self) -> Result<i16, Error> {
        let calibration = chip_temperature::configure()?;
        let saradc = APB_SARADC::regs();
        saradc.tsens_ctrl().modify(|_, w| w.pu().set_bit());
        Delay::new().delay_micros(300);

        let raw = saradc.tsens_ctrl().read().out().bits();
        saradc.tsens_ctrl().modify(|_, w| w.pu().clear_bit());

        Ok(calibration.centi_celsius(raw))
    }

    /// [`Self::read_centi_celsius`] with the SAR ADC register and TSENS
    /// clocks enabled only for the conversion, so neither keeps running
    /// from XTAL through retained light sleep.
    #[allow(dead_code)] // Used by the thermometer bin only.
    pub fn read_gated_centi_celsius(&self) -> Result<i16, Error> {
        set_clocks(true);
        let result = self.read_centi_celsius();
        set_clocks(false);
        result
    }
}

#[allow(dead_code)]
fn set_clocks(on: bool) {
    let system = SYSTEM::regs();
    system
        .saradc_conf()
        .modify(|_, w| w.saradc_reg_clk_en().bit(on));
    system
        .tsens_clk_conf()
        .modify(|_, w| w.tsens_clk_en().bit(on));
}
