//! ESP32-H2 on-chip temperature sensor support.
//!
//! `esp-hal` 1.0 contains the generic TSENS driver, but its generated H2
//! metadata does not expose the peripheral. Keep the small H2-specific
//! register sequence here until the HAL exposes it.

use esp_hal::delay::Delay;
use esp_hal::peripherals::{APB_SARADC, I2C_ANA_MST, MODEM_LPCON, SYSTEM};

pub struct H2TemperatureSensor;

#[derive(Debug)]
pub enum Error {
    AnalogI2cBusy,
}

impl H2TemperatureSensor {
    const SAR_I2C_BLOCK: u8 = 0x69;
    const TSENS_DAC_REGISTER: u8 = 0x06;
    const TSENS_DAC_MASK: u8 = 0x0f;
    const TSENS_DAC_RANGE_MINUS_10_TO_80: u8 = 15;

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

        let current = Self::regi2c_read(Self::TSENS_DAC_REGISTER)?;
        let configured = (current & !Self::TSENS_DAC_MASK)
            | Self::TSENS_DAC_RANGE_MINUS_10_TO_80;
        Self::regi2c_write(Self::TSENS_DAC_REGISTER, configured)?;

        Ok(Self)
    }

    pub fn read_centi_celsius(&self) -> i16 {
        let saradc = APB_SARADC::regs();
        saradc.tsens_ctrl().modify(|_, w| w.pu().set_bit());
        Delay::new().delay_micros(300);

        let raw = saradc.tsens_ctrl().read().out().bits() as i32;
        saradc.tsens_ctrl().modify(|_, w| w.pu().clear_bit());

        // DAC range 15 uses offset 0 and covers -10..80 °C with ±1 °C error.
        ((raw * 4386 - 205200) / 100) as i16
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
        for _ in 0..100_000 {
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
        let master = Self::regi2c_master();
        Self::wait_regi2c(master)?;

        I2C_ANA_MST::regs()
            .i2c_ctrl(master)
            .write(|w| unsafe {
                w.slave_addr().bits(Self::SAR_I2C_BLOCK);
                w.slave_reg_addr().bits(register)
            });
        Self::wait_regi2c(master)?;

        Ok(I2C_ANA_MST::regs()
            .i2c_ctrl(master)
            .read()
            .data()
            .bits())
    }

    fn regi2c_write(register: u8, value: u8) -> Result<(), Error> {
        let master = Self::regi2c_master();
        Self::wait_regi2c(master)?;

        I2C_ANA_MST::regs()
            .i2c_ctrl(master)
            .write(|w| unsafe {
                w.slave_addr().bits(Self::SAR_I2C_BLOCK);
                w.slave_reg_addr().bits(register);
                w.read_write().set_bit();
                w.data().bits(value)
            });

        Self::wait_regi2c(master)
    }
}
