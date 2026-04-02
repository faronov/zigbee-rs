//! Pure-Rust I2C master driver for PHY6222 (DesignWare I2C IP).
//!
//! Polling-mode, 100kHz or 400kHz. FIFO depth 8, reads chunked to 7 bytes.

use crate::regs::*;
use crate::gpio;

/// I2C peripheral instance.
#[derive(Clone, Copy)]
pub enum I2cDev { I2C0, I2C1 }

/// I2C speed mode.
#[derive(Clone, Copy)]
pub enum Speed { Standard100k, Fast400k }

/// I2C configuration.
pub struct Config {
    pub dev: I2cDev,
    pub scl_pin: u8,
    pub sda_pin: u8,
    pub speed: Speed,
}

/// I2C master driver.
pub struct I2cMaster {
    base: u32,
}

impl I2cMaster {
    /// Initialize I2C master.
    pub fn new(config: Config) -> Self {
        let base = match config.dev {
            I2cDev::I2C0 => AP_I2C0_BASE,
            I2cDev::I2C1 => AP_I2C1_BASE,
        };

        // Enable clock gate
        let clk_bit = match config.dev {
            I2cDev::I2C0 => MOD_I2C0_BIT,
            I2cDev::I2C1 => MOD_I2C1_BIT,
        };
        reg_write(PCR_SW_CLK, reg_read(PCR_SW_CLK) | clk_bit);

        // Pin mux
        let fmux_scl = match config.dev {
            I2cDev::I2C0 => FMUX_IIC0_SCL,
            I2cDev::I2C1 => FMUX_IIC1_SCL,
        };
        gpio::set_fmux(config.scl_pin, fmux_scl);
        gpio::set_fmux(config.sda_pin, fmux_scl + 1);
        gpio::set_pull(config.scl_pin, gpio::Pull::StrongPullUp);
        gpio::set_pull(config.sda_pin, gpio::Pull::StrongPullUp);

        // Configure
        reg_write(base + I2C_IC_ENABLE, 0);
        let speed_bits: u32 = match config.speed {
            Speed::Standard100k => 1 << 1,
            Speed::Fast400k => 2 << 1,
        };
        reg_write(base + I2C_IC_CON, 0x61 | speed_bits);

        match config.speed {
            Speed::Standard100k => {
                reg_write(base + I2C_IC_SS_SCL_HCNT, 72);
                reg_write(base + I2C_IC_SS_SCL_LCNT, 88);
            }
            Speed::Fast400k => {
                reg_write(base + I2C_IC_FS_SCL_HCNT, 14);
                reg_write(base + I2C_IC_FS_SCL_LCNT, 24);
            }
        }

        reg_write(base + I2C_IC_INTR_MASK, 0);
        reg_write(base + I2C_IC_RX_TL, 0);
        reg_write(base + I2C_IC_TX_TL, 1);
        reg_write(base + I2C_IC_ENABLE, 1);

        log::info!("[I2C] Init at 0x{:08X}", base);
        Self { base }
    }

    /// Set target slave address (7-bit).
    fn set_target(&self, addr: u8) {
        reg_write(self.base + I2C_IC_ENABLE, 0);
        reg_write(self.base + I2C_IC_TAR, addr as u32);
        reg_write(self.base + I2C_IC_ENABLE, 1);
    }

    fn wait_tx_ready(&self) -> bool {
        for _ in 0..10_000u32 {
            if reg_read(self.base + I2C_IC_STATUS) & I2C_STATUS_TFNF != 0 {
                return true;
            }
        }
        false
    }

    fn wait_rx_ready(&self) -> bool {
        for _ in 0..50_000u32 {
            if reg_read(self.base + I2C_IC_STATUS) & I2C_STATUS_RFNE != 0 {
                return true;
            }
        }
        false
    }

    fn wait_tx_empty(&self) -> bool {
        for _ in 0..50_000u32 {
            if reg_read(self.base + I2C_IC_STATUS) & I2C_STATUS_TFE != 0 {
                return true;
            }
        }
        false
    }

    fn check_abort(&self) -> bool {
        if reg_read(self.base + I2C_IC_RAW_INTR_STAT) & 0x40 != 0 {
            let _ = reg_read(self.base + I2C_IC_CLR_TX_ABRT);
            return true;
        }
        false
    }

    /// Write bytes then read bytes (repeated start).
    pub fn write_read(&self, addr: u8, wr: &[u8], rd: &mut [u8]) -> Result<(), ()> {
        self.set_target(addr);

        for &b in wr {
            if !self.wait_tx_ready() { return Err(()); }
            reg_write(self.base + I2C_IC_DATA_CMD, b as u32);
            if self.check_abort() { return Err(()); }
        }
        if !self.wait_tx_empty() { return Err(()); }

        let mut pos = 0;
        while pos < rd.len() {
            let chunk = (rd.len() - pos).min(7);
            for _ in 0..chunk {
                if !self.wait_tx_ready() { return Err(()); }
                reg_write(self.base + I2C_IC_DATA_CMD, 0x100);
            }
            for i in 0..chunk {
                if !self.wait_rx_ready() { return Err(()); }
                rd[pos + i] = (reg_read(self.base + I2C_IC_DATA_CMD) & 0xFF) as u8;
            }
            pos += chunk;
        }
        Ok(())
    }

    /// Write bytes only.
    pub fn write(&self, addr: u8, data: &[u8]) -> Result<(), ()> {
        self.set_target(addr);
        for &b in data {
            if !self.wait_tx_ready() { return Err(()); }
            reg_write(self.base + I2C_IC_DATA_CMD, b as u32);
            if self.check_abort() { return Err(()); }
        }
        if !self.wait_tx_empty() { return Err(()); }
        Ok(())
    }

    /// Read bytes only.
    pub fn read(&self, addr: u8, buf: &mut [u8]) -> Result<(), ()> {
        self.set_target(addr);
        let mut pos = 0;
        while pos < buf.len() {
            let chunk = (buf.len() - pos).min(7);
            for _ in 0..chunk {
                if !self.wait_tx_ready() { return Err(()); }
                reg_write(self.base + I2C_IC_DATA_CMD, 0x100);
            }
            for i in 0..chunk {
                if !self.wait_rx_ready() { return Err(()); }
                buf[pos + i] = (reg_read(self.base + I2C_IC_DATA_CMD) & 0xFF) as u8;
            }
            pos += chunk;
        }
        Ok(())
    }
}
