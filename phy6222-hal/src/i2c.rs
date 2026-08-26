//! Pure-Rust DesignWare I2C master for PHY62x2.
//!
//! Polling mode, bounded waits, and `embedded-hal` 1.0 transactions.

use crate::gpio;
use crate::peripherals::{I2cInstance, I2cToken};
use crate::regs::*;
use embedded_hal::i2c::{
    Error, ErrorKind, ErrorType, I2c, NoAcknowledgeSource, Operation, SevenBitAddress,
};

const DATA_CMD_READ: u32 = 1 << 8;
const DATA_CMD_STOP: u32 = 1 << 9;
const DATA_CMD_RESTART: u32 = 1 << 10;
const RAW_TX_ABORT: u32 = 1 << 6;
const MAX_PIN_INDEX: u8 = 22;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    Standard100k,
    Fast400k,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    pub scl_pin: u8,
    pub sda_pin: u8,
    pub speed: Speed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2cError {
    InvalidConfiguration,
    Timeout,
    Abort,
}

impl Error for I2cError {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::InvalidConfiguration => ErrorKind::Other,
            Self::Timeout => ErrorKind::Bus,
            Self::Abort => ErrorKind::NoAcknowledge(NoAcknowledgeSource::Unknown),
        }
    }
}

pub struct I2cMaster {
    base: u32,
    _token: I2cToken,
}

impl I2cMaster {
    pub fn new(token: I2cToken, config: Config) -> Result<Self, I2cError> {
        if config.scl_pin > MAX_PIN_INDEX
            || config.sda_pin > MAX_PIN_INDEX
            || config.scl_pin == config.sda_pin
        {
            return Err(I2cError::InvalidConfiguration);
        }

        let (base, clock_bit, fmux_scl) = match token.instance {
            I2cInstance::I2c0 => (AP_I2C0_BASE, MOD_I2C0_BIT, FMUX_IIC0_SCL),
            I2cInstance::I2c1 => (AP_I2C1_BASE, MOD_I2C1_BIT, FMUX_IIC1_SCL),
        };

        reg_write(PCR_SW_CLK, reg_read(PCR_SW_CLK) | clock_bit);
        gpio::set_fmux(config.scl_pin, fmux_scl);
        gpio::set_fmux(config.sda_pin, fmux_scl + 1);
        gpio::set_pull(config.scl_pin, gpio::Pull::StrongPullUp);
        gpio::set_pull(config.sda_pin, gpio::Pull::StrongPullUp);

        reg_write(base + I2C_IC_ENABLE, 0);
        let speed_bits = match config.speed {
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

        Ok(Self {
            base,
            _token: token,
        })
    }

    fn set_target(&mut self, address: u8) -> Result<(), I2cError> {
        if address > 0x7f {
            return Err(I2cError::InvalidConfiguration);
        }
        reg_write(self.base + I2C_IC_ENABLE, 0);
        reg_write(self.base + I2C_IC_TAR, u32::from(address));
        let _ = reg_read(self.base + I2C_IC_CLR_TX_ABRT);
        reg_write(self.base + I2C_IC_ENABLE, 1);
        Ok(())
    }

    fn wait_for(&self, predicate: impl Fn() -> bool, iterations: u32) -> Result<(), I2cError> {
        for _ in 0..iterations {
            self.check_abort()?;
            if predicate() {
                return Ok(());
            }
            cortex_m::asm::nop();
        }
        Err(I2cError::Timeout)
    }

    fn wait_tx_ready(&self) -> Result<(), I2cError> {
        self.wait_for(
            || reg_read(self.base + I2C_IC_STATUS) & I2C_STATUS_TFNF != 0,
            10_000,
        )
    }

    fn wait_rx_ready(&self) -> Result<(), I2cError> {
        self.wait_for(
            || reg_read(self.base + I2C_IC_STATUS) & I2C_STATUS_RFNE != 0,
            50_000,
        )
    }

    fn wait_tx_empty(&self) -> Result<(), I2cError> {
        self.wait_for(
            || reg_read(self.base + I2C_IC_STATUS) & I2C_STATUS_TFE != 0,
            50_000,
        )
    }

    fn check_abort(&self) -> Result<(), I2cError> {
        if reg_read(self.base + I2C_IC_RAW_INTR_STAT) & RAW_TX_ABORT != 0 {
            let _ = reg_read(self.base + I2C_IC_CLR_TX_ABRT);
            Err(I2cError::Abort)
        } else {
            Ok(())
        }
    }

    fn write_operation(&mut self, bytes: &[u8], restart: bool, stop: bool) -> Result<(), I2cError> {
        for (index, byte) in bytes.iter().copied().enumerate() {
            self.wait_tx_ready()?;
            let mut command = u32::from(byte);
            if restart && index == 0 {
                command |= DATA_CMD_RESTART;
            }
            if stop && index + 1 == bytes.len() {
                command |= DATA_CMD_STOP;
            }
            reg_write(self.base + I2C_IC_DATA_CMD, command);
            self.check_abort()?;
        }
        self.wait_tx_empty()
    }

    fn read_operation(
        &mut self,
        bytes: &mut [u8],
        restart: bool,
        stop: bool,
    ) -> Result<(), I2cError> {
        let mut position = 0;
        while position < bytes.len() {
            let chunk = (bytes.len() - position).min(7);
            for index in 0..chunk {
                self.wait_tx_ready()?;
                let absolute = position + index;
                let mut command = DATA_CMD_READ;
                if restart && absolute == 0 {
                    command |= DATA_CMD_RESTART;
                }
                if stop && absolute + 1 == bytes.len() {
                    command |= DATA_CMD_STOP;
                }
                reg_write(self.base + I2C_IC_DATA_CMD, command);
                self.check_abort()?;
            }
            for index in 0..chunk {
                self.wait_rx_ready()?;
                bytes[position + index] = (reg_read(self.base + I2C_IC_DATA_CMD) & 0xff) as u8;
            }
            position += chunk;
        }
        Ok(())
    }

    fn transaction_inner(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), I2cError> {
        let Some(last) = operations.iter().rposition(|operation| match operation {
            Operation::Read(bytes) => !bytes.is_empty(),
            Operation::Write(bytes) => !bytes.is_empty(),
        }) else {
            return Ok(());
        };

        self.set_target(address)?;
        let mut started = false;
        for (index, operation) in operations.iter_mut().enumerate() {
            let final_operation = index == last;
            match operation {
                Operation::Read(bytes) if !bytes.is_empty() => {
                    self.read_operation(bytes, started, final_operation)?;
                    started = true;
                }
                Operation::Write(bytes) if !bytes.is_empty() => {
                    self.write_operation(bytes, started, final_operation)?;
                    started = true;
                }
                Operation::Read(_) | Operation::Write(_) => {}
            }
        }
        self.wait_tx_empty()
    }

    pub fn read(&mut self, address: u8, bytes: &mut [u8]) -> Result<(), I2cError> {
        self.transaction_inner(address, &mut [Operation::Read(bytes)])
    }

    pub fn write(&mut self, address: u8, bytes: &[u8]) -> Result<(), I2cError> {
        self.transaction_inner(address, &mut [Operation::Write(bytes)])
    }

    pub fn write_read(
        &mut self,
        address: u8,
        write: &[u8],
        read: &mut [u8],
    ) -> Result<(), I2cError> {
        self.transaction_inner(
            address,
            &mut [Operation::Write(write), Operation::Read(read)],
        )
    }
}

impl ErrorType for I2cMaster {
    type Error = I2cError;
}

impl I2c<SevenBitAddress> for I2cMaster {
    fn transaction(
        &mut self,
        address: SevenBitAddress,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        self.transaction_inner(address, operations)
    }
}
