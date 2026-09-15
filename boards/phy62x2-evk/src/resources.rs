//! Exclusively owned resources mapped to PHY62x2 EVK wiring.

use crate::InternalFlash;
use crate::pins;
use phy6222_hal::Peripherals;
use phy6222_hal::adc::{Adc, AdcError, Channel};
use phy6222_hal::flash::FlashError;
use phy6222_hal::gpio::{self, Pull};
use phy6222_hal::i2c::{Config as I2cConfig, I2cError, I2cMaster, Speed};
use phy6222_hal::peripherals::{AdcToken, I2cToken};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoardError {
    AlreadyTaken,
    Flash(FlashError),
}

pub struct Resources {
    pub flash: InternalFlash,
    pub supply_monitor: SupplyMonitorToken,
    pub sensor_i2c: SensorI2cToken,
    pub status_led: StatusLed,
    pub user_button: UserButton,
}

impl Resources {
    pub fn take() -> Result<Self, BoardError> {
        let Peripherals {
            adc,
            flash,
            i2c0,
            i2c1: _,
        } = Peripherals::take().ok_or(BoardError::AlreadyTaken)?;

        gpio::set_output(pins::STATUS_LED);
        gpio::write(pins::STATUS_LED, true);
        gpio::set_input(pins::USER_BUTTON);
        gpio::set_pull(pins::USER_BUTTON, Pull::StrongPullUp);

        Ok(Self {
            flash: InternalFlash::new(flash).map_err(BoardError::Flash)?,
            supply_monitor: SupplyMonitorToken { adc },
            sensor_i2c: SensorI2cToken { i2c: i2c0 },
            status_led: StatusLed { _private: () },
            user_button: UserButton { _private: () },
        })
    }
}

/// Exclusive token for the EVK supply divider connected to ADC P11.
pub struct SupplyMonitorToken {
    adc: AdcToken,
}

impl SupplyMonitorToken {
    pub fn into_monitor(self) -> Result<SupplyMonitor, AdcError> {
        Ok(SupplyMonitor {
            adc: Adc::new(self.adc)?,
        })
    }
}

pub struct SupplyMonitor {
    adc: Adc,
}

impl SupplyMonitor {
    pub fn read_millivolts(&mut self) -> Result<u32, AdcError> {
        self.adc.read_mv(Channel::P11)
    }
}

/// Exclusive token for I2C0 on EVK header pins P2/P3.
pub struct SensorI2cToken {
    i2c: I2cToken,
}

impl SensorI2cToken {
    pub fn into_i2c(self) -> Result<I2cMaster, I2cError> {
        I2cMaster::new(
            self.i2c,
            I2cConfig {
                scl_pin: pins::SENSOR_I2C_SCL,
                sda_pin: pins::SENSOR_I2C_SDA,
                speed: Speed::Standard100k,
            },
        )
    }
}

/// Active-low fitted green status LED.
pub struct StatusLed {
    _private: (),
}

impl StatusLed {
    pub fn set_on(&mut self, on: bool) {
        gpio::write(pins::STATUS_LED, !on);
    }
}

/// Active-low fitted user button.
pub struct UserButton {
    _private: (),
}

impl UserButton {
    pub fn is_pressed(&self) -> bool {
        !gpio::read(pins::USER_BUTTON)
    }
}
