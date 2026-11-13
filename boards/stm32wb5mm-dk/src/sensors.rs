//! I2C3 sensor bus (ST BSP `stm32wb5mm_dk_bus.h`, UM2825).

use stm32wb_hal::embassy_stm32::i2c::{self, I2c, Master};
use stm32wb_hal::embassy_stm32::mode::Blocking;
use stm32wb_hal::embassy_stm32::time::Hertz;
use stm32wb_hal::embassy_stm32::{Peri, peripherals};

/// STTS22H temperature sensor, ADDR pin tied to VDD.
pub const STTS22H_ADDRESS: u8 = 0x38;
/// ISM330DHCX accelerometer/gyroscope, SA0 high (BSP `ISM330DHCX_I2C_ADD_H`).
pub const ISM330DHCX_ADDRESS: u8 = 0x6B;
/// VL53L0X time-of-flight sensor (default address, no XSHUT line).
pub const VL53L0X_ADDRESS: u8 = 0x29;
/// Bus clock: standard mode, as in the BSP.
pub const I2C3_HZ: u32 = 100_000;

/// The blocking I2C3 master.
pub type SensorBus = I2c<'static, Blocking, Master>;

/// I2C3 on SCL PB13 / SDA PB11 (AF4) at [`I2C3_HZ`]. The internal
/// pull-ups are enabled in addition to the board's resistors so that the
/// lines never float.
pub fn i2c3(
    i2c3: Peri<'static, peripherals::I2C3>,
    scl: Peri<'static, peripherals::PB13>,
    sda: Peri<'static, peripherals::PB11>,
) -> SensorBus {
    let mut config = i2c::Config::default();
    config.frequency = Hertz(I2C3_HZ);
    config.scl_pullup = true;
    config.sda_pullup = true;
    I2c::new_blocking(i2c3, scl, sda, config)
}
