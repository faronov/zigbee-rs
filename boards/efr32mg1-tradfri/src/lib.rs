//! Board support for the EFR32MG1P TRADFRI module.

#![no_std]

pub mod storage;

use efr32mg1_hal::{
    clock::{self, ClockError, HfxoConfig},
    gpio::{Mode, Pin, Port},
    i2c::{Config as I2cConfig, I2c0, I2cError, PullUp},
};

pub const HCLK_HZ: u32 = 38_400_000;
pub const HFXO_CTUNE: u16 = 360;
pub const SENSOR_I2C_HZ: u32 = 10_000;
pub type SensorI2c = I2c0;

const LED_PIN: Pin = Pin::new(Port::A, 0);
const BUTTON_PIN: Pin = Pin::new(Port::B, 13);
const SENSOR_SDA_PIN: Pin = Pin::new(Port::C, 10);
const SENSOR_SCL_PIN: Pin = Pin::new(Port::C, 11);

/// Select the board's 38.4 MHz crystal before starting SysTick or radio code.
pub fn init_clocks() -> Result<(), ClockError> {
    clock::init_hfxo(HfxoConfig {
        frequency_hz: HCLK_HZ,
        ctune: HFXO_CTUNE,
    })
}

/// PA0 indicator, active high.
pub struct Led;

impl Led {
    pub const fn new() -> Self {
        Self
    }

    pub fn init(&self) {
        LED_PIN.configure(Mode::PushPull, false);
    }

    pub fn on(&self) {
        LED_PIN.set_high();
    }

    pub fn off(&self) {
        LED_PIN.set_low();
    }

    pub fn is_on(&self) -> bool {
        LED_PIN.output_is_high()
    }
}

impl Default for Led {
    fn default() -> Self {
        Self::new()
    }
}

/// PB13 user button, active low with pull-up and input filter.
pub struct Button;

impl Button {
    pub const fn new() -> Self {
        Self
    }

    pub fn init(&self) {
        BUTTON_PIN.configure(Mode::InputPullFilter, true);
    }

    pub fn is_pressed(&self) -> bool {
        !BUTTON_PIN.is_high()
    }
}

impl Default for Button {
    fn default() -> Self {
        Self::new()
    }
}

/// Construct board sensor I2C0: PC10 SDA, PC11 SCL, LOC15.
///
/// This mirrors the native reference's internal-pull-up fallback and therefore
/// limits SCL to 10 kHz. A board fitted with external pull-ups may select
/// `PullUp::External` and 100 kHz in a future board revision.
pub fn sensor_i2c() -> Result<SensorI2c, I2cError> {
    I2c0::new(I2cConfig {
        reference_hz: HCLK_HZ,
        bus_hz: SENSOR_I2C_HZ,
        sda: SENSOR_SDA_PIN,
        scl: SENSOR_SCL_PIN,
        location: 15,
        pull_up: PullUp::Internal,
        timeout_iterations: 200_000,
    })
}
