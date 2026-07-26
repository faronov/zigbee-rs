//! Board support for the EFR32MG1P TRADFRI module.

#![no_std]

pub mod resources;

use efr32mg1_hal::{
    adc::{Adc0, AdcError, Config as AdcConfig, avdd_raw_to_millivolts},
    clock::{self, ClockError, HfxoConfig},
    gpio::{InterruptEdge, Mode, Pin, Port},
    i2c::{Config as I2cConfig, I2c0, I2cError, PullUp},
    pwm::{
        Channel as PwmChannel, Config as PwmConfig, Polarity as PwmPolarity, PwmError, Timer0Pwm,
    },
    spi::{BitOrder, Config as SpiConfig, SpiError, Usart0Spi},
};
use embedded_hal::spi::MODE_0;

pub const HCLK_HZ: u32 = 38_400_000;
pub const HFXO_CTUNE: u16 = 360;
pub const SENSOR_I2C_HZ: u32 = 10_000;
pub const FLASH_SPI_HZ: u32 = 4_000_000;
pub const LED_PWM_HZ: u32 = 1_000;
pub type SensorI2c = I2c0;
pub type FlashSpi = Usart0Spi;
pub type LedPwm = Timer0Pwm;

const SUPPLY_ADC_HZ: u32 = 1_000_000;
const SUPPLY_ADC_TIMEOUT_ITERATIONS: u32 = 200_000;
const SUPPLY_SAMPLE_COUNT: u8 = 4;
const SUPPLY_MIN_VALID_MV: u16 = 1_800;
const SUPPLY_MAX_VALID_MV: u16 = 3_600;

const LED_PIN: Pin = Pin::new(Port::A, 0);
const BUTTON_PIN: Pin = Pin::new(Port::B, 13);
const SENSOR_SDA_PIN: Pin = Pin::new(Port::C, 10);
const SENSOR_SCL_PIN: Pin = Pin::new(Port::C, 11);
const FLASH_MOSI_PIN: Pin = Pin::new(Port::D, 15);
const FLASH_MISO_PIN: Pin = Pin::new(Port::D, 14);
const FLASH_CLOCK_PIN: Pin = Pin::new(Port::D, 13);
const FLASH_CS_PIN: Pin = Pin::new(Port::B, 11);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SupplyReading {
    pub raw_adc: u16,
    pub millivolts: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupplyError {
    Adc(AdcError),
    SupplyOutOfRange { millivolts: u16 },
}

impl From<AdcError> for SupplyError {
    fn from(error: AdcError) -> Self {
        Self::Adc(error)
    }
}

/// ADC0-backed monitor for the board supply rail. Product code decides how
/// millivolts map to a battery chemistry and ZCL attributes.
pub struct SupplyMonitor {
    adc: Adc0,
}

impl SupplyMonitor {
    pub fn new() -> Result<Self, SupplyError> {
        let adc = Adc0::new(AdcConfig {
            reference_hz: HCLK_HZ,
            adc_hz: SUPPLY_ADC_HZ,
            timeout_iterations: SUPPLY_ADC_TIMEOUT_ITERATIONS,
        })?;
        Ok(Self { adc })
    }

    pub fn read(&mut self) -> Result<SupplyReading, SupplyError> {
        let mut sum = 0u32;
        for _ in 0..SUPPLY_SAMPLE_COUNT {
            sum += self.adc.read_avdd_raw()? as u32;
        }
        let raw_adc = (sum / SUPPLY_SAMPLE_COUNT as u32) as u16;
        let millivolts = avdd_raw_to_millivolts(raw_adc)?;
        if !(SUPPLY_MIN_VALID_MV..=SUPPLY_MAX_VALID_MV).contains(&millivolts) {
            return Err(SupplyError::SupplyOutOfRange { millivolts });
        }

        Ok(SupplyReading {
            raw_adc,
            millivolts,
        })
    }
}

pub fn supply_monitor() -> Result<SupplyMonitor, SupplyError> {
    SupplyMonitor::new()
}

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
        BUTTON_PIN.configure_interrupt(InterruptEdge::Falling);
    }

    pub fn is_pressed(&self) -> bool {
        !BUTTON_PIN.is_high()
    }

    pub fn take_interrupt(&self) -> bool {
        if !BUTTON_PIN.interrupt_pending() {
            return false;
        }
        BUTTON_PIN.clear_interrupt();
        true
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

pub struct FlashSpiResources {
    pub bus: FlashSpi,
    pub chip_select: Pin,
}

/// Construct the board's external-flash SPI bus and active-low PB11 CS.
///
/// The bus uses USART0 in mode 0 at up to 4 MHz. `Pin` implements the
/// embedded-hal digital traits, so callers can wrap these resources in an
/// `embedded-hal-bus` exclusive SPI device without board-specific adapters.
pub fn flash_spi() -> Result<FlashSpiResources, SpiError> {
    let bus = Usart0Spi::new(SpiConfig {
        reference_hz: HCLK_HZ,
        bus_hz: FLASH_SPI_HZ,
        mode: MODE_0,
        bit_order: BitOrder::MostSignificantFirst,
        mosi: FLASH_MOSI_PIN,
        miso: FLASH_MISO_PIN,
        clock: FLASH_CLOCK_PIN,
        mosi_location: 23,
        miso_location: 21,
        clock_location: 19,
        timeout_iterations: 200_000,
    })?;
    FLASH_CS_PIN.configure(Mode::PushPull, true);
    Ok(FlashSpiResources {
        bus,
        chip_select: FLASH_CS_PIN,
    })
}

/// Route TIMER0 CC0 LOC0 to the active-high PA0 board LED.
///
/// This is mutually exclusive with `Led`: both own the same physical pin.
pub fn led_pwm() -> Result<LedPwm, PwmError> {
    Timer0Pwm::new(PwmConfig {
        reference_hz: HCLK_HZ,
        frequency_hz: LED_PWM_HZ,
        pin: LED_PIN,
        channel: PwmChannel::Cc0,
        location: 0,
        polarity: PwmPolarity::ActiveHigh,
    })
}

#[cfg(test)]
mod tests {
    use super::{FLASH_SPI_HZ, LED_PWM_HZ};

    #[test]
    fn board_peripheral_defaults_are_conservative() {
        assert_eq!(FLASH_SPI_HZ, 4_000_000);
        assert_eq!(LED_PWM_HZ, 1_000);
    }
}
