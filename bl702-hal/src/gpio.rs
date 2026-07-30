//! Typed BL702 GPIO.

use core::convert::Infallible;
use core::marker::PhantomData;

use embedded_hal::digital::{ErrorType, InputPin, OutputPin, PinState, StatefulOutputPin};

use crate::mmio::{read32, rmw};

const GLB_BASE: u32 = 0x4000_0000;
const HBN_BASE: u32 = 0x4000_f000;
const GPIO_CONFIG_BASE: u32 = GLB_BASE + 0x100;
const GPIO_INPUT: u32 = GLB_BASE + 0x180;
const GPIO_OUTPUT: u32 = GLB_BASE + 0x188;
const GPIO_OUTPUT_ENABLE: u32 = GLB_BASE + 0x190;
const GPIO_USE_PSRAM_IO: u32 = GLB_BASE + 0x88;
const HBN_IRQ_MODE: u32 = HBN_BASE + 0x14;

const FUNCTION_GPIO: u8 = 11;
pub(crate) const FUNCTION_SPI: u8 = 4;
pub(crate) const FUNCTION_I2C: u8 = 6;
pub(crate) const FUNCTION_UART: u8 = 7;
pub(crate) const FUNCTION_PWM: u8 = 8;
pub(crate) const FUNCTION_ANALOG: u8 = 10;

/// Disabled GPIO mode.
pub struct Disabled;
/// Digital input mode.
pub struct Input;
/// Push-pull digital output mode.
pub struct Output;
/// Peripheral alternate-function mode.
pub(crate) struct Alternate;
/// Analog mode for ADC-capable pins.
pub struct Analog;

/// Pull resistor selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pull {
    None,
    Up,
    Down,
}

/// GPIO drive setting documented by the BL702 SDK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Drive {
    Milliamp8 = 0,
    Milliamp9_6 = 1,
    Milliamp11_2 = 2,
    Milliamp12_8 = 3,
}

/// Pin configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError {
    /// GPIO23..GPIO28 are currently routed to the flash/PSRAM pad bank.
    SharedFlashPadActive,
}

/// A uniquely owned BL702 pin.
pub struct Pin<const N: u8, MODE = Disabled> {
    _mode: PhantomData<MODE>,
}

impl<const N: u8, MODE> Pin<N, MODE> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }

    pub const fn number(&self) -> u8 {
        N
    }
}

impl<const N: u8> Pin<N, Disabled> {
    /// Configure a digital input with Schmitt trigger enabled.
    pub fn into_input(self, pull: Pull) -> Result<Pin<N, Input>, ConfigError> {
        configure::<N>(FUNCTION_GPIO, true, false, pull, Drive::Milliamp8, true)?;
        Ok(Pin::new())
    }

    /// Configure a push-pull output and set its initial level before enabling
    /// the output driver.
    pub fn into_push_pull_output(
        self,
        initial: PinState,
        drive: Drive,
    ) -> Result<Pin<N, Output>, ConfigError> {
        match initial {
            PinState::Low => write_output::<N>(false),
            PinState::High => write_output::<N>(true),
        }
        configure::<N>(FUNCTION_GPIO, false, true, Pull::None, drive, true)?;
        Ok(Pin::new())
    }

    pub(crate) fn into_alternate(
        self,
        function: u8,
        pull: Pull,
        drive: Drive,
    ) -> Result<Pin<N, Alternate>, ConfigError> {
        configure::<N>(function, true, false, pull, drive, true)?;
        Ok(Pin::new())
    }

    /// Configure the pad for an analog peripheral. The ADC driver validates
    /// whether the selected pin has an ADC channel.
    pub fn into_analog(self) -> Result<Pin<N, Analog>, ConfigError> {
        configure::<N>(
            FUNCTION_ANALOG,
            false,
            false,
            Pull::None,
            Drive::Milliamp8,
            false,
        )?;
        Ok(Pin::new())
    }
}

impl<const N: u8> ErrorType for Pin<N, Input> {
    type Error = Infallible;
}

impl<const N: u8> InputPin for Pin<N, Input> {
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read32(GPIO_INPUT) & (1 << N) != 0)
    }

    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.is_high().map(|high| !high)
    }
}

impl<const N: u8> ErrorType for Pin<N, Output> {
    type Error = Infallible;
}

impl<const N: u8> OutputPin for Pin<N, Output> {
    fn set_low(&mut self) -> Result<(), Self::Error> {
        write_output::<N>(false);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        write_output::<N>(true);
        Ok(())
    }
}

impl<const N: u8> StatefulOutputPin for Pin<N, Output> {
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        Ok(read32(GPIO_OUTPUT) & (1 << N) != 0)
    }

    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        self.is_set_high().map(|high| !high)
    }
}

fn configure<const N: u8>(
    function: u8,
    input_enable: bool,
    output_enable: bool,
    pull: Pull,
    drive: Drive,
    schmitt: bool,
) -> Result<(), ConfigError> {
    #[cfg(target_arch = "riscv32")]
    {
        riscv::interrupt::free(|| {
            configure_inner::<N>(function, input_enable, output_enable, pull, drive, schmitt)
        })
    }

    #[cfg(not(target_arch = "riscv32"))]
    configure_inner::<N>(function, input_enable, output_enable, pull, drive, schmitt)
}

fn configure_inner<const N: u8>(
    function: u8,
    input_enable: bool,
    output_enable: bool,
    pull: Pull,
    drive: Drive,
    schmitt: bool,
) -> Result<(), ConfigError> {
    debug_assert!(N < 32);
    if (23..=28).contains(&N) && read32(GPIO_USE_PSRAM_IO) & (1 << (N - 23)) != 0 {
        return Err(ConfigError::SharedFlashPadActive);
    }

    let (address, shift) = config_location(N);
    let field_mask = 0x1fff << shift;
    let mut field = u32::from(input_enable)
        | (u32::from(schmitt) << 1)
        | ((drive as u32) << 2)
        | (u32::from(matches!(pull, Pull::Up)) << 4)
        | (u32::from(matches!(pull, Pull::Down)) << 5)
        | (u32::from(function) << 8);
    field <<= shift;
    rmw(address, field_mask, field);

    rmw(GPIO_OUTPUT_ENABLE, 1 << N, u32::from(output_enable) << N);
    update_aon_input::<N>(input_enable);
    Ok(())
}

const fn config_location(pin: u8) -> (u32, u32) {
    (
        GPIO_CONFIG_BASE + (pin as u32 / 2) * 4,
        if pin & 1 == 0 { 0 } else { 16 },
    )
}

fn update_aon_input<const N: u8>(enable: bool) {
    if (9..=13).contains(&N) {
        let bit = 1 << (8 + N - 9);
        rmw(HBN_IRQ_MODE, bit, u32::from(enable) << (8 + N - 9));
    }
}

fn write_output<const N: u8>(high: bool) {
    let mask = 1 << N;
    #[cfg(target_arch = "riscv32")]
    riscv::interrupt::free(|| {
        rmw(GPIO_OUTPUT, mask, u32::from(high) << N);
    });

    #[cfg(not(target_arch = "riscv32"))]
    rmw(GPIO_OUTPUT, mask, u32::from(high) << N);
}

pub(crate) fn configure_open_drain<const N: u8>() -> Result<(), ConfigError> {
    write_output::<N>(false);
    configure::<N>(FUNCTION_GPIO, true, false, Pull::Up, Drive::Milliamp8, true)
}

pub(crate) fn configure_i2c<const N: u8>() -> Result<(), ConfigError> {
    configure::<N>(
        FUNCTION_I2C,
        true,
        false,
        Pull::Up,
        Drive::Milliamp9_6,
        true,
    )
}

pub(crate) fn drive_low<const N: u8>() {
    write_output::<N>(false);
    rmw(GPIO_OUTPUT_ENABLE, 1 << N, 1 << N);
}

pub(crate) fn release<const N: u8>() {
    rmw(GPIO_OUTPUT_ENABLE, 1 << N, 0);
}

pub(crate) fn read_level<const N: u8>() -> bool {
    read32(GPIO_INPUT) & (1 << N) != 0
}

macro_rules! pins {
    ($($field:ident: $number:literal),+ $(,)?) => {
        /// All BL702 pins as uniquely owned disabled tokens.
        pub struct Pins {
            $(pub $field: Pin<$number>,)+
        }

        impl Pins {
            pub(crate) const fn new() -> Self {
                Self {
                    $($field: Pin::new(),)+
                }
            }
        }
    };
}

pins!(
    p0: 0, p1: 1, p2: 2, p3: 3, p4: 4, p5: 5, p6: 6, p7: 7,
    p8: 8, p9: 9, p10: 10, p11: 11, p12: 12, p13: 13, p14: 14, p15: 15,
    p16: 16, p17: 17, p18: 18, p19: 19, p20: 20, p21: 21, p22: 22, p23: 23,
    p24: 24, p25: 25, p26: 26, p27: 27, p28: 28, p29: 29, p30: 30, p31: 31,
);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpio_config_fields_pack_two_pins_per_word() {
        assert_eq!(config_location(0), (GPIO_CONFIG_BASE, 0));
        assert_eq!(config_location(1), (GPIO_CONFIG_BASE, 16));
        assert_eq!(config_location(2), (GPIO_CONFIG_BASE + 4, 0));
        assert_eq!(config_location(31), (GPIO_CONFIG_BASE + 60, 16));
    }
}
