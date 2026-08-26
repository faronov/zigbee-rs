//! Typed LP-EM-CC2340R5 LED and button wiring.

use core::marker::PhantomData;

pub const LED1_DIO: u8 = 7;
pub const LED2_DIO: u8 = 6;
pub const BUTTON1_DIO: u8 = 13;
pub const BUTTON2_DIO: u8 = 14;

#[cfg(target_os = "none")]
const GPIO_BASE: u32 = 0x4002_3000;
#[cfg(target_os = "none")]
const GPIO_DOUTSET: u32 = GPIO_BASE + 0x210;
#[cfg(target_os = "none")]
const GPIO_DOUTCLR: u32 = GPIO_BASE + 0x220;
#[cfg(target_os = "none")]
const GPIO_DOESET: u32 = GPIO_BASE + 0x510;
#[cfg(target_os = "none")]
const GPIO_DOECLR: u32 = GPIO_BASE + 0x520;
#[cfg(target_os = "none")]
const GPIO_DIN: u32 = GPIO_BASE + 0x700;

#[cfg(target_os = "none")]
const IOC_BASE: u32 = 0x4000_3000;
#[cfg(target_os = "none")]
const IOC_DIO0: u32 = IOC_BASE + 0x100;

const IOC_HYSTERESIS_ENABLE: u32 = 0x4000_0000;
const IOC_INPUT_ENABLE: u32 = 0x2000_0000;
const IOC_STANDBY_WAKE_ENABLE: u32 = 0x0004_0000;
const IOC_PULL_UP: u32 = 0x0000_4000;

/// Explicit IOC value for an ordinary push-pull GPIO output.
///
/// TI's LPF3 GPIO output configuration enables the input buffer so software
/// can read the driven level. Pulls, edge detection, inversion, and alternate
/// peripheral functions remain disabled.
pub const LED_IOC_CONFIGURATION: u32 = IOC_INPUT_ENABLE;

/// Explicit IOC value for an active-low button with an internal pull-up.
pub const BUTTON_IOC_CONFIGURATION: u32 =
    IOC_HYSTERESIS_ENABLE | IOC_INPUT_ENABLE | IOC_STANDBY_WAKE_ENABLE | IOC_PULL_UP;

pub struct Output;
pub struct InputPullUp;

/// Active-high board LED. Construction is private to the singleton board
/// resource path, so a DIO cannot be configured simultaneously in two modes.
pub struct Led<const DIO: u8> {
    _mode: PhantomData<Output>,
}

/// Physical LED1 resource on DIO7.
pub type Led1 = Led<LED1_DIO>;
/// Physical LED2 resource on DIO6.
pub type Led2 = Led<LED2_DIO>;

impl<const DIO: u8> Led<DIO> {
    #[cfg(target_os = "none")]
    pub(crate) fn configure(initially_on: bool) -> Self {
        assert_valid_dio(DIO);
        write_output(DIO, initially_on);
        write_ioc(DIO, LED_IOC_CONFIGURATION);
        write_register(GPIO_DOESET, bit(DIO));
        Self { _mode: PhantomData }
    }

    #[cfg(target_os = "none")]
    pub fn on(&mut self) {
        write_register(GPIO_DOUTSET, bit(DIO));
    }

    #[cfg(target_os = "none")]
    pub fn off(&mut self) {
        write_register(GPIO_DOUTCLR, bit(DIO));
    }

    #[cfg(target_os = "none")]
    pub fn set(&mut self, on: bool) {
        if on {
            self.on();
        } else {
            self.off();
        }
    }

    #[cfg(target_os = "none")]
    pub fn is_on(&self) -> bool {
        read_input(DIO)
    }
}

/// Active-low board button with an internal pull-up.
pub struct Button<const DIO: u8> {
    _mode: PhantomData<InputPullUp>,
}

/// Physical BTN1 resource on DIO13.
pub type Button1 = Button<BUTTON1_DIO>;
/// Physical BTN2 resource on DIO14.
pub type Button2 = Button<BUTTON2_DIO>;

impl<const DIO: u8> Button<DIO> {
    #[cfg(target_os = "none")]
    pub(crate) fn configure() -> Self {
        assert_valid_dio(DIO);
        write_register(GPIO_DOECLR, bit(DIO));
        write_ioc(DIO, BUTTON_IOC_CONFIGURATION);
        Self { _mode: PhantomData }
    }

    #[cfg(target_os = "none")]
    pub fn is_pressed(&self) -> bool {
        !read_input(DIO)
    }
}

#[cfg(target_os = "none")]
const fn bit(dio: u8) -> u32 {
    1u32 << dio
}

#[cfg(target_os = "none")]
fn assert_valid_dio(dio: u8) {
    assert!(dio < 32);
}

#[cfg(target_os = "none")]
fn write_ioc(dio: u8, configuration: u32) {
    write_register(IOC_DIO0 + u32::from(dio) * 4, configuration);
}

#[cfg(target_os = "none")]
fn write_output(dio: u8, high: bool) {
    write_register(if high { GPIO_DOUTSET } else { GPIO_DOUTCLR }, bit(dio));
}

#[cfg(target_os = "none")]
fn read_input(dio: u8) -> bool {
    let value = unsafe { core::ptr::read_volatile(GPIO_DIN as *const u32) };
    value & bit(dio) != 0
}

#[cfg(target_os = "none")]
fn write_register(address: u32, value: u32) {
    unsafe { core::ptr::write_volatile(address as *mut u32, value) };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn documented_board_pins_are_unique() {
        let pins = [LED1_DIO, LED2_DIO, BUTTON1_DIO, BUTTON2_DIO];
        for (index, pin) in pins.iter().enumerate() {
            assert!(*pin < 32);
            assert!(!pins[..index].contains(pin));
        }
    }

    #[test]
    fn led_and_button_ioc_modes_are_explicit() {
        assert_eq!(LED_IOC_CONFIGURATION & 0x7, 0);
        assert_eq!(LED_IOC_CONFIGURATION & IOC_INPUT_ENABLE, IOC_INPUT_ENABLE);
        assert_eq!(
            BUTTON_IOC_CONFIGURATION,
            IOC_HYSTERESIS_ENABLE | IOC_INPUT_ENABLE | IOC_STANDBY_WAKE_ENABLE | IOC_PULL_UP
        );
        assert_eq!(BUTTON_IOC_CONFIGURATION & 0x7, 0);
    }
}
