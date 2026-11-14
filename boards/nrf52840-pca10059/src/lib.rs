//! Physical Nordic PCA10059 nRF52840 Dongle wiring.
//!
//! LED1 is active low on P0.06. LED2 is an active-low RGB LED: red P0.08,
//! green P1.09, blue P0.12. SW1 is active low on P1.06 with a pull-up and is
//! available to the application after the bootloader transfers control. The
//! side RESET button drives nRESET (P0.18) and enters the resident bootloader.

#![no_std]

use embassy_nrf::{gpio, peripherals};

pub const STATUS_LED_ACTIVE_LOW: bool = true;
pub const HAS_USER_BUTTON: bool = true;
pub const BUTTON_ACTIVE_LOW: bool = true;

/// P0.06 LED1, initially off.
pub fn status_led(pin: peripherals::P0_06) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::High, gpio::OutputDrive::Standard)
}

/// P1.06 SW1, active low with the board's expected pull-up.
pub fn button(pin: peripherals::P1_06) -> gpio::Input<'static> {
    gpio::Input::new(pin, gpio::Pull::Up)
}

/// LED2 red channel (P0.08), initially off.
pub fn led2_red(pin: peripherals::P0_08) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::High, gpio::OutputDrive::Standard)
}

/// LED2 green channel (P1.09), initially off.
pub fn led2_green(pin: peripherals::P1_09) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::High, gpio::OutputDrive::Standard)
}

/// LED2 blue channel (P0.12), initially off.
pub fn led2_blue(pin: peripherals::P0_12) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::High, gpio::OutputDrive::Standard)
}
