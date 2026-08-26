//! Physical Nordic PCA10059 nRF52840 Dongle wiring.
//!
//! LED1 is active low on P0.06. SW1 is active low on P1.06 with a pull-up and
//! is available to the application after the UF2 bootloader transfers control.

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
