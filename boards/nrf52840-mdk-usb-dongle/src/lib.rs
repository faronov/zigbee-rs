//! Physical Makerdiary nRF52840 MDK USB Dongle wiring.
//!
//! The RGB LED's green channel is active low on P0.22. The UF2 board
//! definition exposes only RESET/unconnected DFU inputs, so this product has
//! no usable application button.

#![no_std]

use embassy_nrf::{gpio, peripherals};

pub const STATUS_LED_ACTIVE_LOW: bool = true;
pub const HAS_USER_BUTTON: bool = false;

/// P0.22 green status LED, initially off.
pub fn status_led(pin: peripherals::P0_22) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::High, gpio::OutputDrive::Standard)
}
