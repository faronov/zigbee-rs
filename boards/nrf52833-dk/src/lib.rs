//! Board support for the Nordic nRF52833 DK (PCA10100).
//!
//! Mirrors `boards/nrf52840-dk`: this crate exposes only the DK's
//! physically fitted, board-specific wiring — LED1, Button 1, and the
//! Arduino-header I2C bus an external BME280/SHT31 breakout is wired to.
//! It has no dependency on `zigbee-runtime` and owns no partition layout,
//! NV placement, security persistence, or firmware identity — those are
//! product concerns. See `products/nrf52833-sensor`.
//!
//! PCA10100 keeps the PCA10056 (nRF52840 DK) LED/button assignment, so the
//! pins below are deliberately identical to `boards/nrf52840-dk`. They are
//! nevertheless declared here rather than shared: a board crate that
//! described two different PCBs could not be corrected for one of them.
//!
//! The radio, RNG, SAADC, TEMP, and internal flash controller (NVMC) are
//! chip-generic mechanisms already provided directly by `embassy-nrf`;
//! callers construct them from `embassy_nrf::Peripherals` in the
//! composition root.
//!
//! ## Fitted hardware (PCA10100)
//!
//! | Function | Pin | Notes |
//! |----------|-----|-------|
//! | LED1     | P0.13 | active low |
//! | LED2..4  | P0.14..P0.16 | not used by this firmware |
//! | Button 1 | P0.11 | active low, internal pull-up |
//! | Button 2..4 | P0.12, P0.24, P0.25 | not used by this firmware |
//! | I2C SDA  | P0.26 | Arduino header (external sensor breakout) |
//! | I2C SCL  | P0.27 | Arduino header (external sensor breakout) |
//! | 32 MHz XTAL | — | fitted; required by the 802.15.4 radio |

#![no_std]

use embassy_nrf::{gpio, interrupt, peripherals, twim};

/// LED1, active low (matches the DK schematic).
pub const LED_ACTIVE_LOW: bool = true;
/// Button 1, active low with an internal pull-up (matches the DK schematic).
pub const BUTTON_ACTIVE_LOW: bool = true;
/// External sensor header I2C bus frequency (BME280/SHT31 breakout).
pub const SENSOR_I2C_FREQUENCY: twim::Frequency = twim::Frequency::K400;

/// Construct LED1 (P0.13) as a push-pull output, initially off (active low).
pub fn led(pin: peripherals::P0_13) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::High, gpio::OutputDrive::Standard)
}

/// Construct Button 1 (P0.11) as an input with an internal pull-up.
pub fn button(pin: peripherals::P0_11) -> gpio::Input<'static> {
    gpio::Input::new(pin, gpio::Pull::Up)
}

/// The board's external sensor I2C bus: TWISPI0 on P0.26 (SDA) / P0.27 (SCL).
pub type SensorI2c<'d> = twim::Twim<'d, peripherals::TWISPI0>;

/// Construct the board's external sensor I2C bus (BME280/SHT31 header) at
/// the board's default 400 kHz.
///
/// `irqs` must bind `embassy_nrf::interrupt::typelevel::TWISPI0` to
/// `twim::InterruptHandler<peripherals::TWISPI0>` (typically via
/// `embassy_nrf::bind_interrupts!` in the composition root).
pub fn sensor_i2c<'d>(
    twispi0: peripherals::TWISPI0,
    irqs: impl interrupt::typelevel::Binding<
        interrupt::typelevel::TWISPI0,
        twim::InterruptHandler<peripherals::TWISPI0>,
    > + 'd,
    sda: peripherals::P0_26,
    scl: peripherals::P0_27,
) -> SensorI2c<'d> {
    let mut config = twim::Config::default();
    config.frequency = SENSOR_I2C_FREQUENCY;
    twim::Twim::new(twispi0, irqs, sda, scl, config)
}
