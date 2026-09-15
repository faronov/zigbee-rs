//! Board support for the Nordic nRF52840 DK (PCA10056).
//!
//! This crate exposes only the DK's physically fitted, board-specific
//! wiring: LED1, LED2, Button 1, and the external I2C sensor header
//! (BME280/SHT31 breakout). It has no dependency on `zigbee-runtime` and owns no
//! partition layout, NV placement, security persistence, or firmware
//! identity — those are product concerns. See `products/nrf52840-sensor`
//! for the product crate that builds on this board and selects them.
//!
//! The radio, RNG, SAADC, and internal flash controller (NVMC) are
//! chip-generic mechanisms already provided directly by `embassy-nrf` (this
//! board's role is analogous to `efr32mg1-hal` for the EFR32MG1P product —
//! the difference is `embassy-nrf` plays that chip-HAL role as an external
//! crate here, so there is no bespoke HAL crate in this workspace). This
//! board crate does not re-wrap them; callers construct
//! `embassy_nrf::radio`, `embassy_nrf::rng`, `embassy_nrf::saadc`, and
//! `embassy_nrf::nvmc` directly from `embassy_nrf::Peripherals`.

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

/// Construct LED1 (P0.13) as the product status output, initially off.
///
/// This named alias lets always-on products state the semantic purpose while
/// preserving [`led`] for the existing sensor composition.
pub fn status_led(pin: peripherals::P0_13) -> gpio::Output<'static> {
    led(pin)
}

/// Construct LED2 (P0.14) as an active-low RX-activity output, initially off.
pub fn rx_activity_led(pin: peripherals::P0_14) -> gpio::Output<'static> {
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
