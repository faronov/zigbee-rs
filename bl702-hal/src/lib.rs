//! Pure-Rust BL702 peripheral support.
//!
//! The crate contains chip mechanisms only. Board wiring, product flash
//! partitions, battery chemistry, and Zigbee application behavior belong in
//! their respective board and product crates.

#![no_std]

#[cfg(test)]
extern crate std;

pub mod adc;
pub mod clock;
pub mod efuse;
pub mod flash;
pub mod gpio;
pub mod i2c;
mod mmio;
pub mod peripherals;
pub mod pm;
pub mod pwm;
pub mod spi;
pub mod timer;
pub mod uart;

pub use peripherals::Peripherals;

#[cfg(target_arch = "riscv32")]
#[allow(dead_code)]
fn target_trait_compile_assertions() {
    fn assert_input<T: embedded_hal::digital::InputPin>() {}
    fn assert_output<T: embedded_hal::digital::OutputPin>() {}
    fn assert_i2c<T: embedded_hal::i2c::I2c>() {}
    fn assert_spi<T: embedded_hal::spi::SpiBus<u8>>() {}
    fn assert_flash<T: embedded_storage::nor_flash::NorFlash>() {}

    assert_input::<gpio::Pin<0, gpio::Input>>();
    assert_output::<gpio::Pin<0, gpio::Output>>();
    assert_i2c::<i2c::I2c0Bus<4, 3>>();
    assert_spi::<spi::Spi0Bus<7, 8, 9>>();
    assert_flash::<flash::XipFlash>();
}
