//! Board support for the ST USB dongle (MB1293C, STM32WB55CGU6) of the
//! P-NUCLEO-WB55 kit.
//!
//! Only the physically fitted wiring (UM2435, ST BSP
//! `stm32wbxx_usb_dongle.h`):
//!
//! | Function | Wiring |
//! |----------|--------|
//! | USB full speed (bus power, USB-A plug) | PA11 DM / PA12 DP |
//! | LED1 blue / LED2 green / LED3 red | PA4 / PB0 / PB1, active high |
//! | Button SW1 | PA10, active low, pull-up |
//! | Switch SW2 | BOOT0: ROM USB DFU bootloader / user flash |
//! | HSE / LSE | 32 MHz crystal / 32.768 kHz crystal |
//!
//! There is no debug probe and no UART bridge: images are loaded through
//! the ROM USB DFU bootloader (SW2 on BOOT0, then replug), and the only
//! console is the USB port itself. No Zigbee runtime, persistence,
//! identity or flash layout lives here: the product
//! (`products/wb55-dongle-router`) selects them.

#![no_std]

use stm32wb_hal::embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use stm32wb_hal::embassy_stm32::interrupt::typelevel::Binding;
use stm32wb_hal::embassy_stm32::usb::{self, Driver};
use stm32wb_hal::embassy_stm32::{Peri, peripherals, rcc};

/// SW1 pulls to ground when pressed.
pub const BUTTON_ACTIVE_LOW: bool = true;
/// A high level lights LED1-LED3.
pub const LED_ACTIVE_HIGH: bool = true;

fn led(pin: Peri<'static, impl stm32wb_hal::embassy_stm32::gpio::Pin>) -> Output<'static> {
    Output::new(pin, Level::Low, Speed::Low)
}

/// LED1, blue (PA4), off.
pub fn led1_blue(pin: Peri<'static, peripherals::PA4>) -> Output<'static> {
    led(pin)
}

/// LED2, green (PB0), off.
pub fn led2_green(pin: Peri<'static, peripherals::PB0>) -> Output<'static> {
    led(pin)
}

/// LED3, red (PB1), off.
pub fn led3_red(pin: Peri<'static, peripherals::PB1>) -> Output<'static> {
    led(pin)
}

/// Button SW1 (PA10).
pub fn button_sw1(pin: Peri<'static, peripherals::PA10>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// The USB full-speed device on the dongle's plug. Needs the 48 MHz clock
/// of [`rcc_config`].
pub fn usb(
    usb: Peri<'static, peripherals::USB>,
    dp: Peri<'static, peripherals::PA12>,
    dm: Peri<'static, peripherals::PA11>,
    irq: impl Binding<
        <peripherals::USB as usb::Instance>::Interrupt,
        usb::InterruptHandler<peripherals::USB>,
    > + 'static,
) -> Driver<'static, peripherals::USB> {
    Driver::new(usb, dp, dm, irq)
}

/// Clock tree of the board: [`rcc::Config::new_wpan`] (HSE 32 MHz, PLL
/// 64 MHz, LSE), the tree CPU2's radio requires, with the USB kernel clock
/// taken from the same crystal-locked PLL (PLLQ = 48 MHz).
pub fn rcc_config() -> rcc::Config {
    let mut config = rcc::Config::new_wpan();
    config.mux.clk48sel = rcc::mux::Clk48sel::Pll1Q;
    config
}
