//! Board support for the ST Nucleo-WB55 (MB1355C, STM32WB55RG) of the
//! P-NUCLEO-WB55 kit.
//!
//! Only the physically fitted wiring (UM2435, ST BSP `stm32wbxx_nucleo.h`):
//!
//! | Function | Wiring |
//! |----------|--------|
//! | Virtual COM port (ST-LINK V2-1, CN15) | USART1 TX PB6 / RX PB7, AF7 |
//! | LED1 blue / LED2 green / LED3 red | PB5 / PB0 / PB1, active high |
//! | Button SW1 / SW2 / SW3 | PC4 / PD0 / PD1, active low, pull-up |
//! | HSE / LSE | 32 MHz crystal / 32.768 kHz crystal |
//!
//! No Zigbee runtime, persistence, identity or flash layout lives here: the
//! product (`products/nucleo-wb55-router`) selects them. The radio, RNG,
//! IPCC and internal flash are chip mechanisms in `stm32wb-hal`.

#![no_std]

use stm32wb_hal::embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use stm32wb_hal::embassy_stm32::interrupt::typelevel::Binding;
use stm32wb_hal::embassy_stm32::mode::{Async, Blocking};
use stm32wb_hal::embassy_stm32::usart::{self, UartTx};
use stm32wb_hal::embassy_stm32::{Peri, dma, peripherals};

/// Buttons SW1-SW3 pull to ground when pressed.
pub const BUTTON_ACTIVE_LOW: bool = true;
/// A high level lights LED1-LED3.
pub const LED_ACTIVE_HIGH: bool = true;
/// ST-LINK virtual COM port line rate, 8N1.
pub const VCOM_BAUDRATE: u32 = 115_200;

/// Transmit half of the ST-LINK virtual COM port.
pub type VcomTx = UartTx<'static, Async>;

/// USART1 TX on PB6 at [`VCOM_BAUDRATE`], driven by `dma`.
pub fn vcom_tx<D: usart::TxDma<peripherals::USART1>>(
    usart1: Peri<'static, peripherals::USART1>,
    tx: Peri<'static, peripherals::PB6>,
    dma: Peri<'static, D>,
    irq: impl Binding<D::Interrupt, dma::InterruptHandler<D>> + 'static,
) -> VcomTx {
    let mut config = usart::Config::default();
    config.baudrate = VCOM_BAUDRATE;
    // The only failure is an unreachable baud rate, which a fixed 115200
    // from the 64 MHz PCLK2 cannot hit.
    UartTx::new(usart1, tx, dma, irq, config).unwrap_or_else(|_| panic!("USART1 115200 8N1"))
}

/// Blocking USART1 TX on PB6 at [`VCOM_BAUDRATE`] (lab and fault output).
pub fn vcom_tx_blocking(
    usart1: Peri<'static, peripherals::USART1>,
    tx: Peri<'static, peripherals::PB6>,
) -> UartTx<'static, Blocking> {
    let mut config = usart::Config::default();
    config.baudrate = VCOM_BAUDRATE;
    UartTx::new_blocking(usart1, tx, config).unwrap_or_else(|_| panic!("USART1 115200 8N1"))
}

fn led(pin: Peri<'static, impl stm32wb_hal::embassy_stm32::gpio::Pin>) -> Output<'static> {
    Output::new(pin, Level::Low, Speed::Low)
}

/// LED1, blue (PB5), off.
pub fn led1_blue(pin: Peri<'static, peripherals::PB5>) -> Output<'static> {
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

/// Button SW1 (PC4).
pub fn button_sw1(pin: Peri<'static, peripherals::PC4>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// Button SW2 (PD0).
pub fn button_sw2(pin: Peri<'static, peripherals::PD0>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// Button SW3 (PD1).
pub fn button_sw3(pin: Peri<'static, peripherals::PD1>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// Clock tree of the board: [`rcc::Config::new_wpan`] (HSE 32 MHz, PLL
/// 64 MHz, LSE), the tree CPU2's radio requires.
///
/// [`rcc::Config::new_wpan`]: stm32wb_hal::embassy_stm32::rcc::Config::new_wpan
pub fn rcc_config() -> stm32wb_hal::embassy_stm32::rcc::Config {
    stm32wb_hal::embassy_stm32::rcc::Config::new_wpan()
}
