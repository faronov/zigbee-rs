//! Board support for the ST STM32WB5MM-DK (MB1292, STM32WB5MMG module).
//!
//! Only the physically fitted wiring (UM2825, ST BSP `stm32wb5mm_dk.h`):
//!
//! | Function | Wiring |
//! |----------|--------|
//! | Virtual COM port (ST-LINK V2-1, CN11) | USART1 TX PB6 / RX PB7, AF7 |
//! | Button B1 / B2 | PC12 / PC13, active low, pull-up |
//!
//! No Zigbee runtime, persistence, identity or flash layout lives here: the
//! product (`products/stm32wb5mm-dk-router`) selects them. The radio, RNG,
//! IPCC and internal flash are chip mechanisms in `stm32wb-hal`.

#![no_std]

use stm32wb_hal::embassy_stm32::gpio::{Input, Pull};
use stm32wb_hal::embassy_stm32::interrupt::typelevel::Binding;
use stm32wb_hal::embassy_stm32::mode::Async;
use stm32wb_hal::embassy_stm32::usart::{self, UartTx};
use stm32wb_hal::embassy_stm32::{Peri, dma, peripherals};

/// Buttons B1 and B2 pull to ground when pressed.
pub const BUTTON_ACTIVE_LOW: bool = true;
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

/// Button B1 (PC12).
pub fn button_b1(pin: Peri<'static, peripherals::PC12>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// Button B2 (PC13).
pub fn button_b2(pin: Peri<'static, peripherals::PC13>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}
