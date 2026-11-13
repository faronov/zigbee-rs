//! Board support for the ST STM32WB5MM-DK (MB1292, STM32WB5MMG module).
//!
//! Only the physically fitted wiring (UM2825, ST BSP `stm32wb5mm_dk.h`):
//!
//! | Function | Wiring |
//! |----------|--------|
//! | Virtual COM port (ST-LINK V2-1, CN11) | USART1 TX PB6 / RX PB7, AF7 |
//! | Button B1 / B2 | PC12 / PC13, active low, pull-up |
//! | SPI1 (feature `lcd` / `rgb-led`) | SCK PA1, MOSI PA7 |
//! | SSD1315 128x64 OLED (`lcd`) | CS PH0, D/C PC9, RES PC8 |
//! | TLC59731 RGB LED (`rgb-led`) | SDI PA7 (= SPI1 MOSI), select PH1 |
//! | I2C3 (`sensors`) | SCL PB13, SDA PB11: STTS22H 0x38, ISM330DHCX 0x6B, VL53L0X 0x29 |
//! | Touch key (`touch`) | TSC G6: IO1 PD10 sample, IO2 PD11 key; G4: IO1 PC6 sample, IO2 PC7 shield |
//! | MP23DB01HP microphone (`mic`) | SAI1 block A PDM: CK2 PA8, D2 PA9 |
//!
//! The peripheral features only add constructors: a product that does not
//! use a peripheral leaves its feature off and links none of it.
//!
//! No Zigbee runtime, persistence, identity or flash layout lives here: the
//! product (`products/stm32wb5mm-dk-router`) selects them. The radio, RNG,
//! IPCC and internal flash are chip mechanisms in `stm32wb-hal`.

#![no_std]

#[cfg(any(feature = "rgb-led", feature = "lcd"))]
mod delay;
#[cfg(feature = "mic")]
pub mod mic;
#[cfg(feature = "sensors")]
pub mod sensors;
#[cfg(any(feature = "rgb-led", feature = "lcd"))]
pub mod spi1;
#[cfg(feature = "touch")]
pub mod touch;

#[cfg(any(feature = "rgb-led", feature = "lcd"))]
pub use delay::CycleDelay;

use stm32wb_hal::embassy_stm32::gpio::{Input, Pull};
use stm32wb_hal::embassy_stm32::interrupt::typelevel::Binding;
use stm32wb_hal::embassy_stm32::mode::{Async, Blocking};
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

/// Blocking USART1 TX on PB6 at [`VCOM_BAUDRATE`] (lab and fault output).
pub fn vcom_tx_blocking(
    usart1: Peri<'static, peripherals::USART1>,
    tx: Peri<'static, peripherals::PB6>,
) -> UartTx<'static, Blocking> {
    let mut config = usart::Config::default();
    config.baudrate = VCOM_BAUDRATE;
    UartTx::new_blocking(usart1, tx, config).unwrap_or_else(|_| panic!("USART1 115200 8N1"))
}

/// Button B1 (PC12).
pub fn button_b1(pin: Peri<'static, peripherals::PC12>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// Button B2 (PC13).
pub fn button_b2(pin: Peri<'static, peripherals::PC13>) -> Input<'static> {
    Input::new(pin, Pull::Up)
}

/// Clock tree of the board: [`rcc::Config::new_wpan`] (HSE 32 MHz, PLL
/// 64 MHz, LSE), plus the SAI1 kernel clock from PLL P (64 MHz) when the
/// microphone is enabled.
///
/// [`rcc::Config::new_wpan`]: stm32wb_hal::embassy_stm32::rcc::Config::new_wpan
pub fn rcc_config() -> stm32wb_hal::embassy_stm32::rcc::Config {
    #[allow(unused_mut)]
    let mut rcc = stm32wb_hal::embassy_stm32::rcc::Config::new_wpan();
    #[cfg(feature = "mic")]
    {
        rcc.mux.sai1sel = stm32wb_hal::embassy_stm32::pac::rcc::vals::Sai1sel::Pll1P;
    }
    rcc
}
