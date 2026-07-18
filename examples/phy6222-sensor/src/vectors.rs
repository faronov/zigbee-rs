//! PHY6222 peripheral interrupt definitions.
//!
//! IRQ numbers come from the PHY6222 SDK `bus_dev.h`, not the placeholder
//! STM32L0 values in the bundled CMSIS header. The ROM dispatch table itself
//! is emitted by `phy6222.x`.

/// PHY6222 peripheral interrupt numbers for NVIC control.
#[derive(Debug, Clone, Copy)]
#[repr(u16)]
#[allow(dead_code)]
pub enum Interrupt {
    /// Baseband / Link Layer radio (BB_IRQn = 4)
    LlIrq = 4,
    /// Key scan (KSCAN_IRQn = 5)
    Kscan = 5,
    /// RTC (RTC_IRQn = 6)
    Rtc = 6,
    /// Watchdog (WDT_IRQn = 10)
    Wdt = 10,
    /// UART 0 (UART0_IRQn = 11)
    Uart0 = 11,
    /// I2C 0 (I2C0_IRQn = 12)
    I2c0 = 12,
    /// I2C 1 (I2C1_IRQn = 13)
    I2c1 = 13,
    /// SPI 0 (SPI0_IRQn = 14)
    Spi0 = 14,
    /// SPI 1 (SPI1_IRQn = 15)
    Spi1 = 15,
    /// GPIO (GPIO_IRQn = 16)
    Gpio = 16,
    /// UART 1 (UART1_IRQn = 17)
    Uart1 = 17,
    /// SPI Flash (SPIF_IRQn = 18)
    Spif = 18,
    /// DMA (DMAC_IRQn = 19)
    Dmac = 19,
    /// Timer 1 (TIM1_IRQn = 20)
    Tim1 = 20,
    /// Timer 2 (TIM2_IRQn = 21)
    Tim2 = 21,
    /// Timer 3 (TIM3_IRQn = 22)
    Tim3 = 22,
    /// Timer 4 (TIM4_IRQn = 23)
    Tim4 = 23,
    /// Timer 5 (TIM5_IRQn = 24)
    Tim5 = 24,
    /// Timer 6 (TIM6_IRQn = 25)
    Tim6 = 25,
    /// AES (AES_IRQn = 28)
    Aes = 28,
    /// ADC (ADCC_IRQn = 29)
    Adcc = 29,
    /// Quadrature decoder (QDEC_IRQn = 30)
    Qdec = 30,
    /// RNG (RNG_IRQn = 31)
    Rng = 31,
}

unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    fn number(self) -> u16 {
        self as u16
    }
}
