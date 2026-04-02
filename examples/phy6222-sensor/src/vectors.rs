//! PHY6222 interrupt vector table and peripheral interrupt definitions.
//!
//! Defines `__INTERRUPTS` for cortex-m-rt and an `Interrupt` enum for NVIC.
//! The names match `device.x` so user code can override any handler.
//!
//! IRQ numbers verified from PHY6222 SDK bus_dev.h — NOT the fake phy6222.h
//! CMSIS header which contains placeholder STM32L0 IRQ numbers.

/// Interrupt vector union (matches cortex-m-rt Vector layout).
#[repr(C)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

unsafe impl Sync for Vector {}

// Declare all interrupt handler symbols from device.x (weak aliases to DefaultHandler)
#[allow(dead_code)]
unsafe extern "C" {
    fn V0();
    fn V1();
    fn V2();
    fn V3();
    fn LL_IRQ();  // IRQ 4: BB (Baseband/radio) — SDK: BB_IRQn = 4
    fn KSCAN();
    fn RTC();
    fn V7();
    fn V8();
    fn V9();
    fn WDT();
    fn UART0();   // IRQ 11 — SDK: UART0_IRQn = 11
    fn I2C0();    // IRQ 12 — SDK: I2C0_IRQn = 12
    fn I2C1();    // IRQ 13 — SDK: I2C1_IRQn = 13
    fn SPI0();    // IRQ 14 — SDK: SPI0_IRQn = 14
    fn SPI1();    // IRQ 15 — SDK: SPI1_IRQn = 15
    fn GPIO();    // IRQ 16 — SDK: GPIO_IRQn = 16
    fn UART1();   // IRQ 17 — SDK: UART1_IRQn = 17
    fn SPIF();    // IRQ 18 — SDK: SPIF_IRQn = 18
    fn DMAC();    // IRQ 19 — SDK: DMAC_IRQn = 19
    fn TIM1();    // IRQ 20 — SDK: TIM1_IRQn = 20
    fn TIM2();    // IRQ 21 — SDK: TIM2_IRQn = 21
    fn TIM3();    // IRQ 22 — SDK: TIM3_IRQn = 22
    fn TIM4();    // IRQ 23 — SDK: TIM4_IRQn = 23
    fn TIM5();    // IRQ 24 — SDK: TIM5_IRQn = 24
    fn TIM6();    // IRQ 25 — SDK: TIM6_IRQn = 25
    fn V26();
    fn V27();
    fn AES();     // IRQ 28 — SDK: AES_IRQn = 28
    fn ADCC();    // IRQ 29 — SDK: ADCC_IRQn = 29
    fn QDEC();    // IRQ 30 — SDK: QDEC_IRQn = 30
    fn RNG();     // IRQ 31 — SDK: RNG_IRQn = 31
}

/// PHY6222 interrupt vector table — 32 entries for Cortex-M0.
///
/// Placed in `.vector_table.interrupts` by cortex-m-rt (requires `device` feature).
/// Each entry is either a handler function or reserved (0).
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
pub static __INTERRUPTS: [Vector; 32] = [
    Vector { reserved: 0 },      // IRQ  0: reserved
    Vector { reserved: 0 },      // IRQ  1: reserved
    Vector { reserved: 0 },      // IRQ  2: reserved
    Vector { reserved: 0 },      // IRQ  3: reserved
    Vector { handler: LL_IRQ },  // IRQ  4: BB (Baseband / LL radio)
    Vector { handler: KSCAN },   // IRQ  5: Key scan
    Vector { handler: RTC },     // IRQ  6: RTC
    Vector { reserved: 0 },      // IRQ  7: reserved
    Vector { reserved: 0 },      // IRQ  8: reserved
    Vector { reserved: 0 },      // IRQ  9: reserved
    Vector { handler: WDT },     // IRQ 10: Watchdog
    Vector { handler: UART0 },   // IRQ 11: UART 0
    Vector { handler: I2C0 },    // IRQ 12: I2C 0
    Vector { handler: I2C1 },    // IRQ 13: I2C 1
    Vector { handler: SPI0 },    // IRQ 14: SPI 0
    Vector { handler: SPI1 },    // IRQ 15: SPI 1
    Vector { handler: GPIO },    // IRQ 16: GPIO
    Vector { handler: UART1 },   // IRQ 17: UART 1
    Vector { handler: SPIF },    // IRQ 18: SPI Flash
    Vector { handler: DMAC },    // IRQ 19: DMA
    Vector { handler: TIM1 },    // IRQ 20: Timer 1
    Vector { handler: TIM2 },    // IRQ 21: Timer 2
    Vector { handler: TIM3 },    // IRQ 22: Timer 3
    Vector { handler: TIM4 },    // IRQ 23: Timer 4
    Vector { handler: TIM5 },    // IRQ 24: Timer 5
    Vector { handler: TIM6 },    // IRQ 25: Timer 6
    Vector { reserved: 0 },      // IRQ 26: reserved
    Vector { reserved: 0 },      // IRQ 27: reserved
    Vector { handler: AES },     // IRQ 28: AES
    Vector { handler: ADCC },    // IRQ 29: ADC
    Vector { handler: QDEC },    // IRQ 30: Quadrature decoder
    Vector { handler: RNG },     // IRQ 31: RNG
];

/// PHY6222 peripheral interrupt numbers for NVIC control.
///
/// Verified from SDK bus_dev.h and mcu_phy_bumbee.h.
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
