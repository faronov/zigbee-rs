//! PHY6222 interrupt vector table and peripheral interrupt definitions.
//!
//! Defines `__INTERRUPTS` for cortex-m-rt and an `Interrupt` enum for NVIC.
//! The names match `device.x` so user code can override any handler.

/// Interrupt vector union (matches cortex-m-rt Vector layout).
#[repr(C)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

unsafe impl Sync for Vector {}

// Declare all interrupt handler symbols from device.x (weak aliases to DefaultHandler)
unsafe extern "C" {
    fn V0();
    fn TIM5();
    fn TIM6();
    fn SPIF();
    fn BB();
    fn KSCAN();
    fn RTC();
    fn V7();
    fn V8();
    fn V9();
    fn WDT();
    fn AP_TIM1();
    fn AP_TIM2();
    fn AP_TIM3();
    fn AP_TIM4();
    fn V15();
    fn V16();
    fn LL_IRQ();
    fn V18();
    fn V19();
    fn I2C0();
    fn I2C1();
    fn V22();
    fn GPIO();
    fn UART0();
    fn V25();
    fn V26();
    fn SPI0();
    fn SPI1();
    fn UART1();
    fn DMAC();
    fn AES();
}

/// PHY6222 interrupt vector table — 32 entries for Cortex-M0.
///
/// Placed in `.vector_table.interrupts` by cortex-m-rt (requires `device` feature).
/// Each entry is either a handler function or reserved (0).
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
pub static __INTERRUPTS: [Vector; 32] = [
    Vector { handler: V0 },      // IRQ  0: reserved
    Vector { handler: TIM5 },    // IRQ  1: Timer 5
    Vector { handler: TIM6 },    // IRQ  2: Timer 6
    Vector { handler: SPIF },    // IRQ  3: SPI Flash
    Vector { handler: BB },      // IRQ  4: Baseband
    Vector { handler: KSCAN },   // IRQ  5: Key scan
    Vector { handler: RTC },     // IRQ  6: RTC
    Vector { handler: V7 },      // IRQ  7: reserved
    Vector { handler: V8 },      // IRQ  8: reserved
    Vector { handler: V9 },      // IRQ  9: reserved
    Vector { handler: WDT },     // IRQ 10: Watchdog
    Vector { handler: AP_TIM1 }, // IRQ 11: App Timer 1
    Vector { handler: AP_TIM2 }, // IRQ 12: App Timer 2
    Vector { handler: AP_TIM3 }, // IRQ 13: App Timer 3
    Vector { handler: AP_TIM4 }, // IRQ 14: App Timer 4
    Vector { handler: V15 },     // IRQ 15: reserved
    Vector { handler: V16 },     // IRQ 16: reserved
    Vector { handler: LL_IRQ },  // IRQ 17: Link Layer (radio)
    Vector { handler: V18 },     // IRQ 18: reserved
    Vector { handler: V19 },     // IRQ 19: reserved
    Vector { handler: I2C0 },    // IRQ 20: I2C 0
    Vector { handler: I2C1 },    // IRQ 21: I2C 1
    Vector { handler: V22 },     // IRQ 22: reserved
    Vector { handler: GPIO },    // IRQ 23: GPIO
    Vector { handler: UART0 },   // IRQ 24: UART 0
    Vector { handler: V25 },     // IRQ 25: reserved
    Vector { handler: V26 },     // IRQ 26: reserved
    Vector { handler: SPI0 },    // IRQ 27: SPI 0
    Vector { handler: SPI1 },    // IRQ 28: SPI 1
    Vector { handler: UART1 },   // IRQ 29: UART 1
    Vector { handler: DMAC },    // IRQ 30: DMA
    Vector { handler: AES },     // IRQ 31: AES engine
];

/// PHY6222 peripheral interrupt numbers for NVIC control.
#[derive(Debug, Clone, Copy)]
#[repr(u16)]
#[allow(dead_code)]
pub enum Interrupt {
    Bb = 4,
    Rtc = 6,
    Wdt = 10,
    ApTim1 = 11,
    ApTim2 = 12,
    ApTim3 = 13,
    ApTim4 = 14,
    LlIrq = 17,
    I2c0 = 20,
    I2c1 = 21,
    Gpio = 23,
    Uart0 = 24,
    Spi0 = 27,
    Spi1 = 28,
    Uart1 = 29,
    Dmac = 30,
    Aes = 31,
}

unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    fn number(self) -> u16 {
        self as u16
    }
}
