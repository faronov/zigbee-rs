//! Interrupt vector table (`__INTERRUPTS`) and `Interrupt` numbers.
//!
//! From the SDK 8.33.00.16 `inc/hw_ints.h` (`INT_*` 16..53, 38 external
//! interrupts; identical for both families). Enable the `rt` feature and
//! link `link/device.x`; handlers are overridden by defining an
//! `extern "C"` function with the vector name.

/// Number of external interrupts.
pub const IRQ_COUNT: usize = 38;

/// External interrupt numbers (`INT_x - 16`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
#[allow(non_camel_case_types, clippy::upper_case_acronyms)]
pub enum Interrupt {
    AON_GPIO_EDGE = 0,
    I2C_IRQ = 1,
    RFC_CPE_1 = 2,
    PKA_IRQ = 3,
    AON_RTC_COMB = 4,
    UART0_COMB = 5,
    AUX_SWEV0 = 6,
    SSI0_COMB = 7,
    SSI1_COMB = 8,
    RFC_CPE_0 = 9,
    RFC_HW_COMB = 10,
    RFC_CMD_ACK = 11,
    I2S_IRQ = 12,
    AUX_SWEV1 = 13,
    WDT_IRQ = 14,
    GPT0A = 15,
    GPT0B = 16,
    GPT1A = 17,
    GPT1B = 18,
    GPT2A = 19,
    GPT2B = 20,
    GPT3A = 21,
    GPT3B = 22,
    CRYPTO_RESULT_AVAIL_IRQ = 23,
    DMA_DONE_COMB = 24,
    DMA_ERR = 25,
    FLASH = 26,
    SWEV0 = 27,
    AUX_COMB = 28,
    AON_PROG0 = 29,
    PROG0 = 30,
    AUX_COMPA = 31,
    AUX_ADC_IRQ = 32,
    TRNG_IRQ = 33,
    OSC_COMB = 34,
    AUX_TIMER2_EV0 = 35,
    UART1_COMB = 36,
    BATMON_COMB = 37,
}

// SAFETY: interrupt numbers are the device IRQ numbers.
unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    fn number(self) -> u16 {
        self as u16
    }
}

/// One entry of the external interrupt vector table.
#[derive(Clone, Copy)]
#[repr(C)]
#[allow(dead_code)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    _reserved: usize,
}

#[cfg(feature = "rt")]
pub mod table {
    unsafe extern "C" {
        fn AON_GPIO_EDGE();
        fn I2C_IRQ();
        fn RFC_CPE_1();
        fn PKA_IRQ();
        fn AON_RTC_COMB();
        fn UART0_COMB();
        fn AUX_SWEV0();
        fn SSI0_COMB();
        fn SSI1_COMB();
        fn RFC_CPE_0();
        fn RFC_HW_COMB();
        fn RFC_CMD_ACK();
        fn I2S_IRQ();
        fn AUX_SWEV1();
        fn WDT_IRQ();
        fn GPT0A();
        fn GPT0B();
        fn GPT1A();
        fn GPT1B();
        fn GPT2A();
        fn GPT2B();
        fn GPT3A();
        fn GPT3B();
        fn CRYPTO_RESULT_AVAIL_IRQ();
        fn DMA_DONE_COMB();
        fn DMA_ERR();
        fn FLASH();
        fn SWEV0();
        fn AUX_COMB();
        fn AON_PROG0();
        fn PROG0();
        fn AUX_COMPA();
        fn AUX_ADC_IRQ();
        fn TRNG_IRQ();
        fn OSC_COMB();
        fn AUX_TIMER2_EV0();
        fn UART1_COMB();
        fn BATMON_COMB();
    }

    #[unsafe(link_section = ".vector_table.interrupts")]
    #[unsafe(no_mangle)]
    #[used]
    pub static __INTERRUPTS: [super::Vector; super::IRQ_COUNT] = [
        super::Vector {
            handler: AON_GPIO_EDGE,
        },
        super::Vector { handler: I2C_IRQ },
        super::Vector { handler: RFC_CPE_1 },
        super::Vector { handler: PKA_IRQ },
        super::Vector {
            handler: AON_RTC_COMB,
        },
        super::Vector {
            handler: UART0_COMB,
        },
        super::Vector { handler: AUX_SWEV0 },
        super::Vector { handler: SSI0_COMB },
        super::Vector { handler: SSI1_COMB },
        super::Vector { handler: RFC_CPE_0 },
        super::Vector {
            handler: RFC_HW_COMB,
        },
        super::Vector {
            handler: RFC_CMD_ACK,
        },
        super::Vector { handler: I2S_IRQ },
        super::Vector { handler: AUX_SWEV1 },
        super::Vector { handler: WDT_IRQ },
        super::Vector { handler: GPT0A },
        super::Vector { handler: GPT0B },
        super::Vector { handler: GPT1A },
        super::Vector { handler: GPT1B },
        super::Vector { handler: GPT2A },
        super::Vector { handler: GPT2B },
        super::Vector { handler: GPT3A },
        super::Vector { handler: GPT3B },
        super::Vector {
            handler: CRYPTO_RESULT_AVAIL_IRQ,
        },
        super::Vector {
            handler: DMA_DONE_COMB,
        },
        super::Vector { handler: DMA_ERR },
        super::Vector { handler: FLASH },
        super::Vector { handler: SWEV0 },
        super::Vector { handler: AUX_COMB },
        super::Vector { handler: AON_PROG0 },
        super::Vector { handler: PROG0 },
        super::Vector { handler: AUX_COMPA },
        super::Vector {
            handler: AUX_ADC_IRQ,
        },
        super::Vector { handler: TRNG_IRQ },
        super::Vector { handler: OSC_COMB },
        super::Vector {
            handler: AUX_TIMER2_EV0,
        },
        super::Vector {
            handler: UART1_COMB,
        },
        super::Vector {
            handler: BATMON_COMB,
        },
    ];
}

#[cfg(feature = "rt")]
pub use table::*;
