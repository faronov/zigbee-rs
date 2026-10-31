//! Interrupt vector table (`__INTERRUPTS`) and `Interrupt` numbers.
//!
//! Generated from the GSDK 4.5.0 device headers (`IRQn_Type`). Enable the
//! `rt` feature and link the matching `link/<chip>/device.x`; handlers are
//! overridden by defining an `extern "C"` function with the vector name.

#[cfg(feature = "efr32mg22")]
mod table {
    /// Number of external interrupts on EFR32MG22.
    pub const IRQ_COUNT: usize = 64;

    /// External interrupt numbers (GSDK `IRQn_Type`).
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[repr(u16)]
    #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
    pub enum Interrupt {
        CRYPTOACC = 0,
        TRNG = 1,
        PKE = 2,
        SMU_SECURE = 3,
        SMU_S_PRIVILEGED = 4,
        SMU_NS_PRIVILEGED = 5,
        EMU = 6,
        TIMER0 = 7,
        TIMER1 = 8,
        TIMER2 = 9,
        TIMER3 = 10,
        TIMER4 = 11,
        RTCC = 12,
        USART0_RX = 13,
        USART0_TX = 14,
        USART1_RX = 15,
        USART1_TX = 16,
        ICACHE0 = 17,
        BURTC = 18,
        LETIMER0 = 19,
        SYSCFG = 20,
        LDMA = 21,
        LFXO = 22,
        LFRCO = 23,
        ULFRCO = 24,
        GPIO_ODD = 25,
        GPIO_EVEN = 26,
        I2C0 = 27,
        I2C1 = 28,
        EMUDG = 29,
        EMUSE = 30,
        AGC = 31,
        BUFC = 32,
        FRC_PRI = 33,
        FRC = 34,
        MODEM = 35,
        PROTIMER = 36,
        RAC_RSM = 37,
        RAC_SEQ = 38,
        RDMAILBOX = 39,
        RFSENSE = 40,
        PRORTC = 41,
        SYNTH = 42,
        WDOG0 = 43,
        HFXO0 = 44,
        HFRCO0 = 45,
        CMU = 46,
        AES = 47,
        IADC = 48,
        MSC = 49,
        DPLL0 = 50,
        PDM = 51,
        SW0 = 52,
        SW1 = 53,
        SW2 = 54,
        SW3 = 55,
        KERNEL0 = 56,
        KERNEL1 = 57,
        M33CTI0 = 58,
        M33CTI1 = 59,
        EMUEFP = 60,
        DCDC = 61,
        EUART0_RX = 62,
        EUART0_TX = 63,
    }

    #[cfg(feature = "rt")]
    unsafe extern "C" {
        fn CRYPTOACC();
        fn TRNG();
        fn PKE();
        fn SMU_SECURE();
        fn SMU_S_PRIVILEGED();
        fn SMU_NS_PRIVILEGED();
        fn EMU();
        fn TIMER0();
        fn TIMER1();
        fn TIMER2();
        fn TIMER3();
        fn TIMER4();
        fn RTCC();
        fn USART0_RX();
        fn USART0_TX();
        fn USART1_RX();
        fn USART1_TX();
        fn ICACHE0();
        fn BURTC();
        fn LETIMER0();
        fn SYSCFG();
        fn LDMA();
        fn LFXO();
        fn LFRCO();
        fn ULFRCO();
        fn GPIO_ODD();
        fn GPIO_EVEN();
        fn I2C0();
        fn I2C1();
        fn EMUDG();
        fn EMUSE();
        fn AGC();
        fn BUFC();
        fn FRC_PRI();
        fn FRC();
        fn MODEM();
        fn PROTIMER();
        fn RAC_RSM();
        fn RAC_SEQ();
        fn RDMAILBOX();
        fn RFSENSE();
        fn PRORTC();
        fn SYNTH();
        fn WDOG0();
        fn HFXO0();
        fn HFRCO0();
        fn CMU();
        fn AES();
        fn IADC();
        fn MSC();
        fn DPLL0();
        fn PDM();
        fn SW0();
        fn SW1();
        fn SW2();
        fn SW3();
        fn KERNEL0();
        fn KERNEL1();
        fn M33CTI0();
        fn M33CTI1();
        fn EMUEFP();
        fn DCDC();
        fn EUART0_RX();
        fn EUART0_TX();
    }

    #[cfg(feature = "rt")]
    #[doc(hidden)]
    #[unsafe(link_section = ".vector_table.interrupts")]
    #[unsafe(no_mangle)]
    pub static __INTERRUPTS: [super::Vector; 64] = [
        super::Vector { handler: CRYPTOACC },
        super::Vector { handler: TRNG },
        super::Vector { handler: PKE },
        super::Vector {
            handler: SMU_SECURE,
        },
        super::Vector {
            handler: SMU_S_PRIVILEGED,
        },
        super::Vector {
            handler: SMU_NS_PRIVILEGED,
        },
        super::Vector { handler: EMU },
        super::Vector { handler: TIMER0 },
        super::Vector { handler: TIMER1 },
        super::Vector { handler: TIMER2 },
        super::Vector { handler: TIMER3 },
        super::Vector { handler: TIMER4 },
        super::Vector { handler: RTCC },
        super::Vector { handler: USART0_RX },
        super::Vector { handler: USART0_TX },
        super::Vector { handler: USART1_RX },
        super::Vector { handler: USART1_TX },
        super::Vector { handler: ICACHE0 },
        super::Vector { handler: BURTC },
        super::Vector { handler: LETIMER0 },
        super::Vector { handler: SYSCFG },
        super::Vector { handler: LDMA },
        super::Vector { handler: LFXO },
        super::Vector { handler: LFRCO },
        super::Vector { handler: ULFRCO },
        super::Vector { handler: GPIO_ODD },
        super::Vector { handler: GPIO_EVEN },
        super::Vector { handler: I2C0 },
        super::Vector { handler: I2C1 },
        super::Vector { handler: EMUDG },
        super::Vector { handler: EMUSE },
        super::Vector { handler: AGC },
        super::Vector { handler: BUFC },
        super::Vector { handler: FRC_PRI },
        super::Vector { handler: FRC },
        super::Vector { handler: MODEM },
        super::Vector { handler: PROTIMER },
        super::Vector { handler: RAC_RSM },
        super::Vector { handler: RAC_SEQ },
        super::Vector { handler: RDMAILBOX },
        super::Vector { handler: RFSENSE },
        super::Vector { handler: PRORTC },
        super::Vector { handler: SYNTH },
        super::Vector { handler: WDOG0 },
        super::Vector { handler: HFXO0 },
        super::Vector { handler: HFRCO0 },
        super::Vector { handler: CMU },
        super::Vector { handler: AES },
        super::Vector { handler: IADC },
        super::Vector { handler: MSC },
        super::Vector { handler: DPLL0 },
        super::Vector { handler: PDM },
        super::Vector { handler: SW0 },
        super::Vector { handler: SW1 },
        super::Vector { handler: SW2 },
        super::Vector { handler: SW3 },
        super::Vector { handler: KERNEL0 },
        super::Vector { handler: KERNEL1 },
        super::Vector { handler: M33CTI0 },
        super::Vector { handler: M33CTI1 },
        super::Vector { handler: EMUEFP },
        super::Vector { handler: DCDC },
        super::Vector { handler: EUART0_RX },
        super::Vector { handler: EUART0_TX },
    ];
}

#[cfg(feature = "efr32mg21")]
mod table {
    /// Number of external interrupts on EFR32MG21.
    pub const IRQ_COUNT: usize = 61;

    /// External interrupt numbers (GSDK `IRQn_Type`).
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    #[repr(u16)]
    #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
    pub enum Interrupt {
        SETAMPERHOST = 0,
        SEMBRX = 1,
        SEMBTX = 2,
        SMU_SECURE = 3,
        SMU_PRIVILEGED = 4,
        EMU = 5,
        TIMER0 = 6,
        TIMER1 = 7,
        TIMER2 = 8,
        TIMER3 = 9,
        RTCC = 10,
        USART0_RX = 11,
        USART0_TX = 12,
        USART1_RX = 13,
        USART1_TX = 14,
        USART2_RX = 15,
        USART2_TX = 16,
        ICACHE0 = 17,
        BURTC = 18,
        LETIMER0 = 19,
        SYSCFG = 20,
        LDMA = 21,
        LFXO = 22,
        LFRCO = 23,
        ULFRCO = 24,
        GPIO_ODD = 25,
        GPIO_EVEN = 26,
        I2C0 = 27,
        I2C1 = 28,
        EMUDG = 29,
        EMUSE = 30,
        AGC = 31,
        BUFC = 32,
        FRC_PRI = 33,
        FRC = 34,
        MODEM = 35,
        PROTIMER = 36,
        RAC_RSM = 37,
        RAC_SEQ = 38,
        PRORTC = 39,
        SYNTH = 40,
        ACMP0 = 41,
        ACMP1 = 42,
        WDOG0 = 43,
        WDOG1 = 44,
        HFXO00 = 45,
        HFRCO0 = 46,
        HFRCOEM23 = 47,
        CMU = 48,
        AES = 49,
        IADC = 50,
        MSC = 51,
        DPLL0 = 52,
        SW0 = 53,
        SW1 = 54,
        SW2 = 55,
        SW3 = 56,
        KERNEL0 = 57,
        KERNEL1 = 58,
        M33CTI0 = 59,
        M33CTI1 = 60,
    }

    #[cfg(feature = "rt")]
    unsafe extern "C" {
        fn SETAMPERHOST();
        fn SEMBRX();
        fn SEMBTX();
        fn SMU_SECURE();
        fn SMU_PRIVILEGED();
        fn EMU();
        fn TIMER0();
        fn TIMER1();
        fn TIMER2();
        fn TIMER3();
        fn RTCC();
        fn USART0_RX();
        fn USART0_TX();
        fn USART1_RX();
        fn USART1_TX();
        fn USART2_RX();
        fn USART2_TX();
        fn ICACHE0();
        fn BURTC();
        fn LETIMER0();
        fn SYSCFG();
        fn LDMA();
        fn LFXO();
        fn LFRCO();
        fn ULFRCO();
        fn GPIO_ODD();
        fn GPIO_EVEN();
        fn I2C0();
        fn I2C1();
        fn EMUDG();
        fn EMUSE();
        fn AGC();
        fn BUFC();
        fn FRC_PRI();
        fn FRC();
        fn MODEM();
        fn PROTIMER();
        fn RAC_RSM();
        fn RAC_SEQ();
        fn PRORTC();
        fn SYNTH();
        fn ACMP0();
        fn ACMP1();
        fn WDOG0();
        fn WDOG1();
        fn HFXO00();
        fn HFRCO0();
        fn HFRCOEM23();
        fn CMU();
        fn AES();
        fn IADC();
        fn MSC();
        fn DPLL0();
        fn SW0();
        fn SW1();
        fn SW2();
        fn SW3();
        fn KERNEL0();
        fn KERNEL1();
        fn M33CTI0();
        fn M33CTI1();
    }

    #[cfg(feature = "rt")]
    #[doc(hidden)]
    #[unsafe(link_section = ".vector_table.interrupts")]
    #[unsafe(no_mangle)]
    pub static __INTERRUPTS: [super::Vector; 61] = [
        super::Vector {
            handler: SETAMPERHOST,
        },
        super::Vector { handler: SEMBRX },
        super::Vector { handler: SEMBTX },
        super::Vector {
            handler: SMU_SECURE,
        },
        super::Vector {
            handler: SMU_PRIVILEGED,
        },
        super::Vector { handler: EMU },
        super::Vector { handler: TIMER0 },
        super::Vector { handler: TIMER1 },
        super::Vector { handler: TIMER2 },
        super::Vector { handler: TIMER3 },
        super::Vector { handler: RTCC },
        super::Vector { handler: USART0_RX },
        super::Vector { handler: USART0_TX },
        super::Vector { handler: USART1_RX },
        super::Vector { handler: USART1_TX },
        super::Vector { handler: USART2_RX },
        super::Vector { handler: USART2_TX },
        super::Vector { handler: ICACHE0 },
        super::Vector { handler: BURTC },
        super::Vector { handler: LETIMER0 },
        super::Vector { handler: SYSCFG },
        super::Vector { handler: LDMA },
        super::Vector { handler: LFXO },
        super::Vector { handler: LFRCO },
        super::Vector { handler: ULFRCO },
        super::Vector { handler: GPIO_ODD },
        super::Vector { handler: GPIO_EVEN },
        super::Vector { handler: I2C0 },
        super::Vector { handler: I2C1 },
        super::Vector { handler: EMUDG },
        super::Vector { handler: EMUSE },
        super::Vector { handler: AGC },
        super::Vector { handler: BUFC },
        super::Vector { handler: FRC_PRI },
        super::Vector { handler: FRC },
        super::Vector { handler: MODEM },
        super::Vector { handler: PROTIMER },
        super::Vector { handler: RAC_RSM },
        super::Vector { handler: RAC_SEQ },
        super::Vector { handler: PRORTC },
        super::Vector { handler: SYNTH },
        super::Vector { handler: ACMP0 },
        super::Vector { handler: ACMP1 },
        super::Vector { handler: WDOG0 },
        super::Vector { handler: WDOG1 },
        super::Vector { handler: HFXO00 },
        super::Vector { handler: HFRCO0 },
        super::Vector { handler: HFRCOEM23 },
        super::Vector { handler: CMU },
        super::Vector { handler: AES },
        super::Vector { handler: IADC },
        super::Vector { handler: MSC },
        super::Vector { handler: DPLL0 },
        super::Vector { handler: SW0 },
        super::Vector { handler: SW1 },
        super::Vector { handler: SW2 },
        super::Vector { handler: SW3 },
        super::Vector { handler: KERNEL0 },
        super::Vector { handler: KERNEL1 },
        super::Vector { handler: M33CTI0 },
        super::Vector { handler: M33CTI1 },
    ];
}

pub use table::*;

/// One entry of the external interrupt vector table.
#[derive(Clone, Copy)]
#[repr(C)]
#[allow(dead_code)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    _reserved: usize,
}

// SAFETY: interrupt numbers are the device IRQn values.
unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    fn number(self) -> u16 {
        self as u16
    }
}
