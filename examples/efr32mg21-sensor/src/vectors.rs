//! EFR32MG21 interrupt vector table and peripheral interrupt definitions.
//!
//! Defines `__INTERRUPTS` for cortex-m-rt and an `Interrupt` enum for NVIC.
//! The names match `device.x` so user code can override any handler.
//!
//! IRQ numbers from EFR32MG21 Reference Manual (EFR32xG21).

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
    fn EMU();
    fn SE();
    fn IADC();
    fn GPIO_EVEN();
    fn TIMER0();
    fn USART0_RX();
    fn USART0_TX();
    fn USART1_RX();
    fn USART1_TX();
    fn USART2_RX();
    fn USART2_TX();
    fn I2C0();
    fn I2C1();
    fn GPIO_ODD();
    fn LDMA();
    fn LDMA_CH0();
    fn LDMA_CH1();
    fn LDMA_CH2();
    fn LDMA_CH3();
    fn LDMA_CH4();
    fn LDMA_CH5();
    fn LDMA_CH6();
    fn LDMA_CH7();
    fn TIMER1();
    fn TIMER2();
    fn TIMER3();
    fn WDOG0();
    fn WDOG1();
    fn SYSCFG();
    fn MSC();
    fn CRYPTO();
    fn CMU();
    fn PRS_ASYNC_CH0();
    fn PRS_ASYNC_CH1();
    fn PRS_ASYNC_CH2();
    fn PRS_ASYNC_CH3();
    fn FRC_PRI(); // IRQ 36: Frame Controller — Priority (radio RX/TX)
    fn FRC();
    fn MODEM();
    fn PROTIMER();
    fn RAC_RSM();
    fn RAC_SEQ();
    fn RDMAILBOX();
    fn RFSENSE();
    fn PRORTC();
    fn SYNTH();
    fn BUFC();
    fn AGC();
    fn LETIMER0();
    fn BURTC();
    fn RTCC();
}

/// EFR32MG21 interrupt vector table — 51 entries for Cortex-M33.
///
/// Placed in `.vector_table.interrupts` by cortex-m-rt (requires `device` feature).
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
pub static __INTERRUPTS: [Vector; 51] = [
    Vector { handler: EMU },       // IRQ  0: Energy Management Unit
    Vector { handler: SE },        // IRQ  1: Secure Element
    Vector { handler: IADC },      // IRQ  2: Incremental ADC
    Vector { handler: GPIO_EVEN }, // IRQ  3: GPIO Even Pins
    Vector { handler: TIMER0 },    // IRQ  4: Timer 0
    Vector { handler: USART0_RX }, // IRQ  5: USART0 RX
    Vector { handler: USART0_TX }, // IRQ  6: USART0 TX
    Vector { handler: USART1_RX }, // IRQ  7: USART1 RX
    Vector { handler: USART1_TX }, // IRQ  8: USART1 TX
    Vector { handler: USART2_RX }, // IRQ  9: USART2 RX
    Vector { handler: USART2_TX }, // IRQ 10: USART2 TX
    Vector { handler: I2C0 },      // IRQ 11: I2C 0
    Vector { handler: I2C1 },      // IRQ 12: I2C 1
    Vector { handler: GPIO_ODD },  // IRQ 13: GPIO Odd Pins
    Vector { handler: LDMA },      // IRQ 14: Linked DMA Controller
    Vector { handler: LDMA_CH0 },  // IRQ 15: LDMA Channel 0
    Vector { handler: LDMA_CH1 },  // IRQ 16: LDMA Channel 1
    Vector { handler: LDMA_CH2 },  // IRQ 17: LDMA Channel 2
    Vector { handler: LDMA_CH3 },  // IRQ 18: LDMA Channel 3
    Vector { handler: LDMA_CH4 },  // IRQ 19: LDMA Channel 4
    Vector { handler: LDMA_CH5 },  // IRQ 20: LDMA Channel 5
    Vector { handler: LDMA_CH6 },  // IRQ 21: LDMA Channel 6
    Vector { handler: LDMA_CH7 },  // IRQ 22: LDMA Channel 7
    Vector { handler: TIMER1 },    // IRQ 23: Timer 1
    Vector { handler: TIMER2 },    // IRQ 24: Timer 2
    Vector { handler: TIMER3 },    // IRQ 25: Timer 3
    Vector { handler: WDOG0 },     // IRQ 26: Watchdog Timer 0
    Vector { handler: WDOG1 },     // IRQ 27: Watchdog Timer 1
    Vector { handler: SYSCFG },    // IRQ 28: System Configuration
    Vector { handler: MSC },       // IRQ 29: Memory System Controller
    Vector { handler: CRYPTO },    // IRQ 30: Crypto Accelerator
    Vector { handler: CMU },       // IRQ 31: Clock Management Unit
    Vector {
        handler: PRS_ASYNC_CH0,
    }, // IRQ 32: PRS Async Channel 0
    Vector {
        handler: PRS_ASYNC_CH1,
    }, // IRQ 33: PRS Async Channel 1
    Vector {
        handler: PRS_ASYNC_CH2,
    }, // IRQ 34: PRS Async Channel 2
    Vector {
        handler: PRS_ASYNC_CH3,
    }, // IRQ 35: PRS Async Channel 3
    Vector { handler: FRC_PRI },   // IRQ 36: Frame Controller — Priority
    Vector { handler: FRC },       // IRQ 37: Frame Controller
    Vector { handler: MODEM },     // IRQ 38: Modem
    Vector { handler: PROTIMER },  // IRQ 39: Protocol Timer
    Vector { handler: RAC_RSM },   // IRQ 40: Radio Controller — State Machine
    Vector { handler: RAC_SEQ },   // IRQ 41: Radio Controller — Sequencer
    Vector { handler: RDMAILBOX }, // IRQ 42: Radio DMA Mailbox
    Vector { handler: RFSENSE },   // IRQ 43: RF Sense
    Vector { handler: PRORTC },    // IRQ 44: Protocol Real-Time Counter
    Vector { handler: SYNTH },     // IRQ 45: Frequency Synthesizer
    Vector { handler: BUFC },      // IRQ 46: Buffer Controller
    Vector { handler: AGC },       // IRQ 47: Automatic Gain Control
    Vector { handler: LETIMER0 },  // IRQ 48: Low Energy Timer 0
    Vector { handler: BURTC },     // IRQ 49: Backup Real-Time Counter
    Vector { handler: RTCC },      // IRQ 50: Real-Time Counter and Calendar
];

/// EFR32MG21 peripheral interrupt numbers for NVIC control.
#[derive(Debug, Clone, Copy)]
#[repr(u16)]
#[allow(dead_code)]
pub enum Interrupt {
    Emu = 0,
    Se = 1,
    Iadc = 2,
    GpioEven = 3,
    Timer0 = 4,
    Usart0Rx = 5,
    Usart0Tx = 6,
    Usart1Rx = 7,
    Usart1Tx = 8,
    Usart2Rx = 9,
    Usart2Tx = 10,
    I2c0 = 11,
    I2c1 = 12,
    GpioOdd = 13,
    Ldma = 14,
    LdmaCh0 = 15,
    LdmaCh1 = 16,
    LdmaCh2 = 17,
    LdmaCh3 = 18,
    LdmaCh4 = 19,
    LdmaCh5 = 20,
    LdmaCh6 = 21,
    LdmaCh7 = 22,
    Timer1 = 23,
    Timer2 = 24,
    Timer3 = 25,
    Wdog0 = 26,
    Wdog1 = 27,
    Syscfg = 28,
    Msc = 29,
    Crypto = 30,
    Cmu = 31,
    PrsAsyncCh0 = 32,
    PrsAsyncCh1 = 33,
    PrsAsyncCh2 = 34,
    PrsAsyncCh3 = 35,
    FrcPri = 36,
    Frc = 37,
    Modem = 38,
    Protimer = 39,
    RacRsm = 40,
    RacSeq = 41,
    Rdmailbox = 42,
    Rfsense = 43,
    Prortc = 44,
    Synth = 45,
    Bufc = 46,
    Agc = 47,
    Letimer0 = 48,
    Burtc = 49,
    Rtcc = 50,
}

unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    fn number(self) -> u16 {
        self as u16
    }
}
