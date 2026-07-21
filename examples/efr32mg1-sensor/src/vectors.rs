//! EFR32MG1P interrupt vector table and peripheral interrupt definitions.
//!
//! Defines `__INTERRUPTS` for cortex-m-rt and an `Interrupt` enum for NVIC.
//! The names match `device.x` so user code can override any handler.
//!
//! IRQ numbers from EFR32MG1P Reference Manual (EFR32xG1).

/// Silicon Labs application properties consumed by the resident bootloader.
#[repr(C)]
pub struct ApplicationProperties {
    magic: [u8; 16],
    struct_version: u32,
    signature_type: u32,
    signature_location: u32,
    app_type: u32,
    app_version: u32,
    app_capabilities: u32,
    app_product_id: [u8; 16],
}

#[used]
#[unsafe(no_mangle)]
#[unsafe(link_section = ".app_properties")]
pub static APP_PROPERTIES: ApplicationProperties = ApplicationProperties {
    magic: [
        0x13, 0xB7, 0x79, 0xFA, 0xC9, 0x25, 0xDD, 0xB7, 0xAD, 0xF3, 0xCF, 0xE0, 0xF1, 0xB6, 0x14,
        0xB8,
    ],
    struct_version: 0x0000_0100,
    signature_type: 0,
    signature_location: 0xFFFF_FFFF,
    app_type: 1,
    app_version: crate::FIRMWARE_VERSION,
    app_capabilities: 0,
    app_product_id: [0; 16],
};

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
    fn FRC_PRI(); // IRQ  1: Frame Controller — Priority (radio RX/TX)
    fn WDOG0();
    fn FRC();
    fn MODEM();
    fn RAC_SEQ();
    fn RAC_RSM();
    fn BUFC();
    fn LDMA();
    fn GPIO_EVEN();
    fn TIMER0();
    fn USART0_RX();
    fn USART0_TX();
    fn ACMP0();
    fn ADC0();
    fn IDAC0();
    fn I2C0();
    fn GPIO_ODD();
    fn TIMER1();
    fn USART1_RX();
    fn USART1_TX();
    fn LEUART0();
    fn PCNT0();
    fn CMU();
    fn MSC();
    fn CRYPTO();
    fn LETIMER0();
    fn AGC();
    fn PROTIMER();
    fn RTCC();
    fn SYNTH();
    fn CRYOTIMER();
    fn RFSENSE();
    fn FPUEH();
}

/// EFR32MG1P interrupt vector table — 34 entries for Cortex-M4F.
///
/// Placed in `.vector_table.interrupts` by cortex-m-rt (requires `device` feature).
#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
pub static __INTERRUPTS: [Vector; 34] = [
    Vector { handler: EMU },       // IRQ  0: Energy Management Unit
    Vector { handler: FRC_PRI },   // IRQ  1: Frame Controller — Priority
    Vector { handler: WDOG0 },     // IRQ  2: Watchdog Timer 0
    Vector { handler: FRC },       // IRQ  3: Frame Controller
    Vector { handler: MODEM },     // IRQ  4: Modem
    Vector { handler: RAC_SEQ },   // IRQ  5: Radio Controller — Sequencer
    Vector { handler: RAC_RSM },   // IRQ  6: Radio Controller — State Machine
    Vector { handler: BUFC },      // IRQ  7: Buffer Controller
    Vector { handler: LDMA },      // IRQ  8: Linked DMA Controller
    Vector { handler: GPIO_EVEN }, // IRQ  9: GPIO Even Pins
    Vector { handler: TIMER0 },    // IRQ 10: Timer 0
    Vector { handler: USART0_RX }, // IRQ 11: USART0 RX
    Vector { handler: USART0_TX }, // IRQ 12: USART0 TX
    Vector { handler: ACMP0 },     // IRQ 13: Analog Comparator 0
    Vector { handler: ADC0 },      // IRQ 14: ADC 0
    Vector { handler: IDAC0 },     // IRQ 15: Current DAC 0
    Vector { handler: I2C0 },      // IRQ 16: I2C 0
    Vector { handler: GPIO_ODD },  // IRQ 17: GPIO Odd Pins
    Vector { handler: TIMER1 },    // IRQ 18: Timer 1
    Vector { handler: USART1_RX }, // IRQ 19: USART1 RX
    Vector { handler: USART1_TX }, // IRQ 20: USART1 TX
    Vector { handler: LEUART0 },   // IRQ 21: Low Energy UART 0
    Vector { handler: PCNT0 },     // IRQ 22: Pulse Counter 0
    Vector { handler: CMU },       // IRQ 23: Clock Management Unit
    Vector { handler: MSC },       // IRQ 24: Memory System Controller
    Vector { handler: CRYPTO },    // IRQ 25: Crypto Engine
    Vector { handler: LETIMER0 },  // IRQ 26: Low Energy Timer 0
    Vector { handler: AGC },       // IRQ 27: Automatic Gain Control
    Vector { handler: PROTIMER },  // IRQ 28: Protocol Timer
    Vector { handler: RTCC },      // IRQ 29: Real-Time Counter and Calendar
    Vector { handler: SYNTH },     // IRQ 30: Frequency Synthesizer
    Vector { handler: CRYOTIMER }, // IRQ 31: Ultra-Low Energy Timer
    Vector { handler: RFSENSE },   // IRQ 32: RF Sense
    Vector { handler: FPUEH },     // IRQ 33: FPU Exception Handler
];

/// EFR32MG1P peripheral interrupt numbers for NVIC control.
#[derive(Debug, Clone, Copy)]
#[repr(u16)]
#[allow(dead_code)]
pub enum Interrupt {
    Emu = 0,
    FrcPri = 1,
    Wdog0 = 2,
    Frc = 3,
    Modem = 4,
    RacSeq = 5,
    RacRsm = 6,
    Bufc = 7,
    Ldma = 8,
    GpioEven = 9,
    Timer0 = 10,
    Usart0Rx = 11,
    Usart0Tx = 12,
    Acmp0 = 13,
    Adc0 = 14,
    Idac0 = 15,
    I2c0 = 16,
    GpioOdd = 17,
    Timer1 = 18,
    Usart1Rx = 19,
    Usart1Tx = 20,
    Leuart0 = 21,
    Pcnt0 = 22,
    Cmu = 23,
    Msc = 24,
    Crypto = 25,
    Letimer0 = 26,
    Agc = 27,
    Protimer = 28,
    Rtcc = 29,
    Synth = 30,
    Cryotimer = 31,
    Rfsense = 32,
    Fpueh = 33,
}

unsafe impl cortex_m::interrupt::InterruptNumber for Interrupt {
    fn number(self) -> u16 {
        self as u16
    }
}
