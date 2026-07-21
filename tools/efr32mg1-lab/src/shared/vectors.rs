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
    app_version: 1,
    app_capabilities: 0,
    app_product_id: [0; 16],
};

#[repr(C)]
pub union Vector {
    handler: unsafe extern "C" fn(),
    reserved: usize,
}

unsafe impl Sync for Vector {}

#[allow(dead_code)]
unsafe extern "C" {
    fn EMU();
    fn FRC_PRI();
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

#[unsafe(link_section = ".vector_table.interrupts")]
#[unsafe(no_mangle)]
pub static __INTERRUPTS: [Vector; 34] = [
    Vector { handler: EMU },
    Vector { handler: FRC_PRI },
    Vector { handler: WDOG0 },
    Vector { handler: FRC },
    Vector { handler: MODEM },
    Vector { handler: RAC_SEQ },
    Vector { handler: RAC_RSM },
    Vector { handler: BUFC },
    Vector { handler: LDMA },
    Vector { handler: GPIO_EVEN },
    Vector { handler: TIMER0 },
    Vector { handler: USART0_RX },
    Vector { handler: USART0_TX },
    Vector { handler: ACMP0 },
    Vector { handler: ADC0 },
    Vector { handler: IDAC0 },
    Vector { handler: I2C0 },
    Vector { handler: GPIO_ODD },
    Vector { handler: TIMER1 },
    Vector { handler: USART1_RX },
    Vector { handler: USART1_TX },
    Vector { handler: LEUART0 },
    Vector { handler: PCNT0 },
    Vector { handler: CMU },
    Vector { handler: MSC },
    Vector { handler: CRYPTO },
    Vector { handler: LETIMER0 },
    Vector { handler: AGC },
    Vector { handler: PROTIMER },
    Vector { handler: RTCC },
    Vector { handler: SYNTH },
    Vector { handler: CRYOTIMER },
    Vector { handler: RFSENSE },
    Vector { handler: FPUEH },
];

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
