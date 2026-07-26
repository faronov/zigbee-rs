/* EFR32MG1P Interrupt Vector Names
 *
 * Weak aliases to DefaultHandler — override by defining a function
 * with the same name in Rust code.
 *
 * IRQ numbers from EFR32MG1P Reference Manual (EFR32xG1):
 *
 *   0: EMU          (Energy Management Unit)
 *   1: FRC_PRI      (Frame Controller — Priority)
 *   2: WDOG0        (Watchdog Timer 0)
 *   3: FRC          (Frame Controller)
 *   4: MODEM        (Modem)
 *   5: RAC_SEQ      (Radio Controller — Sequencer)
 *   6: RAC_RSM      (Radio Controller — State Machine)
 *   7: BUFC         (Buffer Controller)
 *   8: LDMA         (Linked DMA Controller)
 *   9: GPIO_EVEN    (GPIO Even Pins)
 *  10: TIMER0       (Timer 0)
 *  11: USART0_RX    (USART0 RX)
 *  12: USART0_TX    (USART0 TX)
 *  13: ACMP0        (Analog Comparator 0)
 *  14: ADC0         (ADC 0)
 *  15: IDAC0        (Current DAC 0)
 *  16: I2C0         (I2C 0)
 *  17: GPIO_ODD     (GPIO Odd Pins)
 *  18: TIMER1       (Timer 1)
 *  19: USART1_RX    (USART1 RX)
 *  20: USART1_TX    (USART1 TX)
 *  21: LEUART0      (Low Energy UART 0)
 *  22: PCNT0        (Pulse Counter 0)
 *  23: CMU          (Clock Management Unit)
 *  24: MSC          (Memory System Controller)
 *  25: CRYPTO       (Crypto Engine)
 *  26: LETIMER0     (Low Energy Timer 0)
 *  27: AGC          (Automatic Gain Control)
 *  28: PROTIMER     (Protocol Timer)
 *  29: RTCC         (Real-Time Counter and Calendar)
 *  30: SYNTH        (Frequency Synthesizer)
 *  31: CRYOTIMER    (Ultra-Low Energy Timer)
 *  32: RFSENSE      (RF Sense)
 *  33: FPUEH        (FPU Exception Handler)
 */

PROVIDE(EMU         = DefaultHandler);
PROVIDE(FRC_PRI     = DefaultHandler);
PROVIDE(WDOG0       = DefaultHandler);
PROVIDE(FRC         = DefaultHandler);
PROVIDE(MODEM       = DefaultHandler);
PROVIDE(RAC_SEQ     = DefaultHandler);
PROVIDE(RAC_RSM     = DefaultHandler);
PROVIDE(BUFC        = DefaultHandler);
PROVIDE(LDMA        = DefaultHandler);
PROVIDE(GPIO_EVEN   = DefaultHandler);
PROVIDE(TIMER0      = DefaultHandler);
PROVIDE(USART0_RX   = DefaultHandler);
PROVIDE(USART0_TX   = DefaultHandler);
PROVIDE(ACMP0       = DefaultHandler);
PROVIDE(ADC0        = DefaultHandler);
PROVIDE(IDAC0       = DefaultHandler);
PROVIDE(I2C0        = DefaultHandler);
PROVIDE(GPIO_ODD    = DefaultHandler);
PROVIDE(TIMER1      = DefaultHandler);
PROVIDE(USART1_RX   = DefaultHandler);
PROVIDE(USART1_TX   = DefaultHandler);
PROVIDE(LEUART0     = DefaultHandler);
PROVIDE(PCNT0       = DefaultHandler);
PROVIDE(CMU         = DefaultHandler);
PROVIDE(MSC         = DefaultHandler);
PROVIDE(CRYPTO      = DefaultHandler);
PROVIDE(LETIMER0    = DefaultHandler);
PROVIDE(AGC         = DefaultHandler);
PROVIDE(PROTIMER    = DefaultHandler);
PROVIDE(RTCC        = DefaultHandler);
PROVIDE(SYNTH       = DefaultHandler);
PROVIDE(CRYOTIMER   = DefaultHandler);
PROVIDE(RFSENSE     = DefaultHandler);
PROVIDE(FPUEH       = DefaultHandler);
