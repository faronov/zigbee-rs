/* EFR32MG21 development-target Interrupt Vector Names
 *
 * Weak aliases to DefaultHandler — override by defining a function
 * with the same name in Rust code.
 *
 * IRQ numbers from EFR32MG21 Reference Manual (EFR32xG21):
 *
 *   0: EMU          (Energy Management Unit)
 *   1: SE           (Secure Element)
 *   2: IADC         (Incremental ADC)
 *   3: GPIO_EVEN    (GPIO Even Pins)
 *   4: TIMER0       (Timer 0)
 *   5: USART0_RX    (USART0 RX)
 *   6: USART0_TX    (USART0 TX)
 *   7: USART1_RX    (USART1 RX)
 *   8: USART1_TX    (USART1 TX)
 *   9: USART2_RX    (USART2 RX)
 *  10: USART2_TX    (USART2 TX)
 *  11: I2C0         (I2C 0)
 *  12: I2C1         (I2C 1)
 *  13: GPIO_ODD     (GPIO Odd Pins)
 *  14: LDMA         (Linked DMA Controller)
 *  15: LDMA_CH0     (LDMA Channel 0)
 *  16: LDMA_CH1     (LDMA Channel 1)
 *  17: LDMA_CH2     (LDMA Channel 2)
 *  18: LDMA_CH3     (LDMA Channel 3)
 *  19: LDMA_CH4     (LDMA Channel 4)
 *  20: LDMA_CH5     (LDMA Channel 5)
 *  21: LDMA_CH6     (LDMA Channel 6)
 *  22: LDMA_CH7     (LDMA Channel 7)
 *  23: TIMER1       (Timer 1)
 *  24: TIMER2       (Timer 2)
 *  25: TIMER3       (Timer 3)
 *  26: WDOG0        (Watchdog Timer 0)
 *  27: WDOG1        (Watchdog Timer 1)
 *  28: SYSCFG       (System Configuration)
 *  29: MSC          (Memory System Controller)
 *  30: CRYPTO       (Crypto Accelerator)
 *  31: CMU          (Clock Management Unit)
 *  32: PRS_ASYNC_CH0 (PRS Async Channel 0)
 *  33: PRS_ASYNC_CH1 (PRS Async Channel 1)
 *  34: PRS_ASYNC_CH2 (PRS Async Channel 2)
 *  35: PRS_ASYNC_CH3 (PRS Async Channel 3)
 *  36: FRC_PRI      (Frame Controller — Priority)
 *  37: FRC          (Frame Controller)
 *  38: MODEM        (Modem)
 *  39: PROTIMER     (Protocol Timer)
 *  40: RAC_RSM      (Radio Controller — State Machine)
 *  41: RAC_SEQ      (Radio Controller — Sequencer)
 *  42: RDMAILBOX    (Radio DMA Mailbox)
 *  43: RFSENSE      (RF Sense)
 *  44: PRORTC       (Protocol Real-Time Counter)
 *  45: SYNTH        (Frequency Synthesizer)
 *  46: BUFC         (Buffer Controller)
 *  47: AGC          (Automatic Gain Control)
 *  48: LETIMER0     (Low Energy Timer 0)
 *  49: BURTC        (Backup Real-Time Counter)
 *  50: RTCC         (Real-Time Counter and Calendar)
 */

PROVIDE(EMU            = DefaultHandler);
PROVIDE(SE             = DefaultHandler);
PROVIDE(IADC           = DefaultHandler);
PROVIDE(GPIO_EVEN      = DefaultHandler);
PROVIDE(TIMER0         = DefaultHandler);
PROVIDE(USART0_RX      = DefaultHandler);
PROVIDE(USART0_TX      = DefaultHandler);
PROVIDE(USART1_RX      = DefaultHandler);
PROVIDE(USART1_TX      = DefaultHandler);
PROVIDE(USART2_RX      = DefaultHandler);
PROVIDE(USART2_TX      = DefaultHandler);
PROVIDE(I2C0           = DefaultHandler);
PROVIDE(I2C1           = DefaultHandler);
PROVIDE(GPIO_ODD       = DefaultHandler);
PROVIDE(LDMA           = DefaultHandler);
PROVIDE(LDMA_CH0       = DefaultHandler);
PROVIDE(LDMA_CH1       = DefaultHandler);
PROVIDE(LDMA_CH2       = DefaultHandler);
PROVIDE(LDMA_CH3       = DefaultHandler);
PROVIDE(LDMA_CH4       = DefaultHandler);
PROVIDE(LDMA_CH5       = DefaultHandler);
PROVIDE(LDMA_CH6       = DefaultHandler);
PROVIDE(LDMA_CH7       = DefaultHandler);
PROVIDE(TIMER1         = DefaultHandler);
PROVIDE(TIMER2         = DefaultHandler);
PROVIDE(TIMER3         = DefaultHandler);
PROVIDE(WDOG0          = DefaultHandler);
PROVIDE(WDOG1          = DefaultHandler);
PROVIDE(SYSCFG         = DefaultHandler);
PROVIDE(MSC            = DefaultHandler);
PROVIDE(CRYPTO         = DefaultHandler);
PROVIDE(CMU            = DefaultHandler);
PROVIDE(PRS_ASYNC_CH0  = DefaultHandler);
PROVIDE(PRS_ASYNC_CH1  = DefaultHandler);
PROVIDE(PRS_ASYNC_CH2  = DefaultHandler);
PROVIDE(PRS_ASYNC_CH3  = DefaultHandler);
PROVIDE(FRC_PRI        = DefaultHandler);
PROVIDE(FRC            = DefaultHandler);
PROVIDE(MODEM          = DefaultHandler);
PROVIDE(PROTIMER       = DefaultHandler);
PROVIDE(RAC_RSM        = DefaultHandler);
PROVIDE(RAC_SEQ        = DefaultHandler);
PROVIDE(RDMAILBOX      = DefaultHandler);
PROVIDE(RFSENSE        = DefaultHandler);
PROVIDE(PRORTC         = DefaultHandler);
PROVIDE(SYNTH          = DefaultHandler);
PROVIDE(BUFC           = DefaultHandler);
PROVIDE(AGC            = DefaultHandler);
PROVIDE(LETIMER0       = DefaultHandler);
PROVIDE(BURTC          = DefaultHandler);
PROVIDE(RTCC           = DefaultHandler);
