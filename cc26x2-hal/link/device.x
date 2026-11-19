/* CC26x2/CC13x2 interrupt handler names for cortex-m-rt.
 * From SDK 8.33.00.16 inc/hw_ints.h (INT_* 16..53), 38 IRQs.
 * The table itself is `cc26x2_hal::vectors::__INTERRUPTS` (feature `rt`).
 */
PROVIDE(AON_GPIO_EDGE = DefaultHandler);
PROVIDE(I2C_IRQ = DefaultHandler);
PROVIDE(RFC_CPE_1 = DefaultHandler);
PROVIDE(PKA_IRQ = DefaultHandler);
PROVIDE(AON_RTC_COMB = DefaultHandler);
PROVIDE(UART0_COMB = DefaultHandler);
PROVIDE(AUX_SWEV0 = DefaultHandler);
PROVIDE(SSI0_COMB = DefaultHandler);
PROVIDE(SSI1_COMB = DefaultHandler);
PROVIDE(RFC_CPE_0 = DefaultHandler);
PROVIDE(RFC_HW_COMB = DefaultHandler);
PROVIDE(RFC_CMD_ACK = DefaultHandler);
PROVIDE(I2S_IRQ = DefaultHandler);
PROVIDE(AUX_SWEV1 = DefaultHandler);
PROVIDE(WDT_IRQ = DefaultHandler);
PROVIDE(GPT0A = DefaultHandler);
PROVIDE(GPT0B = DefaultHandler);
PROVIDE(GPT1A = DefaultHandler);
PROVIDE(GPT1B = DefaultHandler);
PROVIDE(GPT2A = DefaultHandler);
PROVIDE(GPT2B = DefaultHandler);
PROVIDE(GPT3A = DefaultHandler);
PROVIDE(GPT3B = DefaultHandler);
PROVIDE(CRYPTO_RESULT_AVAIL_IRQ = DefaultHandler);
PROVIDE(DMA_DONE_COMB = DefaultHandler);
PROVIDE(DMA_ERR = DefaultHandler);
PROVIDE(FLASH = DefaultHandler);
PROVIDE(SWEV0 = DefaultHandler);
PROVIDE(AUX_COMB = DefaultHandler);
PROVIDE(AON_PROG0 = DefaultHandler);
PROVIDE(PROG0 = DefaultHandler);
PROVIDE(AUX_COMPA = DefaultHandler);
PROVIDE(AUX_ADC_IRQ = DefaultHandler);
PROVIDE(TRNG_IRQ = DefaultHandler);
PROVIDE(OSC_COMB = DefaultHandler);
PROVIDE(AUX_TIMER2_EV0 = DefaultHandler);
PROVIDE(UART1_COMB = DefaultHandler);
PROVIDE(BATMON_COMB = DefaultHandler);
