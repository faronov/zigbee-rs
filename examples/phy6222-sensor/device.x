/* PHY6222 Interrupt Vector Names
 *
 * Weak aliases to DefaultHandler — override by defining a function
 * with the same name in Rust code (e.g., `#[no_mangle] pub extern "C" fn LL_IRQ()`).
 *
 * IRQ numbers from PHY6222 SDK (CMSIS device header):
 *   0: V0          (reserved)
 *   1: TIM5        (Timer 5 — SDK sleep timer)
 *   2: TIM6        (Timer 6)
 *   3: SPIF        (SPI Flash)
 *   4: BB          (Baseband controller)
 *   5: KSCAN       (Key scan matrix)
 *   6: RTC         (Real-time counter)
 *   7-9: reserved
 *  10: WDT         (Watchdog timer)
 *  11: AP_TIM1     (Application Timer 1)
 *  12: AP_TIM2     (Application Timer 2)
 *  13: AP_TIM3     (Application Timer 3)
 *  14: AP_TIM4     (Application Timer 4)
 *  15-16: reserved
 *  17: LL_IRQ      (Link Layer / radio hardware)
 *  18-19: reserved
 *  20: I2C0        (I2C controller 0)
 *  21: I2C1        (I2C controller 1)
 *  22: reserved
 *  23: GPIO        (GPIO port interrupt)
 *  24: UART0       (UART 0)
 *  25-26: reserved
 *  27: SPI0        (SPI controller 0)
 *  28: SPI1        (SPI controller 1)
 *  29: UART1       (UART 1)
 *  30: DMAC        (DMA controller)
 *  31: AES         (AES encryption engine)
 */

PROVIDE(V0      = DefaultHandler);
PROVIDE(TIM5    = DefaultHandler);
PROVIDE(TIM6    = DefaultHandler);
PROVIDE(SPIF    = DefaultHandler);
PROVIDE(BB      = DefaultHandler);
PROVIDE(KSCAN   = DefaultHandler);
PROVIDE(RTC     = DefaultHandler);
PROVIDE(V7      = DefaultHandler);
PROVIDE(V8      = DefaultHandler);
PROVIDE(V9      = DefaultHandler);
PROVIDE(WDT     = DefaultHandler);
PROVIDE(AP_TIM1 = DefaultHandler);
PROVIDE(AP_TIM2 = DefaultHandler);
PROVIDE(AP_TIM3 = DefaultHandler);
PROVIDE(AP_TIM4 = DefaultHandler);
PROVIDE(V15     = DefaultHandler);
PROVIDE(V16     = DefaultHandler);
PROVIDE(LL_IRQ  = DefaultHandler);
PROVIDE(V18     = DefaultHandler);
PROVIDE(V19     = DefaultHandler);
PROVIDE(I2C0    = DefaultHandler);
PROVIDE(I2C1    = DefaultHandler);
PROVIDE(V22     = DefaultHandler);
PROVIDE(GPIO    = DefaultHandler);
PROVIDE(UART0   = DefaultHandler);
PROVIDE(V25     = DefaultHandler);
PROVIDE(V26     = DefaultHandler);
PROVIDE(SPI0    = DefaultHandler);
PROVIDE(SPI1    = DefaultHandler);
PROVIDE(UART1   = DefaultHandler);
PROVIDE(DMAC    = DefaultHandler);
PROVIDE(AES     = DefaultHandler);
