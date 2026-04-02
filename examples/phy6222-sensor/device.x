/* PHY6222 Interrupt Vector Names
 *
 * Weak aliases to DefaultHandler — override by defining a function
 * with the same name in Rust code (e.g., `#[no_mangle] pub extern "C" fn LL_IRQ()`).
 *
 * IRQ numbers verified from PHY6222 SDK bus_dev.h (NOT the fake phy6222.h CMSIS header
 * which contains placeholder STM32L0 IRQ numbers):
 *
 *   Source: bthome_phy6222/SDK/components/inc/bus_dev.h
 *   Confirmed: bthome_phy6222/SDK/components/inc/mcu_phy_bumbee.h (jump table mapping)
 *
 *   0-3: reserved
 *   4: BB          (Baseband / LL radio — SDK uses JUMP_FUNCTION(V4_IRQ_HANDLER))
 *   5: KSCAN       (Key scan matrix)
 *   6: RTC         (Real-time counter)
 *   7-9: reserved
 *  10: WDT         (Watchdog timer)
 *  11: UART0       (UART 0)
 *  12: I2C0        (I2C controller 0)
 *  13: I2C1        (I2C controller 1)
 *  14: SPI0        (SPI controller 0)
 *  15: SPI1        (SPI controller 1)
 *  16: GPIO        (GPIO port interrupt)
 *  17: UART1       (UART 1)
 *  18: SPIF        (SPI Flash)
 *  19: DMAC        (DMA controller)
 *  20: TIM1        (Timer 1 — LL event in SDK)
 *  21: TIM2        (Timer 2 — OSAL 625µs tick in SDK)
 *  22: TIM3        (Timer 3 — base-time timer in SDK)
 *  23: TIM4        (Timer 4 — extended adv in SDK)
 *  24: TIM5        (Timer 5)
 *  25: TIM6        (Timer 6)
 *  26-27: reserved
 *  28: AES         (AES encryption engine)
 *  29: ADCC        (ADC controller)
 *  30: QDEC        (Quadrature decoder)
 *  31: RNG         (Random number generator)
 */

PROVIDE(V0      = DefaultHandler);
PROVIDE(V1      = DefaultHandler);
PROVIDE(V2      = DefaultHandler);
PROVIDE(V3      = DefaultHandler);
PROVIDE(LL_IRQ  = DefaultHandler);
PROVIDE(KSCAN   = DefaultHandler);
PROVIDE(RTC     = DefaultHandler);
PROVIDE(V7      = DefaultHandler);
PROVIDE(V8      = DefaultHandler);
PROVIDE(V9      = DefaultHandler);
PROVIDE(WDT     = DefaultHandler);
PROVIDE(UART0   = DefaultHandler);
PROVIDE(I2C0    = DefaultHandler);
PROVIDE(I2C1    = DefaultHandler);
PROVIDE(SPI0    = DefaultHandler);
PROVIDE(SPI1    = DefaultHandler);
PROVIDE(GPIO    = DefaultHandler);
PROVIDE(UART1   = DefaultHandler);
PROVIDE(SPIF    = DefaultHandler);
PROVIDE(DMAC    = DefaultHandler);
PROVIDE(TIM1    = DefaultHandler);
PROVIDE(TIM2    = DefaultHandler);
PROVIDE(TIM3    = DefaultHandler);
PROVIDE(TIM4    = DefaultHandler);
PROVIDE(TIM5    = DefaultHandler);
PROVIDE(TIM6    = DefaultHandler);
PROVIDE(V26     = DefaultHandler);
PROVIDE(V27     = DefaultHandler);
PROVIDE(AES     = DefaultHandler);
PROVIDE(ADCC    = DefaultHandler);
PROVIDE(QDEC    = DefaultHandler);
PROVIDE(RNG     = DefaultHandler);
