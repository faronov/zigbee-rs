/* PHY62x2 Interrupt Vector Names
 *
 * Weak aliases to DefaultHandler — override by defining a function
 * with the same name in Rust code (e.g., `#[no_mangle] pub extern "C" fn LL_IRQ()`).
 *
 * IRQ numbers verified from PHY6222 SDK bus_dev.h (NOT the fake phy6222.h CMSIS header
 * which contains placeholder STM32L0 IRQ numbers):
 *
 *   Source: bthome_phy6222/SDK/components/inc/bus_dev.h
 *   Confirmed: bthome_phy6222/SDK/components/inc/mcu_phy_bumbee.h (jump table mapping)
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
