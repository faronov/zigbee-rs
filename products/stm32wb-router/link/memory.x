MEMORY
{
  /* Every 1 MiB STM32WB55 router product (STM32WB5MM-DK's STM32WB55VG,
   * Nucleo-WB55's STM32WB55RG, USB dongle's STM32WB55CG): CPU2
   * (FUS 1.2.0 + 802.15.4 MAC 1.24.0) owns 0x080E1000.. (SFSA = 0xE1).
   * 0x080CF000..0x080D3000 holds the Green Power Proxy Table journal,
   * 0x080D3000..0x080D5000 the settings journal and 0x080D5000..0x080E1000
   * the router journals (src/layout.rs). */
  FLASH            : ORIGIN = 0x08000000, LENGTH = 828K
  GP_PROXY_TABLE   : ORIGIN = 0x080CF000, LENGTH = 16K
  SETTINGS         : ORIGIN = 0x080D3000, LENGTH = 8K
  ROUTER_JOURNALS  : ORIGIN = 0x080D5000, LENGTH = 48K
  /* SRAM1. SRAM2a starts with the 10 KiB IPCC mailbox (tl_mbox.x, from
   * embassy-stm32-wpan); its upper part and SRAM2b belong to CPU2. */
  RAM              : ORIGIN = 0x20000000, LENGTH = 192K
}

ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == ORIGIN(GP_PROXY_TABLE),
  "STM32WB application must end exactly at the GP Proxy Table journal");
ASSERT(ORIGIN(GP_PROXY_TABLE) + LENGTH(GP_PROXY_TABLE) == ORIGIN(SETTINGS),
  "STM32WB GP Proxy Table journal must end exactly at the settings journal");
ASSERT(ORIGIN(SETTINGS) == 0x080D3000,
  "STM32WB settings journal must start at 0x080D3000 (src/layout.rs)");
ASSERT(ORIGIN(SETTINGS) + LENGTH(SETTINGS) == ORIGIN(ROUTER_JOURNALS),
  "STM32WB settings journal must end exactly at the router journals");
ASSERT(ORIGIN(ROUTER_JOURNALS) == 0x080D5000,
  "STM32WB router journals must start at 0x080D5000 (src/layout.rs)");
ASSERT(ORIGIN(ROUTER_JOURNALS) + LENGTH(ROUTER_JOURNALS) == 0x080E1000,
  "STM32WB router journals must end at the CPU2 secure area (SFSA 0xE1)");
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == 0x20030000,
  "STM32WB CPU1 RAM must end at SRAM2a (IPCC mailbox)");
ASSERT(_stack_start - __euninit >= 16K,
  "STM32WB static task storage must leave at least 16 KiB for the stack");
