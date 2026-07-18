INCLUDE memory.x
INCLUDE device.x

EXTERN(__RESET_VECTOR);
EXTERN(Reset);
ENTRY(Reset);

EXTERN(DefaultHandler);
EXTERN(HardFaultTrampoline);

PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(MemoryManagement = DefaultHandler);
PROVIDE(BusFault = DefaultHandler);
PROVIDE(UsageFault = DefaultHandler);
PROVIDE(SecureFault = DefaultHandler);
PROVIDE(SVCall = DefaultHandler);
PROVIDE(DebugMonitor = DefaultHandler);
PROVIDE(PendSV = DefaultHandler);
PROVIDE(SysTick = DefaultHandler);
PROVIDE(DefaultHandler = DefaultHandler_);
PROVIDE(HardFault = HardFault_);
PROVIDE(__pre_init = DefaultPreInit);

SECTIONS
{
  PROVIDE(_ram_start = ORIGIN(RAM));
  PROVIDE(_ram_end = ORIGIN(RAM) + LENGTH(RAM));
  PROVIDE(_stack_start = _ram_end);

  /* PHY62x2 ROM dispatches exceptions and peripheral IRQs through application
   * pointers 219..255 in the 256-entry table at 0x1fff0000. */
  .jump_table ORIGIN(JUMP_TABLE) :
  {
    __jump_table = .;
    FILL(0);
    . = ORIGIN(JUMP_TABLE) + 219 * 4;
    LONG(NonMaskableInt);
    LONG(HardFault);
    LONG(SVCall);
    LONG(PendSV);
    LONG(SysTick);
    LONG(V0);
    LONG(V1);
    LONG(V2);
    LONG(V3);
    LONG(LL_IRQ);
    LONG(KSCAN);
    LONG(RTC);
    LONG(V7);
    LONG(V8);
    LONG(V9);
    LONG(WDT);
    LONG(UART0);
    LONG(I2C0);
    LONG(I2C1);
    LONG(SPI0);
    LONG(SPI1);
    LONG(GPIO);
    LONG(UART1);
    LONG(SPIF);
    LONG(DMAC);
    LONG(TIM1);
    LONG(TIM2);
    LONG(TIM3);
    LONG(TIM4);
    LONG(TIM5);
    LONG(TIM6);
    LONG(V26);
    LONG(V27);
    LONG(AES);
    LONG(ADCC);
    LONG(QDEC);
    LONG(RNG);
    . = ORIGIN(JUMP_TABLE) + LENGTH(JUMP_TABLE);
  } > JUMP_TABLE

  /* The PHY6 ROM loader obtains the initial stack and Reset entry from this
   * descriptor. Interrupts use .jump_table above, not this address. */
  .vector_table ORIGIN(RAM) :
  {
    __vector_table = .;
    LONG(_stack_start & 0xfffffff8);
    KEEP(*(.vector_table.reset_vector));
  } > RAM

  .text ORIGIN(FLASH) :
  {
    __stext = .;
    *(.Reset);
    *(.text .text.*);
    *(.HardFaultTrampoline);
    *(.HardFault.*);
    . = ALIGN(4);
    __etext = .;
  } > FLASH

  .rodata : ALIGN(4)
  {
    . = ALIGN(4);
    __srodata = .;
    *(.rodata .rodata.*);
    . = ALIGN(4);
    __erodata = .;
  } > FLASH

  .data : ALIGN(4)
  {
    . = ALIGN(4);
    __sdata = .;
    *(.data .data.*);
    . = ALIGN(4);
  } > RAM AT>FLASH
  . = ALIGN(4);
  __edata = .;
  __sidata = LOADADDR(.data);

  .gnu.sgstubs : ALIGN(32)
  {
    . = ALIGN(32);
    __veneer_base = .;
    *(.gnu.sgstubs*);
    . = ALIGN(32);
  } > FLASH
  . = ALIGN(32);
  __veneer_limit = .;

  .bss (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __sbss = .;
    *(.bss .bss.*);
    *(COMMON);
    . = ALIGN(4);
  } > RAM
  . = ALIGN(4);
  __ebss = .;

  .uninit (NOLOAD) : ALIGN(4)
  {
    . = ALIGN(4);
    __suninit = .;
    *(.uninit .uninit.*);
    . = ALIGN(4);
    __euninit = .;
  } > RAM

  PROVIDE(__sheap = __euninit);
  PROVIDE(_stack_end = __euninit);

  .got (NOLOAD) :
  {
    KEEP(*(.got .got.*));
  }

  /DISCARD/ :
  {
    *(.ARM.exidx);
    *(.ARM.exidx.*);
    *(.ARM.extab.*);
  }
}

ASSERT(ADDR(.vector_table) == 0x1fff1838,
       "PHY62x2 run descriptor must start at the ROM run address");
ASSERT(SIZEOF(.vector_table) == 8,
       "PHY62x2 run descriptor must contain only stack and Reset");
ASSERT(ADDR(.jump_table) == 0x1fff0000,
       "PHY62x2 ROM jump table has the wrong address");
ASSERT(SIZEOF(.jump_table) == 0x400,
       "PHY62x2 ROM jump table must contain 256 entries");
ASSERT(__stext == 0x11010100,
       "PHY62x2 XIP text must start immediately after the PHY6 header");
ASSERT(__sdata >= ADDR(.vector_table) + SIZEOF(.vector_table),
       "PHY62x2 data overlaps the ROM-loaded run descriptor");
ASSERT(__ebss <= _stack_start,
       "PHY62x2 RAM sections overlap the stack");
ASSERT(SIZEOF(.got) == 0,
       "PHY62x2 firmware cannot contain dynamic relocations");
