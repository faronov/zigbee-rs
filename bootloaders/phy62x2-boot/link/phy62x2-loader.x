/* PHY62x2 zigbee-rs loader, started by the boot ROM.
 *
 * memory.x is generated from the product layout (FLASH = the loader's XIP
 * code, LOADER_INFO = its last 64 bytes, RAM). `phy62x2-image --rom-boot`
 * turns the ELF into the ROM-boot table (flash 0x2000) and the SRAM load
 * data (flash 0x5000: this jump table and the run descriptor) followed by
 * the XIP code. The ROM copies the jump table to 0x1fff0000 and the
 * `[sp][reset]` run descriptor to the start of RAM (0x1fff1838), then
 * starts Reset. The loader enables no interrupt; its jump table only routes
 * faults. Before the hand-over the loader replaces the jump table with the
 * application's (see src/main.rs).
 */
INCLUDE memory.x

MEMORY
{
    JUMP_TABLE (rw) : ORIGIN = 0x1fff0000, LENGTH = 0x400
}

EXTERN(__RESET_VECTOR);
EXTERN(Reset);
ENTRY(Reset);

EXTERN(DefaultHandler);
EXTERN(HardFaultTrampoline);

PROVIDE(NonMaskableInt = DefaultHandler);
PROVIDE(SVCall = DefaultHandler);
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

  /* Entries 0..218 stay zero (no ROM function overrides); 219..223 are the
   * core exceptions, 224..255 the 32 peripheral interrupts. */
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
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
    LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler); LONG(DefaultHandler);
  } > JUMP_TABLE

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
       "PHY62x2 loader: the run descriptor must start at the ROM run address");
ASSERT(SIZEOF(.vector_table) == 8,
       "PHY62x2 loader: the run descriptor must contain only stack and Reset");
ASSERT(ADDR(.jump_table) == 0x1fff0000 && SIZEOF(.jump_table) == 0x400,
       "PHY62x2 loader: the ROM jump table must be 256 entries at 0x1fff0000");
ASSERT(__sdata >= ADDR(.vector_table) + SIZEOF(.vector_table),
       "PHY62x2 loader: data overlaps the ROM-loaded run descriptor");
ASSERT(__ebss <= _stack_start - 2K,
       "PHY62x2 loader: static RAM leaves less than 2 KiB of stack");
ASSERT(SIZEOF(.got) == 0,
       "PHY62x2 loader: firmware cannot contain dynamic relocations");
