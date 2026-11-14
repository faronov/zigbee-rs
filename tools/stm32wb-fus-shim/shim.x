/* Flash-resident sequencer: the vector table and code at the start of the
 * user flash (the ROM DFU bootloader programs it there), the stack at the
 * top of SRAM1. SRAM2a (0x20030000) holds the coprocessor mailbox and is
 * never part of the image. There is no .data/.bss: all state is on the
 * stack or in the flash log page, so no startup copy runs. */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 0x2000
  RAM   : ORIGIN = 0x20004000, LENGTH = 0x2C000
}

ENTRY(seq_reset);

SECTIONS
{
  .vector_table ORIGIN(FLASH) :
  {
    KEEP(*(.vector_table));
  } > FLASH

  .text :
  {
    *(.text .text.*);
    *(.rodata .rodata.*);
  } > FLASH

  .data : { *(.data .data.*); } > RAM AT > FLASH
  .bss (NOLOAD) : { *(.bss .bss.* COMMON); } > RAM

  /DISCARD/ : { *(.ARM.exidx .ARM.exidx.* .ARM.extab.*); }
}

ASSERT(SIZEOF(.data) == 0, "the sequencer must not have .data");
ASSERT(SIZEOF(.bss) == 0, "the sequencer must not have .bss");
