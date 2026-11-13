/* The whole helper runs from SRAM1. The ROM DFU bootloader keeps its own
 * data below 0x20004000 and hwlab only writes [0x20004000, 0x20030000).
 * SRAM2a (0x20030000) holds the coprocessor mailbox and is never part of
 * the image. There is no .data/.bss: all state is on the stack or in the
 * fixed parameter/result block, so no startup copy runs before interrupts
 * are masked. */
MEMORY
{
  IMAGE  : ORIGIN = 0x20008000, LENGTH = 0x8000
  STACK  : ORIGIN = 0x20010000, LENGTH = 0x1F000
  /* 0x2002F000: parameter block (host -> shim), 0x2002F100: results. */
}

ENTRY(shim_reset);

SECTIONS
{
  .vector_table ORIGIN(IMAGE) :
  {
    KEEP(*(.vector_table));
  } > IMAGE

  .text :
  {
    *(.text .text.*);
    *(.rodata .rodata.*);
  } > IMAGE

  .data : { *(.data .data.*); } > IMAGE
  .bss (NOLOAD) : { *(.bss .bss.* COMMON); } > IMAGE

  /DISCARD/ : { *(.ARM.exidx .ARM.exidx.* .ARM.extab.*); }
}

ASSERT(SIZEOF(.data) == 0, "the shim must not have .data");
ASSERT(SIZEOF(.bss) == 0, "the shim must not have .bss");
ASSERT(ORIGIN(IMAGE) % 0x200 == 0, "DFU leave needs a 0x200-aligned vector table");
