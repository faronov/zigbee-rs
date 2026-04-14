/* TLSR8258 memory layout — pure Rust, no vendor SDK
 *
 * RAM starts at 0x840000 but first 0x900 bytes are used by boot ROM
 * for I-cache tags (256 bytes at 0x840000) and cache data (0x840100-0x8408FF).
 * User code data starts at 0x840900.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 512K
    RAM   : ORIGIN = 0x00840900, LENGTH = 64K - 0x900
}

ENTRY(_reset_vector);

SECTIONS
{
    /*
     * Telink boot vector at flash offset 0.
     * Layout matches cstartup_8258.S exactly — see global_asm! in main.rs.
     */
    .vectors :
    {
        KEEP(*(.vectors));
        KEEP(*(.vectors.*));
    } > FLASH

    /* Code — starts right after vectors section */
    .text ALIGN(4) :
    {
        *(.text._start);
        *(.text._start.*);
        *(.text .text.*);
        *(.rodata .rodata.*);
        *(.ARM.exidx .ARM.exidx.*);
    } > FLASH
    _etext = .;
    _code_size_ = .;

    /* Initialized data (loaded from flash, copied to RAM on startup) */
    .data : AT(_etext)
    {
        _sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
        _edata = .;
    } > RAM

    /* Zero-initialized data */
    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*);
        *(.bss.irq_stk);
        *(COMMON);
        . = ALIGN(4);
        _ebss = .;
    } > RAM

    /* Stack at end of RAM (grows down) */
    _stack_top = ORIGIN(RAM) + LENGTH(RAM);

    /* Boot header size fields (used by boot ROM) */
    _bin_size_ = _code_size_ + SIZEOF(.data);
    _bin_size_div_16 = (_bin_size_ + 15) / 16;

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
        *(.debug*);
    }
}
