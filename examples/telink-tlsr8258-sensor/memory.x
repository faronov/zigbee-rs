/* TLSR8258 memory layout — pure Rust, no vendor SDK
 *
 * Default I-cache is 32KB (reg 0x6C bits[3:2]=00), so the lower SRAM window
 * 0x840000-0x847FFF must be treated as cache territory. Earlier revisions put
 * .data/.bss there, which is acceptable only for fragile bring-up. For stable
 * runtime we place all writable sections in the upper 32KB SRAM window.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 512K
    RAM   : ORIGIN = 0x00848000, LENGTH = 0x8000
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

    /* Code — right after vectors so tj _start can reach.
     * VMA = LMA because code runs in-place from flash. */
    .text :
    {
        *(.text._start);
        *(.text._start.*);
        *(.text .text.*);
        *(.rodata .rodata.*);
        *(.ARM.exidx .ARM.exidx.*);
    } > FLASH
    _etext = .;

    /* RAM-resident code (HAL functions that must survive flash ops).
     * LMA in flash after .text, VMA in RAM. Copied by _start on boot. */
    .ram_code : AT(_etext)
    {
        . = ALIGN(256);
        _ramcode_start_ = .;
        *(.ram_code .ram_code.*);
        . = ALIGN(256);
        _ramcode_end_ = .;
    } > RAM
    _ramcode_stored_ = LOADADDR(.ram_code);
    _ramcode_size_ = _ramcode_end_ - _ramcode_start_;
    _ramcode_size_align_256_ = (_ramcode_size_ + 255) & 0xFFFFFF00;
    _code_size_ = _etext + SIZEOF(.ram_code);

    /* Initialized data (loaded from flash after ram_code, copied to RAM on startup) */
    .data : AT(_code_size_)
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

    /* Linker symbol aliases (for startup code compatibility) */
    _dstored_ = LOADADDR(.data);
    _start_data_ = _sdata;
    _end_data_ = _edata;
    _start_bss_ = _sbss;
    _end_bss_ = _ebss;
    _stack_end_ = _stack_top;
    /* I-cache tags: 256 bytes at start of RAM */
    _ictag_start_ = 0x840000;
    _ictag_end_ = 0x840100;
    /* Custom data sections (unused — set to zero-length) */
    _custom_stored_ = _etext;
    _start_custom_data_ = _edata;
    _end_custom_data_ = _edata;
    _start_custom_bss_ = _ebss;
    _end_custom_bss_ = _ebss;

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
        *(.debug*);
    }
}
