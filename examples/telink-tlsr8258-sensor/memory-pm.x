/* TLSR8258 LOW32K-retention diagnostic layout.
 *
 * The retained SRAM window is 0x840000..0x848000. All writable state, DMA,
 * and both banked stacks used by diag-pm must remain inside that window.
 * Production keeps its existing larger top-of-SRAM stacks until this layout
 * and the deep-retention wake path are hardware-proven.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 0x74000
    RAM   : ORIGIN = 0x00840000, LENGTH = 0x10000
}

ENTRY(_reset_vector);

SECTIONS
{
    .vectors :
    {
        KEEP(*(.vectors));
        KEEP(*(.vectors.*));
    } > FLASH

    .ram_code :
    {
        _ramcode_start_ = .;
        *(.ram_code .ram_code.*);
        _ramcode_end_ = .;
    } > FLASH
    . = ALIGN(4);
    _rstored_ = .;
    _ramcode_size_ = .;
    _ramcode_size_div_16_ = (. + 15) / 16;
    _ramcode_size_div_256_ = (. + 255) / 256;
    _ramcode_size_div_16_align_256_ = ((. + 255) / 256) * 16;
    _ramcode_size_align_256_ = _ramcode_size_div_16_align_256_ * 16;

    .text 0x8000 :
    {
        *(.text._start);
        *(.text._start.*);
        *(.text .text.*);
        *(.rodata .rodata.*);
        *(.ARM.exidx .ARM.exidx.*);
    } > FLASH
    . = ALIGN(4);
    _dstored_ = .;
    _code_size_ = .;

    _ictag_start_ = 0x840000 + _ramcode_size_align_256_;
    _ictag_end_ = _ictag_start_ + 0x100;
    _icache_data_start_ = _ictag_end_;
    _icache_data_end_ = _icache_data_start_ + 0x800;
    _sram_data_start_ = 0x840900 + _ramcode_size_align_256_;
    _retention_limit_ = 0x848000;

    .data _sram_data_start_ : AT(_dstored_)
    {
        _sdata = .;
        *(.data .data.*);
        . = ALIGN(4);
        _edata = .;
    } > RAM

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

    .rf_dma (NOLOAD) :
    {
        . = ALIGN(4);
        _rf_dma_start_ = .;
        KEEP(*(.rf_dma));
        . = ALIGN(4);
        _rf_dma_end_ = .;
    } > RAM

    /* 12 KiB SVC + 1 KiB IRQ, with 1 KiB guard below the retention limit. */
    _svc_stack_bottom = 0x00844800;
    _svc_stack_top    = 0x00847800;
    _irq_stack_bottom = 0x00847800;
    _irq_stack_top    = 0x00847C00;
    _stack_top = _svc_stack_top;

    /* Diagnostics need not survive retention; they are rewritten after wake. */
    .debug_sram 0x0084F000 (NOLOAD) :
    {
        . = ALIGN(4);
        _debug_sram_start = .;
        KEEP(*(.debug_sram));
        . = ALIGN(4);
        _debug_sram_end = .;
    } > RAM

    _bin_size_ = _code_size_ + SIZEOF(.data);
    _bin_size_div_16 = (_bin_size_ + 15) / 16;
    _etext = _dstored_;
    _security_nv_start_ = 0x74000;
    _security_nv_end_ = 0x76000;

    _ramcode_stored_ = LOADADDR(.ram_code);
    _start_data_ = _sdata;
    _end_data_ = _edata;
    _start_bss_ = _sbss;
    _end_bss_ = _ebss;
    _stack_end_ = _stack_top;
    _custom_stored_ = _etext;
    _start_custom_data_ = _edata;
    _end_custom_data_ = _edata;
    _start_custom_bss_ = _ebss;
    _end_custom_bss_ = _ebss;

    _assert_ramcode_fits = ASSERT(_ramcode_end_ <= 0x8000,
        "ERROR: .ram_code overflows the absolute .text base at FLASH+0x8000");
    _assert_cache_layout = ASSERT(_sdata >= _icache_data_end_,
        "ERROR: .data overlaps the TLSR8258 I-cache tag/data reservation");
    _assert_bss_under_stack = ASSERT(_ebss <= _svc_stack_bottom,
        "ERROR: .bss/.data extends into the retained SVC stack");
    _assert_dma_outside_cache = ASSERT(_rf_dma_start_ >= _icache_data_end_,
        "ERROR: .rf_dma overlaps the TLSR8258 I-cache tag/data reservation");
    _assert_dma_under_stack = ASSERT(_rf_dma_end_ <= _svc_stack_bottom,
        "ERROR: .rf_dma extends into the retained SVC stack");
    _assert_data_retained = ASSERT(_ebss <= _retention_limit_,
        "ERROR: .data/.bss exceeds LOW32K retention");
    _assert_dma_retained = ASSERT(_rf_dma_end_ <= _retention_limit_,
        "ERROR: .rf_dma exceeds LOW32K retention");
    _assert_stacks_retained = ASSERT(_irq_stack_top <= _retention_limit_,
        "ERROR: stacks exceed LOW32K retention");
    _assert_image_below_security_nv = ASSERT(_bin_size_ <= _security_nv_start_,
        "ERROR: firmware image overlaps security journal at 0x74000");

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
        *(.debug*);
    }
}
