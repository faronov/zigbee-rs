/* Dedicated TLSR8258 LOW32K reset-on-wake lab layout.
 *
 * All writable state, explicit hand-off cells, RF DMA, a 256-byte underflow
 * guard and both banked stacks are below 0x848000. Debug SRAM remains outside
 * retention and is rewritten on each fresh entry.
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

    .retained (NOLOAD) :
    {
        . = ALIGN(4);
        _retained_start_ = .;
        KEEP(*(.retained));
        KEEP(*(.retained.*));
        . = ALIGN(4);
        _retained_end_ = .;
    } > RAM

    .rf_dma (NOLOAD) :
    {
        . = ALIGN(4);
        _rf_dma_start_ = .;
        KEEP(*(.rf_dma));
        . = ALIGN(4);
        _rf_dma_end_ = .;
    } > RAM

    _retention_stack_guard_start_ = (_rf_dma_end_ + 15) & ~15;
    _retention_stack_guard_end_ = _retention_stack_guard_start_ + 0x100;
    _svc_stack_bottom = (_retention_stack_guard_end_ + 0xFF) & ~0xFF;
    _svc_stack_top    = 0x00847800;
    _irq_stack_bottom = 0x00847800;
    _irq_stack_top    = 0x00847C00;
    _stack_top = _svc_stack_top;

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
        "ERROR: .ram_code overflows FLASH+0x8000");
    _assert_cache_layout = ASSERT(_sdata >= _icache_data_end_,
        "ERROR: writable data overlaps cache reservation");
    _assert_retained_state = ASSERT(_retained_end_ <= _retention_limit_,
        "ERROR: retained state exceeds LOW32K");
    _assert_dma_retained = ASSERT(_rf_dma_end_ <= _retention_stack_guard_start_,
        "ERROR: RF DMA overlaps retained guard");
    _assert_guard = ASSERT(
        (_retention_stack_guard_end_ - _retention_stack_guard_start_) >= 0x100,
        "ERROR: retained stack guard is too small");
    _assert_stack_size = ASSERT((_svc_stack_top - _svc_stack_bottom) >= 0x2000,
        "ERROR: fresh SVC stack is smaller than 8 KiB");
    _assert_stacks_retained = ASSERT(_irq_stack_top <= _retention_limit_,
        "ERROR: stacks exceed LOW32K");
    _assert_top_guard = ASSERT((_retention_limit_ - _irq_stack_top) >= 0x400,
        "ERROR: top guard is smaller than 1 KiB");
    _assert_image_below_security_nv = ASSERT(_bin_size_ <= _security_nv_start_,
        "ERROR: image overlaps security journal");

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
        *(.debug*);
    }
}
