/* Production TLSR8258 TB-04 product layout.
 *
 * Writable data follows the Telink SDK cache reservation:
 *   0x840000 + A         RAM-code backing end / I-cache tag start
 *   0x840100 + A         I-cache tag end / I-cache data start
 *   0x840900 + A         I-cache data end / .data start
 *   0x850000             top of SRAM
 *
 * A is the RAM-code preload size rounded up to 256 bytes. The product-owned
 * NV partitions occupy flash 0x72000..0x76000:
 *   0x72000..0x74000  child-table journal (router/coordinator child records)
 *   0x74000..0x76000  security journal (frame counters, keys, network state)
 * Telink factory data remains untouched:
 *   0x76000..0x77000  factory EUI-64
 *   0x77000..0x78000  factory config and ADC calibration
 * They are separate two-sector journals on purpose: the security record is
 * rewritten on every frame-counter reservation, the child table only on a
 * child lifecycle transition.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 0x72000
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

    /* Keep a 16 KiB SVC stack and 1 KiB IRQ stack at the top of SRAM. */
    _svc_stack_bottom = 0x0084BC00;
    _svc_stack_top    = 0x0084FC00;
    _irq_stack_bottom = 0x0084FC00;
    _irq_stack_top    = 0x00850000;
    _stack_top = _svc_stack_top;

    _bin_size_ = _code_size_ + SIZEOF(.data);
    _bin_size_div_16 = (_bin_size_ + 15) / 16;
    _etext = _dstored_;
    _child_nv_start_ = 0x72000;
    _child_nv_end_ = 0x74000;
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
        "ERROR: .bss/.data extends into the SVC stack region");
    _assert_dma_outside_cache = ASSERT(_rf_dma_start_ >= _icache_data_end_,
        "ERROR: .rf_dma overlaps the TLSR8258 I-cache tag/data reservation");
    _assert_dma_under_stack = ASSERT(_rf_dma_end_ <= _svc_stack_bottom,
        "ERROR: .rf_dma extends into the SVC stack region");
    _assert_image_below_child_nv = ASSERT(_bin_size_ <= _child_nv_start_,
        "ERROR: firmware image overlaps child-table journal at 0x72000");
    _assert_child_nv_before_security_nv = ASSERT(_child_nv_end_ <= _security_nv_start_,
        "ERROR: child-table journal overlaps the security journal");
    _assert_security_nv_before_factory_eui = ASSERT(_security_nv_end_ <= 0x76000,
        "ERROR: security journal overlaps the Telink factory EUI-64 sector");

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
    }
}
