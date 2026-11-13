/* Production TLSR8258 TB-04 product layout.
 *
 * Writable data follows the Telink SDK cache reservation:
 *   0x840000 + A         RAM-code backing end / I-cache tag start
 *   0x840100 + A         I-cache tag end / I-cache data start
 *   0x840900 + A         I-cache data end / .data start
 *   0x850000             top of SRAM
 *
 * A is the RAM-code preload size rounded up to 256 bytes. The product-owned
 * NV partitions occupy flash 0x6E000..0x76000:
 *   0x6E000..0x70000  router Green Power Proxy Table journal (reserved in
 *                     every image; only the router writes it)
 *   0x70000..0x72000  APS binding/group journal
 *   0x72000..0x74000  child-table journal (router/coordinator child records)
 *   0x74000..0x76000  security journal (frame counters, keys, network state)
 * Telink factory data remains untouched:
 *   0x76000..0x77000  factory EUI-64
 *   0x77000..0x78000  factory config and ADC calibration
 * They are separate two-sector journals on purpose: the security record is
 * rewritten on every frame-counter reservation, while the APS and child
 * journals are rewritten only when their respective tables change.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 0x6E000
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
        KEEP(*(.retention_image_marker));
        KEEP(*(.tb04_router_image_marker));
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

    /* Key application/profile/lifecycle cells and the PM hand-off record are
     * explicit NOLOAD retained state in the proof image. Cold startup
     * initializes them manually; retention startup never copies/clears them.
     */
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

    /*
     * Both layouts keep the stacks at the top of the 64 KiB SRAM: the SVC
     * stack grows down from 0x84FC00, a 64-byte guard sits immediately below
     * it, and the 1 KiB IRQ stack occupies [0x84FC00, 0x850000). At least
     * 4 KiB of unused physical RAM must separate the last static allocation
     * from the guard; that gap is not part of the stack budget.
     *
     * The leaf images (default sensor, End Device, retention sensor) use a
     * 24 KiB SVC stack: their analyzed worst cases exceed or nearly fill
     * 20 KiB (tools/tlsr8258-{sensor,ed,sensor-retention}-stack-gate.args).
     * The router image emits TELINK_TB04_ROUTER_IMAGE and keeps a 20 KiB SVC
     * stack: its larger .bss leaves no room for more above the 4 KiB gap,
     * and its analyzed worst case fits (tools/tlsr8258-router-stack-
     * gate.args). A router that lost its marker would get 24 KiB and fail
     * the static-gap assertion below.
     *
     * The retention image emits TELINK_RETENTION_IMAGE. In that image the
     * stacks and their guard live entirely in the upper 32 KiB, which is NOT
     * retained in LOW32K deep retention: a retention wake re-enters
     * `__reset`, which reloads both stack pointers and re-arms the SVC guard
     * before any stack-deep work, so no stack content is needed across
     * sleep. Everything that must survive sleep (.ram_code, .data, .bss,
     * .retained, .rf_dma) stays below _retention_limit_, followed by a
     * 256-byte retained tripwire that is painted on cold boot and verified
     * before a wake is accepted.
     */
    _retention_image_enabled_ = DEFINED(TELINK_RETENTION_IMAGE);
    _router_image_enabled_ = DEFINED(TELINK_TB04_ROUTER_IMAGE);
    _retention_limit_ = 0x00848000;
    _retention_stack_guard_start_ =
        _retention_image_enabled_ ? ((_rf_dma_end_ + 15) & ~15) : 0;
    _retention_stack_guard_end_ =
        _retention_image_enabled_ ? _retention_stack_guard_start_ + 0x100 : 0;

    _default_svc_stack_top_ = 0x0084FC00;
    _leaf_svc_stack_bytes_ = 24 * 1024;
    _router_svc_stack_bytes_ = 20 * 1024;
    _default_svc_stack_guard_bytes_ = 64;
    _default_min_static_gap_ = 4096;
    _svc_stack_top    = _default_svc_stack_top_;
    _svc_stack_bottom = _default_svc_stack_top_ - (_router_image_enabled_
        ? _router_svc_stack_bytes_ : _leaf_svc_stack_bytes_);
    _svc_stack_guard_end = _svc_stack_bottom;
    _svc_stack_guard_start = _svc_stack_bottom - _default_svc_stack_guard_bytes_;
    _irq_stack_bottom = 0x0084FC00;
    _irq_stack_top    = 0x00850000;
    _stack_top = _svc_stack_top;

    _bin_size_ = _code_size_ + SIZEOF(.data);
    _bin_size_div_16 = (_bin_size_ + 15) / 16;
    _etext = _dstored_;
    _gp_nv_start_ = 0x6E000;
    _gp_nv_end_ = 0x70000;
    _aps_nv_start_ = 0x70000;
    _aps_nv_end_ = 0x72000;
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
    _assert_single_layout_marker = ASSERT(
        !(_retention_image_enabled_ && _router_image_enabled_),
        "ERROR: an image cannot be both the router and the retention sensor");
    _assert_router_svc_stack = ASSERT(!_router_image_enabled_ ||
        (_svc_stack_top - _svc_stack_bottom) == _router_svc_stack_bytes_,
        "ERROR: router SVC stack must be exactly 20 KiB");
    _assert_leaf_svc_stack = ASSERT(_router_image_enabled_ ||
        (_svc_stack_top - _svc_stack_bottom) == _leaf_svc_stack_bytes_,
        "ERROR: sensor/End Device SVC stack must be exactly 24 KiB");
    _assert_svc_guard = ASSERT(
        (_svc_stack_guard_end - _svc_stack_guard_start) == _default_svc_stack_guard_bytes_,
        "ERROR: SVC stack guard must be 64 bytes");
    _assert_static_gap = ASSERT(
        _svc_stack_guard_start >= _rf_dma_end_ + _default_min_static_gap_ &&
        _svc_stack_guard_start >= _ebss + _default_min_static_gap_,
        "ERROR: less than 4 KiB between static RAM and the SVC stack guard");
    _assert_stacks_above_irq = ASSERT(_svc_stack_top <= _irq_stack_bottom &&
        _irq_stack_top <= 0x00850000,
        "ERROR: SVC/IRQ stacks overlap or exceed physical SRAM");
    _assert_retained_state_low32 = ASSERT(!_retention_image_enabled_ ||
        (_retained_end_ <= _retention_limit_ && _rf_dma_end_ <= _retention_limit_ &&
         _ebss <= _retention_limit_ &&
         _retention_stack_guard_end_ <= _retention_limit_),
        "ERROR: retained writable/RF state or its tripwire exceeds LOW32K");
    _assert_retention_guard = ASSERT(!_retention_image_enabled_ ||
        (_retention_stack_guard_end_ - _retention_stack_guard_start_) >= 0x100,
        "ERROR: LOW32K retained tripwire is smaller than 256 bytes");
    _assert_retention_stacks_unretained = ASSERT(!_retention_image_enabled_ ||
        _svc_stack_guard_start >= _retention_limit_,
        "ERROR: retention stacks/guard must live in the non-retained upper 32 KiB");
    _assert_image_below_gp_nv = ASSERT(_bin_size_ <= _gp_nv_start_,
        "ERROR: firmware image overlaps the GP Proxy Table journal at 0x6E000");
    _assert_gp_nv_before_aps_nv = ASSERT(_gp_nv_end_ <= _aps_nv_start_,
        "ERROR: GP Proxy Table journal overlaps the APS-table journal");
    _assert_image_below_aps_nv = ASSERT(_bin_size_ <= _aps_nv_start_,
        "ERROR: firmware image overlaps APS-table journal at 0x70000");
    _assert_aps_nv_before_child_nv = ASSERT(_aps_nv_end_ <= _child_nv_start_,
        "ERROR: APS-table journal overlaps the child-table journal");
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
