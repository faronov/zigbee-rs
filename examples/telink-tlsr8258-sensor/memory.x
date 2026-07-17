/* TLSR8258 memory layout — pure Rust, no vendor SDK.
 *
 * This follows Telink SDK V3.7.2.0 platform/boot/8258/boot_8258.link:
 *
 *   0x840000 + A         RAM-code backing end / I-cache tag start
 *   0x840100 + A         I-cache tag end / I-cache data start
 *   0x840900 + A         I-cache data end / .data start
 *   0x850000             top of the 64 KiB SRAM window
 *
 * where A is the RAM-code preload size rounded up to 256 bytes. The 0x800
 * bytes after the 0x100-byte tag array are reserved for I-cache data and must
 * not contain mutable program state or RF DMA buffers.
 */
MEMORY
{
    FLASH : ORIGIN = 0x00000000, LENGTH = 512K
    RAM   : ORIGIN = 0x00840000, LENGTH = 0x10000
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

    /* Telink boot ROM preloads the beginning of the image into the code SRAM
     * alias at 0x00880000. Keep RAM-resident routines in that preload area, as
     * the vendor B85 linker does.
     */
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

    /* Cached flash code starts at the same offset used by the Telink SDK. */
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

    /* Telink reserves RAM-code backing, 0x100 bytes of I-cache tags, then
     * 0x800 bytes of I-cache data before writable program data. */
    _ictag_start_ = 0x840000 + _ramcode_size_align_256_;
    _ictag_end_ = _ictag_start_ + 0x100;
    _icache_data_start_ = _ictag_end_;
    _icache_data_end_ = _icache_data_start_ + 0x800;
    _sram_data_start_ = 0x840900 + _ramcode_size_align_256_;

    /* Initialized data is copied here from flash by the reset handler. */
    .data _sram_data_start_ : AT(_dstored_)
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

    /* SWire-readable diagnostics near the top of SRAM, above the stacks. The
     * telink-debug probes also poke raw counters/SP-min slots up to ~0x84F800,
     * so the stacks are kept below 0x0084F000. */
    .debug_sram 0x0084F000 (NOLOAD) :
    {
        . = ALIGN(4);
        _debug_sram_start = .;
        KEEP(*(.debug_sram));
        . = ALIGN(4);
        _debug_sram_end = .;
    } > RAM

    /* Stack layout (tc32 uses banked, descending stacks). The debug-heavy
     * bring-up image keeps diagnostics at 0x84F000, so its stacks remain below
     * that reservation rather than using the SDK's normal 0x850000 stack top.
     *
     *   SVC stack:  _svc_stack_bottom (0x0084B400) .. _svc_stack_top (0x0084E000)   [11 KiB]
     *   IRQ stack:  _irq_stack_bottom (0x0084E000) .. _irq_stack_top (0x0084F000)   [4 KiB]
     * Debug SRAM sits at 0x0084F000+, so the IRQ stack abuts it from below.
     *
     * The `_ebss <= _svc_stack_bottom` assert below still guards against any
     * future .bss growth large enough to climb back into the stack region.
     *
     * The reset/IRQ asm initialises SP from these symbols so the linker
     * remains the single source of truth for stack placement.
     */
    _svc_stack_bottom = 0x0084B400;
    _svc_stack_top    = 0x0084E000;
    _irq_stack_bottom = 0x0084E000;
    _irq_stack_top    = 0x0084F000;

    /* Backwards-compatible alias for older code paths. */
    _stack_top = _svc_stack_top;

    /* Boot header size fields (used by boot ROM) */
    _bin_size_ = _code_size_ + SIZEOF(.data);
    _bin_size_div_16 = (_bin_size_ + 15) / 16;
    _etext = _dstored_;

    /* Linker symbol aliases (for startup code compatibility) */
    _ramcode_stored_ = LOADADDR(.ram_code);
    _start_data_ = _sdata;
    _end_data_ = _edata;
    _start_bss_ = _sbss;
    _end_bss_ = _ebss;
    _stack_end_ = _stack_top;
    /* Custom data sections (unused — set to zero-length) */
    _custom_stored_ = _etext;
    _start_custom_data_ = _edata;
    _end_custom_data_ = _edata;
    _start_custom_bss_ = _ebss;
    _end_custom_bss_ = _ebss;

    /* Safety asserts (assigned to dummy symbols so ld.lld evaluates them). */
    _assert_ramcode_fits = ASSERT(_ramcode_end_ <= 0x8000,
        "ERROR: .ram_code overflows the absolute .text base at FLASH+0x8000");
    _assert_cache_layout = ASSERT(_sdata >= _icache_data_end_,
        "ERROR: .data overlaps the TLSR8258 I-cache tag/data reservation");
    _assert_bss_under_stack = ASSERT(_ebss <= _svc_stack_bottom,
        "ERROR: .bss/.data extends into the SVC stack region; shrink statics or lower _svc_stack_bottom in memory.x");

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
        *(.debug*);
    }
}
