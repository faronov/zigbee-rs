/* TLSR8258 memory layout for the standalone raw-radio bring-up crate.
 *
 * This mirrors the corrected layout validated on hardware for
 * examples/telink-tlsr8258-sensor/memory.x (tc32-45, diag-beacon passed):
 *
 *   0x840000 + A         RAM-code backing end / I-cache tag start
 *   0x840000 + A + 0x100 I-cache tag end / I-cache data start
 *   0x840900 + A         I-cache data end / .data start
 *   0x850000             top of the 64 KiB SRAM window
 *
 * where A = align256(ram_code_size) — the RAM-code preload size rounded up
 * to 256 bytes. The 0x800 bytes after the 0x100-byte tag array are reserved
 * for I-cache data and must never contain mutable program state or RF DMA
 * buffers. See platform::layout for the Rust-side mirror of these symbols
 * and scripts/tlsr8258.sh:verify_layout for the post-link re-check (ld.lld
 * silently drops ASSERT() failures in some configurations, so the shell
 * script is the authoritative gate).
 */
MEMORY
{
    /* The final 48 KiB are not available to the linked image:
     *   0x74000..0x76000 security-state journal
     *   0x76000..0x80000 factory/configuration data */
    FLASH : ORIGIN = 0x00000000, LENGTH = 0x74000
    RAM   : ORIGIN = 0x00840000, LENGTH = 0x10000
}

ENTRY(_reset_vector);

SECTIONS
{
    /* Telink boot vector table at flash offset 0. Layout matches
     * cstartup_8258.S — see platform::vectors global_asm!. */
    .vectors :
    {
        KEEP(*(.vectors));
        KEEP(*(.vectors.*));
    } > FLASH

    /* Telink boot ROM preloads the beginning of the image into the code
     * SRAM alias at 0x00880000. Keep RAM-resident routines (flash-critical
     * timing paths) in that preload area, matching the vendor B85 linker. */
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

    /* Initialized data is copied here from flash by the reset handler.
     * `.data.canary_first` is placed explicitly ahead of the generic
     * `.data`/`.data.*` wildcard so the cache-boundary canary defined in
     * diag::CACHE_CANARY is guaranteed to be the first four bytes at
     * `_icache_data_end_` / `_sdata`. A cache-write overrun past the
     * reserved I-cache data region corrupts this word first, and only
     * this word, before touching any other static. */
    .data _sram_data_start_ : AT(_dstored_)
    {
        _sdata = .;
        KEEP(*(.data.canary_first));
        *(.data .data.*);
        . = ALIGN(4);
        _edata = .;
    } > RAM

    /* Zero-initialized data. */
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

    /* RF DMA buffers get their own explicitly-aligned output section so the
     * post-link check can find them by section name instead of relying on
     * symbol-name pattern matching, and so their placement is trivially
     * provable to sit after the I-cache reservation and below the stacks. */
    .rf_dma (NOLOAD) :
    {
        . = ALIGN(4);
        _rf_dma_start_ = .;
        KEEP(*(.rf_dma));
        . = ALIGN(4);
        _rf_dma_end_ = .;
    } > RAM

    /* Stack layout (tc32 uses banked, descending stacks). Both stacks sit
     * strictly below the diagnostic record reserved at the top of SRAM, so
     * a stack overflow (SP descending) moves *away* from the diagnostics,
     * never towards them.
     *
     *   SVC stack:  _svc_stack_bottom (0x0084BE00) .. _svc_stack_top (0x0084FE00) [16 KiB]
     *   IRQ stack:  _irq_stack_bottom (0x0084BA00) .. _irq_stack_top (0x0084BE00)  [1 KiB]
     *   Diagnostic: _diag_start_      (0x0084FE00) .. _diag_end_      (0x00850000) [512 B]
     *
     * The `_ebss <= _irq_stack_bottom` assert below guards against any
     * future .bss/.rf_dma growth large enough to climb back into the IRQ
     * stack region.
     */
    _svc_stack_top    = 0x0084FE00;
    _svc_stack_bottom = 0x0084BE00;
    _irq_stack_top    = 0x0084BE00;
    _irq_stack_bottom = 0x0084BA00;
    _stack_top = _svc_stack_top;

    /* Diagnostic record: fixed, documented address near the top of SRAM,
     * strictly outside `.bss`/`.data`/stacks and outside the startup zero
     * loop (which only touches [_sbss, _ebss)). NOINIT so the record
     * survives any reset that does not power-cycle SRAM; diag::init()
     * re-validates magic/version/checksum and re-initializes deterministically
     * on mismatch (cold power-on, corrupted record, or first flash). */
    .diag 0x0084FE00 (NOLOAD) :
    {
        . = ALIGN(4);
        _diag_start_ = .;
        KEEP(*(.diag));
        /* diag::hw accesses the record through a raw `DIAG_ADDR as *mut
         * DiagRecord` cast rather than a linked Rust `static`, so there is
         * no input section named `.diag` for `KEEP(*(.diag))` above to
         * actually match — reserve the documented 512 B explicitly via the
         * location counter so `_diag_end_` still lands at the true region
         * boundary (0x00850000, top of SRAM) instead of collapsing to
         * `_diag_start_` (which `llvm-readelf -S`/`llvm-size` would then
         * report as a 0-byte section). */
        . = 0x0084FE00 + 0x200;
        _diag_end_ = .;
    } > RAM

    /* Boot header size fields (used by boot ROM) */
    _bin_size_ = _code_size_ + SIZEOF(.data);
    _bin_size_div_16 = (_bin_size_ + 15) / 16;
    _etext = _dstored_;
    _security_nv_start_ = 0x74000;
    _security_nv_end_ = 0x76000;

    /* Linker symbol aliases (for startup code compatibility) */
    _ramcode_stored_ = LOADADDR(.ram_code);
    _start_data_ = _sdata;
    _end_data_ = _edata;
    _start_bss_ = _sbss;
    _end_bss_ = _ebss;
    _stack_end_ = _stack_top;

    /* Safety asserts (assigned to dummy symbols so ld.lld evaluates them).
     * These are documentation-grade only — scripts/tlsr8258.sh:verify_layout
     * re-checks the same invariants from the linked ELF symbol table because
     * some lld configurations silently drop ASSERT() failures. */
    _assert_ramcode_fits = ASSERT(_ramcode_end_ <= 0x8000,
        "ERROR: .ram_code overflows the absolute .text base at FLASH+0x8000");
    _assert_cache_layout = ASSERT(_sdata >= _icache_data_end_,
        "ERROR: .data overlaps the TLSR8258 I-cache tag/data reservation");
    _assert_dma_outside_cache = ASSERT(_rf_dma_start_ >= _icache_data_end_,
        "ERROR: .rf_dma overlaps the TLSR8258 I-cache tag/data reservation");
    _assert_bss_under_irq_stack = ASSERT(_ebss <= _irq_stack_bottom,
        "ERROR: .bss extends into the IRQ stack region; shrink statics or lower _irq_stack_bottom in memory.x");
    _assert_dma_under_irq_stack = ASSERT(_rf_dma_end_ <= _irq_stack_bottom,
        "ERROR: .rf_dma extends into the IRQ stack region");
    _assert_stack_under_diag = ASSERT(_svc_stack_top <= _diag_start_,
        "ERROR: SVC stack overlaps the diagnostic record");
    _assert_image_below_security_nv = ASSERT(_bin_size_ <= _security_nv_start_,
        "ERROR: firmware image overlaps security journal at 0x74000");

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
        *(.debug*);
    }
}
