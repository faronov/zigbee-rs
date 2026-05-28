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

    /* Initialized data (loaded from flash after ram_code, copied to RAM on startup) */
    .data 0x00848300 : AT(_dstored_)
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

    /* SWire-readable diagnostics near the top of SRAM. Keep this away from the
     * lower SRAM area used by Telink's instruction cache while executing XIP.
     */
    .debug_sram 0x0084F000 (NOLOAD) :
    {
        . = ALIGN(4);
        _debug_sram_start = .;
        KEEP(*(.debug_sram));
        . = ALIGN(4);
        _debug_sram_end = .;
    } > RAM

    /* Stack layout (tc32 uses banked, descending stacks). The SVC stack must
     * start well above `_ebss`; the runtime-sensor build's .bss approaches
     * 0x0084B000 with all cluster MaybeUninit slots, so the prior hard-coded
     * 0x0084B000 SVC top left <1 KiB of usable stack before corrupting .bss.
     *
     *   SVC stack:  _svc_stack_bottom (0x0084B400) .. _svc_stack_top (0x0084E000)   [11 KiB]
     *   IRQ stack:  _irq_stack_bottom (0x0084E000) .. _irq_stack_top (0x0084F000)   [4 KiB]
     * Debug SRAM sits at 0x0084F000+, so the IRQ stack abuts it from below.
     *
     * NOTE: stack was 8 KiB (bottom=0x0084C000) and the SVC high-water mark
     * (painted-pattern scan recorded at MODE+0x1AC) showed full 8 KiB
     * occupancy — i.e., a SINGLE-WORD safety margin away from clobbering
     * .bss. That correlates with the observed `aes_ccm_decrypt` hang during
     * BDB Transport-Key wait: when BDB ran multiple TC-link-key tries the
     * descending stack briefly punched into NwkSecurity state in .bss,
     * leaving the RustCrypto `ccm` crate spinning on corrupt context. The
     * stack was grown to 11 KiB so even the deepest async/BDB call paths
     * stay within budget.
     *
     * _ebss after the runtime-sensor build sits at 0x0084B224; the assert
     * below catches any future growth that would re-collide.
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
    /* I-cache tags sit immediately after the boot/RAM-code preload area. */
    _ictag_start_ = 0x840000 + _ramcode_size_div_256_ * 0x100;
    _ictag_end_ = _ictag_start_ + 0x100;
    /* Custom data sections (unused — set to zero-length) */
    _custom_stored_ = _etext;
    _start_custom_data_ = _edata;
    _end_custom_data_ = _edata;
    _start_custom_bss_ = _ebss;
    _end_custom_bss_ = _ebss;

    /* Safety asserts (assigned to dummy symbols so ld.lld evaluates them). */
    _assert_ramcode_fits = ASSERT(_ramcode_end_ <= 0x8000,
        "ERROR: .ram_code overflows the absolute .text base at FLASH+0x8000");
    _assert_bss_under_stack = ASSERT(_ebss <= _svc_stack_bottom,
        "ERROR: .bss/.data extends into the SVC stack region; shrink statics or lower _svc_stack_bottom in memory.x");

    /DISCARD/ :
    {
        *(.ARM.attributes);
        *(.comment);
        *(.debug*);
    }
}
