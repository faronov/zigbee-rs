//! Extern linker symbols and the layout self-checks that consume them.
//! Symbol *names* mirror `examples/telink-tlsr8258-sensor/memory.x` so the
//! two crates' `memory.x` files and post-link tooling stay directly
//! comparable. Only compiled for the real target (`target_arch = "tc32"`);
//! see `platform` module docs.

unsafe extern "C" {
    static _sdata: u8;
    static _edata: u8;
    static _sbss: u8;
    static _ebss: u8;
    static _etext: u8;
    static _icache_data_end_: u8;
    static _ictag_start_: u8;
    static _ictag_end_: u8;
    static _ramcode_start_: u8;
    static _ramcode_end_: u8;
    static _rf_dma_start_: u8;
    static _rf_dma_end_: u8;
    static _svc_stack_top: u8;
    static _svc_stack_bottom: u8;
    static _irq_stack_top: u8;
    static _irq_stack_bottom: u8;
    static _diag_start_: u8;
    static _diag_end_: u8;
}

#[used]
static mut BSS_PROBE: [u32; 4] = [0; 4];

// The accessor is named `addr_<symbol>` rather than reusing `$name` itself:
// the extern `static`s above already occupy `$name` in the value namespace,
// so a same-named `fn` would be a duplicate-definition error (E0428).
macro_rules! addr_of_sym {
    ($name:ident, $accessor:ident) => {
        pub fn $accessor() -> u32 {
            core::ptr::addr_of!($name) as u32
        }
    };
}
addr_of_sym!(_sdata, addr_sdata);
addr_of_sym!(_edata, addr_edata);
addr_of_sym!(_sbss, addr_sbss);
addr_of_sym!(_ebss, addr_ebss);
addr_of_sym!(_etext, addr_etext);
addr_of_sym!(_icache_data_end_, addr_icache_data_end);
addr_of_sym!(_ictag_start_, addr_ictag_start);
addr_of_sym!(_ictag_end_, addr_ictag_end);
addr_of_sym!(_ramcode_start_, addr_ramcode_start);
addr_of_sym!(_ramcode_end_, addr_ramcode_end);
addr_of_sym!(_rf_dma_start_, addr_rf_dma_start);
addr_of_sym!(_rf_dma_end_, addr_rf_dma_end);
addr_of_sym!(_svc_stack_top, addr_svc_stack_top);
addr_of_sym!(_svc_stack_bottom, addr_svc_stack_bottom);
addr_of_sym!(_irq_stack_top, addr_irq_stack_top);
addr_of_sym!(_irq_stack_bottom, addr_irq_stack_bottom);
addr_of_sym!(_diag_start_, addr_diag_start);
addr_of_sym!(_diag_end_, addr_diag_end);

/// Spot-check that `.data` was actually copied from flash: the cache canary
/// word (first bytes of `.data`) must already read back as
/// [`crate::diag::CANARY_VALUE`] by the time this runs (after the asm
/// `.data`-copy loop, before `diag::init()`).
pub fn data_init_ok() -> bool {
    let canary =
        unsafe { core::ptr::read_volatile(core::ptr::addr_of_mut!(crate::diag::CACHE_CANARY)) };
    crate::diag::canary_matches(canary)
}

/// Spot-check that `.bss` was actually zeroed: read a handful of words
/// spread across `[_sbss, _ebss)` and confirm they are zero. A full scan
/// would work too but is unnecessary: the zero loop is a few straight-line
/// instructions with no data dependency on `.bss` content, so a sparse
/// sample is just as diagnostic as a full one and costs less at every boot.
pub fn bss_zero_ok() -> bool {
    for i in 0..4 {
        let word = unsafe {
            core::ptr::read_volatile(core::ptr::addr_of_mut!(BSS_PROBE).cast::<u32>().add(i))
        };
        if word != 0 {
            return false;
        }
    }

    let start = addr_sbss();
    let end = addr_ebss();
    if end <= start {
        return true; // empty .bss is trivially zeroed
    }
    let len = end - start;
    let samples = [0u32, len / 4, len / 2, (len * 3) / 4, len.saturating_sub(4)];
    for off in samples {
        let off = off & !0x3; // word-align
        if off >= len {
            continue;
        }
        let word = unsafe { core::ptr::read_volatile((start + off) as *const u32) };
        if word != 0 {
            return false;
        }
    }
    true
}
