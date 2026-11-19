//! The CCFG linked at the end of flash (`.ccfg`, see `link/memory-*.x`):
//! the board's physical options, with the serial ROM bootloader and its
//! backdoor kept enabled (compile-time gate). The ROM bootloader is the
//! NCP's firmware-update path.

use cc26x2_hal::ccfg::{Ccfg, WORDS};

pub const PRODUCT_CCFG: Ccfg = cc26x2_dongle::CCFG;

const _: () = match PRODUCT_CCFG.check_recoverable() {
    Ok(()) => {}
    Err(reason) => panic!("{}", reason),
};

/// The 88 bytes the boot ROM reads. Referenced by the application so the
/// linker keeps it.
#[cfg_attr(target_os = "none", unsafe(link_section = ".ccfg"))]
#[used]
#[unsafe(no_mangle)]
pub static CCFG: [u32; WORDS] = PRODUCT_CCFG.words();
