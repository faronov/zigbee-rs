//! The CCFG linked at the end of flash (`.ccfg`, see `link/memory-*.x`).
//!
//! The product ships the board's physical options unchanged
//! (`cc26x2_dongle::CCFG`). The compile-time gate keeps the serial ROM
//! bootloader and its DIO15 backdoor enabled: on a dongle without a debug
//! header this is the only recovery path, so an image that would disable it
//! does not build.

use cc26x2_hal::ccfg::{Ccfg, WORDS};

pub const PRODUCT_CCFG: Ccfg = cc26x2_dongle::CCFG;

const _: () = match PRODUCT_CCFG.check_recoverable() {
    Ok(()) => {}
    Err(reason) => panic!("{}", reason),
};

/// The 88 bytes the boot ROM reads (`CCFG_BASE` alias of the last flash
/// bytes). Referenced by the application so the linker keeps it.
#[cfg_attr(target_os = "none", unsafe(link_section = ".ccfg"))]
#[used]
#[unsafe(no_mangle)]
pub static CCFG: [u32; WORDS] = PRODUCT_CCFG.words();
