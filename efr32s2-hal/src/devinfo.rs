//! Factory device-information page (`DEVINFO`, identical layout on xG21 and
//! xG22: GSDK `efr32mg2{1,2}_devinfo.h`).

const DEVINFO_BASE: u32 = 0x0FE0_8000;
const EUI64L: u32 = DEVINFO_BASE + 0x048;
const EUI64H: u32 = DEVINFO_BASE + 0x04C;

/// Factory EUI-64 in IEEE 802.15.4 over-the-air (little-endian) byte order:
/// `EUI64L` bytes first, then `EUI64H`.
pub fn eui64() -> [u8; 8] {
    // SAFETY: DEVINFO is an always-readable, read-only flash page.
    let (lo, hi) = unsafe {
        (
            core::ptr::read_volatile(EUI64L as *const u32),
            core::ptr::read_volatile(EUI64H as *const u32),
        )
    };
    eui64_from_words(lo, hi)
}

/// Silicon Labs manufacturing token `MFG_CTUNE` in the user-data page
/// (GSDK `USERDATA_BASE + 0x100`, `sl_device_init_hfxo_s2.c`).
const MFG_CTUNE: u32 = 0x0FE0_0100;

/// Board HFXO tuning stored by the manufacturer in the `MFG_CTUNE` token,
/// if programmed.
///
/// GSDK precedence (`sl_device_init_hfxo`): module DEVINFO `MODXOCAL`
/// (modules only), then this token, then the board default. Products built
/// on a bare chip (e.g. the Home Assistant Connect ZBT-1) use this token
/// with a board fallback; an erased token (`0xFFFF`) or a value that does
/// not fit the 8-bit `CTUNEXIANA` field yields `None`.
pub fn mfg_ctune() -> Option<u8> {
    // SAFETY: the user-data page is always-readable memory-mapped flash.
    let raw = unsafe { core::ptr::read_volatile(MFG_CTUNE as *const u16) };
    ctune_from_token(raw)
}

pub(crate) const fn ctune_from_token(raw: u16) -> Option<u8> {
    if raw <= u8::MAX as u16 {
        Some(raw as u8)
    } else {
        None
    }
}

pub(crate) const fn eui64_from_words(lo: u32, hi: u32) -> [u8; 8] {
    let l = lo.to_le_bytes();
    let h = hi.to_le_bytes();
    [l[0], l[1], l[2], l[3], h[0], h[1], h[2], h[3]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mfg_ctune_rejects_erased_and_out_of_range_tokens() {
        assert_eq!(ctune_from_token(0xFFFF), None);
        assert_eq!(ctune_from_token(0x0100), None);
        assert_eq!(ctune_from_token(133), Some(133));
        assert_eq!(ctune_from_token(0), Some(0));
    }

    #[test]
    fn eui64_is_little_endian_low_word_first() {
        // EUI64H:EUI64L = 0x588E81FF:FE6F2C29 (illustrative value).
        assert_eq!(
            eui64_from_words(0xFE6F_2C29, 0x588E_81FF),
            [0x29, 0x2C, 0x6F, 0xFE, 0xFF, 0x81, 0x8E, 0x58]
        );
    }
}
