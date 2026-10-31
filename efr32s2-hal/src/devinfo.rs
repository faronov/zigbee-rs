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

pub(crate) const fn eui64_from_words(lo: u32, hi: u32) -> [u8; 8] {
    let l = lo.to_le_bytes();
    let h = hi.to_le_bytes();
    [l[0], l[1], l[2], l[3], h[0], h[1], h[2], h[3]]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn eui64_is_little_endian_low_word_first() {
        // EUI64H:EUI64L = 0x588E81FF:FE6F2C29 (illustrative value).
        assert_eq!(
            eui64_from_words(0xFE6F_2C29, 0x588E_81FF),
            [0x29, 0x2C, 0x6F, 0xFE, 0xFF, 0x81, 0x8E, 0x58]
        );
    }
}
