//! Factory device-information page (`DEVINFO`) and the manufacturing
//! tokens of the user-data page (`USERDATA`). The offsets used here are the
//! same on xG21, xG22 and xG24 (GSDK 4.5.0 `efr32mg2{1,2,4}_devinfo.h`,
//! `offsetof` dumps; `sl_token_manufacturing_series_2.h`).

const DEVINFO_BASE: u32 = 0x0FE0_8000;
const EUI64L: u32 = DEVINFO_BASE + 0x048;
const EUI64H: u32 = DEVINFO_BASE + 0x04C;
/// `DEVINFO.MODULEINFO` (Silicon Labs modules only; erased on bare chips).
const MODULEINFO: u32 = DEVINFO_BASE + 0x14C;
/// `DEVINFO.MODXOCAL`: the module's HFXO tuning.
const MODXOCAL: u32 = DEVINFO_BASE + 0x150;
/// `MODULEINFO.HFXOCALVAL`: 0 = `MODXOCAL` is valid.
const MODULEINFO_HFXOCALVAL: u32 = 1 << 19;

/// User-data page (`USERDATA_BASE`), where the manufacturing tokens live.
const USERDATA_BASE: u32 = 0x0FE0_0000;
/// `MFG_CUSTOM_EUI_64` (`USERDATA_TOKENS | 0x002`, 8 bytes).
const MFG_CUSTOM_EUI_64: u32 = USERDATA_BASE + 0x002;
/// Silicon Labs manufacturing token `MFG_CTUNE` (GSDK `USERDATA_BASE +
/// 0x100`, `sl_device_init_hfxo_s2.c`).
const MFG_CTUNE: u32 = USERDATA_BASE + 0x100;

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

/// The `MFG_CUSTOM_EUI_64` manufacturing token, if programmed (not all
/// `0xFF`), in the same over-the-air byte order as [`eui64`] (GSDK copies
/// the 8 token bytes verbatim into the `EmberEUI64`).
pub fn mfg_custom_eui64() -> Option<[u8; 8]> {
    let mut token = [0u8; 8];
    for (index, byte) in token.iter_mut().enumerate() {
        // SAFETY: the user-data page is always-readable memory-mapped flash.
        *byte =
            unsafe { core::ptr::read_volatile((MFG_CUSTOM_EUI_64 + index as u32) as *const u8) };
    }
    custom_eui64_from_token(token)
}

/// Where the IEEE address came from ([`ieee_address`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EuiSource {
    /// `MFG_CUSTOM_EUI_64` token in the user-data page.
    CustomToken,
    /// Factory `DEVINFO.EUI64`.
    Devinfo,
}

/// The node's IEEE address with GSDK's precedence (EmberZNet's
/// `TOKEN_MFG_EUI_64`, `sl_token_manufacturing.c`): the `MFG_CUSTOM_EUI_64`
/// token when it is not all `0xFF`, else the factory DEVINFO EUI-64. A
/// stick that leaves the factory with a custom EUI keeps the coordinator
/// IEEE of its stock EmberZNet firmware.
pub fn ieee_address() -> ([u8; 8], EuiSource) {
    match mfg_custom_eui64() {
        Some(custom) => (custom, EuiSource::CustomToken),
        None => (eui64(), EuiSource::Devinfo),
    }
}

/// Board HFXO tuning stored by the manufacturer in the `MFG_CTUNE` token,
/// if programmed. An erased token (`0xFFFF`) or a value that does not fit
/// the 8-bit `CTUNEXIANA` field yields `None` (GSDK would truncate it).
pub fn mfg_ctune() -> Option<u8> {
    // SAFETY: the user-data page is always-readable memory-mapped flash.
    let raw = unsafe { core::ptr::read_volatile(MFG_CTUNE as *const u16) };
    ctune_from_token(raw)
}

/// The module calibration `DEVINFO.MODXOCAL.HFXOCTUNEXIANA`, when
/// `MODULEINFO.HFXOCALVAL` marks it valid (Silicon Labs modules; erased on
/// bare chips such as the BRD4187A and EK2703A dies).
pub fn module_ctune() -> Option<u8> {
    // SAFETY: DEVINFO is an always-readable, read-only flash page.
    let (info, cal) = unsafe {
        (
            core::ptr::read_volatile(MODULEINFO as *const u32),
            core::ptr::read_volatile(MODXOCAL as *const u32),
        )
    };
    module_ctune_from_words(info, cal)
}

/// Where the HFXO CTUNE came from ([`hfxo_ctune`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CtuneSource {
    /// `DEVINFO.MODXOCAL` (module calibration).
    Module,
    /// `MFG_CTUNE` token in the user-data page.
    MfgToken,
    /// The board crate's default.
    BoardDefault,
}

/// HFXO CTUNE (`CTUNEXIANA`) with GSDK's precedence
/// (`sl_device_init_hfxo_s2.c`): the module calibration, then the
/// `MFG_CTUNE` token, then `board_default`.
pub fn hfxo_ctune(board_default: u16) -> (u16, CtuneSource) {
    select_ctune(module_ctune(), mfg_ctune(), board_default)
}

pub(crate) const fn select_ctune(
    module: Option<u8>,
    token: Option<u8>,
    board_default: u16,
) -> (u16, CtuneSource) {
    match (module, token) {
        (Some(value), _) => (value as u16, CtuneSource::Module),
        (None, Some(value)) => (value as u16, CtuneSource::MfgToken),
        (None, None) => (board_default, CtuneSource::BoardDefault),
    }
}

pub(crate) const fn module_ctune_from_words(moduleinfo: u32, modxocal: u32) -> Option<u8> {
    if moduleinfo & MODULEINFO_HFXOCALVAL == 0 {
        Some(modxocal as u8)
    } else {
        None
    }
}

pub(crate) const fn ctune_from_token(raw: u16) -> Option<u8> {
    if raw <= u8::MAX as u16 {
        Some(raw as u8)
    } else {
        None
    }
}

pub(crate) const fn custom_eui64_from_token(token: [u8; 8]) -> Option<[u8; 8]> {
    let mut index = 0;
    while index < 8 {
        if token[index] != 0xFF {
            return Some(token);
        }
        index += 1;
    }
    None
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
    fn ctune_precedence_follows_gsdk() {
        // Erased DEVINFO module fields (the BRD4187A and EK2703A dies:
        // MODULEINFO and MODXOCAL read 0xFFFFFFFF) are not a calibration.
        assert_eq!(module_ctune_from_words(0xFFFF_FFFF, 0xFFFF_FFFF), None);
        assert_eq!(
            module_ctune_from_words(0xFFF7_FFFF, 0x007F_7A78),
            Some(0x78)
        );
        assert_eq!(
            select_ctune(Some(0x78), Some(133), 100),
            (0x78, CtuneSource::Module)
        );
        assert_eq!(
            select_ctune(None, Some(133), 100),
            (133, CtuneSource::MfgToken)
        );
        assert_eq!(
            select_ctune(None, None, 100),
            (100, CtuneSource::BoardDefault)
        );
    }

    #[test]
    fn custom_eui_token_wins_unless_erased() {
        assert_eq!(custom_eui64_from_token([0xFF; 8]), None);
        let custom = [0x03, 0x4D, 0x5D, 0xFE, 0xFF, 0xE8, 0xE8, 0xB0];
        assert_eq!(custom_eui64_from_token(custom), Some(custom));
        // Partially programmed is still programmed (GSDK compares all 8).
        let mut partial = [0xFF; 8];
        partial[7] = 0x00;
        assert_eq!(custom_eui64_from_token(partial), Some(partial));
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
