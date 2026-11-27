//! LDMA (linked DMA) controller of EFR32 Series 2 (xG21, xG22, xG24).
//!
//! Register map and descriptor formats follow GSDK 4.5.0
//! `efr32mg2{1,2,4}_ldma.h`, `efr32mg2{1,2,4}_ldmaxbar.h` and `em_ldma.h`
//! (checked with `offsetof()` dumps of the three headers). They are
//! identical on the three dies, except the LDMAXBAR request-select register,
//! which is at `+0x0` on xG21/xG22 and `+0x4` on xG24 (an `IPVERSION`
//! register precedes it). xG22 and xG24 gate the LDMA and LDMAXBAR bus clocks
//! (`CMU_CLKEN0` bits 0 and 1); xG21 has no gates.
//!
//! Ownership: [`Ldma`] is the whole controller: 8 channels and the 8
//! `SYNCTRIG` bits that SYNC descriptors and software share. Its only user
//! is the host UART receiver ([`crate::usart::host`], feature
//! `usart0-host`), which takes channel [`HOST_RX_CHANNEL`] and `SYNCTRIG`
//! bits 0..=3. Another user would have to split the token first.

// The register map's only user is the host UART receiver.
#![cfg_attr(not(feature = "usart0-host"), allow(dead_code))]

/// Exclusive ownership of the LDMA controller and the LDMAXBAR.
///
/// Created once by [`crate::peripherals::Peripherals::take`].
pub struct Ldma {
    _private: (),
}

impl Ldma {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }
}

/// Secure-alias bases (`LDMA_S_BASE`, `LDMAXBAR_S_BASE`, all three dies).
pub(crate) const LDMA: u32 = 0x4004_0000;
pub(crate) const LDMAXBAR: u32 = 0x4004_4000;

pub(crate) const EN: u32 = 0x004;
pub(crate) const SYNCSWSET: u32 = 0x010;
pub(crate) const SYNCSWCLR: u32 = 0x014;
pub(crate) const SYNCHWEN: u32 = 0x018;
pub(crate) const CHDIS: u32 = 0x028;
pub(crate) const CHBUSY: u32 = 0x030;
pub(crate) const CHDONE: u32 = 0x034;
pub(crate) const DBGHALT: u32 = 0x038;
pub(crate) const REQDIS: u32 = 0x040;
pub(crate) const LINKLOAD: u32 = 0x048;
pub(crate) const IF: u32 = 0x050;
pub(crate) const IEN: u32 = 0x054;
/// Clear alias of the Series 2 register blocks.
pub(crate) const CLR: u32 = 0x2000;

/// `CH[n]` register group: `0x05C + 0x30 * n`.
pub(crate) const fn ch(n: u32) -> u32 {
    0x05C + 0x30 * n
}
pub(crate) const CH_CFG: u32 = 0x00;
pub(crate) const CH_LOOP: u32 = 0x04;
pub(crate) const CH_DST: u32 = 0x10;
pub(crate) const CH_LINK: u32 = 0x14;

/// `LDMAXBAR CH[n].REQSEL`.
#[cfg(not(feature = "efr32mg24"))]
pub(crate) const fn xbar_reqsel(n: u32) -> u32 {
    4 * n
}
/// `LDMAXBAR CH[n].REQSEL` (after `IPVERSION` on xG24).
#[cfg(feature = "efr32mg24")]
pub(crate) const fn xbar_reqsel(n: u32) -> u32 {
    0x4 + 4 * n
}

/// `LDMAXBAR_CH_REQSEL`: USART0 (`SOURCESEL` 4) `RXDATAV` (`SIGSEL` 0),
/// the same on the three dies.
pub(crate) const REQSEL_USART0_RXDATAV: u32 = 4 << 16;

/// The channel of the host UART receiver.
pub const HOST_RX_CHANNEL: u32 = 0;

const CTRL_STRUCTTYPE_SYNC: u32 = 1;
const CTRL_STRUCTREQ: u32 = 1 << 3;
const CTRL_XFERCNT_SHIFT: u32 = 4;
/// `XFERCNT` is 11 bits: at most 2048 transfers per descriptor.
pub(crate) const MAX_XFER: usize = 2048;
const CTRL_SRCINC_NONE: u32 = 3 << 24;
const LINK_LINK: u32 = 1 << 1;

/// One descriptor in RAM: `CTRL`, `SRC` (SYNC: set/clear), `DST` (SYNC:
/// match), `LINK`.
pub(crate) type Descriptor = [u32; 4];

/// Peripheral-to-memory byte transfer (GSDK `LDMA_DESCRIPTOR_LINKABS_P2M_BYTE`
/// without the done interrupt): `count` bytes from the register `src` to
/// `dst..`, one byte per request (`BLOCKSIZE` UNIT1, `REQMODE` BLOCK), then
/// load the descriptor at `link` (absolute).
pub(crate) const fn p2m_bytes(src: u32, dst: u32, count: usize, link: u32) -> Descriptor {
    assert!(count >= 1 && count <= MAX_XFER);
    [
        ((count as u32 - 1) << CTRL_XFERCNT_SHIFT) | CTRL_SRCINC_NONE,
        src,
        dst,
        (link & !3) | LINK_LINK,
    ]
}

/// SYNC descriptor (GSDK `LDMA_DESCRIPTOR_LINKABS_SYNC`): set `set` and
/// clear `clear` in `SYNCTRIG`, wait until `SYNCTRIG & match_en ==
/// match_val`, then load the descriptor at `link`.
pub(crate) const fn sync(set: u8, clear: u8, match_val: u8, match_en: u8, link: u32) -> Descriptor {
    [
        CTRL_STRUCTTYPE_SYNC | CTRL_STRUCTREQ,
        set as u32 | (clear as u32) << 8,
        match_val as u32 | (match_en as u32) << 8,
        (link & !3) | LINK_LINK,
    ]
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
#[inline]
pub(crate) unsafe fn read(offset: u32) -> u32 {
    unsafe { core::ptr::read_volatile((LDMA + offset) as *const u32) }
}

#[cfg(all(target_arch = "arm", target_os = "none"))]
#[inline]
pub(crate) unsafe fn write(offset: u32, value: u32) {
    unsafe { core::ptr::write_volatile((LDMA + offset) as *mut u32, value) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_offsets_match_gsdk() {
        // offsetof(LDMA_TypeDef, ...) in efr32mg2{1,2,4}_ldma.h.
        assert_eq!((EN, SYNCSWSET, SYNCSWCLR, SYNCHWEN), (4, 0x10, 0x14, 0x18));
        assert_eq!((CHDIS, CHBUSY, CHDONE), (0x28, 0x30, 0x34));
        assert_eq!(
            (DBGHALT, REQDIS, LINKLOAD, IF, IEN),
            (0x38, 0x40, 0x48, 0x50, 0x54)
        );
        assert_eq!((ch(0), ch(1) - ch(0)), (0x5C, 0x30));
        assert_eq!(ch(0) + CH_DST, 0x6C);
        assert_eq!(ch(0) + CH_LINK, 0x70);
        assert_eq!((CH_CFG, CH_LOOP), (0, 4));
        assert_eq!(IF + CLR, 0x2050);
    }

    #[test]
    fn xbar_request_select_offset_per_die() {
        #[cfg(feature = "efr32mg24")]
        assert_eq!(xbar_reqsel(0), 0x4);
        #[cfg(not(feature = "efr32mg24"))]
        assert_eq!(xbar_reqsel(0), 0x0);
        assert_eq!(xbar_reqsel(1) - xbar_reqsel(0), 4);
        // _LDMAXBAR_CH_REQSEL_SOURCESEL_USART0 = 4, SIGSEL_USART0RXDATAV = 0.
        assert_eq!(REQSEL_USART0_RXDATAV, 0x0004_0000);
    }

    #[test]
    fn descriptors_match_the_gsdk_bitfields() {
        // xferCnt 2047 at bit 4, srcInc NONE (3) at bit 24, byte size and
        // dstInc ONE (0), blockSize UNIT1 (0), reqMode BLOCK (0).
        let d = p2m_bytes(0x4005_C024, 0x2000_1000, 2048, 0x2000_0010);
        assert_eq!(d, [0x0300_7FF0, 0x4005_C024, 0x2000_1000, 0x2000_0012]);
        // structType SYNC (1), structReq; syncSet/syncClr in word 1;
        // matchVal [7:0], matchEn [15:8] in word 2 (no CTRL.EXTEND on these
        // dies).
        let s = sync(0, 0x08, 0x01, 0x01, 0x2000_0020);
        assert_eq!(s, [0x9, 0x0800, 0x0101, 0x2000_0022]);
    }
}
