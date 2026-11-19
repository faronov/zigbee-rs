//! AON real-time clock (`inc/hw_aon_rtc.h`, driverlib `aon_rtc.[ch]`).
//!
//! The RTC counts seconds (`SEC`) and 2^-32 s fractions (`SUBSEC`) from the
//! 32 kHz SCLK_LF and keeps running in standby. Channel 0 compares against
//! `SEC[15:0]:SUBSEC[31:16]` and raises the combined `AON_RTC_COMB`
//! interrupt that [`crate::setup::trim_device`] routes (CH0..CH2). The time
//! driver uses 32,768 Hz ticks: `ticks = SEC << 15 | SUBSEC >> 17`.

use crate::mmio::{self, AON_RTC_BASE};

const CTL: u32 = AON_RTC_BASE;
const EVFLAGS: u32 = AON_RTC_BASE + 0x04;
const SEC: u32 = AON_RTC_BASE + 0x08;
const SUBSEC: u32 = AON_RTC_BASE + 0x0C;
const CHCTL: u32 = AON_RTC_BASE + 0x14;
const CH0CMP: u32 = AON_RTC_BASE + 0x18;

/// Tick rate of [`now_ticks`].
pub const TICK_HZ: u32 = 32_768;

/// Minimum lead of a compare over the current time: AON writes take a few
/// SCLK_LF periods to land, and a compare set into the past never fires.
pub const MIN_LEAD_TICKS: u64 = 4;

/// Ownership token for the RTC channel used by the time driver.
pub struct RtcToken(pub(crate) ());

/// 64-bit tick count (32,768 Hz) with a consistent SEC/SUBSEC pair.
pub fn now_ticks() -> u64 {
    loop {
        let sec = mmio::read(SEC);
        let subsec = mmio::read(SUBSEC);
        if mmio::read(SEC) == sec {
            return to_ticks(sec, subsec);
        }
    }
}

pub const fn to_ticks(sec: u32, subsec: u32) -> u64 {
    ((sec as u64) << 15) | (subsec >> 17) as u64
}

/// `CHxCMP` value for tick `at` (16 bits of seconds, 16 bits of fraction).
pub const fn compare_value(at: u64) -> u32 {
    (((at >> 15) as u32 & 0xFFFF) << 16) | (((at & 0x7FFF) as u32) << 1)
}

/// Arm channel 0 for tick `at` (no earlier than `now + MIN_LEAD_TICKS`; at
/// most 65,535 s ahead, the compare range — the caller re-arms).
pub fn arm_ch0(at: u64) {
    let now = now_ticks();
    let at = at.max(now + MIN_LEAD_TICKS).min(now + (0xFFFFu64 << 15));
    mmio::modify(CHCTL, 1, 0);
    mmio::write(EVFLAGS, 1);
    mmio::write(CH0CMP, compare_value(at));
    mmio::modify(CHCTL, 0, 1);
    mmio::aon_sync();
}

pub fn disarm_ch0() {
    mmio::modify(CHCTL, 1, 0);
    mmio::write(EVFLAGS, 1);
    mmio::aon_sync();
}

/// Clear and return the channel 0 event flag.
pub fn take_ch0_event() -> bool {
    let set = mmio::read(EVFLAGS) & 1 != 0;
    if set {
        mmio::write(EVFLAGS, 1);
        mmio::aon_sync();
    }
    set
}

/// The RTC is enabled (`CTL.EN`, set by [`crate::setup::trim_device`]).
pub fn running() -> bool {
    mmio::read(CTL) & 1 != 0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ticks_and_compare_round_trip() {
        // 3.5 s.
        let t = to_ticks(3, 0x8000_0000);
        assert_eq!(t, 3 * 32_768 + 16_384);
        assert_eq!(compare_value(t), (3 << 16) | 0x8000);
        // Compare keeps only 16 bits of seconds.
        assert_eq!(
            compare_value((70_000u64 << 15) + 1),
            ((70_000 & 0xFFFF) << 16) | 2
        );
    }
}
