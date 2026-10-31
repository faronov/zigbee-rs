//! RTCC on the 32.768 kHz LFRCO: the EM2-capable time base.
//!
//! TIMER0 (see [`crate::delay`]) stops in EM2, so anything that must keep
//! time across deep sleep (the Embassy time driver, sleep deadlines) runs on
//! the RTCC. The register layout is identical on xG21 and xG22 (GSDK 4.5.0
//! `efr32mg2{1,2}_rtcc.h`, base `0x4800_0000`); the clock source is selected
//! in `CMU_RTCCCLKCTRL` (offset `0x240`, `LFRCO = 1`) on both dies.
//!
//! Channel ownership: CC0 belongs to the Embassy time driver
//! (`time-driver` feature), CC1 to blocking EM2 sleeps in
//! [`crate::power`]. The overflow flag extends the 32-bit counter to 64
//! bits (one wrap every ~36.4 h).
//!
//! The LFRCO is the uncalibrated RC oscillator (datasheet: ±3 % over
//! temperature in normal mode). That is ample for SED poll scheduling; a
//! product that needs wall-clock accuracy should add an LFXO source.

use core::sync::atomic::{AtomicU32, Ordering};

use crate::chip::{BusClock, base};

/// RTCC tick rate (LFRCO, prescaler DIV1).
pub const RTCC_HZ: u32 = 32_768;

const CMU_LOCK: u32 = base::CMU + 0x010;
const CMU_LOCK_UNLOCK: u32 = 0x93F7;
const CMU_RTCCCLKCTRL: u32 = base::CMU + 0x240;
const CMU_RTCCCLKCTRL_LFRCO: u32 = 1;

const RTCC_EN: u32 = base::RTCC + 0x04;
const RTCC_CFG: u32 = base::RTCC + 0x08;
const RTCC_CMD: u32 = base::RTCC + 0x0C;
const RTCC_STATUS: u32 = base::RTCC + 0x10;
const RTCC_IF: u32 = base::RTCC + 0x14;
const RTCC_IEN: u32 = base::RTCC + 0x18;
const RTCC_CNT: u32 = base::RTCC + 0x20;
const RTCC_SYNCBUSY: u32 = base::RTCC + 0x28;
const RTCC_LOCK: u32 = base::RTCC + 0x2C;
/// `CC[n]` is `{CTRL, OCVALUE, ICVALUE}` at `0x30 + 12 n`.
const fn rtcc_cc_ctrl(channel: usize) -> u32 {
    base::RTCC + 0x30 + 12 * channel as u32
}
const fn rtcc_cc_ocvalue(channel: usize) -> u32 {
    rtcc_cc_ctrl(channel) + 4
}

const SET: u32 = 0x1000;
const CLR: u32 = 0x2000;

const RTCC_CMD_START: u32 = 1 << 0;
const RTCC_CMD_STOP: u32 = 1 << 1;
const RTCC_STATUS_RUNNING: u32 = 1 << 0;
const RTCC_LOCK_UNLOCK: u32 = 0xAEE8;
const RTCC_CC_CTRL_MODE_OUTPUTCOMPARE: u32 = 2;

/// `RTCC_IF` overflow flag.
pub const IF_OF: u32 = 1 << 0;
/// `RTCC_IF` channel-0 compare flag (Embassy time driver).
pub const IF_CC0: u32 = 1 << 4;
/// `RTCC_IF` channel-1 compare flag (blocking EM2 sleep).
pub const IF_CC1: u32 = 1 << 6;
const IF_OWNED: u32 = IF_OF | IF_CC0 | IF_CC1;

const START_TIMEOUT_ITERATIONS: u32 = 1_000_000;

/// Minimum distance between `now` and an armed compare, in ticks.
///
/// GSDK 4.5.0 `sl_sleeptimer_hal_rtcc.c` `SLEEPTIMER_COMPARE_MIN_DIFF`
/// (`2 + 1`): the `OCVALUE` write crosses into the LF clock domain, and a
/// compare closer than this can be passed before it takes effect.
pub const MIN_ARM_TICKS: u32 = 3;
/// Longest single compare hop; further deadlines are re-armed on wake.
pub const MAX_ARM_TICKS: u32 = u32::MAX / 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtccError {
    /// The RTCC did not report RUNNING (LFRCO not starting).
    StartTimeout,
}

/// Compare channel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Channel {
    /// Reserved for the Embassy time driver.
    Cc0 = 0,
    /// Reserved for [`crate::power`] blocking sleeps.
    Cc1 = 1,
}

impl Channel {
    const fn flag(self) -> u32 {
        match self {
            Channel::Cc0 => IF_CC0,
            Channel::Cc1 => IF_CC1,
        }
    }
}

/// Snapshot of the RTCC flags this crate owns.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RtccFlags {
    pub overflow: bool,
    pub cc0: bool,
    pub cc1: bool,
}

/// Decode a raw `RTCC_IF` value.
pub const fn decode_flags(raw: u32) -> RtccFlags {
    RtccFlags {
        overflow: raw & IF_OF != 0,
        cc0: raw & IF_CC0 != 0,
        cc1: raw & IF_CC1 != 0,
    }
}

/// `OCVALUE` for a compare that should fire when the counter reaches
/// `deadline`: GSDK programs `value - 1` (`sleeptimer_hal_set_compare`).
pub const fn compare_value(deadline: u32) -> u32 {
    deadline.wrapping_sub(1)
}

/// True once the wrapping counter `now` has reached `deadline`
/// (deadlines are always armed less than half the range ahead).
pub const fn reached(now: u32, deadline: u32) -> bool {
    now.wrapping_sub(deadline) < 0x8000_0000
}

/// How far ahead a single compare must be armed for a 64-bit `deadline`:
/// `None` if it already passed, otherwise clamped to
/// [`MIN_ARM_TICKS`]..=[`MAX_ARM_TICKS`].
pub const fn ticks_until(now: u64, deadline: u64) -> Option<u32> {
    if deadline <= now {
        return None;
    }
    let remaining = deadline - now;
    Some(if remaining > MAX_ARM_TICKS as u64 {
        MAX_ARM_TICKS
    } else if remaining < MIN_ARM_TICKS as u64 {
        MIN_ARM_TICKS
    } else {
        remaining as u32
    })
}

/// Resolve a 64-bit time from two counter samples taken between two reads
/// of the wrap epoch. `None` means the overflow handler ran in between and
/// the sample must be retaken. A wrap is owed when the hardware overflow
/// flag is latched but not yet handled, or the counter went backwards.
pub const fn extend(
    epoch_before: u32,
    epoch_after: u32,
    low_before: u32,
    low_after: u32,
    overflow_pending: bool,
) -> Option<u64> {
    if epoch_before != epoch_after {
        return None;
    }
    let epoch = epoch_before as u64 + (overflow_pending || low_after < low_before) as u64;
    Some((epoch << 32) | low_after as u64)
}

/// Ticks to milliseconds (floor).
pub const fn ticks_to_ms(ticks: u64) -> u64 {
    ticks * 1000 / RTCC_HZ as u64
}

/// Milliseconds to ticks (ceiling, saturating at `u32::MAX`).
pub const fn ms_to_ticks(ms: u32) -> u32 {
    let ticks = (ms as u64 * RTCC_HZ as u64).div_ceil(1000);
    if ticks > u32::MAX as u64 {
        u32::MAX
    } else {
        ticks as u32
    }
}

static WRAPS: AtomicU32 = AtomicU32::new(0);

/// Exclusive RTCC ownership.
pub struct RtccToken {
    _private: (),
}

impl RtccToken {
    pub(crate) const fn new() -> Self {
        Self { _private: () }
    }

    /// Clock the RTCC from the LFRCO and start it at zero.
    ///
    /// Mirrors GSDK `sleeptimer_hal_init_timer` + `RTCC_Init`: compare
    /// channels in output-compare mode with interrupts disabled, all flags
    /// cleared, prescaler DIV1, `DEBUGRUN = 0`.
    pub fn start(self) -> Result<Rtcc, RtccError> {
        crate::clock::enable_bus_clock(BusClock::LFRCO);
        crate::clock::enable_bus_clock(BusClock::RTCC);
        unsafe {
            write(CMU_LOCK, CMU_LOCK_UNLOCK);
            write(CMU_RTCCCLKCTRL, CMU_RTCCCLKCTRL_LFRCO);
            write(RTCC_LOCK, RTCC_LOCK_UNLOCK);
            if read(RTCC_EN) != 0 {
                wait_sync()?;
                write(RTCC_CMD, RTCC_CMD_STOP);
                wait_sync()?;
            }
            write(RTCC_EN + CLR, 1);
            write(RTCC_CFG, 0);
            write(rtcc_cc_ctrl(0), RTCC_CC_CTRL_MODE_OUTPUTCOMPARE);
            write(rtcc_cc_ctrl(1), RTCC_CC_CTRL_MODE_OUTPUTCOMPARE);
            write(RTCC_EN + SET, 1);
            write(RTCC_IEN, 0);
            write(RTCC_IF + CLR, u32::MAX);
            wait_sync()?;
            write(RTCC_CNT, 0);
            wait_sync()?;
            write(RTCC_CMD, RTCC_CMD_START);
            wait_sync()?;
            let mut running = false;
            for _ in 0..START_TIMEOUT_ITERATIONS {
                if read(RTCC_STATUS) & RTCC_STATUS_RUNNING != 0 {
                    running = true;
                    break;
                }
            }
            if !running {
                return Err(RtccError::StartTimeout);
            }
        }
        WRAPS.store(0, Ordering::Relaxed);
        Ok(Rtcc { _token: self })
    }
}

/// Running RTCC. The free functions in this module operate on it.
pub struct Rtcc {
    _token: RtccToken,
}

impl Rtcc {
    pub fn now(&self) -> u32 {
        now()
    }

    pub fn now64(&self) -> u64 {
        now64()
    }
}

unsafe fn wait_sync() -> Result<(), RtccError> {
    for _ in 0..START_TIMEOUT_ITERATIONS {
        if unsafe { read(RTCC_SYNCBUSY) } == 0 {
            return Ok(());
        }
    }
    Err(RtccError::StartTimeout)
}

/// Raw 32-bit counter.
#[inline]
pub fn now() -> u32 {
    unsafe { read(RTCC_CNT) }
}

/// Counter extended to 64 bits with the overflow epoch.
///
/// Requires the overflow interrupt to be serviced by a handler that calls
/// [`take_flags`] and [`note_overflow`] (the `time-driver` RTCC handler
/// does); safe inside critical sections thanks to the pending-flag check.
pub fn now64() -> u64 {
    loop {
        let epoch_before = WRAPS.load(Ordering::Acquire);
        let low_before = now();
        let pending = unsafe { read(RTCC_IF) } & IF_OF != 0;
        let low_after = now();
        let epoch_after = WRAPS.load(Ordering::Acquire);
        if let Some(value) = extend(epoch_before, epoch_after, low_before, low_after, pending) {
            return value;
        }
    }
}

/// Record one handled overflow (call once per `overflow` in [`take_flags`]).
pub fn note_overflow() {
    WRAPS.fetch_add(1, Ordering::AcqRel);
}

/// Arm `channel` to fire `ticks` ahead (clamped to [`MIN_ARM_TICKS`]);
/// returns the absolute deadline.
pub fn arm(channel: Channel, ticks: u32) -> u32 {
    let deadline = now().wrapping_add(ticks.max(MIN_ARM_TICKS));
    unsafe {
        write(rtcc_cc_ocvalue(channel as usize), compare_value(deadline));
        write(RTCC_IF + CLR, channel.flag());
        write(RTCC_IEN + SET, channel.flag());
    }
    deadline
}

/// Disable the compare interrupt of `channel`.
pub fn disarm(channel: Channel) {
    unsafe { write(RTCC_IEN + CLR, channel.flag()) };
}

/// Enable the overflow interrupt (needed for [`now64`] across wraps).
pub fn enable_overflow_interrupt() {
    unsafe { write(RTCC_IEN + SET, IF_OF) };
}

/// Read without clearing.
pub fn peek_flags() -> RtccFlags {
    decode_flags(unsafe { read(RTCC_IF) })
}

/// Read and clear the flags this crate owns.
pub fn take_flags() -> RtccFlags {
    let raw = unsafe { read(RTCC_IF) } & IF_OWNED;
    if raw != 0 {
        unsafe { write(RTCC_IF + CLR, raw) };
    }
    decode_flags(raw)
}

#[inline]
unsafe fn read(address: u32) -> u32 {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::read_volatile(address as *const u32)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    {
        let _ = address;
        0
    }
}

#[inline]
unsafe fn write(address: u32, value: u32) {
    #[cfg(all(target_arch = "arm", target_os = "none"))]
    unsafe {
        core::ptr::write_volatile(address as *mut u32, value)
    }
    #[cfg(not(all(target_arch = "arm", target_os = "none")))]
    let _ = (address, value);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn register_map_matches_gsdk_offsets() {
        // offsetof() from GSDK 4.5.0 efr32mg2{1,2}_rtcc.h / _cmu.h.
        assert_eq!(rtcc_cc_ctrl(0), 0x4800_0030);
        assert_eq!(rtcc_cc_ocvalue(0), 0x4800_0034);
        assert_eq!(rtcc_cc_ctrl(1), 0x4800_003C);
        assert_eq!(rtcc_cc_ocvalue(1), 0x4800_0040);
        assert_eq!(CMU_RTCCCLKCTRL, 0x4000_8240);
        assert_eq!(RTCC_SYNCBUSY, 0x4800_0028);
    }

    #[test]
    fn flags_decode() {
        assert_eq!(
            decode_flags(IF_OF | IF_CC1 | (1 << 8)),
            RtccFlags {
                overflow: true,
                cc0: false,
                cc1: true
            }
        );
    }

    #[test]
    fn compare_value_and_reached_wrap() {
        assert_eq!(compare_value(0), u32::MAX);
        assert!(reached(5, 5));
        assert!(reached(2, u32::MAX - 1));
        assert!(!reached(u32::MAX - 1, 2));
    }

    #[test]
    fn ticks_until_clamps() {
        assert_eq!(ticks_until(10, 10), None);
        assert_eq!(ticks_until(10, 11), Some(MIN_ARM_TICKS));
        assert_eq!(ticks_until(0, 1000), Some(1000));
        assert_eq!(ticks_until(0, u64::MAX), Some(MAX_ARM_TICKS));
    }

    #[test]
    fn extend_handles_pending_and_torn_samples() {
        assert_eq!(extend(1, 2, 0, 0, false), None);
        assert_eq!(extend(0, 0, 5, 6, false), Some(6));
        assert_eq!(extend(0, 0, u32::MAX, 1, false), Some((1 << 32) | 1));
        assert_eq!(extend(3, 3, 7, 8, true), Some((4 << 32) | 8));
    }

    #[test]
    fn ms_tick_conversion() {
        assert_eq!(ms_to_ticks(1000), RTCC_HZ);
        assert_eq!(ms_to_ticks(1), 33);
        assert_eq!(ticks_to_ms(RTCC_HZ as u64 * 7), 7000);
        assert_eq!(ms_to_ticks(u32::MAX), u32::MAX);
    }
}
