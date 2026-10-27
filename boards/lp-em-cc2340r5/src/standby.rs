//! Device standby between radio polls.
//!
//! Mirrors TI's FreeRTOS standby policy for CC23X0
//! (`kernel/freertos/dpl/PowerCC23X0_freertos.c`, `PowerCC23X0_standbyPolicy()`)
//! together with `Power_sleep()` and `PowerCC23X0_enterStandby()`
//! (`source/ti/drivers/power/PowerCC23X0.c`):
//!
//! 1. find the soonest armed SYSTIM channel (our Embassy alarm is channel 0)
//!    and the caller's deadline, and refuse if it is closer than
//!    `PowerCC23X0_TOTALTIMESTANDBY`;
//! 2. stash the SYSTIM channel compares/arm/mask state (lost in standby);
//! 3. route `CPUIRQ16` to `AON_RTC_COMB` and program RTC channel 0 for the
//!    wake-up, `PowerCC23X0_WAKEDELAYSTANDBY` early;
//! 4. `CKMD.LDOCTL = 0` and `HapiEnterStandby(NULL)` from privileged thread
//!    mode on the MSP (the ROM saves/restores the CPU state);
//! 5. on wake: disarm/clear the RTC event, route `CPUIRQ16` back to SYSTIM,
//!    wait until SYSTIM has resynchronised with the RTC (`STATUS == RUN`) and
//!    restore the channel state; anything that expired meanwhile fires as
//!    soon as interrupts are re-enabled.
//!
//! SYSTIM `TIME1U` therefore stays a continuous microsecond time base across
//! standby (hardware resync from the LF-clocked RTC), so the Embassy time
//! driver needs no correction. Button edges wake the device through the IOC
//! (`WUENSB` is set by [`crate::pins`]); the latched GPIO interrupt is serviced
//! after the critical section.
//!
//! The caller must have powered the radio down (LRFD registers are lost) and
//! re-initialise it afterwards. HFXT is stopped by the radio sleep; the
//! hardware keeps it off in standby and the radio wake restarts it, as
//! `Power_sleep()` does after `PowerCC23X0_enterStandby()`.
//!
//! Debug trade-off: while a debugger holds the debug power domain up
//! (`CDBGPWRUPREQ`, e.g. during an RTT session) the ROM performs a "fake"
//! standby — functionally identical, but without the current saving. Real
//! standby requires the probe to release debug power (see the HIL notes).

/// `PowerCC23X0_TOTALTIMESTANDBY` (`PowerCC23X0.h`): minimum time to the next
/// event for standby to make sense, µs.
pub const TOTAL_TIME_STANDBY_US: u32 = 500;
/// `PowerCC23X0_WAKEDELAYSTANDBY` (`PowerCC23X0.h`): wake this early, µs.
pub const WAKE_DELAY_STANDBY_US: u32 = 185;
/// `RTC_TO_SYSTIMER_TICKS`: RTC `TIME8U` counts 8 µs.
pub const RTC_TO_SYSTIMER_TICKS: u32 = 8;
/// `MAX_SYSTIMER_DELTA`: larger compare deltas are in the past.
pub const MAX_SYSTIMER_DELTA: u32 = 0xFFBF_FFFF;
/// Longest single standby; keeps the 64-bit SYSTIM extension (wraps after
/// ~71 min) safe and bounds a lost wake source. The caller loops.
pub const MAX_STANDBY_US: u32 = 60_000_000;
/// SYSTIM channels; 0–1 count 1 µs, 2–4 count 250 ns
/// (`sysTimerResolutionShift` in the TI policy).
pub const SYSTIM_CHANNELS: usize = 5;
pub const SYSTIM_RESOLUTION_SHIFT: [u32; SYSTIM_CHANNELS] = [0, 0, 2, 2, 2];

/// Why standby was not entered; the caller should idle (WFI) instead.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StandbyRefusal {
    /// The LF clock (LFOSC) is not qualified yet: the RTC could not wake us.
    LfClockNotReady,
    /// The next event is closer than [`TOTAL_TIME_STANDBY_US`].
    TooShort,
    /// An interrupt is already pending; it must be serviced first.
    WakePending,
    /// Not in privileged thread mode on the MSP (`HapiEnterStandby` needs it).
    WrongStack,
}

/// Delta from `now_1u` to one channel's compare value, in µs, as computed by
/// the TI policy: in the channel's native resolution to handle wrapping, and
/// a compare in the past yields 0.
pub const fn channel_delta_us(compare: u32, now_1u: u32, shift: u32) -> u32 {
    let delta = compare.wrapping_sub(now_1u << shift);
    if delta > MAX_SYSTIMER_DELTA {
        0
    } else {
        delta >> shift
    }
}

/// RTC channel-0 compare for a wake `soonest_us` from now, or `None` when
/// that is too close for standby.
pub const fn rtc_compare(rtc_now_8u: u32, soonest_us: u32) -> Option<u32> {
    if soonest_us <= TOTAL_TIME_STANDBY_US {
        return None;
    }
    Some(rtc_now_8u.wrapping_add((soonest_us - WAKE_DELAY_STANDBY_US) / RTC_TO_SYSTIMER_TICKS))
}

/// Soonest wake in µs: the caller's deadline (capped at [`MAX_STANDBY_US`])
/// or the earliest armed and unmasked SYSTIM channel.
pub fn soonest_wake_us(
    deadline_delta_us: u64,
    now_1u: u32,
    active_channels: u32,
    compares: &[u32; SYSTIM_CHANNELS],
) -> u32 {
    let mut soonest = deadline_delta_us.min(u64::from(MAX_STANDBY_US)) as u32;
    for (channel, compare) in compares.iter().enumerate() {
        if active_channels & (1 << channel) != 0 {
            let delta = channel_delta_us(*compare, now_1u, SYSTIM_RESOLUTION_SHIFT[channel]);
            soonest = soonest.min(delta);
        }
    }
    soonest
}

/// Standby counters for diagnostics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct StandbyStats {
    pub entries: u32,
    pub refused_lf_clock: u32,
    pub refused_too_short: u32,
    pub refused_pending: u32,
}

#[cfg(target_os = "none")]
pub use target::{init, standby_until, stats};

#[cfg(target_os = "none")]
mod target {
    use super::*;
    use portable_atomic::{AtomicU32, Ordering};

    const CKMD_LDOCTL: u32 = 0x4000_10A8;

    const RTC_BASE: u32 = 0x4000_2000;
    const RTC_DESC: u32 = RTC_BASE;
    const RTC_ARMCLR: u32 = RTC_BASE + 0x0C;
    const RTC_TIME8U: u32 = RTC_BASE + 0x18;
    const RTC_CH0CC8U: u32 = RTC_BASE + 0x28;
    const RTC_ICLR: u32 = RTC_BASE + 0x54;
    const RTC_IMSET: u32 = RTC_BASE + 0x58;
    const RTC_CH0: u32 = 1 << 0;
    const RTC_EV0: u32 = 1 << 0;

    const EVTULL_WKUPMASK: u32 = 0x4000_5800;
    const EVTULL_WKUPMASK_AON_RTC_COMB: u32 = 0x10;

    const SYSTIM_BASE: u32 = 0x4002_2000;
    const SYSTIM_IMASK: u32 = SYSTIM_BASE + 0x44;
    const SYSTIM_TIME1U: u32 = SYSTIM_BASE + 0x104;
    const SYSTIM_STATUS: u32 = SYSTIM_BASE + 0x140;
    const SYSTIM_ARMSET: u32 = SYSTIM_BASE + 0x144;
    const SYSTIM_CH0CCSR: u32 = SYSTIM_BASE + 0x14C;
    const SYSTIM_STATUS_VAL_RUN: u32 = 1;

    const EVTSVT_CPUIRQ16SEL: u32 = 0x4002_5444;
    const PUBID_SYSTIM0: u32 = 0x1B;
    const PUBID_AON_RTC_COMB: u32 = 0x04;

    /// `HAPI_TABLE_BASE_ADDR` (`driverlib/hapi.h`); entry 0 is
    /// `enterStandby(const uint32_t *copyList)`.
    const HAPI_TABLE: u32 = 0x0F00_004C;

    const NVIC_ISER: u32 = 0xE000_E100;
    const NVIC_ISPR: u32 = 0xE000_E200;
    const NVIC_ICPR: u32 = 0xE000_E280;
    const IRQ_CPUIRQ16: u32 = 1 << 16;

    static ENTRIES: AtomicU32 = AtomicU32::new(0);
    static REFUSED_LF: AtomicU32 = AtomicU32::new(0);
    static REFUSED_SHORT: AtomicU32 = AtomicU32::new(0);
    static REFUSED_PENDING: AtomicU32 = AtomicU32::new(0);

    #[inline(always)]
    fn rd(address: u32) -> u32 {
        // SAFETY: fixed CC2340R52 peripheral/system registers.
        unsafe { (address as *const u32).read_volatile() }
    }

    #[inline(always)]
    fn wr(address: u32, value: u32) {
        // SAFETY: fixed CC2340R52 peripheral/system registers.
        unsafe { (address as *mut u32).write_volatile(value) }
    }

    /// `Power_init()` part: RTC channel 0 compare as the standby wake source
    /// (`RTC.IMSET = EV0`, `EVTULL.WKUPMASK = AON_RTC_COMB`). `CPUIRQ16`
    /// stays on SYSTIM outside standby, so the RTC never reaches the CPU.
    pub fn init() {
        wr(RTC_ARMCLR, RTC_CH0);
        wr(RTC_ICLR, RTC_EV0);
        wr(RTC_IMSET, RTC_EV0);
        wr(EVTULL_WKUPMASK, EVTULL_WKUPMASK_AON_RTC_COMB);
    }

    pub fn stats() -> StandbyStats {
        StandbyStats {
            entries: ENTRIES.load(Ordering::Relaxed),
            refused_lf_clock: REFUSED_LF.load(Ordering::Relaxed),
            refused_too_short: REFUSED_SHORT.load(Ordering::Relaxed),
            refused_pending: REFUSED_PENDING.load(Ordering::Relaxed),
        }
    }

    /// Enter standby until `deadline` (Embassy ticks, 1 µs), an earlier
    /// SYSTIM alarm, or a wake-enabled button edge.
    ///
    /// Returns `Ok(())` after a standby (the wake may be early; the caller
    /// re-checks its condition) or the reason it was refused, in which case
    /// nothing was changed. The radio must already be asleep.
    pub fn standby_until(deadline: u64) -> Result<(), StandbyRefusal> {
        let result = cortex_m::interrupt::free(|_| enter(deadline));
        let counter = match result {
            Ok(()) => &ENTRIES,
            Err(StandbyRefusal::LfClockNotReady) => &REFUSED_LF,
            Err(StandbyRefusal::TooShort) => &REFUSED_SHORT,
            Err(StandbyRefusal::WakePending | StandbyRefusal::WrongStack) => &REFUSED_PENDING,
        };
        counter.fetch_add(1, Ordering::Relaxed);
        result
    }

    fn enter(deadline: u64) -> Result<(), StandbyRefusal> {
        if !crate::power::lf_clock_good() {
            return Err(StandbyRefusal::LfClockNotReady);
        }
        // HapiEnterStandby() "must execute from MSP" in privileged thread
        // mode (PowerCC23X0_enterStandby()); cortex-m-rt never leaves it.
        if cortex_m::register::control::read().bits() != 0 {
            return Err(StandbyRefusal::WrongStack);
        }
        // An enabled, pending interrupt (button edge, expired alarm) would
        // abort the ROM's WFI anyway; service it first. Pending but disabled
        // lines cannot wake the core and are ignored.
        if rd(NVIC_ISPR) & rd(NVIC_ISER) != 0 {
            return Err(StandbyRefusal::WakePending);
        }

        let now = embassy_time::Instant::now().as_ticks();
        let imask = rd(SYSTIM_IMASK);
        let armset = rd(SYSTIM_ARMSET);
        let now_1u = rd(SYSTIM_TIME1U);
        let rtc_now = rd(RTC_TIME8U);
        // CHnCCSR, not CHnCC: reading it does not clear pending events.
        let mut compares = [0u32; SYSTIM_CHANNELS];
        for (channel, compare) in compares.iter_mut().enumerate() {
            *compare = rd(SYSTIM_CH0CCSR + 4 * channel as u32);
        }
        // TI considers every unmasked channel; a disarmed one cannot fire,
        // so only armed channels shorten the sleep here.
        let soonest = soonest_wake_us(
            deadline.saturating_sub(now),
            now_1u,
            imask & armset,
            &compares,
        );
        let rtc_wake = rtc_compare(rtc_now, soonest).ok_or(StandbyRefusal::TooShort)?;

        // Route CPUIRQ16 to the RTC; the wake event is consumed below
        // without vectoring (interrupts are masked).
        wr(EVTSVT_CPUIRQ16SEL, PUBID_AON_RTC_COMB);
        wr(NVIC_ICPR, IRQ_CPUIRQ16);
        // Writing CH0CC8U arms the RTC compare.
        wr(RTC_CH0CC8U, rtc_wake);

        // PowerCC23X0_enterStandby(): clear all CKM LDO SW control bits, or
        // the LDO stays on in standby.
        wr(CKMD_LDOCTL, 0);
        // SAFETY: ROM HAPI entry 0, called from privileged thread mode on the
        // MSP (checked above) with interrupts masked; the ROM saves the CPU
        // state, enters standby and returns here after wake-up.
        unsafe {
            let enter_standby: extern "C" fn(*const u32) =
                core::mem::transmute((HAPI_TABLE as *const usize).read_volatile());
            enter_standby(core::ptr::null());
        }

        // Disarm in case a button woke us, and clear the RTC event; read
        // back from the ULL domain (ULLSync()) before clearing the NVIC.
        wr(RTC_ARMCLR, RTC_CH0);
        wr(RTC_ICLR, RTC_EV0);
        let _ = rd(RTC_DESC);
        wr(NVIC_ICPR, IRQ_CPUIRQ16);
        wr(EVTSVT_CPUIRQ16SEL, PUBID_SYSTIM0);

        // SYSTIM resynchronises with the RTC after standby: wait for RUN
        // with SYNCUP clear (STATUS exactly RUN), then restore the channels.
        while rd(SYSTIM_STATUS) != SYSTIM_STATUS_VAL_RUN {}
        for (channel, compare) in compares.iter().enumerate() {
            wr(SYSTIM_CH0CCSR + 4 * channel as u32, *compare);
        }
        wr(SYSTIM_ARMSET, armset);
        wr(SYSTIM_IMASK, imask);
        // A compare that passed during the resync never matches again; pend
        // CPUIRQ16 so the time driver re-evaluates its queue (idempotent).
        wr(NVIC_ISPR, IRQ_CPUIRQ16);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_delta_matches_the_ti_policy() {
        // 1 µs channel, 10 ms ahead.
        assert_eq!(channel_delta_us(1_010_000, 1_000_000, 0), 10_000);
        // Across the 32-bit wrap.
        assert_eq!(channel_delta_us(100, u32::MAX - 99, 0), 200);
        // In the past -> 0 (wake at once).
        assert_eq!(channel_delta_us(999_999, 1_000_000, 0), 0);
        // 250 ns channel: native delta of 4000 ticks is 1000 µs.
        assert_eq!(channel_delta_us((1_000 << 2) + 4_000, 1_000, 2), 1_000);
    }

    #[test]
    fn rtc_compare_wakes_early_and_refuses_short_sleeps() {
        assert_eq!(rtc_compare(0, TOTAL_TIME_STANDBY_US), None);
        assert_eq!(
            rtc_compare(1_000, 10_000_000),
            Some(1_000 + (10_000_000 - WAKE_DELAY_STANDBY_US) / 8)
        );
        assert_eq!(
            rtc_compare(u32::MAX, 8 + WAKE_DELAY_STANDBY_US + 600),
            Some(75)
        );
    }

    #[test]
    fn soonest_wake_honours_armed_channels_and_the_cap() {
        let compares = [1_005_000, 0, 0, 0, 0];
        // Deadline 10 s away, channel 0 armed 5 ms away.
        assert_eq!(
            soonest_wake_us(10_000_000, 1_000_000, 0b1, &compares),
            5_000
        );
        // Channel 0 not active: the deadline wins.
        assert_eq!(
            soonest_wake_us(10_000_000, 1_000_000, 0, &compares),
            10_000_000
        );
        // Long deadlines are capped.
        assert_eq!(soonest_wake_us(u64::MAX, 0, 0, &compares), MAX_STANDBY_US);
        // An active channel already in the past forbids standby.
        let past = [999_000, 0, 0, 0, 0];
        assert_eq!(soonest_wake_us(10_000_000, 1_000_000, 0b1, &past), 0);
    }
}
