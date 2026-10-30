//! BL702 free-running timer and cycle-count delays.

use core::hint::spin_loop;

use crate::clock::{Clocks, Peripheral, enable_and_reset};
use crate::mmio::{read32, rmw, write32};
use crate::peripherals::Timer0;

const TIMER_BASE: u32 = 0x4000_a500;
const TIMER_CLOCK_CONFIG: u32 = TIMER_BASE;
const TIMER_COMPARE_0: u32 = TIMER_BASE + 0x010;
const TIMER_COMPARE_1: u32 = TIMER_BASE + 0x014;
const TIMER_COMPARE_2: u32 = TIMER_BASE + 0x018;
const TIMER_COUNTER: u32 = TIMER_BASE + 0x02c;
const TIMER_MATCH_STATUS: u32 = TIMER_BASE + 0x038;
const TIMER_INTERRUPT_ENABLE: u32 = TIMER_BASE + 0x044;
const TIMER_PRELOAD_CONTROL: u32 = TIMER_BASE + 0x05c;
const TIMER_INTERRUPT_CLEAR: u32 = TIMER_BASE + 0x078;
const TIMER_ENABLE: u32 = TIMER_BASE + 0x084;
const TIMER_COUNT_MODE: u32 = TIMER_BASE + 0x088;
const TIMER_CLOCK_DIVIDER: u32 = TIMER_BASE + 0x0bc;

const COMPARE_0: u32 = 1 << 0;

/// Longest interval the comparator alarm is armed for.
///
/// The raw counter wraps every 2^32 us (about 71.6 minutes) and the 64-bit
/// extension needs at least one counter read per wrap. Capping every alarm at
/// 30 minutes guarantees that the alarm interrupt itself reads the counter
/// often enough even when the CPU sleeps with no other timer pending.
pub const ALARM_MAX_INTERVAL_TICKS: u64 = 30 * 60 * 1_000_000;

/// Minimum lead between "now" and an armed comparator value.
///
/// The comparator fires only on equality. A target closer than this is
/// treated as already due so a racing counter cannot step past it and delay
/// the wake by a full 71-minute wrap.
pub const ALARM_MIN_LEAD_TICKS: u64 = 20;

/// Comparator plan for one alarm request.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlarmPlan {
    /// The target is due now (or within [`ALARM_MIN_LEAD_TICKS`]).
    Due,
    /// Arm comparator 0 at `compare`; the interrupt represents tick `at`,
    /// which is the target or the capped wrap-safety deadline.
    Arm { at: u64, compare: u32 },
}

/// Plan comparator 0 for `target` given the extended counter value `now`.
pub const fn plan_alarm(now: u64, target: u64) -> AlarmPlan {
    if target <= now.saturating_add(ALARM_MIN_LEAD_TICKS) {
        return AlarmPlan::Due;
    }
    let cap = now.saturating_add(ALARM_MAX_INTERVAL_TICKS);
    let at = if target < cap { target } else { cap };
    // The extended value is `epoch | raw`, so the raw comparator value is the
    // low 32 bits. The cap keeps `at - now` far below one wrap.
    AlarmPlan::Arm {
        at,
        compare: at as u32,
    }
}

/// Result of arming the comparator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlarmArm {
    /// Comparator 0 is armed for tick `at`.
    Armed { at: u64 },
    /// The requested tick is already due; the comparator interrupt is
    /// disabled and the caller must service the expiry itself.
    Due,
}

/// Free-running one-megahertz TIMER channel used as a monotonic clock.
pub struct Monotonic {
    _token: Timer0,
    last_ticks: u32,
    epoch: u64,
}

impl Monotonic {
    /// Configure the same 1 MHz free-running timer used by the hardware-proven
    /// BL702 radio path.
    pub fn new_1mhz(token: Timer0, clocks: Clocks) -> Self {
        assert_eq!(clocks.fclk_hz(), 32_000_000);
        enable_and_reset(Peripheral::Timer);

        rmw(TIMER_ENABLE, 1 << 1, 0);
        rmw(TIMER_CLOCK_CONFIG, 0x3 << 2, 0);
        rmw(TIMER_CLOCK_DIVIDER, 0xff << 8, 31 << 8);
        rmw(TIMER_COUNT_MODE, 1 << 1, 1 << 1);
        write32(TIMER_PRELOAD_CONTROL, 0);
        write32(TIMER_COMPARE_0, u32::MAX - 2);
        write32(TIMER_COMPARE_1, u32::MAX - 2);
        write32(TIMER_COMPARE_2, u32::MAX - 2);
        write32(TIMER_INTERRUPT_ENABLE, 0);
        write32(TIMER_INTERRUPT_CLEAR, 0x7);
        rmw(TIMER_ENABLE, 1 << 1, 1 << 1);
        Self {
            _token: token,
            last_ticks: read32(TIMER_COUNTER),
            epoch: 0,
        }
    }

    /// Return active-time microseconds as an extended 64-bit monotonic value.
    ///
    /// Callers must read the counter at least once per approximately
    /// 71-minute wrap. The interrupt-driven alarm guarantees this by never
    /// arming further than [`ALARM_MAX_INTERVAL_TICKS`] ahead. The counter
    /// keeps running in CPU idle (WFI); PDS/HBN, which stop this clock, remain
    /// unsupported.
    pub fn ticks(&mut self) -> u64 {
        let current = read32(TIMER_COUNTER);
        extend_ticks(&mut self.last_ticks, &mut self.epoch, current)
    }

    /// Read the raw wrapping one-megahertz hardware counter.
    pub fn raw_ticks(&self) -> u32 {
        read32(TIMER_COUNTER)
    }

    /// Arm comparator 0 to raise the TIMER channel 0 interrupt
    /// ([`crate::interrupt::TIMER_CH0_IRQ`]) at extended tick `target`, capped
    /// at [`ALARM_MAX_INTERVAL_TICKS`] from now.
    ///
    /// Returns [`AlarmArm::Due`] when the target is already due or the
    /// counter reached the armed value while it was being programmed. In the
    /// latter case a match may still have latched; the resulting interrupt
    /// is a harmless spurious wake.
    ///
    /// Call with interrupts disabled so the extended counter and comparator
    /// state change atomically with respect to the alarm interrupt.
    pub fn arm_alarm(&mut self, target: u64) -> AlarmArm {
        let now = self.ticks();
        match plan_alarm(now, target) {
            AlarmPlan::Due => {
                self.disable_alarm();
                AlarmArm::Due
            }
            AlarmPlan::Arm { at, compare } => {
                rmw(TIMER_INTERRUPT_ENABLE, COMPARE_0, 0);
                write32(TIMER_COMPARE_0, compare);
                write32(TIMER_INTERRUPT_CLEAR, COMPARE_0);
                rmw(TIMER_INTERRUPT_ENABLE, COMPARE_0, COMPARE_0);
                if self.ticks() >= at {
                    AlarmArm::Due
                } else {
                    AlarmArm::Armed { at }
                }
            }
        }
    }

    /// Disable the comparator 0 interrupt and clear its latched match.
    pub fn disable_alarm(&mut self) {
        rmw(TIMER_INTERRUPT_ENABLE, COMPARE_0, 0);
        write32(TIMER_INTERRUPT_CLEAR, COMPARE_0);
    }

    /// Acknowledge a comparator 0 match. The interrupt stays enabled; the
    /// comparator only matches again after the counter wraps unless rearmed.
    pub fn clear_alarm(&mut self) {
        write32(TIMER_INTERRUPT_CLEAR, COMPARE_0);
    }

    /// Return whether comparator 0 has latched a match.
    pub fn alarm_matched(&self) -> bool {
        read32(TIMER_MATCH_STATUS) & COMPARE_0 != 0
    }
}

fn extend_ticks(last: &mut u32, epoch: &mut u64, current: u32) -> u64 {
    if current < *last {
        *epoch += 1u64 << 32;
    }
    *last = current;
    *epoch | u64::from(current)
}

/// Busy-wait for microseconds using the 32 MHz RISC-V cycle counter.
///
/// This function intentionally retains a plain function-pointer ABI for the
/// existing pure-Rust radio calibration path.
pub fn delay_us(duration_us: u32) {
    let target_cycles = duration_us.saturating_mul(32);
    let started = cycle_count();
    while cycle_count().wrapping_sub(started) < target_cycles {
        spin_loop();
    }
}

#[inline(always)]
fn cycle_count() -> u32 {
    #[cfg(target_arch = "riscv32")]
    {
        let value: u32;
        // SAFETY: Reading `mcycle` has no memory-safety side effects.
        unsafe {
            core::arch::asm!("csrr {}, mcycle", out(reg) value);
        }
        value
    }

    #[cfg(not(target_arch = "riscv32"))]
    {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::{
        ALARM_MAX_INTERVAL_TICKS, ALARM_MIN_LEAD_TICKS, AlarmPlan, extend_ticks, plan_alarm,
    };

    #[test]
    fn near_or_past_targets_are_due() {
        assert_eq!(plan_alarm(1_000, 999), AlarmPlan::Due);
        assert_eq!(plan_alarm(1_000, 1_000), AlarmPlan::Due);
        assert_eq!(
            plan_alarm(1_000, 1_000 + ALARM_MIN_LEAD_TICKS),
            AlarmPlan::Due
        );
        assert_eq!(
            plan_alarm(1_000, 1_001 + ALARM_MIN_LEAD_TICKS),
            AlarmPlan::Arm {
                at: 1_001 + ALARM_MIN_LEAD_TICKS,
                compare: 1_001 + ALARM_MIN_LEAD_TICKS as u32,
            }
        );
    }

    #[test]
    fn distant_and_absent_targets_are_capped_below_one_wrap() {
        let now = (7u64 << 32) | 0xffff_0000;
        let AlarmPlan::Arm { at, compare } = plan_alarm(now, u64::MAX) else {
            panic!("an absent target must still arm the wrap-safety alarm");
        };
        assert_eq!(at, now + ALARM_MAX_INTERVAL_TICKS);
        assert!(at - now < 1u64 << 32);
        // The comparator value is the wrapped raw counter value.
        assert_eq!(compare, at as u32);
        assert!(compare < 0xffff_0000, "the cap crosses a raw wrap");
    }

    #[test]
    fn saturates_at_the_end_of_time() {
        assert_eq!(plan_alarm(u64::MAX - 5, u64::MAX), AlarmPlan::Due);
    }

    #[test]
    fn capped_alarm_reads_keep_the_extension_correct() {
        // Reading only at capped-alarm cadence still observes every wrap.
        let mut last = 0;
        let mut epoch = 0;
        let mut now = 0u64;
        for _ in 0..10 {
            now += ALARM_MAX_INTERVAL_TICKS;
            assert_eq!(extend_ticks(&mut last, &mut epoch, now as u32), now);
        }
    }

    #[test]
    fn active_polling_extends_one_mhz_counter_rollover() {
        let mut last = u32::MAX - 10;
        let mut epoch = 0;
        assert_eq!(
            extend_ticks(&mut last, &mut epoch, u32::MAX - 1),
            u64::from(u32::MAX - 1)
        );
        assert_eq!(extend_ticks(&mut last, &mut epoch, 5), (1u64 << 32) | 5);
    }
}
