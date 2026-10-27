//! Hardware-independent pieces of the tickless SYSTIM time driver.
//!
//! Kept free of register access so the 32-bit extension and the timer queue
//! can be unit-tested on the host.

use core::task::Waker;

/// Longest single SYSTIM channel 0 compare the driver programs, in µs.
///
/// `SYSTIM.TIME1U` is a 32-bit microsecond counter that wraps every
/// ~71.6 minutes. The 64-bit extension below only sees a wrap if the counter
/// is read at least once per wrap period, so the alarm is never programmed
/// further out than this; an otherwise idle system then still wakes (and
/// reads the counter) every 30 minutes.
pub const MAX_ALARM_DELTA_US: u64 = 30 * 60 * 1_000_000;
// The cap must stay well inside one 32-bit counter wrap so `extend()` never
// misses a wrap between two alarms.
const _: () = assert!(MAX_ALARM_DELTA_US < (1 << 32) / 2);

/// Extend a free-running 32-bit counter to 64 bits.
///
/// `state` holds the previously returned 64-bit value. Valid as long as it is
/// called at least once per 2^32 counts.
pub fn extend(state: &mut u64, low: u32) -> u64 {
    let previous_low = *state as u32;
    let mut high = *state & !0xFFFF_FFFF;
    if low < previous_low {
        high = high.wrapping_add(1 << 32);
    }
    *state = high | u64::from(low);
    *state
}

/// Next value to program into the 32-bit compare register for a 64-bit
/// deadline, capped at [`MAX_ALARM_DELTA_US`] from `now`.
pub fn compare_target(now: u64, deadline: u64) -> u64 {
    deadline.min(now.saturating_add(MAX_ALARM_DELTA_US))
}

struct Entry {
    at: u64,
    waker: Waker,
}

/// Small fixed-capacity timer queue with embassy's generic-queue semantics:
/// one entry per waker at its earliest requested time, woken once expired.
pub struct TimerQueue<const N: usize> {
    entries: [Option<Entry>; N],
}

impl<const N: usize> TimerQueue<N> {
    pub const fn new() -> Self {
        Self {
            entries: [const { None }; N],
        }
    }

    /// Returns `true` when the earliest deadline may have changed.
    pub fn schedule_wake(&mut self, at: u64, waker: &Waker) -> bool {
        for entry in self.entries.iter_mut().flatten() {
            if entry.waker.will_wake(waker) {
                if at < entry.at {
                    entry.at = at;
                    return true;
                }
                return false;
            }
        }

        if let Some(slot) = self.entries.iter_mut().find(|slot| slot.is_none()) {
            *slot = Some(Entry {
                at,
                waker: waker.clone(),
            });
            return true;
        }

        // Full: wake the latest entry early (a spurious wake is always
        // allowed) and reuse its slot.
        let latest = self
            .entries
            .iter_mut()
            .max_by_key(|slot| slot.as_ref().map_or(0, |entry| entry.at))
            .expect("queue capacity is non-zero");
        if let Some(evicted) = latest.replace(Entry {
            at,
            waker: waker.clone(),
        }) {
            evicted.waker.wake();
        }
        true
    }

    /// Wake every expired entry and return the earliest remaining deadline
    /// (`u64::MAX` when empty).
    pub fn next_expiration(&mut self, now: u64) -> u64 {
        let mut next = u64::MAX;
        for slot in self.entries.iter_mut() {
            let expired = matches!(slot, Some(entry) if entry.at <= now);
            if expired {
                if let Some(entry) = slot.take() {
                    entry.waker.wake();
                }
            } else if let Some(entry) = slot {
                next = next.min(entry.at);
            }
        }
        next
    }
}

impl<const N: usize> Default for TimerQueue<N> {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use super::*;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::task::Wake;

    struct Counter(AtomicUsize);

    impl Wake for Counter {
        fn wake(self: Arc<Self>) {
            self.0.fetch_add(1, Ordering::SeqCst);
        }
    }

    fn waker() -> (Arc<Counter>, Waker) {
        let counter = Arc::new(Counter(AtomicUsize::new(0)));
        (counter.clone(), Waker::from(counter))
    }

    #[test]
    fn extension_carries_across_the_32_bit_wrap() {
        let mut state = 0;
        assert_eq!(extend(&mut state, 10), 10);
        assert_eq!(extend(&mut state, 0xFFFF_FFF0), 0xFFFF_FFF0);
        assert_eq!(extend(&mut state, 5), (1 << 32) + 5);
        assert_eq!(extend(&mut state, 5), (1 << 32) + 5);
        assert_eq!(extend(&mut state, 0xFFFF_FFFF), (1 << 32) + 0xFFFF_FFFF);
        assert_eq!(extend(&mut state, 0), 2 << 32);
    }

    #[test]
    fn alarms_are_capped_well_inside_one_counter_wrap() {
        assert_eq!(compare_target(100, 200), 200);
        assert_eq!(compare_target(100, u64::MAX), 100 + MAX_ALARM_DELTA_US);
        assert_eq!(compare_target(u64::MAX - 1, u64::MAX), u64::MAX);
    }

    #[test]
    fn queue_keeps_the_earliest_deadline_per_waker() {
        let mut queue = TimerQueue::<2>::new();
        let (count, w) = waker();
        assert!(queue.schedule_wake(1_000, &w));
        // Order of polling must not matter: a later deadline never replaces
        // an earlier one for the same waker.
        assert!(!queue.schedule_wake(5_000, &w));
        assert!(queue.schedule_wake(500, &w));
        assert_eq!(queue.next_expiration(0), 500);
        assert_eq!(queue.next_expiration(499), 500);
        assert_eq!(count.0.load(Ordering::SeqCst), 0);
        assert_eq!(queue.next_expiration(500), u64::MAX);
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
    }

    #[test]
    fn a_full_queue_wakes_the_latest_entry_early_instead_of_losing_one() {
        let mut queue = TimerQueue::<2>::new();
        let (a_count, a) = waker();
        let (b_count, b) = waker();
        let (c_count, c) = waker();
        queue.schedule_wake(100, &a);
        queue.schedule_wake(900, &b);
        queue.schedule_wake(300, &c);
        assert_eq!(b_count.0.load(Ordering::SeqCst), 1);
        assert_eq!(queue.next_expiration(0), 100);
        queue.next_expiration(1_000);
        assert_eq!(a_count.0.load(Ordering::SeqCst), 1);
        assert_eq!(c_count.0.load(Ordering::SeqCst), 1);
    }
}
