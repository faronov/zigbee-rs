//! Adaptive persistence of GPD security frame counters, shared by the Proxy
//! Table and the Sink Table.
//!
//! A protected GPD's frame counter must survive a reboot without ever
//! accepting a counter twice. Two persistence modes trade flash wear against
//! frames rejected after a reboot:
//!
//! - **batched** (the original behaviour): the counter is written only after
//!   it advanced by `stride` since the last write, and a restore skips it
//!   ahead by `stride - 1`, the most that can have been accepted unwritten.
//!   Up to `stride - 1` genuinely new frames are rejected after a reboot;
//!   right for frequent reporters (a sensor every 60 s writes once per
//!   ~64 min).
//! - **exact**: every accepted frame is written and a restore uses the
//!   written counter itself, so no new frame is lost after a reboot; right
//!   for rare, user-visible commands (a switch press).
//!
//! The mode is chosen per accepted frame: a GPD *event* command
//! ([`is_event_command`]: anything below the attribute-reporting range
//! 0xA0) uses exact persistence while a shared write budget
//! ([`ExactBudget`]: `exact_burst` writes, refilled one per
//! `exact_refill_ms`) allows it; reports, and events beyond the budget, use
//! batched persistence. Every mode change is itself written together with
//! the counter, so the stored mode always describes the stored counter:
//!
//! - entering exact writes the current counter (nothing is unwritten);
//! - leaving exact writes the current counter as batched, so the restore
//!   floor `counter + stride - 1` covers the frames that follow unwritten.
//!
//! The persistence layer stores the mode next to each counter; journals
//! without it restore as batched, which is always safe.
//!
//! The one remaining window is shared by both modes: a frame accepted (and
//! acted upon) after its write was requested but before that write
//! completed in the same router loop pass can be accepted once more after a
//! power loss in between.

/// Default [`CounterPolicy::exact_burst`]: up to 16 exact writes in a burst.
pub const DEFAULT_EXACT_COUNTER_BURST: u8 = 16;
/// Default [`CounterPolicy::exact_refill_ms`]: one exact write per minute
/// sustained (at most 1,440 a day per table).
pub const DEFAULT_EXACT_COUNTER_REFILL_MS: u32 = 60_000;

/// Whether a GPD command is an *event* (switch, level, scene, door lock,
/// generic switch, ... below 0xA0) rather than a report, request or
/// manufacturer-specific command (0xA0..=0xBF) or commissioning (0xE0..).
pub const fn is_event_command(command_id: u8) -> bool {
    command_id < 0xA0
}

/// Counter persistence policy of a table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CounterPolicy {
    /// Batched stride (see the module docs); 1 writes every frame.
    pub stride: u32,
    /// Exact writes allowed in a burst; 0 disables exact persistence.
    pub exact_burst: u8,
    /// One exact write is refilled per this many milliseconds.
    pub exact_refill_ms: u32,
}

impl CounterPolicy {
    /// Lowest counter a restored entry may accept next minus one: the stored
    /// counter itself when it was written exactly, otherwise skipped ahead
    /// by `stride - 1`.
    pub const fn restore_floor(&self, stored: u32, exact: bool) -> u32 {
        if exact {
            stored
        } else {
            stored.saturating_add(if self.stride == 0 { 0 } else { self.stride - 1 })
        }
    }
}

/// Token bucket bounding exact writes of one table (flash wear).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExactBudget {
    tokens: u8,
    refilled_ms: u32,
    started: bool,
}

impl Default for ExactBudget {
    fn default() -> Self {
        Self::new()
    }
}

impl ExactBudget {
    /// A budget that starts full on its first use.
    pub const fn new() -> Self {
        Self {
            tokens: 0,
            refilled_ms: 0,
            started: false,
        }
    }

    /// Take one exact write at `now_ms`, if the budget allows it.
    pub fn take(&mut self, now_ms: u32, policy: &CounterPolicy) -> bool {
        let burst = policy.exact_burst;
        if burst == 0 {
            return false;
        }
        if !self.started {
            self.started = true;
            self.tokens = burst;
            self.refilled_ms = now_ms;
        } else if let Some(gained) = now_ms
            .wrapping_sub(self.refilled_ms)
            .checked_div(policy.exact_refill_ms)
        {
            if gained != 0 {
                let room = u32::from(burst.saturating_sub(self.tokens));
                self.tokens = burst.min(self.tokens.saturating_add(gained.min(room) as u8));
                self.refilled_ms = self
                    .refilled_ms
                    .wrapping_add(gained.wrapping_mul(policy.exact_refill_ms));
            }
            if self.tokens == burst {
                self.refilled_ms = now_ms;
            }
        }
        if self.tokens == 0 {
            return false;
        }
        self.tokens -= 1;
        true
    }
}

/// Record an accepted protected `counter` of an entry whose last write was
/// `persisted` in mode `exact`. Returns whether the entry must be written
/// now; `persisted` and `exact` then describe that write.
pub fn on_accepted(
    persisted: &mut u32,
    exact: &mut bool,
    counter: u32,
    event: bool,
    budget: &mut ExactBudget,
    policy: &CounterPolicy,
    now_ms: u32,
) -> bool {
    if event && budget.take(now_ms, policy) {
        *exact = true;
    } else if *exact {
        // Leave exact mode with a write, so the batched restore floor covers
        // the frames that follow unwritten.
        *exact = false;
    } else if counter.wrapping_sub(*persisted) < policy.stride.max(1) {
        return false;
    }
    *persisted = counter;
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    const POLICY: CounterPolicy = CounterPolicy {
        stride: 64,
        exact_burst: 4,
        exact_refill_ms: 1_000,
    };

    /// A table entry backed by a simulated journal: `stored` is what flash
    /// holds (counter, exact).
    struct Entry {
        live: u32,
        persisted: u32,
        exact: bool,
        stored: (u32, bool),
    }

    impl Entry {
        fn new(counter: u32, exact: bool) -> Self {
            Self {
                live: counter,
                persisted: counter,
                exact,
                stored: (counter, exact),
            }
        }

        /// Accept `counter` if fresh; write it when the policy says so.
        fn rx(&mut self, counter: u32, event: bool, budget: &mut ExactBudget, now: u32) -> bool {
            if counter <= self.live {
                return false;
            }
            self.live = counter;
            if on_accepted(
                &mut self.persisted,
                &mut self.exact,
                counter,
                event,
                budget,
                &POLICY,
                now,
            ) {
                self.stored = (self.live, self.exact);
            }
            true
        }

        fn reboot(&self) -> Self {
            let floor = POLICY.restore_floor(self.stored.0, self.stored.1);
            Self {
                live: floor,
                persisted: self.stored.0,
                exact: self.stored.1,
                stored: self.stored,
            }
        }
    }

    #[test]
    fn event_commands_are_below_the_reporting_range() {
        for id in [0x00, 0x20, 0x21, 0x22, 0x34, 0x60, 0x69, 0x9F] {
            assert!(is_event_command(id), "{id:#04x}");
        }
        for id in [0xA0, 0xA2, 0xAF, 0xB0, 0xE0, 0xE2, 0xF0] {
            assert!(!is_event_command(id), "{id:#04x}");
        }
    }

    #[test]
    fn budget_refills_and_caps_at_the_burst() {
        let mut b = ExactBudget::new();
        for _ in 0..4 {
            assert!(b.take(0, &POLICY));
        }
        assert!(!b.take(999, &POLICY));
        assert!(b.take(1_000, &POLICY));
        assert!(!b.take(1_500, &POLICY));
        // A long pause refills to the burst only.
        for _ in 0..4 {
            assert!(b.take(1_000_000, &POLICY));
        }
        assert!(!b.take(1_000_000, &POLICY));
        // Disabled.
        let off = CounterPolicy {
            exact_burst: 0,
            ..POLICY
        };
        assert!(!ExactBudget::new().take(0, &off));
    }

    #[test]
    fn budget_survives_clock_wrap() {
        let mut b = ExactBudget::new();
        for _ in 0..4 {
            assert!(b.take(u32::MAX - 100, &POLICY));
        }
        assert!(!b.take(u32::MAX, &POLICY));
        assert!(b.take(1_000, &POLICY));
    }

    /// Reboot after `n` accepted frames in a mode; every accepted counter is
    /// rejected afterwards and the first new one is accepted when the mode
    /// promises it.
    fn reboot_after(n: u32, event: bool) {
        let mut budget = ExactBudget::new();
        // Commissioned exactly at 1000 (a new entry is written exactly).
        let mut e = Entry::new(1000, true);
        let mut now = 0;
        let mut accepted = heapless::Vec::<u32, 128>::new();
        for i in 1..=n {
            now += 60_000;
            assert!(e.rx(1000 + i, event, &mut budget, now));
            accepted.push(1000 + i).unwrap();
        }
        let mut rebooted = e.reboot();
        for &c in accepted.iter().chain([1000].iter()) {
            assert!(
                !rebooted.rx(c, event, &mut ExactBudget::new(), now),
                "{n} frames, event {event}: replayed {c} accepted"
            );
        }
        let next = 1000 + n + 1;
        if event {
            assert!(
                rebooted.rx(next, event, &mut ExactBudget::new(), now + 60_000),
                "{n} frames: the next press must be accepted"
            );
        } else {
            // Batched: the floor never undercuts an accepted counter.
            assert!(rebooted.live >= 1000 + n);
            assert!(rebooted.live < 1000 + n + POLICY.stride);
        }
    }

    #[test]
    fn reboot_after_1_2_64_65_frames_in_both_modes() {
        for n in [1, 2, 64, 65] {
            reboot_after(n, true);
            reboot_after(n, false);
        }
    }

    #[test]
    fn exact_mode_writes_every_event_and_batched_mode_every_stride() {
        let mut budget = ExactBudget::new();
        let mut e = Entry::new(0, true);
        let mut writes = 0;
        for i in 1..=10 {
            let before = e.stored;
            e.rx(i, true, &mut budget, i * 60_000);
            writes += u32::from(e.stored != before);
        }
        assert_eq!(writes, 10, "one write per press");
        let mut e = Entry::new(0, false);
        let mut writes = 0;
        for i in 1..=640 {
            let before = e.stored;
            e.rx(i, false, &mut budget, i * 60_000);
            writes += u32::from(e.stored != before);
        }
        assert_eq!(writes, 10, "one write per stride");
    }

    #[test]
    fn mode_changes_are_written_and_never_reuse_a_counter() {
        // Events beyond the budget, mixed with reports, with a reboot after
        // every frame: no counter is ever accepted twice.
        let mut budget = ExactBudget::new();
        let mut e = Entry::new(5000, true);
        let mut seen = heapless::Vec::<u32, 512>::new();
        let mut now = 0u32;
        for i in 1..=300u32 {
            let event = i % 7 != 0;
            now = now.wrapping_add(if i % 50 < 30 { 10 } else { 2_000 });
            let c = 5000 + i;
            assert!(e.rx(c, event, &mut budget, now));
            seen.push(c).unwrap();
            let mut r = e.reboot();
            for &old in seen.iter() {
                assert!(
                    !r.rx(old, event, &mut ExactBudget::new(), now),
                    "frame {i}: replayed {old}"
                );
            }
        }
    }

    #[test]
    fn leaving_exact_mode_writes_the_batched_floor() {
        let mut budget = ExactBudget::new();
        let mut e = Entry::new(100, true);
        assert!(e.rx(101, true, &mut budget, 0));
        assert_eq!(e.stored, (101, true));
        // A report leaves exact mode with a write.
        assert!(e.rx(102, false, &mut budget, 0));
        assert_eq!(e.stored, (102, false));
        // Unwritten frames inside the stride are covered by the floor.
        assert!(e.rx(150, false, &mut budget, 0));
        assert_eq!(e.stored, (102, false));
        assert_eq!(e.reboot().live, 102 + 63);
    }
}
