//! Observed poll interval of sleepy children (`adaptive-indirect`).
//!
//! A parent buffers frames for a sleepy child only until the child's next
//! MAC Data Request. The default transaction lifetime (≈7.68 s,
//! `macTransactionPersistenceTime` 0x01F4) loses every frame sent to a child
//! whose long-poll interval is longer — a sensor polling every 30 s outside
//! its fast-poll windows cannot be read. With this feature the parent learns
//! each child's poll interval from its Data Requests and keeps that child's
//! indirect frames until the next expected poll, bounded by
//! [`MAX_HOLD_SECS`] and by the child's negotiated End Device Timeout.
//!
//! The estimate is the largest interval seen within [`WINDOW_US`]: a sensor
//! that fast-polls for a few seconds after every exchange keeps its long-poll
//! hold, and a child that permanently polls faster drops to it once the
//! window has passed without a long interval. Holding a frame longer than
//! needed costs only the queue slot: it still goes out at the next poll.
//!
//! The estimates live in RAM. A child restored from the durable child table
//! after a reboot is seeded instead ([`PollEstimates::seed`]): until it is
//! heard to long-poll again, its frames are held up to its End Device
//! Timeout (capped by [`MAX_HOLD_SECS`]). Zigbee leaves a parent's
//! buffering time to its `macTransactionPersistenceTime`; the only bound a
//! sleepy child guarantees is its End Device Timeout, within which it must
//! poll (R21+ keepalive) or be aged out, so a frame held that long reaches
//! a live child at its next poll. Without the seed every frame to a
//! long-polling child after a parent restart expired after the default
//! 8 s, and APS retries spaced by that hold all missed its 30 s polls.

use zigbee_types::ShortAddress;

/// Hold used without an estimate: the spec default transaction lifetime,
/// rounded up to whole seconds as the indirect queue ages in seconds.
pub const DEFAULT_HOLD_SECS: u8 = 8;
/// Upper bound of an adaptive hold (one queue slot per frame, at most this
/// long; the host's confirm timeout is derived from it, ZRHP 7.3).
pub const MAX_HOLD_SECS: u8 = 64;

/// How long the largest observed interval stays authoritative (10 min).
pub const WINDOW_US: u32 = 600_000_000;

/// Children tracked at once (the NCP's `max_children`).
pub const MAX_TRACKED: usize = crate::ncp_capacity_tier(16, 64, 64, 16);

#[derive(Clone, Copy)]
struct Estimate {
    child: ShortAddress,
    last_poll_us: u32,
    /// Largest poll interval (ms) seen since `max_since_us` (0 = one poll).
    interval_ms: u32,
    max_since_us: u32,
    /// `last_poll_us` is a real poll (not the seeding time).
    polled: bool,
    /// Conservative seed of a restored child, kept until a long poll is
    /// measured or a whole window passed.
    seeded: bool,
}

/// Seed interval of a restored child: holds of [`MAX_HOLD_SECS`], bounded
/// by its End Device Timeout.
const SEED_INTERVAL_MS: u32 = MAX_HOLD_SECS as u32 * 1_000;

/// Hold for a measured poll interval: the interval plus a quarter and one
/// second (whole seconds), before the timeout/cap bounds.
const fn wanted_hold_secs(interval_ms: u32) -> u32 {
    (interval_ms + interval_ms / 4) / 1_000 + 1
}

/// Per-child poll-interval estimates.
pub struct PollEstimates {
    entries: heapless::Vec<Estimate, MAX_TRACKED>,
}

impl PollEstimates {
    pub const fn new() -> Self {
        Self {
            entries: heapless::Vec::new(),
        }
    }

    /// Record one Data Request from `child` at `now_us`.
    pub fn note_poll(&mut self, child: ShortAddress, now_us: u32) {
        if let Some(entry) = self.entries.iter_mut().find(|entry| entry.child == child) {
            if !entry.polled {
                // The first poll after seeding: no interval yet.
                entry.last_poll_us = now_us;
                entry.polled = true;
                return;
            }
            let interval_ms = now_us.wrapping_sub(entry.last_poll_us) / 1_000;
            if entry.seeded {
                // A fast-poll burst does not replace the seed; the first
                // long poll (one that needs more than the default hold) or
                // a whole window without one does.
                if wanted_hold_secs(interval_ms) > u32::from(DEFAULT_HOLD_SECS)
                    || now_us.wrapping_sub(entry.max_since_us) >= WINDOW_US
                {
                    entry.seeded = false;
                    entry.interval_ms = interval_ms;
                    entry.max_since_us = now_us;
                }
            } else if interval_ms >= entry.interval_ms
                || now_us.wrapping_sub(entry.max_since_us) >= WINDOW_US
            {
                entry.interval_ms = interval_ms;
                entry.max_since_us = now_us;
            }
            entry.last_poll_us = now_us;
            return;
        }
        self.insert(Estimate {
            child,
            last_poll_us: now_us,
            interval_ms: 0,
            max_since_us: now_us,
            polled: true,
            seeded: false,
        });
    }

    /// Seed a child restored after a reboot (nothing learned this power
    /// cycle): hold its frames up to its End Device Timeout until it is heard
    /// to long-poll. Replaces any estimate of that address. Out of line: it
    /// runs once per restored child and stays off its callers' frames.
    #[inline(never)]
    pub fn seed(&mut self, child: ShortAddress, now_us: u32) {
        self.forget(child);
        self.insert(Estimate {
            child,
            last_poll_us: now_us,
            interval_ms: SEED_INTERVAL_MS,
            max_since_us: now_us,
            polled: false,
            seeded: true,
        });
    }

    fn insert(&mut self, estimate: Estimate) {
        let now_us = estimate.last_poll_us;
        if self.entries.push(estimate).is_err() {
            // Full: replace the child heard from least recently.
            if let Some(oldest) = self
                .entries
                .iter_mut()
                .max_by_key(|entry| now_us.wrapping_sub(entry.last_poll_us))
            {
                *oldest = estimate;
            }
        }
    }

    /// Forget a child (left or aged out).
    pub fn forget(&mut self, child: ShortAddress) {
        self.entries.retain(|entry| entry.child != child);
    }

    /// Seconds to hold an indirect frame for `child`: the estimated poll
    /// interval plus a quarter and one second of margin, within
    /// [`DEFAULT_HOLD_SECS`]..=[`MAX_HOLD_SECS`] and never longer than
    /// `timeout_secs` (the child's End Device Timeout).
    pub fn hold_secs(&self, child: ShortAddress, timeout_secs: u32) -> u8 {
        let interval_ms = self
            .entries
            .iter()
            .find(|entry| entry.child == child)
            .map_or(0, |entry| entry.interval_ms);
        let wanted = wanted_hold_secs(interval_ms);
        let hold = wanted
            .min(timeout_secs)
            .clamp(u32::from(DEFAULT_HOLD_SECS), u32::from(MAX_HOLD_SECS));
        hold as u8
    }
}

impl Default for PollEstimates {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CHILD: ShortAddress = ShortAddress(0x2D69);

    #[test]
    fn an_unknown_child_gets_the_default_hold() {
        let estimates = PollEstimates::new();
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), DEFAULT_HOLD_SECS);
    }

    #[test]
    fn a_long_poll_interval_extends_the_hold() {
        let mut estimates = PollEstimates::new();
        estimates.note_poll(CHILD, 1_000_000);
        estimates.note_poll(CHILD, 31_000_000);
        // 30 s + 7.5 s + 1 s.
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), 38);
        // Bounded by the child's End Device Timeout and the global cap.
        assert_eq!(estimates.hold_secs(CHILD, 20), 20);
        estimates.note_poll(CHILD, 131_000_000);
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), MAX_HOLD_SECS);
    }

    #[test]
    fn a_fast_poll_burst_keeps_the_long_poll_hold_for_the_window() {
        let mut estimates = PollEstimates::new();
        estimates.note_poll(CHILD, 0);
        estimates.note_poll(CHILD, 30_000_000);
        // A fast-poll burst (250 ms) after an exchange keeps the hold.
        let mut now = 30_000_000;
        for _ in 0..200 {
            now += 250_000;
            estimates.note_poll(CHILD, now);
        }
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), 38);
        // Polling only fast for a whole window drops back to the default.
        while now < 30_000_000 + WINDOW_US + 1_000_000 {
            now += 250_000;
            estimates.note_poll(CHILD, now);
        }
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), DEFAULT_HOLD_SECS);
    }

    /// A child restored after a reboot is held up to its End Device Timeout
    /// (capped) until it is heard to long-poll; fast polls do not undo the
    /// seed, the first long poll replaces it.
    #[test]
    fn a_restored_child_is_held_until_its_long_poll_is_relearned() {
        let mut estimates = PollEstimates::new();
        estimates.seed(CHILD, 5_000_000);
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), MAX_HOLD_SECS);
        assert_eq!(estimates.hold_secs(CHILD, 20), 20, "End Device Timeout");
        assert_eq!(estimates.hold_secs(CHILD, 2), DEFAULT_HOLD_SECS);
        // First poll 50 s after the restore: no interval measured from the
        // seeding time.
        let mut now = 55_000_000;
        estimates.note_poll(CHILD, now);
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), MAX_HOLD_SECS);
        // A fast-poll burst keeps the seed.
        for _ in 0..20 {
            now += 250_000;
            estimates.note_poll(CHILD, now);
        }
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), MAX_HOLD_SECS);
        // The first 30 s long poll is learned.
        now += 30_000_000;
        estimates.note_poll(CHILD, now);
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), 38);
        // From then on the usual window rules: a burst keeps 38 s.
        now += 250_000;
        estimates.note_poll(CHILD, now);
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), 38);
    }

    #[test]
    fn a_restored_child_that_only_fast_polls_drops_the_seed_after_a_window() {
        let mut estimates = PollEstimates::new();
        estimates.seed(CHILD, 0);
        let mut now = 0;
        while now < WINDOW_US + 1_000_000 {
            now += 1_000_000;
            estimates.note_poll(CHILD, now);
        }
        assert_eq!(estimates.hold_secs(CHILD, 60 << 14), DEFAULT_HOLD_SECS);
    }

    #[test]
    fn a_full_table_replaces_the_least_recent_child() {
        let mut estimates = PollEstimates::new();
        for index in 0..MAX_TRACKED as u16 {
            estimates.note_poll(ShortAddress(index + 1), u32::from(index));
        }
        estimates.note_poll(CHILD, 1_000);
        estimates.note_poll(CHILD, 40_001_000);
        assert!(estimates.hold_secs(CHILD, 60 << 14) > DEFAULT_HOLD_SECS);
        // Child 1 (the least recent) was replaced.
        assert_eq!(
            estimates.hold_secs(ShortAddress(1), 60 << 14),
            DEFAULT_HOLD_SECS
        );
    }
}
