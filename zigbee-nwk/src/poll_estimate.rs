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
pub const MAX_TRACKED: usize = crate::ncp_capacity(16, 64);

#[derive(Clone, Copy)]
struct Estimate {
    child: ShortAddress,
    last_poll_us: u32,
    /// Largest poll interval (ms) seen since `max_since_us` (0 = one poll).
    interval_ms: u32,
    max_since_us: u32,
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
            let interval_ms = now_us.wrapping_sub(entry.last_poll_us) / 1_000;
            if interval_ms >= entry.interval_ms
                || now_us.wrapping_sub(entry.max_since_us) >= WINDOW_US
            {
                entry.interval_ms = interval_ms;
                entry.max_since_us = now_us;
            }
            entry.last_poll_us = now_us;
            return;
        }
        let estimate = Estimate {
            child,
            last_poll_us: now_us,
            interval_ms: 0,
            max_since_us: now_us,
        };
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
        let wanted = (interval_ms + interval_ms / 4) / 1_000 + 1;
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
