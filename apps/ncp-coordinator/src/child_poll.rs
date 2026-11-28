//! `EVT_CHILD_POLL` rate limiting (spec 7.2).
//!
//! Every MAC Data Request of a child of the NCP reaches the dispatcher
//! ([`crate::port::CoordinatorPort::take_child_poll`]). While a host that set
//! `host_caps` `CHILD_POLL_EVENTS` is negotiated, [`PollLimiter`] decides
//! which of them become events:
//!
//! * per child, at most one report per `child_poll_interval_ms` (default
//!   [`child_poll::DEFAULT_INTERVAL_MS`]); a child's first poll the limiter
//!   tracks, and a poll whose ACK Frame Pending differs from the child's
//!   last report, are reported at once. Polls in between are coalesced and
//!   counted into the next report's `unreported`;
//! * globally, a token bucket of [`GLOBAL_BURST`] reports refilled with one
//!   token every [`GLOBAL_REFILL_MS`] (10 reports/s sustained, about 350 B/s
//!   of a 115 200 baud link). A due poll without a token, or without room in
//!   the ZIGBEE queue, is dropped and the child's next poll is due again.
//!
//! Memory is fixed: [`TRACKED`] children (the least recently reported one is
//! replaced; its next poll counts as a first poll, still bounded by the
//! global bucket).

use heapless::Vec;
use zigbee_zrhp::msg::network::{AckFramePending, child_poll};

/// Children tracked for the per-child interval.
pub const TRACKED: usize = [32, 16][cfg!(feature = "compact") as usize];
/// Reports the global bucket holds.
pub const GLOBAL_BURST: u8 = 16;
/// One global token per this many milliseconds.
pub const GLOBAL_REFILL_MS: u32 = 100;

#[derive(Debug, Clone, Copy)]
struct Tracked {
    nwk: u16,
    /// Last report (ms), or when tracking started.
    last_ms: u32,
    /// Frame Pending of the last report (`None`: not reported yet).
    reported: Option<AckFramePending>,
    unreported: u16,
}

/// What became of one poll.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Admit {
    /// Send `EVT_CHILD_POLL` with this `unreported` count.
    Report { unreported: u16 },
    /// Within the child's interval, Frame Pending unchanged.
    Coalesced,
    /// Due, but no global token or no queue room.
    Dropped,
}

/// See the module documentation.
#[derive(Debug)]
pub struct PollLimiter {
    enabled: bool,
    interval_ms: u16,
    tracked: Vec<Tracked, TRACKED>,
    tokens: u8,
    refill_ms: u32,
}

impl Default for PollLimiter {
    fn default() -> Self {
        Self::new()
    }
}

impl PollLimiter {
    pub const fn new() -> Self {
        Self {
            enabled: false,
            interval_ms: child_poll::DEFAULT_INTERVAL_MS,
            tracked: Vec::new(),
            tokens: GLOBAL_BURST,
            refill_ms: 0,
        }
    }

    /// The negotiated host consumes `EVT_CHILD_POLL`.
    pub const fn enabled(&self) -> bool {
        self.enabled
    }

    /// A new negotiation: enable or disable, forget every child, full
    /// bucket. `child_poll_interval_ms` is kept (volatile until reboot).
    pub fn restart(&mut self, enabled: bool, now_ms: u32) {
        self.enabled = enabled;
        self.tracked.clear();
        self.tokens = GLOBAL_BURST;
        self.refill_ms = now_ms;
    }

    /// `child_poll_interval_ms`.
    pub const fn interval_ms(&self) -> u16 {
        self.interval_ms
    }

    /// Set `child_poll_interval_ms`; `false` outside
    /// [`child_poll::MIN_INTERVAL_MS`]..=[`child_poll::MAX_INTERVAL_MS`].
    pub fn set_interval_ms(&mut self, interval_ms: u16) -> bool {
        let valid =
            (child_poll::MIN_INTERVAL_MS..=child_poll::MAX_INTERVAL_MS).contains(&interval_ms);
        if valid {
            self.interval_ms = interval_ms;
        }
        valid
    }

    /// Decide one poll of `nwk` whose ACK carried `ack`. `room`: the event
    /// queue can take the report.
    pub fn admit(&mut self, nwk: u16, ack: AckFramePending, now_ms: u32, room: bool) -> Admit {
        let interval = u32::from(self.interval_ms);
        let index = self.track(nwk, now_ms);
        let entry = &mut self.tracked[index];
        let due = match entry.reported {
            None => true,
            Some(reported) => reported != ack || now_ms.wrapping_sub(entry.last_ms) >= interval,
        };
        if !due {
            entry.unreported = entry.unreported.saturating_add(1);
            return Admit::Coalesced;
        }
        if !room || !Self::take_token(&mut self.tokens, &mut self.refill_ms, now_ms) {
            entry.unreported = entry.unreported.saturating_add(1);
            return Admit::Dropped;
        }
        entry.last_ms = now_ms;
        entry.reported = Some(ack);
        Admit::Report {
            unreported: core::mem::take(&mut entry.unreported),
        }
    }

    /// The entry of `nwk`, replacing the least recently reported one when
    /// every slot is taken.
    fn track(&mut self, nwk: u16, now_ms: u32) -> usize {
        if let Some(index) = self.tracked.iter().position(|entry| entry.nwk == nwk) {
            return index;
        }
        let entry = Tracked {
            nwk,
            last_ms: now_ms,
            reported: None,
            unreported: 0,
        };
        if self.tracked.push(entry).is_ok() {
            return self.tracked.len() - 1;
        }
        let oldest = self
            .tracked
            .iter()
            .enumerate()
            .max_by_key(|(_, entry)| now_ms.wrapping_sub(entry.last_ms))
            .map_or(0, |(index, _)| index);
        self.tracked[oldest] = entry;
        oldest
    }

    fn take_token(tokens: &mut u8, refill_ms: &mut u32, now_ms: u32) -> bool {
        let earned = now_ms.wrapping_sub(*refill_ms) / GLOBAL_REFILL_MS;
        if earned != 0 {
            let total = u32::from(*tokens).saturating_add(earned);
            if total >= u32::from(GLOBAL_BURST) {
                *tokens = GLOBAL_BURST;
                *refill_ms = now_ms;
            } else {
                *tokens = total as u8;
                *refill_ms = refill_ms.wrapping_add(earned * GLOBAL_REFILL_MS);
            }
        }
        if *tokens == 0 {
            return false;
        }
        *tokens -= 1;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SET: AckFramePending = AckFramePending::Set;

    #[test]
    fn the_bucket_refills_one_report_per_period() {
        let mut limiter = PollLimiter::new();
        limiter.restart(true, 0);
        let mut nwk = 0u16;
        let mut admit = |limiter: &mut PollLimiter, now_ms| {
            nwk += 1;
            limiter.admit(nwk, SET, now_ms, true)
        };
        for _ in 0..GLOBAL_BURST {
            assert!(matches!(admit(&mut limiter, 0), Admit::Report { .. }));
        }
        assert_eq!(admit(&mut limiter, GLOBAL_REFILL_MS - 1), Admit::Dropped);
        assert!(matches!(
            admit(&mut limiter, GLOBAL_REFILL_MS),
            Admit::Report { .. }
        ));
        assert_eq!(admit(&mut limiter, GLOBAL_REFILL_MS + 50), Admit::Dropped);
        // Idle time refills up to the burst, never beyond.
        for _ in 0..GLOBAL_BURST {
            assert!(matches!(admit(&mut limiter, 60_000), Admit::Report { .. }));
        }
        assert_eq!(admit(&mut limiter, 60_000), Admit::Dropped);
    }

    #[test]
    fn the_least_recently_reported_child_is_replaced() {
        let mut limiter = PollLimiter::new();
        limiter.restart(true, 0);
        assert!(limiter.set_interval_ms(child_poll::MAX_INTERVAL_MS));
        for nwk in 0..TRACKED as u16 {
            let now_ms = u32::from(nwk) * GLOBAL_REFILL_MS;
            assert!(matches!(
                limiter.admit(nwk, SET, now_ms, true),
                Admit::Report { .. }
            ));
        }
        let now_ms = TRACKED as u32 * GLOBAL_REFILL_MS;
        // A new child replaces child 0 (the oldest report)...
        assert!(matches!(
            limiter.admit(0xBEEF, SET, now_ms, true),
            Admit::Report { .. }
        ));
        // ...so child 0 counts as new again, while child 1 is still limited.
        assert_eq!(limiter.admit(1, SET, now_ms + 100, true), Admit::Coalesced);
        assert!(matches!(
            limiter.admit(0, SET, now_ms + 200, true),
            Admit::Report { unreported: 0 }
        ));
    }
}
