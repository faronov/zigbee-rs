//! Bounded always-on router scheduling policy.

const MAX_RELATIVE_DELAY_US: u32 = 0x7FFF_FFFF;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RouterPolicy {
    /// Longest single MAC receive window.
    pub max_receive_slice_us: u32,
    /// Delay after the first failed fresh commissioning attempt.
    pub join_retry_initial_ms: u32,
    /// Maximum exponential commissioning retry delay.
    pub join_retry_max_ms: u32,
    /// Consecutive secured-rejoin failures before the router discards its
    /// network state and commissions afresh.
    ///
    /// `0` (the default) never self-resets: a commissioned router keeps
    /// retrying the secured rejoin on the `join_retry_initial_ms` ..
    /// `join_retry_max_ms` exponential backoff until its network returns.
    /// Network state is then only discarded by a Leave or an explicit user
    /// factory reset. A nonzero value is an opt-in "give up" limit.
    pub secure_rejoin_failure_limit: u8,
}

impl RouterPolicy {
    pub const DEFAULT: Self = Self {
        max_receive_slice_us: 20_000,
        join_retry_initial_ms: 5_000,
        join_retry_max_ms: 60_000,
        secure_rejoin_failure_limit: 0,
    };

    pub const fn is_valid(&self) -> bool {
        self.max_receive_slice_us > 0
            && self.max_receive_slice_us <= MAX_RELATIVE_DELAY_US
            && self.join_retry_initial_ms > 0
            && self.join_retry_initial_ms <= self.join_retry_max_ms
            && self.join_retry_max_ms <= MAX_RELATIVE_DELAY_US / 1_000
    }

    /// Whether `failures` consecutive secured-rejoin failures exhaust the
    /// opt-in give-up limit.
    pub const fn secure_rejoin_limit_reached(&self, failures: u8) -> bool {
        self.secure_rejoin_failure_limit != 0 && failures >= self.secure_rejoin_failure_limit
    }

    pub(crate) const fn max_relative_delay_us() -> u32 {
        MAX_RELATIVE_DELAY_US
    }
}
