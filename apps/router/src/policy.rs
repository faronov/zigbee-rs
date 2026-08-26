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
    /// Consecutive secured-rejoin failures before reset and fresh commissioning.
    pub secure_rejoin_failure_limit: u8,
}

impl RouterPolicy {
    pub const DEFAULT: Self = Self {
        max_receive_slice_us: 20_000,
        join_retry_initial_ms: 5_000,
        join_retry_max_ms: 60_000,
        secure_rejoin_failure_limit: 3,
    };

    pub const fn is_valid(&self) -> bool {
        self.max_receive_slice_us > 0
            && self.max_receive_slice_us <= MAX_RELATIVE_DELAY_US
            && self.join_retry_initial_ms > 0
            && self.join_retry_initial_ms <= self.join_retry_max_ms
            && self.join_retry_max_ms <= MAX_RELATIVE_DELAY_US / 1_000
            && self.secure_rejoin_failure_limit > 0
    }

    pub(crate) const fn max_relative_delay_us() -> u32 {
        MAX_RELATIVE_DELAY_US
    }
}
