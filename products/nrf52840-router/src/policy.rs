//! Product-selected bounded scheduling policy for the always-on End Device.

use router_app::RouterPolicy;

/// No receive operation may monopolize the application future for more than
/// 20 ms. The next slice starts immediately after runtime maintenance.
pub const MAX_RECEIVE_SLICE_US: u32 = 20_000;

pub static ALWAYS_ON_END_DEVICE_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: MAX_RECEIVE_SLICE_US,
    join_retry_initial_ms: 15_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 3,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn always_on_end_device_policy_is_valid_and_keeps_waits_bounded() {
        assert!(ALWAYS_ON_END_DEVICE_POLICY.is_valid());
        assert_eq!(
            ALWAYS_ON_END_DEVICE_POLICY.max_receive_slice_us,
            MAX_RECEIVE_SLICE_US
        );
        assert!(ALWAYS_ON_END_DEVICE_POLICY.max_receive_slice_us <= 20_000);
        assert!(
            ALWAYS_ON_END_DEVICE_POLICY.join_retry_initial_ms
                <= ALWAYS_ON_END_DEVICE_POLICY.join_retry_max_ms
        );
        assert!(ALWAYS_ON_END_DEVICE_POLICY.secure_rejoin_failure_limit > 0);
    }
}
