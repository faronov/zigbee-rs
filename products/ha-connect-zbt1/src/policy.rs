//! Product-selected bounded scheduling policy for the ZBT-1 always-on End Device.

use router_app::RouterPolicy;

/// No receive operation may monopolize the application future for more than
/// 20 ms. The next slice starts immediately after runtime maintenance.
pub const MAX_RECEIVE_SLICE_US: u32 = 20_000;

pub static ALWAYS_ON_END_DEVICE_POLICY: RouterPolicy = RouterPolicy {
    max_receive_slice_us: MAX_RECEIVE_SLICE_US,
    join_retry_initial_ms: 15_000,
    join_retry_max_ms: 60_000,
    secure_rejoin_failure_limit: 0,
};

/// The ZBT-1 has no button to request commissioning after a Leave (or a
/// coordinator's Remove Device), so Network Steering is re-armed
/// automatically after this hold-off. It outlasts ZHA's default 254 s
/// permit-join window, so a removed device does not immediately rejoin the
/// window that removed it.
pub const RECOMMISSION_AFTER_LEAVE_MS: u64 = 5 * 60 * 1000;

/// ZHA's default permit-join window, in milliseconds.
const DEFAULT_PERMIT_JOIN_WINDOW_MS: u64 = 254 * 1000;
const _: () = assert!(RECOMMISSION_AFTER_LEAVE_MS > DEFAULT_PERMIT_JOIN_WINDOW_MS);

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
        // Never self-reset: network state is discarded only by Leave/user.
        assert_eq!(ALWAYS_ON_END_DEVICE_POLICY.secure_rejoin_failure_limit, 0);
    }
}
