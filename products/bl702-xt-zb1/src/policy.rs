//! Product-selected environmental SED behavior.

use sensor_sed_app::{ButtonPolicy, NoUserAction, SensorPolicy, SleepDepth, StatusPolicy};

pub const USER_ACTIONS: NoUserAction = NoUserAction;

pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 30_000,
    // The polling-only BL702 path has been proven at a steady 250 ms cadence.
    // Do not claim PDS/HBN or lengthen the parent poll until wake restoration
    // and child-aging behavior have been validated on hardware.
    fast_poll_ms: 250,
    slow_poll_ms: 250,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 120_000,
    wake_duration_ms: 500,
    join_retry_ms: 15_000,
    announce_retry_ms: 8_000,
    announce_retries: 5,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 300,
    },
    // The board exposes no proven fitted status indicator. These deadlines
    // are intentionally absent and compile out with `NoStatus`.
    status: StatusPolicy {
        unjoined_blink_period_ms: 0,
        blink_on_ms: 0,
        blink_gap_ms: 0,
        reset_blinks: 0,
        reset_phase_ms: 0,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Active,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn policy_is_valid_for_the_no_status_active_only_product() {
        assert!(SENSOR_POLICY.is_valid_for_status(false));
        assert!(!SENSOR_POLICY.is_valid_for_status(true));
        assert_eq!(SENSOR_POLICY.fast_poll_ms, 250);
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 250);
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Active);
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Active);
    }
}
