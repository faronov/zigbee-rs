//! Product-selected environmental SED behavior.

use sensor_sed_app::{ButtonPolicy, SensorPolicy, SleepDepth, StatusPolicy, ToggleJoinAction};

pub const USER_ACTIONS: ToggleJoinAction = ToggleJoinAction;

pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 60_000,
    fast_poll_ms: 250,
    slow_poll_ms: 30_000,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 120_000,
    wake_duration_ms: 500,
    join_retry_ms: 15_000,
    announce_retry_ms: 8_000,
    announce_retries: 5,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: Some(3_000),
        debounce_ms: 300,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: 1_000,
        blink_on_ms: 80,
        blink_gap_ms: 120,
        reset_blinks: 5,
        reset_phase_ms: 100,
    },
    // The BRD4181A adapter can only keep the radio active or gate its radio
    // clocks while the Cortex-M executor idles. It has no qualified retention
    // restore path, so neither policy requests Retention.
    fast_sleep_depth: SleepDepth::Idle,
    slow_sleep_depth: SleepDepth::Idle,
};

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_sed_app::{ShortPressAction, UserActionPolicy};

    #[test]
    fn policy_preserves_legacy_cadence_and_toggle_action() {
        assert!(SENSOR_POLICY.is_valid());
        assert_eq!(SENSOR_POLICY.fast_poll_ms, 250);
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 30_000);
        assert_eq!(SENSOR_POLICY.sample_interval_ms, 60_000);
        assert_eq!(
            <ToggleJoinAction as UserActionPolicy>::SHORT_PRESS,
            ShortPressAction::ToggleJoin
        );
    }

    #[test]
    fn policy_never_requests_unqualified_retention() {
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Idle);
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Idle);
        assert_ne!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Retention);
        assert_ne!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Retention);
    }
}
