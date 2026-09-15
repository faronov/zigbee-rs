//! Product-selected sleepy environmental-sensor behavior.

use sensor_sed_app::{ButtonPolicy, SensorPolicy, SleepDepth, StatusPolicy, ToggleJoinAction};

/// BTN1 short press joins when uncommissioned and durably leaves when joined.
/// A three-second hold performs the shared durable factory-reset-and-reboot
/// path.
pub const USER_ACTIONS: ToggleJoinAction = ToggleJoinAction;

pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 30_000,
    fast_poll_ms: 250,
    slow_poll_ms: 10_000,
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
    // The repaired direct-register radio has no proven retention/off restore
    // sequence. Both phases therefore retain active clocks and use only the
    // driver's verified active-operation quiesce transition.
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Active,
};

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_sed_app::{ShortPressAction, UserActionPolicy};

    #[test]
    fn policy_is_valid_and_preserves_manual_polling_cadence() {
        assert!(SENSOR_POLICY.is_valid());
        assert_eq!(SENSOR_POLICY.sample_interval_ms, 30_000);
        assert_eq!(SENSOR_POLICY.fast_poll_ms, 250);
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 10_000);
    }

    #[test]
    fn radio_waits_are_active_only_and_button_is_semantic() {
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Active);
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Active);
        assert_eq!(ToggleJoinAction::SHORT_PRESS, ShortPressAction::ToggleJoin);
    }
}
