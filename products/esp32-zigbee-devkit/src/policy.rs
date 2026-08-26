//! Shared sleepy-sensor application policy for both ESP32 devkits.
//!
//! The current platform wait implementation is deliberately active-only:
//! neither the polling Embassy time driver nor the ESP MAC backend exposes the
//! atomic radio quiesce/restore transition required for idle or retention
//! sleep. Both policy depths therefore remain [`SleepDepth::Active`].

use sensor_sed_app::{ButtonPolicy, SensorPolicy, SleepDepth, StatusPolicy};

/// Fast-poll extension applied whenever OTA traffic is consumed.
pub const OTA_KEEP_AWAKE_MS: u32 = 120_000;

/// Product policy shared by the ESP32-H2 and ESP32-C6 sensor compositions.
pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 60_000,
    fast_poll_ms: 250,
    slow_poll_ms: 30_000,
    fresh_join_fast_ms: OTA_KEEP_AWAKE_MS,
    restored_fast_ms: 60_000,
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
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Active,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sensor_policy_is_valid_with_and_without_status_hardware() {
        assert!(SENSOR_POLICY.is_valid_for_status(true));
        assert!(SENSOR_POLICY.is_valid_for_status(false));
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Active);
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Active);
    }
}
