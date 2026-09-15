//! Product-selected lifecycle behavior for the TRÅDFRI SHT3x sensor.

use sensor_sed_app::{ButtonPolicy, ForceReportAction, SensorPolicy, SleepDepth, StatusPolicy};

/// OTA traffic and a fresh coordinator interview use the proven 120-second
/// fast-poll window.
pub const OTA_KEEP_AWAKE_MS: u32 = 120_000;

pub const USER_ACTIONS: ForceReportAction = ForceReportAction;

pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 60_000,
    fast_poll_ms: 250,
    slow_poll_ms: 30_000,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 60_000,
    wake_duration_ms: 250,
    join_retry_ms: 15_000,
    announce_retry_ms: 8_000,
    announce_retries: 5,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    button: ButtonPolicy {
        long_press_ms: Some(3_000),
        debounce_ms: 80,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: 1_000,
        blink_on_ms: 80,
        blink_gap_ms: 120,
        reset_blinks: 5,
        reset_phase_ms: 100,
    },
    // Fast polling retains clocks/radio so coordinator traffic can arrive
    // without an EM2 transition. Only steady-state polling enters EM2.
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Retention,
};

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::power::PowerMode;

    #[test]
    fn policy_preserves_poll_button_and_em2_contracts() {
        assert!(SENSOR_POLICY.is_valid_for_status(true));
        assert_eq!(SENSOR_POLICY.fast_poll_ms, 250);
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 30_000);
        assert_eq!(SENSOR_POLICY.button.debounce_ms, 80);
        assert_eq!(SENSOR_POLICY.button.long_press_ms, Some(3_000));
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Active);
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Retention);
        assert_eq!(
            SENSOR_POLICY.power_mode(),
            PowerMode::Sleepy {
                poll_interval_ms: 30_000,
                wake_duration_ms: 250,
            }
        );
    }

    #[test]
    fn interview_timeout_seam_keeps_the_proven_windows() {
        // SensorApp installs local default reporting eagerly, while remote
        // Configure Reporting remains tracked separately. Its public policy
        // has no separate fallback deadline, so the proven 120 s fresh
        // interview timeout is represented by this finite fast-poll window.
        assert_eq!(SENSOR_POLICY.fresh_join_fast_ms, 120_000);
        assert_eq!(SENSOR_POLICY.restored_fast_ms, 60_000);
        assert_eq!(SENSOR_POLICY.interview_complete_grace_ms, 5_000);
        assert_eq!(OTA_KEEP_AWAKE_MS, SENSOR_POLICY.fresh_join_fast_ms);
    }
}
