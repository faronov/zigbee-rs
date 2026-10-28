//! Lifecycle policy for the battery (CR2032) flower sensor.

use sensor_sed_app::{ButtonPolicy, ForceReportAction, SensorPolicy, SleepDepth, StatusPolicy};

/// SW1 short press: join when uncommissioned, otherwise refresh every
/// measurement and report immediately. A five-second hold performs the
/// shared durable factory reset and reboot (which also re-runs sensor
/// detection, since the device is then factory-new).
pub const USER_ACTIONS: ForceReportAction = ForceReportAction;

pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    // Lifecycle sample tick; [`crate::measurement::CADENCE`] spreads soil
    // (10 min) and battery (1 h) over it.
    sample_interval_ms: 300_000,
    fast_poll_ms: 250,
    // Long poll: well inside every coordinator's end-device timeout.
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
        long_press_ms: Some(5_000),
        debounce_ms: 300,
    },
    status: StatusPolicy {
        unjoined_blink_period_ms: 1_000,
        blink_on_ms: 80,
        blink_gap_ms: 120,
        reset_blinks: 3,
        reset_phase_ms: 150,
    },
    // As on the LaunchPad sensor: idle WFE between fast polls, standby
    // (RTC on LFOSC; the E104-BT55SP has no 32-kHz crystal) between slow
    // polls.
    fast_sleep_depth: SleepDepth::Idle,
    slow_sleep_depth: SleepDepth::Retention,
};

#[cfg(test)]
mod tests {
    use super::*;
    use sensor_sed_app::{ShortPressAction, UserActionPolicy};

    #[test]
    fn policy_is_valid_and_battery_friendly() {
        assert!(SENSOR_POLICY.is_valid());
        assert_eq!(SENSOR_POLICY.sample_interval_ms, 300_000);
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 30_000);
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Retention);
    }

    #[test]
    fn button_matches_the_reference_semantics() {
        assert_eq!(SENSOR_POLICY.button.long_press_ms, Some(5_000));
        assert_eq!(
            ForceReportAction::SHORT_PRESS,
            ShortPressAction::ForceReport
        );
    }

    #[test]
    fn cadence_matches_the_product_sampling_plan() {
        use crate::measurement::CADENCE;
        let interval = SENSOR_POLICY.sample_interval_ms;
        assert_eq!(interval * u32::from(CADENCE.soil_every), 10 * 60_000);
        assert_eq!(interval * u32::from(CADENCE.battery_every), 60 * 60_000);
        assert_eq!(interval * u32::from(CADENCE.ambient_every), 5 * 60_000);
    }
}
