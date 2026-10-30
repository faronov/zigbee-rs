//! Product-selected environmental SED behavior.

use sensor_sed_app::{ButtonPolicy, NoUserAction, SensorPolicy, SleepDepth, StatusPolicy};

pub const USER_ACTIONS: NoUserAction = NoUserAction;

pub static SENSOR_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 30_000,
    // Fast polling covers join, interview and command windows. Afterwards the
    // device long-polls its parent every 10 s, far below parent child-aging
    // limits. Idle waits and the 10 s long poll were exercised on XT-ZB1
    // hardware (join, interview, reconfigure, leave, resume); sleep current
    // is not yet measured.
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
    // Between polls the M154 radio is quiesced (RX off, state machine idle,
    // RF and calibration retained) and the CPU waits in WFI for the TIMER
    // channel 0 alarm interrupt. PDS/HBN (`Retention`) is not implemented and
    // the wake controller rejects it.
    fast_sleep_depth: SleepDepth::Idle,
    slow_sleep_depth: SleepDepth::Idle,
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn policy_is_valid_for_the_no_status_product() {
        assert!(SENSOR_POLICY.is_valid_for_status(false));
        assert!(!SENSOR_POLICY.is_valid_for_status(true));
        assert_eq!(SENSOR_POLICY.sample_interval_ms, 30_000);
        assert_eq!(SENSOR_POLICY.fast_poll_ms, 250);
        assert_eq!(SENSOR_POLICY.slow_poll_ms, 10_000);
    }

    #[test]
    fn waits_use_radio_quiesced_cpu_idle_never_retention() {
        // PDS/HBN restore is unproven; `Retention` would be rejected.
        assert_eq!(SENSOR_POLICY.fast_sleep_depth, SleepDepth::Idle);
        assert_eq!(SENSOR_POLICY.slow_sleep_depth, SleepDepth::Idle);
    }

    #[test]
    fn long_poll_stays_within_the_shared_wait_bound() {
        assert!(SENSOR_POLICY.slow_poll_ms <= sensor_sed_app::policy::MAX_WAIT_MS);
        assert!(SENSOR_POLICY.fast_poll_ms < SENSOR_POLICY.slow_poll_ms);
    }
}
