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
    //
    // OTA reachability: 10 s is longer than the parent's indirect hold
    // (6..7.68 s) and the board has no button, so an idle sensor misses the
    // OTA server's Image Notify (`SensorPolicy::ota_reachable` is false).
    // ZHA can start an OTA only inside the fast-poll window after a boot
    // with a restored network (`restored_fast_ms`) or after a join/rejoin
    // (`fresh_join_fast_ms`).
    fast_poll_ms: 250,
    slow_poll_ms: 10_000,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 120_000,
    wake_duration_ms: 500,
    join_retry_ms: 15_000,
    announce_retry_ms: 8_000,
    announce_retries: 5,
    secure_rejoin_failure_limit: 0,
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
    recovery: sensor_sed_app::RecoveryPolicy::DEFAULT,
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

    /// Documented limitation: an idle sensor cannot take an OTA server's
    /// Image Notify; the post-boot and post-join fast-poll windows can.
    #[test]
    fn ota_starts_only_in_the_post_boot_or_join_fast_poll_window() {
        assert!(
            !SENSOR_POLICY
                .ota_reachable(<NoUserAction as sensor_sed_app::UserActionPolicy>::SHORT_PRESS)
        );
        assert!(!SENSOR_POLICY.slow_poll_within_parent_hold());
        assert_eq!(SENSOR_POLICY.restored_fast_ms, 120_000);
        assert_eq!(SENSOR_POLICY.fresh_join_fast_ms, 120_000);
        assert!(SENSOR_POLICY.fast_poll_ms < sensor_sed_app::RESPONSIVE_SLOW_POLL_MS);
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
