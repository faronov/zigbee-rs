//! Sleepy End Device role for the ZBT-1 (feature `sed`).
//!
//! The dongle has no sensor, button or LED, so the SED is an identity-only
//! Basic + Identify node driven by `sensor-sed-app` with no measurement, no
//! battery and no status capability. Its value is proving EM2 sleep, parent
//! polling and RTCC wake on the EFR32MG21.

use sensor_sed_app::{ButtonPolicy, NoUserAction, SensorPolicy, SleepDepth, StatusPolicy};
use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_runtime::profile::{DeviceProfile, RangeExtender};
use zigbee_zcl::DeviceId;

use crate::ENDPOINT;

/// No button is fitted.
pub const USER_ACTIONS: NoUserAction = NoUserAction;

/// Same cadence as the MG22 sensor: 250 ms fast polling with the radio and
/// HFXO live, 30 s slow polling from EM2 (well below the 256 s default
/// child-aging timeout of a ZHA/EmberZNet parent).
pub static SED_POLICY: SensorPolicy = SensorPolicy {
    sample_interval_ms: 60_000,
    fast_poll_ms: 250,
    slow_poll_ms: 30_000,
    fresh_join_fast_ms: 120_000,
    restored_fast_ms: 60_000,
    wake_duration_ms: 500,
    join_retry_ms: 15_000,
    announce_retry_ms: 8_000,
    announce_retries: 5,
    secure_rejoin_failure_limit: 4,
    interview_complete_grace_ms: 5_000,
    // Unused (no button), but must be valid.
    button: ButtonPolicy {
        long_press_ms: None,
        debounce_ms: 300,
    },
    // Unused (no status indicator; the waits are compiled out).
    status: StatusPolicy {
        unjoined_blink_period_ms: 0,
        blink_on_ms: 0,
        blink_gap_ms: 0,
        reset_blinks: 0,
        reset_phase_ms: 0,
    },
    fast_sleep_depth: SleepDepth::Active,
    slow_sleep_depth: SleepDepth::Retention,
};

pub type SedProfile = DeviceProfile<RangeExtender>;

/// Basic + Identify with the Simple Sensor device ID (not a router type).
pub const fn sed_profile() -> SedProfile {
    DeviceProfile::new(
        ENDPOINT,
        PROFILE_HOME_AUTOMATION,
        DeviceId::SIMPLE_SENSOR,
        RangeExtender,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sed_policy_is_valid_without_a_status_indicator() {
        assert!(SED_POLICY.is_valid_for_status(false));
        assert!(SED_POLICY.slow_poll_ms < 256_000);
        assert_eq!(SED_POLICY.slow_sleep_depth, SleepDepth::Retention);
    }
}
