//! Product-selected sleepy-sensor policy and pure deadline arbitration.

use zigbee_runtime::event_loop::TickResult;
use zigbee_runtime::power::PowerMode;

/// What a short user-button press means for this product.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShortPressAction {
    None,
    JoinOnly,
    ForceReport,
    ToggleJoin,
}

pub trait UserActionPolicy {
    const SHORT_PRESS: ShortPressAction;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct NoUserAction;

impl UserActionPolicy for NoUserAction {
    const SHORT_PRESS: ShortPressAction = ShortPressAction::None;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct JoinOnlyAction;

impl UserActionPolicy for JoinOnlyAction {
    const SHORT_PRESS: ShortPressAction = ShortPressAction::JoinOnly;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ForceReportAction;

impl UserActionPolicy for ForceReportAction {
    const SHORT_PRESS: ShortPressAction = ShortPressAction::ForceReport;
}

#[derive(Debug, Default, Clone, Copy)]
pub struct ToggleJoinAction;

impl UserActionPolicy for ToggleJoinAction {
    const SHORT_PRESS: ShortPressAction = ShortPressAction::ToggleJoin;
}

/// Maximum sleep depth a platform may use while waiting between parent polls.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SleepDepth {
    /// Keep clocks and radio ownership active; only await the timer/button.
    Active,
    /// Quiesce the radio and use a light/suspend sleep with retained state.
    Idle,
    /// Use a retention sleep that requires explicit MAC/radio restoration.
    Retention,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ButtonPolicy {
    pub long_press_ms: Option<u32>,
    pub debounce_ms: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusPolicy {
    pub unjoined_blink_period_ms: u32,
    pub blink_on_ms: u32,
    pub blink_gap_ms: u32,
    pub reset_blinks: u8,
    pub reset_phase_ms: u32,
}

/// All product-tunable behavior owned by the environmental SED archetype.
///
/// Durations are bounded `u32` milliseconds so 32-bit MCUs do not pay for
/// repeated `u64` arithmetic. Platform-native rollover is handled by the
/// selected wake controller's mark/elapsed implementation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorPolicy {
    pub sample_interval_ms: u32,
    pub fast_poll_ms: u32,
    pub slow_poll_ms: u32,
    pub fresh_join_fast_ms: u32,
    pub restored_fast_ms: u32,
    pub wake_duration_ms: u32,
    pub join_retry_ms: u32,
    pub announce_retry_ms: u32,
    pub announce_retries: u8,
    pub secure_rejoin_failure_limit: u8,
    pub interview_complete_grace_ms: u32,
    pub button: ButtonPolicy,
    pub status: StatusPolicy,
    /// Wait depth while joined and the fast-poll window is active.
    pub fast_sleep_depth: SleepDepth,
    /// Wait depth while joined in the steady-state slow-poll period.
    pub slow_sleep_depth: SleepDepth,
}

impl SensorPolicy {
    pub const fn is_valid(&self) -> bool {
        self.is_valid_for_status(true)
    }

    /// Validate the policy for a statically known status capability.
    ///
    /// Products without a fitted status indicator do not need meaningful
    /// blink periods because those deadlines and waits are compiled out.
    pub const fn is_valid_for_status(&self, status_present: bool) -> bool {
        self.sample_interval_ms != 0
            && self.fast_poll_ms != 0
            && self.slow_poll_ms != 0
            && self.wake_duration_ms != 0
            && self.join_retry_ms != 0
            && self.announce_retry_ms != 0
            && self.secure_rejoin_failure_limit != 0
            && self.interview_complete_grace_ms != 0
            && self.button.debounce_ms != 0
            && match self.button.long_press_ms {
                Some(duration_ms) => duration_ms != 0,
                None => true,
            }
            && (!status_present
                || (self.status.unjoined_blink_period_ms != 0
                    && self.status.blink_on_ms != 0
                    && self.status.blink_gap_ms != 0
                    && self.status.reset_phase_ms != 0))
    }

    pub const fn power_mode(&self) -> PowerMode {
        PowerMode::Sleepy {
            poll_interval_ms: self.slow_poll_ms,
            wake_duration_ms: self.wake_duration_ms,
        }
    }
}

/// Extract the runtime's requested "run again within this many ms".
pub fn run_again_delay_ms(result: &TickResult) -> Option<u32> {
    match result {
        TickResult::Idle | TickResult::Event(_) => None,
        TickResult::RunAgain(delay_ms) => Some(*delay_ms),
    }
}

/// Remaining time until an interval expires.
pub const fn remaining_ms(elapsed_ms: u32, interval_ms: u32) -> u32 {
    interval_ms.saturating_sub(elapsed_ms)
}

/// Select the next bounded wait from the base parent-poll cadence and every
/// independently scheduled piece of application work.
///
/// Zero means "run as soon as the executor can schedule us" and is converted
/// to one millisecond to avoid an accidental busy loop.
pub fn resolve_wait_delay_ms(base_poll_ms: u32, deadlines_ms: &[Option<u32>]) -> u32 {
    let mut delay_ms = base_poll_ms.max(1);
    for deadline_ms in deadlines_ms.iter().flatten() {
        delay_ms = delay_ms.min((*deadline_ms).max(1));
    }
    delay_ms
}

/// Whether Configure Reporting activity should extend generic fast polling.
pub fn configure_reporting_requests_generic_extension(reporting_complete: bool) -> bool {
    !reporting_complete
}

#[cfg(test)]
mod tests {
    use super::*;
    use zigbee_runtime::event_loop::StackEvent;

    #[test]
    fn idle_and_event_carry_no_run_again_request() {
        assert_eq!(run_again_delay_ms(&TickResult::Idle), None);
        assert_eq!(
            run_again_delay_ms(&TickResult::Event(StackEvent::Left)),
            None
        );
    }

    #[test]
    fn run_again_is_extracted_verbatim() {
        assert_eq!(run_again_delay_ms(&TickResult::RunAgain(42)), Some(42));
    }

    #[test]
    fn every_due_deadline_can_shorten_the_poll_wait() {
        assert_eq!(
            resolve_wait_delay_ms(30_000, &[Some(8_000), Some(60_000), None]),
            8_000
        );
        assert_eq!(resolve_wait_delay_ms(250, &[Some(1_500), Some(8_000)]), 250);
        assert_eq!(resolve_wait_delay_ms(30_000, &[None, None]), 30_000);
    }

    #[test]
    fn a_due_deadline_does_not_busy_loop() {
        assert_eq!(resolve_wait_delay_ms(30_000, &[Some(0)]), 1);
    }

    #[test]
    fn remaining_time_saturates_at_zero() {
        assert_eq!(remaining_ms(400, 1_000), 600);
        assert_eq!(remaining_ms(1_000, 1_000), 0);
        assert_eq!(remaining_ms(1_500, 1_000), 0);
    }

    #[test]
    fn configure_reporting_after_completion_keeps_the_short_grace() {
        assert!(!configure_reporting_requests_generic_extension(true));
        assert!(configure_reporting_requests_generic_extension(false));
    }
}
