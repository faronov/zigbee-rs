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
    /// Retained for source compatibility; no longer scheduled.
    ///
    /// An unjoined sensor sleeps between commissioning attempts and shows
    /// the double blink only when an attempt actually starts, so there is no
    /// periodic idle blink to configure.
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
    /// First delay of the unjoined commissioning backoff.
    ///
    /// Each failed join, resume, or secure-rejoin attempt doubles the next
    /// delay up to [`JOIN_RETRY_MAX_MS`]; a successful join or a user
    /// button press restores this initial delay.
    pub join_retry_ms: u32,
    pub announce_retry_ms: u32,
    pub announce_retries: u8,
    pub secure_rejoin_failure_limit: u8,
    pub interview_complete_grace_ms: u32,
    pub button: ButtonPolicy,
    pub status: StatusPolicy,
    /// Wait depth while joined and the fast-poll window is active.
    pub fast_sleep_depth: SleepDepth,
    /// Wait depth while joined in the steady-state slow-poll period, and
    /// while unjoined between commissioning attempts or while waiting for
    /// the user button after an explicit leave.
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
            && (self.slow_poll_ms <= MAX_WAIT_MS || self.sample_interval_ms <= MAX_WAIT_MS)
            && self.wake_duration_ms != 0
            && self.join_retry_ms != 0
            && self.join_retry_ms <= JOIN_RETRY_MAX_MS
            && self.announce_retry_ms != 0
            && self.secure_rejoin_failure_limit != 0
            && self.interview_complete_grace_ms != 0
            && self.button.debounce_ms != 0
            && match self.button.long_press_ms {
                Some(duration_ms) => duration_ms != 0,
                None => true,
            }
            && (!status_present
                || (self.status.blink_on_ms != 0
                    && self.status.blink_gap_ms != 0
                    && self.status.reset_phase_ms != 0))
    }

    /// Whether the joined slow poll fetches a frame that the parent queued
    /// just after the previous poll before a common parent expires it.
    ///
    /// When true, an unsolicited command (Identify, attribute read/write,
    /// Configure Reporting, bind) reaches the device on its first
    /// transmission. When false, delivery relies on the sender's APS
    /// retries, and a command sent between polls may be lost.
    pub const fn slow_poll_within_parent_hold(&self) -> bool {
        self.slow_poll_ms <= RESPONSIVE_SLOW_POLL_MS
    }

    pub const fn power_mode(&self) -> PowerMode {
        PowerMode::Sleepy {
            poll_interval_ms: self.slow_poll_ms,
            wake_duration_ms: self.wake_duration_ms,
        }
    }
}

/// Shortest time a common Zigbee parent keeps an indirect frame for a sleepy
/// child before discarding it.
///
/// The IEEE 802.15.4 default `macTransactionPersistenceTime` (0x01F4 ×
/// 15.36 ms = 7.68 s) is used by EmberZNet, ZBOSS and this stack's
/// `SoftMacCore` parent. Z-Stack holds an indirect frame for
/// `NWK_INDIRECT_MSG_TIMEOUT` = 7 s, aged on a one-second tick, so a frame
/// can expire after about 6 s.
pub const PARENT_INDIRECT_HOLD_MIN_MS: u32 = 6_000;

/// Part of [`PARENT_INDIRECT_HOLD_MIN_MS`] reserved for the awake service
/// time between two polls and for wake-timer drift (the poll-to-poll gap is
/// the slow poll plus the service iteration, measured on an RC or crystal
/// sleep clock).
pub const PARENT_INDIRECT_HOLD_MARGIN_MS: u32 = 1_000;

/// Longest joined slow poll that keeps a sleepy sensor commandable on the
/// first transmission behind any common parent (5 s).
pub const RESPONSIVE_SLOW_POLL_MS: u32 =
    PARENT_INDIRECT_HOLD_MIN_MS - PARENT_INDIRECT_HOLD_MARGIN_MS;

// The shortest hold must not exceed the 802.15.4 default
// macTransactionPersistenceTime (0x01F4 unit backoff periods of 15.36 ms).
const _: () = assert!(PARENT_INDIRECT_HOLD_MIN_MS <= 0x01F4 * 15_360 / 1_000);
const _: () = assert!(PARENT_INDIRECT_HOLD_MARGIN_MS < PARENT_INDIRECT_HOLD_MIN_MS);

/// Upper bound of the unjoined commissioning backoff (30 minutes).
pub const JOIN_RETRY_MAX_MS: u32 = 30 * 60 * 1_000;

/// Longest single platform wait.
///
/// Unjoined idle periods (backoff or waiting for the user button) are split
/// into chunks of at most this length and the measured time of each chunk is
/// accumulated. Joined waits are bounded by `min(slow_poll_ms,
/// sample_interval_ms)`, which [`SensorPolicy::is_valid`] limits to this
/// value. Together this keeps every wait and elapsed-time comparison inside
/// the narrowest supported platform counter (TLSR8258 Timer0 at 24 MHz wraps
/// after about 179 seconds). The supervisor may bound each wait further.
pub const MAX_WAIT_MS: u32 = 60_000;

/// Delay to apply after the next failed attempt, given the delay just used.
pub const fn next_join_retry_ms(current_ms: u32, initial_ms: u32) -> u32 {
    let doubled = current_ms.saturating_mul(2);
    let capped = if doubled > JOIN_RETRY_MAX_MS {
        JOIN_RETRY_MAX_MS
    } else {
        doubled
    };
    if capped < initial_ms {
        initial_ms
    } else {
        capped
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
    fn join_backoff_doubles_from_the_initial_delay_to_the_thirty_minute_cap() {
        let mut delay_ms = 15_000;
        let mut seen = [0u32; 10];
        for slot in &mut seen {
            *slot = delay_ms;
            delay_ms = next_join_retry_ms(delay_ms, 15_000);
        }
        assert_eq!(
            seen,
            [
                15_000, 30_000, 60_000, 120_000, 240_000, 480_000, 960_000, 1_800_000, 1_800_000,
                1_800_000
            ]
        );
        assert_eq!(next_join_retry_ms(u32::MAX, 15_000), JOIN_RETRY_MAX_MS);
        assert_eq!(next_join_retry_ms(0, 15_000), 15_000);
    }

    #[test]
    fn join_retry_beyond_the_backoff_cap_is_rejected() {
        let mut policy = VALID;
        assert!(policy.is_valid());
        policy.join_retry_ms = JOIN_RETRY_MAX_MS + 1;
        assert!(!policy.is_valid());
    }

    #[test]
    fn joined_wait_bound_must_fit_one_platform_wait() {
        let mut policy = VALID;
        policy.slow_poll_ms = MAX_WAIT_MS + 1;
        policy.sample_interval_ms = MAX_WAIT_MS;
        assert!(policy.is_valid(), "the sample interval bounds the wait");
        policy.sample_interval_ms = MAX_WAIT_MS + 1;
        assert!(!policy.is_valid());
        policy.slow_poll_ms = MAX_WAIT_MS;
        assert!(policy.is_valid(), "the slow poll bounds the wait");
    }

    #[test]
    fn responsive_slow_poll_leaves_margin_below_the_shortest_parent_hold() {
        assert_eq!(RESPONSIVE_SLOW_POLL_MS, 5_000);

        let mut policy = VALID;
        policy.slow_poll_ms = RESPONSIVE_SLOW_POLL_MS;
        assert!(policy.slow_poll_within_parent_hold());
        policy.slow_poll_ms = RESPONSIVE_SLOW_POLL_MS + 1;
        assert!(!policy.slow_poll_within_parent_hold());
    }

    const VALID: SensorPolicy = SensorPolicy {
        sample_interval_ms: 1,
        fast_poll_ms: 1,
        slow_poll_ms: 1,
        fresh_join_fast_ms: 1,
        restored_fast_ms: 1,
        wake_duration_ms: 1,
        join_retry_ms: 15_000,
        announce_retry_ms: 1,
        announce_retries: 0,
        secure_rejoin_failure_limit: 1,
        interview_complete_grace_ms: 1,
        button: ButtonPolicy {
            long_press_ms: None,
            debounce_ms: 1,
        },
        status: StatusPolicy {
            unjoined_blink_period_ms: 0,
            blink_on_ms: 1,
            blink_gap_ms: 1,
            reset_blinks: 0,
            reset_phase_ms: 1,
        },
        fast_sleep_depth: SleepDepth::Active,
        slow_sleep_depth: SleepDepth::Active,
    };

    #[test]
    fn configure_reporting_after_completion_keeps_the_short_grace() {
        assert!(!configure_reporting_requests_generic_extension(true));
        assert!(configure_reporting_requests_generic_extension(false));
    }
}
