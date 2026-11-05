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
    /// Failed secure-rejoin attempts after which the device gives up its
    /// network, factory-resets and searches for a new one.
    ///
    /// `0` (recommended, see [`RecoveryPolicy`]) never gives up: a
    /// commissioned sensor keeps its network and retries on the bounded
    /// backoff until the network returns, a user action resets it, or the
    /// network removes it with a Leave.
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
    /// Parent-loss detection and rejoin scheduling.
    pub recovery: RecoveryPolicy,
}

/// How a commissioned sensor notices a dead parent and how it schedules
/// its recovery attempts.
///
/// Every recovery attempt runs the runtime's staged R22/BDB rejoin (secured
/// rejoin on the last channel, then on every channel, then the Trust Center
/// rejoin on a centralized network). Between attempts the device sleeps at
/// [`SensorPolicy::slow_sleep_depth`] with its radio off; the delay starts at
/// [`SensorPolicy::join_retry_ms`], doubles after each failure up to
/// [`join_retry_max_ms`](Self::join_retry_max_ms), and is shortened by a
/// random jitter so a whole network of sensors that lost the same parent does
/// not retry in lock step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecoveryPolicy {
    /// Upper bound of the commissioning/rejoin backoff.
    pub join_retry_max_ms: u32,
    /// Each backoff delay is shortened by a random `0..=pct` percent
    /// (`0` disables jitter; at most 50). Jitter never exceeds the cap.
    pub join_retry_jitter_pct: u8,
    /// Consecutive pieces of parent-loss evidence that start a rejoin while
    /// the device still believes it is joined: acknowledged unicasts that got
    /// no APS ACK after every retry, and pending protocol work that outlived
    /// [`max_active_ms`](Self::max_active_ms). `0` disables this detection
    /// (MAC-level poll failures are still handled by the runtime).
    pub parent_loss_strikes: u8,
    /// Longest time pending protocol work (APS ACK wait, Trust Center link
    /// key exchange, queued responses, a scheduled rejoin) may keep the chip
    /// awake with the radio on. Afterwards the device returns to its normal
    /// poll cadence and sleep depth and counts one parent-loss strike. `0`
    /// leaves pending work unbounded (not recommended for battery devices).
    /// Must not exceed [`MAX_WAIT_MS`].
    pub max_active_ms: u32,
    /// Request an APS acknowledgement on unicast attribute reports so a
    /// parent that acknowledges at the MAC level but delivers nothing end to
    /// end is detected. Costs one extra poll per report.
    pub report_aps_ack: bool,
    /// Longest an OTA session (query, download, verification, activation
    /// wait) may keep the chip at the fast cadence with the radio on. The
    /// stalled session is then aborted through
    /// [`OtaLifecycle::abort`](crate::OtaLifecycle::abort); a later query
    /// may resume it. `0` leaves OTA unbounded.
    pub max_ota_awake_ms: u32,
    /// Longest Identify may keep the fast poll. Afterwards Identify keeps
    /// running at the normal cadence until it ends. `0` leaves it unbounded.
    pub max_identify_awake_ms: u32,
    /// Longest uninterrupted run of timed fast-poll windows (commissioning
    /// and interview grace, Poll Control fast poll, control events). When it
    /// fires the windows are closed. `0` leaves the windows unbounded; a
    /// non-zero value must cover the longest configured window.
    pub max_fast_poll_ms: u32,
    /// Count a missed report as one parent-loss strike: no acknowledged
    /// report within the shortest configured reporting maximum interval
    /// plus this grace. Requires [`report_aps_ack`](Self::report_aps_ack).
    /// `0` disables the detector.
    pub missed_report_grace_ms: u32,
    /// Consecutive failed low-power waits (a sleep transition the platform
    /// reports as recoverable) after which the device stays at
    /// [`SleepDepth::Active`] for the rest of the boot and records
    /// [`AnomalyReason::SleepDisabled`](crate::AnomalyReason::SleepDisabled).
    /// Earlier failures finish their iteration with an active wait. A
    /// non-recoverable failure, or any failure when this is `0`, resets
    /// through the supervisor with the error as the retained reset cause.
    pub sleep_failure_limit: u8,
}

impl RecoveryPolicy {
    /// Recommended battery-sensor recovery: 15-minute backoff cap, 25 %
    /// jitter, rejoin after three strikes, 30-second pending-work bound,
    /// two-hour OTA, two-minute Identify and ten-minute fast-poll bounds,
    /// a two-minute missed-report grace, and sleep disabled for the boot
    /// after three consecutive failed sleep transitions.
    pub const DEFAULT: Self = Self {
        join_retry_max_ms: 15 * 60 * 1_000,
        join_retry_jitter_pct: 25,
        parent_loss_strikes: 3,
        max_active_ms: 30_000,
        report_aps_ack: true,
        max_ota_awake_ms: 2 * 60 * 60 * 1_000,
        max_identify_awake_ms: 2 * 60 * 1_000,
        max_fast_poll_ms: 10 * 60 * 1_000,
        missed_report_grace_ms: 2 * 60 * 1_000,
        sleep_failure_limit: 3,
    };

    pub const fn is_valid(&self) -> bool {
        self.join_retry_max_ms != 0
            && self.join_retry_max_ms <= JOIN_RETRY_MAX_MS
            && self.join_retry_jitter_pct <= MAX_JOIN_RETRY_JITTER_PCT
            && self.max_active_ms <= MAX_WAIT_MS
            && (self.missed_report_grace_ms == 0 || self.report_aps_ack)
    }

    /// `delay_ms` shortened by a share of up to `join_retry_jitter_pct`
    /// percent selected by `random`, a uniform value in `0..=u16::MAX`.
    pub const fn jittered_ms(&self, delay_ms: u32, random: u16) -> u32 {
        let span = (delay_ms as u64 * self.join_retry_jitter_pct as u64) / 100;
        let cut = (span * random as u64) / u16::MAX as u64;
        delay_ms - cut as u32
    }
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
            && self.recovery.is_valid()
            && self.join_retry_ms <= self.recovery.join_retry_max_ms
            && (self.recovery.max_fast_poll_ms == 0
                || (self.recovery.max_fast_poll_ms >= self.fresh_join_fast_ms
                    && self.recovery.max_fast_poll_ms >= self.restored_fast_ms
                    && self.recovery.max_fast_poll_ms >= self.interview_complete_grace_ms))
            && self.announce_retry_ms != 0
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

/// Largest configurable backoff cap (30 minutes); see
/// [`RecoveryPolicy::join_retry_max_ms`].
pub const JOIN_RETRY_MAX_MS: u32 = 30 * 60 * 1_000;

/// Largest configurable backoff jitter in percent.
pub const MAX_JOIN_RETRY_JITTER_PCT: u8 = 50;

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
pub const fn next_join_retry_ms(current_ms: u32, initial_ms: u32, max_ms: u32) -> u32 {
    let doubled = current_ms.saturating_mul(2);
    let capped = if doubled > max_ms { max_ms } else { doubled };
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
    fn join_backoff_doubles_from_the_initial_delay_to_the_configured_cap() {
        let cap = RecoveryPolicy::DEFAULT.join_retry_max_ms;
        let mut delay_ms = 15_000;
        let mut seen = [0u32; 9];
        for slot in &mut seen {
            *slot = delay_ms;
            delay_ms = next_join_retry_ms(delay_ms, 15_000, cap);
        }
        assert_eq!(
            seen,
            [
                15_000, 30_000, 60_000, 120_000, 240_000, 480_000, 900_000, 900_000, 900_000
            ]
        );
        assert_eq!(next_join_retry_ms(u32::MAX, 15_000, cap), cap);
        assert_eq!(next_join_retry_ms(0, 15_000, cap), 15_000);
        assert_eq!(
            next_join_retry_ms(1_200_000, 15_000, JOIN_RETRY_MAX_MS),
            JOIN_RETRY_MAX_MS
        );
    }

    #[test]
    fn jitter_only_shortens_the_delay_within_the_configured_share() {
        let recovery = RecoveryPolicy::DEFAULT;
        let cap = recovery.join_retry_max_ms;
        assert_eq!(recovery.jittered_ms(cap, 0), cap);
        assert_eq!(recovery.jittered_ms(cap, u16::MAX), cap - cap / 4);
        for random in [1u16, 999, 0x7FFF, 0xFFFE] {
            let delay = recovery.jittered_ms(cap, random);
            assert!(delay <= cap && delay >= cap - cap / 4, "{delay}");
        }
        let none = RecoveryPolicy {
            join_retry_jitter_pct: 0,
            ..recovery
        };
        assert_eq!(none.jittered_ms(cap, u16::MAX), cap);
        // No overflow at the largest representable delay.
        let max = RecoveryPolicy {
            join_retry_jitter_pct: MAX_JOIN_RETRY_JITTER_PCT,
            ..recovery
        };
        assert_eq!(max.jittered_ms(u32::MAX, u16::MAX), u32::MAX - u32::MAX / 2);
    }

    #[test]
    fn join_retry_beyond_the_backoff_cap_is_rejected() {
        let mut policy = VALID;
        assert!(policy.is_valid());
        policy.join_retry_ms = policy.recovery.join_retry_max_ms + 1;
        assert!(!policy.is_valid());
    }

    #[test]
    fn recovery_bounds_are_validated() {
        let mut policy = VALID;
        policy.recovery.join_retry_max_ms = JOIN_RETRY_MAX_MS + 1;
        assert!(!policy.is_valid());
        policy = VALID;
        policy.recovery.join_retry_jitter_pct = MAX_JOIN_RETRY_JITTER_PCT + 1;
        assert!(!policy.is_valid());
        policy = VALID;
        policy.recovery.max_active_ms = MAX_WAIT_MS + 1;
        assert!(!policy.is_valid());
        policy = VALID;
        policy.secure_rejoin_failure_limit = 0;
        assert!(policy.is_valid(), "0 means never give the network up");
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
        recovery: RecoveryPolicy::DEFAULT,
    };

    #[test]
    fn configure_reporting_after_completion_keeps_the_short_grace() {
        assert!(!configure_reporting_requests_generic_extension(true));
        assert!(configure_reporting_requests_generic_extension(false));
    }
}
