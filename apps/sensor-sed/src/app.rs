//! Platform-independent sleepy-sensor application state machine.
//!
//! Extracted without changing behavior from the nRF sensor lifecycle: the
//! same bounded (four-round) MAC
//! receive/poll window, the same two-level fast/slow polling with a
//! post-join fast-poll window ended early by a real `ConfigureReporting`
//! from the coordinator, the same Device_annce retry policy (skipped only
//! for the very first cold-boot silent resume), the same sensor/report cadence, button
//! behavior, and durable checkpointing at the same points in the
//! lifecycle.
//!
//! The chip/board/product couplings are static capabilities bound by the
//! composition root:
//!
//! | was | now |
//! |-----|-----|
//! | `nrf52840_sensor_product::storage::SecurityStore` | `S: SecurityStateStore` |
//! | `nrf52840_sensor_product::profile::SensorProfile` | `P: SensorProfile` |
//! | `nrf52840_sensor_product::ENDPOINT` | `profile.endpoint()` (same value) |
//! | nRF SAADC + product battery curve | `B: BatterySource` |
//! | `Temp` / `crate::sensor::Sensor` (cfg-selected) | `E: SensorMeasurement<P>` (any `EnvironmentSource`) |
//! | Embassy clock/button + radio sleep | `W: WakeController<M>` |
//! | LED and reset/watchdog | `St: StatusSink`, `Sv: Supervisor` |
//! | `defmt` calls | `D: Diagnostics` |
//!
//! Every capability is monomorphized. This crate has no platform `cfg`, HAL
//! dependency, allocator, or trait object.
//!
//! Three behaviors are new relative to the original `main.rs`:
//!
//! - Every [`StackEvent`] variant is now matched explicitly in the internal
//!   control-event handler, instead of a
//!   wildcard arm that silently dropped anything not already special-cased.
//!   `BasicResetToFactoryDefaults` is the Basic cluster Reset to Factory Defaults
//!   notification, so it preserves the network and security state; the Basic
//!   cluster has already reset its writable attributes before the event is
//!   emitted. This also unifies event handling between the two places it used
//!   to be split across (after `process_incoming`, and after the periodic
//!   `tick()`) — the periodic-tick path previously never actually acted on
//!   `LeaveRequested`/`RejoinRequested`, only logged them.
//! - [`TickResult::RunAgain`] now records a rollover-safe mark + duration and shortens
//!   the next poll/sleep wait (see `crate::policy`) instead of being
//!   discarded. Runtime elapsed time is tracked independently from the
//!   sensor-report cadence, so those additional wakeups cannot advance stack
//!   timers faster than wall clock.
//! - Secure-rejoin failures are counted once per boot-time, direct, or
//!   runtime-driven attempt. A commissioned device never gives its network
//!   up on its own unless the product sets a nonzero
//!   `secure_rejoin_failure_limit`; it retries on the bounded, jittered
//!   backoff of [`crate::RecoveryPolicy`] and sleeps between attempts.
//! - Parent loss that MAC acknowledgements cannot reveal (a parent that
//!   acknowledges polls but delivers nothing) is detected from APS
//!   end-to-end failures and from pending protocol work that outlives
//!   `RecoveryPolicy::max_active_ms`; either starts the staged rejoin. A
//!   report that is due by its maximum interval but was not acknowledged
//!   counts as the same evidence.
//! - Every keep-awake reason (pending work, OTA, Identify, timed fast-poll
//!   windows) is bounded by its own `RecoveryPolicy` limit; a fired limit
//!   forces the reason to release the chip and is recorded in the
//!   [`AnomalyRecord`] a product may persist and expose.
//! - Construction rejects a device whose runtime automatic parent polling is
//!   still enabled, so the bounded manual poll window has exactly one owner.

use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::role::EndDevice;
use zigbee_runtime::security_store::SecurityStateStore;
use zigbee_zcl::ClusterId;
use zigbee_zcl::clusters::poll_control::{CMD_CHECK_IN, FastPollEvent};
use zigbee_zcl::foundation::reporting::ReportDirection;

use crate::anomaly::{AnomalyReason, AnomalyRecord, AwakeBound, BoundStep};
use crate::battery::BatterySource;
use crate::capabilities::{
    SensorStatus, StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use crate::diagnostics::{DiagnosticEvent, Diagnostics};
use crate::measurement::{SampleTrigger, SensorMeasurement, SensorProfile};
use crate::ota::{OtaActivationOutcome, OtaEventOutcome, OtaLifecycle, is_ota_event};
use crate::parts::{SensorSedParts, SensorSedResources};
use crate::policy::{self, SensorPolicy, ShortPressAction, SleepDepth, UserActionPolicy};
use crate::reset_cause::ResetCause;

type SensorNode<'a, M, S, P> = ZigbeeNode<'a, M, S, P, EndDevice>;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorAppError {
    AutomaticPollingEnabled,
    InvalidPolicy,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SensorLifecycleError {
    AlreadyInitialized,
    NotInitialized,
}

/// Starting BDB Finding & Binding target mode was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FindingBindingTargetError {
    Rejected,
}

pub struct SensorApp<'a, M, S, P, R>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: SensorProfile,
    R: SensorSedResources<M>,
{
    node: SensorNode<'a, M, S, P>,
    policy: &'static SensorPolicy,
    resources: R,
    /// Cached `profile.endpoint()`. Identical to the product's `ENDPOINT`
    /// constant this used to import; read once so the hot paths do not
    /// re-borrow the profile just to name the endpoint.
    endpoint: u8,
    last_report: R::Mark,
    last_tick: R::Mark,
    /// Timed fast-poll window `(start, duration)`. Cleared as soon as it
    /// expires so a narrow platform counter cannot make an old start mark
    /// look recent again after rollover.
    fast_poll: Option<(R::Mark, u32)>,
    /// Last bounded MAC poll window; `None` until the first one runs.
    last_poll: Option<R::Mark>,
    /// Delay applied after the next failed unjoined commissioning attempt.
    join_backoff_ms: u32,
    /// Measured time still to sleep before the next unjoined attempt.
    ///
    /// Decremented by the measured duration of each bounded wait instead of
    /// comparing against a long-lived mark, so a 30-minute backoff remains
    /// correct on platforms whose native counter wraps in minutes.
    join_wait_ms: u32,
    /// Factory-new after an explicit leave: do not search for a network
    /// until the user presses the commissioning button.
    awaiting_button: bool,
    /// Whether the previous service iteration observed a joined network.
    network_up: bool,
    rejoin_count: u8,
    annce_retries_left: u8,
    last_annce: R::Mark,
    was_fast_polling: bool,
    was_identifying: bool,
    identify_phase_on: bool,
    interview_done: bool,
    consecutive_rejoin_failures: u8,
    run_again: Option<(R::Mark, u32)>,
    /// Keep-awake budgets: pending protocol work, OTA, Identify and the
    /// timed fast-poll windows.
    pending_bound: AwakeBound,
    ota_bound: AwakeBound,
    identify_bound: AwakeBound,
    fast_bound: AwakeBound,
    /// When the budgets were last advanced (once per joined wait request).
    last_bound_check: R::Mark,
    /// Parent-loss strikes from bounded pending work and missed reports
    /// since the last APS ACK.
    active_bound_strikes: u8,
    /// Measured time since the last end-to-end proof (an APS ACK).
    report_proof_ms: u32,
    /// APS ACK table state at the previous budget check, to recognise an
    /// acknowledgement between two checks.
    prev_ack_pending: bool,
    prev_ack_failures: u8,
    /// Anomalies recorded since boot (plus any the product restored), and
    /// whether the product has not yet seen the latest change.
    anomalies: AnomalyRecord,
    anomaly_dirty: bool,
    /// Consecutive recoverable sleep-transition failures.
    sleep_failures: u8,
    /// Low-power waits are disabled for the rest of this boot.
    sleep_disabled: bool,
    /// Runtime APS end-to-end failure count seen at the previous check.
    seen_aps_ack_failures: u8,
    /// Joined poll intervals in force: the policy's, or the profile's Poll
    /// Control LongPollInterval/ShortPollInterval when one is fitted.
    slow_poll_ms: u32,
    fast_poll_ms: u32,
    /// Poll Control fast-poll window `(start, duration)`: waiting for a
    /// Check-in Response, or a client's requested fast-poll period.
    poll_control_fast: Option<(R::Mark, u32)>,
    /// Measured time since the last Check-in, accumulated per service like
    /// [`join_wait_ms`](Self::join_wait_ms) so an hour-long CheckInInterval
    /// survives narrow platform counters.
    check_in_elapsed_ms: u32,
    last_poll_control: R::Mark,
    initialized: bool,
}

impl<'a, M, S, P, W, St, E, B, O, A, Sv, D>
    SensorApp<'a, M, S, P, SensorSedParts<W, St, E, B, O, A, Sv, D>>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: SensorProfile,
    W: WakeController<M>,
    St: StatusSink,
    E: SensorMeasurement<P>,
    B: BatterySource,
    O: OtaLifecycle<M, S, P>,
    A: UserActionPolicy,
    Sv: Supervisor,
    D: Diagnostics,
{
    pub fn new(
        node: SensorNode<'a, M, S, P>,
        policy: &'static SensorPolicy,
        parts: SensorSedParts<W, St, E, B, O, A, Sv, D>,
    ) -> Result<Self, SensorAppError> {
        if node.device().automatic_polling_enabled() {
            return Err(SensorAppError::AutomaticPollingEnabled);
        }
        if !policy.is_valid_for_status(St::PRESENT) {
            return Err(SensorAppError::InvalidPolicy);
        }

        let mut node = node;
        node.device_mut()
            .set_report_aps_ack(policy.recovery.report_aps_ack);
        let now = parts.wake.mark();
        let endpoint = node.profile().endpoint();
        Ok(Self {
            node,
            policy,
            resources: parts,
            endpoint,
            last_report: now,
            last_tick: now,
            fast_poll: None,
            last_poll: None,
            join_backoff_ms: policy.join_retry_ms,
            join_wait_ms: 0,
            awaiting_button: false,
            network_up: false,
            rejoin_count: 0,
            annce_retries_left: 0,
            last_annce: now,
            was_fast_polling: false,
            was_identifying: false,
            identify_phase_on: false,
            interview_done: false,
            consecutive_rejoin_failures: 0,
            run_again: None,
            pending_bound: AwakeBound::default(),
            ota_bound: AwakeBound::default(),
            identify_bound: AwakeBound::default(),
            fast_bound: AwakeBound::default(),
            last_bound_check: now,
            active_bound_strikes: 0,
            report_proof_ms: 0,
            prev_ack_pending: false,
            prev_ack_failures: 0,
            anomalies: AnomalyRecord::EMPTY,
            anomaly_dirty: false,
            sleep_failures: 0,
            sleep_disabled: false,
            seen_aps_ack_failures: 0,
            slow_poll_ms: policy.slow_poll_ms,
            fast_poll_ms: policy.fast_poll_ms,
            poll_control_fast: None,
            check_in_elapsed_ms: 0,
            last_poll_control: now,
            initialized: false,
        })
    }

    /// Anomalies recorded so far (see [`crate::anomaly`]).
    pub const fn anomalies(&self) -> &AnomalyRecord {
        &self.anomalies
    }

    /// Seed the anomaly counters from a product's persistent copy, before
    /// [`initialize`](Self::initialize).
    pub fn restore_anomalies(&mut self, record: AnomalyRecord) {
        self.anomalies = record;
        self.anomaly_dirty = false;
    }

    /// The anomaly record once after each change, for a product that
    /// persists or publishes it. A product that does neither never calls it.
    pub fn take_anomaly_update(&mut self) -> Option<AnomalyRecord> {
        if core::mem::take(&mut self.anomaly_dirty) {
            Some(self.anomalies)
        } else {
            None
        }
    }

    fn note_anomaly(&mut self, reason: AnomalyReason) {
        self.anomalies.note(reason);
        self.anomaly_noted(reason);
    }

    fn anomaly_noted(&mut self, reason: AnomalyReason) {
        self.anomaly_dirty = true;
        self.resources.diagnostics.record(DiagnosticEvent::Anomaly {
            reason,
            total: self.anomalies.total,
        });
    }

    /// Note the previous boot's unplanned reset, carried by the product (see
    /// [`crate::reset_cause`]), after
    /// [`restore_anomalies`](Self::restore_anomalies). A cause equal to the
    /// newest recorded one is not noted again, so a reset loop changes the
    /// record (and the product's flash copy) once per distinct cause.
    pub fn note_reset(&mut self, cause: ResetCause) {
        if self.anomalies.note_reset(cause.word()) {
            self.anomaly_noted(AnomalyReason::Reset);
        }
    }

    /// Keep every remaining wait of this boot at [`SleepDepth::Active`], for
    /// a product whose rapid-reset guard tripped. `record` notes
    /// [`AnomalyReason::SleepDisabled`] with `cause` (the carried reset
    /// cause, if any); pass `false` when this burst was already recorded.
    pub fn disable_sleep(&mut self, record: bool, cause: Option<ResetCause>) {
        self.sleep_disabled = true;
        if record {
            self.anomalies.note_with_cause(
                AnomalyReason::SleepDisabled,
                cause.map_or(0, ResetCause::word),
            );
            self.anomaly_noted(AnomalyReason::SleepDisabled);
        }
    }

    /// Whether low-power waits are disabled for the rest of this boot.
    pub const fn sleep_disabled(&self) -> bool {
        self.sleep_disabled
    }

    /// The Zigbee device, for composition roots that add product services
    /// (for example reporting persistence) around [`step`](Self::step).
    pub const fn device(&self) -> &zigbee_runtime::ZigbeeDevice<M, EndDevice> {
        self.node.device()
    }

    /// Mutable access to the Zigbee device; see [`device`](Self::device).
    pub fn device_mut(&mut self) -> &mut zigbee_runtime::ZigbeeDevice<M, EndDevice> {
        self.node.device_mut()
    }

    /// The application profile, for composition roots that publish product
    /// state (for example the anomaly record) in its clusters.
    pub fn profile_mut(&mut self) -> &mut P {
        self.node.profile_mut()
    }

    /// The Zigbee device together with the profile's Poll Control server,
    /// if fitted, for composition roots that persist both (see
    /// `zigbee_runtime::reporting_store::ReportingPersistence::restore_with`).
    pub fn device_and_poll_control_mut(
        &mut self,
    ) -> (
        &mut zigbee_runtime::ZigbeeDevice<M, EndDevice>,
        Option<&mut zigbee_zcl::clusters::poll_control::PollControlCluster>,
    ) {
        let (device, profile) = self.node.device_and_profile_mut();
        (device, profile.poll_control_mut())
    }

    fn mark(&self) -> W::Mark {
        self.resources.wake.mark()
    }

    fn elapsed_ms(&self, since: W::Mark) -> u32 {
        W::elapsed_ms(self.mark(), since)
    }

    fn start_fast_poll(&mut self, duration_ms: u32) {
        self.fast_poll = Some((self.mark(), duration_ms));
    }

    /// Remaining time in the timed fast-poll window, clearing it on expiry.
    ///
    /// Called on every service iteration (through
    /// [`next_wait_ms`](Self::next_wait_ms)), so an expired window is dropped
    /// long before any platform counter can roll over and alias its start.
    fn fast_poll_remaining_ms(&mut self) -> Option<u32> {
        let now = self.mark();
        let timed = Self::window_remaining_ms(now, &mut self.fast_poll);
        let requested = Self::window_remaining_ms(now, &mut self.poll_control_fast);
        match (timed, requested) {
            (Some(timed), Some(requested)) => Some(timed.max(requested)),
            (timed, requested) => timed.or(requested),
        }
    }

    fn window_remaining_ms(now: W::Mark, window: &mut Option<(W::Mark, u32)>) -> Option<u32> {
        let (started, duration_ms) = (*window)?;
        let elapsed_ms = W::elapsed_ms(now, started);
        if elapsed_ms < duration_ms {
            Some(duration_ms - elapsed_ms)
        } else {
            *window = None;
            None
        }
    }

    /// Poll Control server (0x0020) duties, when the profile fits one:
    /// adopt its poll intervals, follow the client's fast-poll decisions and
    /// send the periodic Check-in, after which the device fast-polls for the
    /// configured response timeout.
    async fn service_poll_control(&mut self) {
        const QS_MS: u32 = 250;
        let now = self.mark();
        let elapsed_ms = W::elapsed_ms(now, self.last_poll_control);
        self.last_poll_control = now;
        let Some(poll_control) = self.node.profile_mut().poll_control_mut() else {
            return;
        };
        let slow_ms = poll_control
            .long_poll_interval_qs()
            .saturating_mul(QS_MS)
            .min(policy::MAX_WAIT_MS);
        let fast_ms = (u32::from(poll_control.short_poll_interval_qs()) * QS_MS).min(slow_ms);
        let event = poll_control.take_fast_poll_event();
        let check_in_ms = poll_control.check_in_interval_qs().saturating_mul(QS_MS);
        let response_ms = u32::from(poll_control.check_in_response_timeout_qs()) * QS_MS;
        if slow_ms != self.slow_poll_ms {
            // The runtime's sleepy power manager schedules its own polls
            // from the same interval.
            self.node
                .device_mut()
                .power_mut()
                .set_mode(zigbee_runtime::power::PowerMode::Sleepy {
                    poll_interval_ms: slow_ms,
                    wake_duration_ms: self.policy.wake_duration_ms,
                });
        }
        self.slow_poll_ms = slow_ms;
        self.fast_poll_ms = fast_ms;
        match event {
            Some(FastPollEvent::Start { timeout_qs }) => {
                self.poll_control_fast = Some((now, u32::from(timeout_qs) * QS_MS));
            }
            Some(FastPollEvent::Stop) => self.poll_control_fast = None,
            None => {}
        }
        if check_in_ms == 0 {
            self.check_in_elapsed_ms = 0;
            return;
        }
        self.check_in_elapsed_ms = self.check_in_elapsed_ms.saturating_add(elapsed_ms);
        if self.check_in_elapsed_ms < check_in_ms {
            return;
        }
        self.check_in_elapsed_ms = 0;
        self.checkpoint_security();
        let sent = self
            .node
            .device_mut()
            .send_bound_cluster_command(
                self.endpoint,
                ClusterId::POLL_CONTROL.0,
                true,
                CMD_CHECK_IN.0,
                &[],
            )
            .await;
        self.checkpoint_security();
        if sent.is_ok() {
            self.poll_control_fast = Some((self.mark(), response_ms));
        }
    }

    fn fast_poll_active(&mut self) -> bool {
        self.fast_poll_remaining_ms().is_some()
    }

    /// Whether a short button press can start commissioning on this product.
    const fn button_commissions() -> bool {
        !matches!(A::SHORT_PRESS, ShortPressAction::None)
    }

    /// Sleep for the current (jittered) backoff delay before the next
    /// unjoined attempt and double the following delay up to the product's
    /// cap.
    fn schedule_join_backoff(&mut self) {
        let recovery = self.policy.recovery;
        let delay_ms = if recovery.join_retry_jitter_pct == 0 {
            self.join_backoff_ms
        } else {
            recovery.jittered_ms(self.join_backoff_ms, self.random_u16())
        };
        self.join_wait_ms = delay_ms;
        self.join_backoff_ms = policy::next_join_retry_ms(
            self.join_backoff_ms,
            self.policy.join_retry_ms,
            recovery.join_retry_max_ms,
        );
        // Runtime RunAgain hints are serviced by the next attempt's tick; an
        // idle unjoined device must not wake early for them.
        self.run_again = None;
        self.resources
            .diagnostics
            .record(DiagnosticEvent::JoinBackoff { delay_ms });
    }

    /// Backoff jitter source.
    ///
    /// Jitter only needs to de-synchronize devices, not to be unpredictable,
    /// so this mixes the local IEEE address with the monotonic clock instead
    /// of calling the MAC entropy source: on some backends (TLSR8258) that
    /// source is a CTR-DRBG which would link a software AES into every
    /// sleepy image.
    fn random_u16(&mut self) -> u16 {
        let device = self.node.device_mut();
        let ieee = device.bdb().zdo().nwk().nib().ieee_address;
        let now = u64::from(device.mac_mut().monotonic_micros());
        let mut x = u64::from_le_bytes(ieee) ^ now ^ ((self.join_backoff_ms as u64) << 32);
        // splitmix64 finaliser.
        x ^= x >> 30;
        x = x.wrapping_mul(0xbf58_476d_1ce4_e5b9);
        x ^= x >> 27;
        x = x.wrapping_mul(0x94d0_49bb_1331_11eb);
        x ^= x >> 31;
        x as u16
    }

    /// Restart the backoff sequence at the product's initial delay.
    fn reset_join_backoff(&mut self) {
        self.join_backoff_ms = self.policy.join_retry_ms;
        self.join_wait_ms = 0;
    }

    /// Enter the factory-new state after an explicit leave.
    ///
    /// Products with a commissioning button sleep until it is pressed and
    /// never search for a network on their own. A product without a usable
    /// short-press action would otherwise be unrecoverable without a power
    /// cycle, so it falls back to the bounded unjoined backoff instead.
    fn enter_factory_new_after_leave(&mut self) {
        self.network_up = false;
        self.fast_poll = None;
        self.run_again = None;
        self.reset_join_backoff();
        self.resources.status.set(SensorStatus::Off);
        if Self::button_commissions() {
            self.awaiting_button = true;
            self.join_wait_ms = 0;
            self.resources
                .diagnostics
                .record(DiagnosticEvent::WaitingForButton);
        } else {
            self.awaiting_button = false;
            self.schedule_join_backoff();
        }
    }

    /// Start Finding & Binding target mode on this sensor endpoint.
    ///
    /// The BDB layer requests `IdentifyTime =
    /// bdbcMinCommissioningTime`; the runtime applies it on its next tick and
    /// handles Identify Query requests during that interval.
    pub async fn finding_binding_target(&mut self) -> Result<(), FindingBindingTargetError> {
        let result = self
            .node
            .device_mut()
            .bdb_mut()
            .finding_binding_target(self.endpoint)
            .await;
        if result.is_err() {
            return Err(FindingBindingTargetError::Rejected);
        }
        self.start_fast_poll(self.policy.fresh_join_fast_ms);
        Ok(())
    }

    #[inline(never)]
    fn wake_failure(&mut self, code: u32) -> ! {
        self.resources.status.set(SensorStatus::Fault);
        self.resources
            .supervisor
            .reset_with_cause(ResetCause::wake(code))
    }

    /// A low-power wait failed. A recoverable failure finishes this
    /// iteration with an active wait; `sleep_failure_limit` consecutive ones
    /// disable low-power waits for the boot and record the error. Anything
    /// else resets with the error as the retained cause.
    #[inline(never)]
    async fn recover_wake_failure(&mut self, error: W::Error, timeout_ms: u32) -> WakeReason {
        let code = W::error_code(&error);
        self.resources
            .diagnostics
            .record(DiagnosticEvent::WakeFailed);
        let limit = self.policy.recovery.sleep_failure_limit;
        if limit == 0 || !W::is_recoverable(&error) {
            self.wake_failure(code);
        }
        self.sleep_failures = self.sleep_failures.saturating_add(1);
        if self.sleep_failures >= limit && !self.sleep_disabled {
            self.sleep_disabled = true;
            self.anomalies
                .note_with_cause(AnomalyReason::SleepDisabled, ResetCause::wake(code).word());
            self.anomaly_noted(AnomalyReason::SleepDisabled);
        }
        let result = {
            let mac = self.node.device_mut().mac_mut();
            self.resources
                .wake
                .wait(
                    mac,
                    WaitRequest {
                        timeout_ms,
                        sleep_depth: SleepDepth::Active,
                    },
                )
                .await
        };
        match result {
            Ok(reason) => reason,
            Err(error) => self.wake_failure(W::error_code(&error)),
        }
    }

    #[inline(never)]
    fn persistence_failure(
        &mut self,
        error: zigbee_runtime::security_store::SecurityStoreError,
    ) -> ! {
        self.resources
            .diagnostics
            .record(DiagnosticEvent::SecurityFailure(error));
        self.resources.status.set(SensorStatus::Fault);
        core::panic!("security persistence failure");
    }

    #[inline(never)]
    fn node_failure(&mut self, error: NodeError) -> ! {
        match error {
            NodeError::Persistence(error) => self.persistence_failure(error),
            NodeError::Profile(error) => {
                // A profile/cluster configuration error does not risk
                // security-counter reuse, so restart through the product
                // supervisor instead of halting forever without a watchdog.
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::ProfileFailure(error));
                self.resources.status.set(SensorStatus::Fault);
                self.resources
                    .supervisor
                    .reset_with_cause(ResetCause::profile())
            }
        }
    }

    fn checkpoint_security(&mut self) {
        if let Err(error) = self.node.checkpoint_security() {
            self.persistence_failure(error);
        }
    }

    /// Tick the runtime and replace the previously requested deadline.
    ///
    /// Calling `tick()` services any earlier `RunAgain` request. A new
    /// request returned by this call is therefore measured from completion
    /// of this call, not merged with an already-serviced relative delay.
    async fn tick(&mut self, elapsed_secs: u16) -> TickResult {
        self.run_again = None;
        let result = match self.node.tick(elapsed_secs).await {
            Ok(result) => result,
            Err(error) => self.node_failure(error),
        };
        if let Some(delay_ms) = policy::run_again_delay_ms(&result) {
            self.run_again = Some((self.mark(), delay_ms));
        }
        result
    }

    /// Tick the runtime, and if it produced a control event, run it through
    /// [`handle_control_event`](Self::handle_control_event).
    async fn tick_and_handle(&mut self, elapsed_secs: u16) {
        let result = self.tick(elapsed_secs).await;
        if let TickResult::Event(event) = &result
            && self.handle_control_event(event).await
        {
            self.start_fast_poll(self.policy.fresh_join_fast_ms);
        }
    }

    fn reset_post_join_state(&mut self) {
        let now = self.mark();
        self.resources
            .status
            .set(SensorStatus::Joined { active: true });
        self.start_fast_poll(self.policy.fresh_join_fast_ms);
        self.last_tick = now;
        self.last_poll = None;
        self.network_up = true;
        self.awaiting_button = false;
        self.reset_join_backoff();
        self.annce_retries_left = self.policy.announce_retries;
        self.last_annce = now;
        self.interview_done = false;
        self.was_identifying = false;
        self.identify_phase_on = false;
        // A fresh join/rejoin means the coordinator re-runs its interview.
        self.node.reset_remote_reporting();
        self.was_fast_polling = true;
        self.consecutive_rejoin_failures = 0;
        self.clear_parent_loss_evidence();
        self.poll_control_fast = None;
        self.check_in_elapsed_ms = 0;
        self.last_poll_control = now;
    }

    fn clear_parent_loss_evidence(&mut self) {
        self.node.device_mut().clear_aps_ack_failures();
        self.seen_aps_ack_failures = 0;
        self.active_bound_strikes = 0;
        self.pending_bound.reset();
        self.report_proof_ms = 0;
        self.prev_ack_pending = false;
        self.prev_ack_failures = 0;
    }

    /// Advance every keep-awake budget by the time measured since the last
    /// joined wait request and apply the fired limits. `windows` tells
    /// whether a timed fast-poll window is still open (expired ones are
    /// already cleared, so a single window ending on time never fires).
    /// Returns whether pending work, OTA and Identify may still keep the
    /// chip awake, and whether the fast-poll limit closed the windows.
    ///
    /// - pending work: every further `max_active_ms` it persists counts one
    ///   parent-loss strike;
    /// - OTA: the session is aborted;
    /// - Identify: it keeps running but no longer holds the fast poll;
    /// - fast-poll windows: they are closed.
    fn apply_awake_bounds(&mut self, now: W::Mark, windows: bool) -> (bool, bool, bool, bool) {
        let delta_ms = W::elapsed_ms(now, self.last_bound_check);
        self.last_bound_check = now;
        let recovery = &self.policy.recovery;

        self.sample_report_proof(delta_ms);

        let pending = self.node.device().has_pending_protocol_work();
        let pending = match self
            .pending_bound
            .step(pending, delta_ms, recovery.max_active_ms)
        {
            BoundStep::Fired => {
                self.active_bound_strikes = self.active_bound_strikes.saturating_add(1);
                self.note_anomaly(AnomalyReason::PendingWork);
                false
            }
            // Persisting work keeps earning strikes, recorded once.
            BoundStep::Refired => {
                self.active_bound_strikes = self.active_bound_strikes.saturating_add(1);
                false
            }
            step => step.allows(),
        };

        let ota = O::ENABLED && self.resources.ota.is_active(self.node.profile());
        let ota = match self
            .ota_bound
            .step(ota, delta_ms, recovery.max_ota_awake_ms)
        {
            BoundStep::Fired => {
                self.resources.ota.abort(&mut self.node);
                self.ota_bound.reset();
                self.note_anomaly(AnomalyReason::Ota);
                false
            }
            step => step.allows(),
        };

        let identifying = self.node.device().is_identifying(self.endpoint);
        let identifying =
            match self
                .identify_bound
                .step(identifying, delta_ms, recovery.max_identify_awake_ms)
            {
                BoundStep::Fired => {
                    self.note_anomaly(AnomalyReason::Identify);
                    false
                }
                step => step.allows(),
            };

        let windows_closed = self
            .fast_bound
            .step(windows, delta_ms, recovery.max_fast_poll_ms)
            == BoundStep::Fired;
        if windows_closed {
            self.fast_poll = None;
            self.poll_control_fast = None;
            self.fast_bound.reset();
            self.note_anomaly(AnomalyReason::FastPoll);
        }
        (pending, ota, identifying, windows_closed)
    }

    /// Missed-report detector.
    ///
    /// An APS ACK is end-to-end proof. One is recognised when the ACK table
    /// stops waiting without a new exhausted-retry failure, or when the
    /// failure count returns to zero. Without proof for the shortest
    /// configured reporting maximum interval plus `missed_report_grace_ms`
    /// a report was due but not acknowledged: that counts one parent-loss
    /// strike, and every report is forced due so the next tick produces
    /// fresh evidence (an ACK, or an APS failure).
    fn sample_report_proof(&mut self, delta_ms: u32) {
        let aps = self.node.device().bdb().zdo().aps();
        let ack_pending = aps.has_pending_ack();
        let failures = aps.consecutive_ack_failures();
        let acked = (self.prev_ack_pending && !ack_pending && failures <= self.prev_ack_failures)
            || (failures == 0 && self.prev_ack_failures != 0);
        self.prev_ack_pending = ack_pending;
        self.prev_ack_failures = failures;

        let grace_ms = self.policy.recovery.missed_report_grace_ms;
        let max_interval_s = if grace_ms == 0 {
            None
        } else {
            self.shortest_report_max_interval_s()
        };
        let Some(max_interval_s) = max_interval_s.filter(|_| !acked) else {
            self.report_proof_ms = 0;
            return;
        };
        self.report_proof_ms = self.report_proof_ms.saturating_add(delta_ms);
        let window_ms = (u32::from(max_interval_s) * 1_000).saturating_add(grace_ms);
        if self.report_proof_ms >= window_ms {
            self.report_proof_ms = 0;
            self.active_bound_strikes = self.active_bound_strikes.saturating_add(1);
            self.node.device_mut().reporting_mut().force_all_due();
            self.note_anomaly(AnomalyReason::MissedReport);
        }
    }

    /// Shortest periodic maximum interval among this endpoint's reporting
    /// configurations, if any attribute reports periodically.
    fn shortest_report_max_interval_s(&self) -> Option<u16> {
        self.node
            .device()
            .reporting()
            .configs()
            .filter(|(endpoint, _, config)| {
                *endpoint == self.endpoint
                    && config.direction == ReportDirection::Send
                    && config.max_interval != 0
                    && config.max_interval != 0xFFFF
            })
            .map(|(_, _, config)| config.max_interval)
            .min()
    }

    /// Start the staged rejoin once parent-loss evidence reaches the
    /// product's strike limit while the device still believes it is joined.
    ///
    /// An APS acknowledgement proves end-to-end delivery, so it also clears
    /// the strikes earned by bounded pending work.
    async fn check_parent_loss(&mut self) {
        let limit = self.policy.recovery.parent_loss_strikes;
        if limit == 0 || !self.node.device().is_joined() {
            return;
        }
        let aps_failures = self.node.device().consecutive_aps_ack_failures();
        if aps_failures < self.seen_aps_ack_failures {
            self.active_bound_strikes = 0;
        }
        self.seen_aps_ack_failures = aps_failures;
        let strikes = aps_failures.saturating_add(self.active_bound_strikes);
        if strikes < limit {
            return;
        }
        self.resources
            .diagnostics
            .record(DiagnosticEvent::ParentLossSuspected { strikes });
        self.note_anomaly(AnomalyReason::ParentLoss);
        self.clear_parent_loss_evidence();
        if self.secure_rejoin().await {
            self.reset_post_join_state();
        }
    }

    /// Attempt to join or silently resume. Every call site other than
    /// [`cold_start`](Self::cold_start) treats success the same way — see
    /// [`reset_post_join_state`](Self::reset_post_join_state) — regardless
    /// of whether this was a fresh join or a resume, matching the original
    /// firmware exactly: only the very first cold-boot join is allowed to
    /// suppress its Device_annce retries.
    async fn join_or_resume(&mut self) -> bool {
        self.run_again = None;
        match self.node.start_or_resume().await {
            Ok(short_address) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::JoinedOrResumed {
                        short_address,
                        channel: self.node.device().channel(),
                        pan_id: self.node.device().pan_id(),
                    });
                // The runtime owns the R22 End Device Timeout lifecycle: a
                // fresh join or secured rejoin sends exactly one initial
                // request, and a silent resume reuses the persisted parent
                // relationship. Sending one here as well would duplicate
                // the negotiation.
                self.checkpoint_security();
                true
            }
            Err(StartError::InitFailed) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::ZigbeeInitializationFailed);
                false
            }
            Err(StartError::CommissioningFailed(status)) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::CommissioningFailed {
                        status: status as u8,
                    });
                false
            }
            Err(StartError::PersistenceFailed(error)) => self.persistence_failure(error),
        }
    }

    /// Start or resume while counting a restored pending secure-rejoin
    /// failure as one real over-the-air attempt.
    ///
    /// The raw [`join_or_resume`](Self::join_or_resume) remains separate so
    /// [`rejoin_after_reset`](Self::rejoin_after_reset) can perform its fresh
    /// post-reset join without creating a recursive async call graph through
    /// [`record_failed_rejoin`](Self::record_failed_rejoin).
    async fn join_or_resume_with_rejoin_tracking(&mut self) -> bool {
        let was_secure_rejoin_pending = self.node.device().secure_rejoin_pending();
        if self.join_or_resume().await {
            true
        } else if was_secure_rejoin_pending || self.node.device().secure_rejoin_pending() {
            // On the first boot call, start_or_resume() restores the pending
            // flag from the journal before attempting the rejoin, so it only
            // becomes visible after the failed call returns.
            self.record_failed_rejoin().await
        } else {
            false
        }
    }

    async fn secure_rejoin(&mut self) -> bool {
        self.run_again = None;
        match self.node.secure_rejoin().await {
            Ok(short_address) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::SecureRejoinSucceeded { short_address });
                self.checkpoint_security();
                true
            }
            Err(StartError::InitFailed) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::SecureRejoinInitializationFailed);
                self.record_failed_rejoin().await
            }
            Err(StartError::CommissioningFailed(status)) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::SecureRejoinFailed {
                        status: status as u8,
                    });
                self.record_failed_rejoin().await
            }
            Err(StartError::PersistenceFailed(error)) => self.persistence_failure(error),
        }
    }

    async fn factory_reset(&mut self) -> bool {
        self.run_again = None;
        match self.node.factory_reset().await {
            Ok(()) => true,
            Err(StartError::InitFailed) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::FactoryResetInitializationFailed);
                false
            }
            Err(StartError::CommissioningFailed(status)) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::FactoryResetFailed {
                        status: status as u8,
                    });
                false
            }
            Err(StartError::PersistenceFailed(error)) => self.persistence_failure(error),
        }
    }

    /// Clear durable security/network state and immediately attempt a
    /// fresh join. Used only for the repeated-secure-rejoin-failure fallback;
    /// an explicit leave waits for the user instead (see
    /// [`enter_factory_new_after_leave`](Self::enter_factory_new_after_leave)).
    async fn rejoin_after_reset(&mut self) -> bool {
        if self.factory_reset().await && self.join_or_resume().await {
            self.reset_post_join_state();
            true
        } else {
            false
        }
    }

    /// Count one failed secure-rejoin attempt. Only a product that sets a
    /// nonzero `secure_rejoin_failure_limit` falls back to a durable reset
    /// and fresh join once it is reached; the default (`0`) keeps the
    /// network and lets the backoff schedule the next attempt.
    ///
    /// Both direct `secure_rejoin()` failures and runtime-driven retry
    /// failures use this path, so one over-the-air attempt is counted once.
    async fn record_failed_rejoin(&mut self) -> bool {
        self.consecutive_rejoin_failures = self.consecutive_rejoin_failures.saturating_add(1);
        let limit = self.policy.secure_rejoin_failure_limit;
        if limit == 0 || self.consecutive_rejoin_failures < limit {
            self.resources
                .diagnostics
                .record(DiagnosticEvent::SecureRejoinPending {
                    failures: self.consecutive_rejoin_failures,
                });
            return false;
        }
        self.resources
            .diagnostics
            .record(DiagnosticEvent::SecureRejoinLimitReached {
                failures: self.consecutive_rejoin_failures,
            });
        self.consecutive_rejoin_failures = 0;
        self.rejoin_after_reset().await
    }

    /// Read all sensors and update the profile's measurement/battery
    /// clusters — unchanged from the original firmware's `read_sensors`:
    /// a failed measurement still leaves the previous cluster values in
    /// place and still proceeds to the battery measurement. What is sampled
    /// for each `trigger` is the product's [`SensorMeasurement`] decision.
    async fn read_sensors(&mut self, trigger: SampleTrigger) {
        self.resources
            .environment
            .measure(
                self.node.profile_mut(),
                trigger,
                &mut self.resources.diagnostics,
            )
            .await;

        match self.resources.battery.sample().await {
            Ok(Some(battery)) => {
                self.resources.diagnostics.record(DiagnosticEvent::Battery {
                    millivolts: battery.millivolts,
                    percentage: battery.measurement.percentage_remaining / 2,
                });
                self.node.profile_mut().update_battery(battery.measurement);
            }
            Ok(None) => {}
            Err(_) => self
                .resources
                .diagnostics
                .record(DiagnosticEvent::BatteryReadFailed),
        }
    }

    /// One-time cold-boot bootstrap: silent-resume detection (only this
    /// call suppresses Device_annce retries), the initial join/resume
    /// attempt, the first sensor read (so ZHA's interview sees real cluster
    /// values immediately), and unconditional default reporting so the
    /// device reports even without a coordinator `ConfigureReporting`.
    async fn cold_start(&mut self) {
        let resumed_at_boot = match self.node.load_security_state() {
            Ok(Some(state)) => state.commissioned && !state.rejoin_pending,
            Ok(None) => false,
            Err(error) => self.persistence_failure(error),
        };

        if !resumed_at_boot {
            // Power-on commissioning is a real network search: show it.
            self.blink_join_attempt().await;
        }
        if self.join_or_resume_with_rejoin_tracking().await {
            self.resources
                .status
                .set(SensorStatus::Joined { active: true });
        }

        self.read_sensors(SampleTrigger::ColdStart).await;

        if let Err(error) = self.node.configure_default_reporting() {
            self.node_failure(NodeError::Profile(error));
        }
        self.resources
            .diagnostics
            .record(DiagnosticEvent::DefaultReportingConfigured);

        let now = self.mark();
        self.last_report = now;
        self.last_tick = now;
        self.last_poll = None;
        self.rejoin_count = 0;
        self.last_annce = now;
        self.interview_done = false;
        self.node.reset_remote_reporting();
        let joined = self.node.device().is_joined();
        self.was_fast_polling = joined;
        self.was_identifying = false;
        self.identify_phase_on = false;
        if joined {
            let duration_ms = if resumed_at_boot {
                self.policy.restored_fast_ms
            } else {
                self.policy.fresh_join_fast_ms
            };
            self.resources
                .diagnostics
                .record(DiagnosticEvent::FastPollStarted { duration_ms });
            self.resources
                .status
                .set(SensorStatus::Joined { active: true });
            self.start_fast_poll(duration_ms);
        } else {
            // A factory-new or network-lost device found nothing at power-on:
            // sleep through the bounded exponential backoff.
            self.fast_poll = None;
            self.resources.status.set(SensorStatus::Off);
            self.reset_join_backoff();
            self.schedule_join_backoff();
        }
        self.network_up = joined;
        self.annce_retries_left = if joined && !resumed_at_boot {
            self.policy.announce_retries
        } else {
            0
        };
    }

    fn apply_ota_outcome(&mut self, keep_awake_ms: Option<u32>, activation_pending: bool) {
        self.resources.status.set(SensorStatus::Ota);
        if let Some(duration_ms) = keep_awake_ms {
            self.start_fast_poll(duration_ms);
        }
        if activation_pending {
            // Activation may reset immediately. Persist network keys and
            // security counters before handing control back to the product.
            self.checkpoint_security();
            if matches!(
                self.resources.ota.activate(&mut self.node),
                OtaActivationOutcome::Failed
            ) {
                self.resources.status.set(SensorStatus::Fault);
            }
        }
    }

    /// Give every stack event to the selected OTA lifecycle before generic
    /// application matching. `None` is the only fall-through result.
    async fn handle_ota_event(&mut self, event: &StackEvent) -> Option<bool> {
        match self.resources.ota.handle_event(&mut self.node, event).await {
            OtaEventOutcome::NotHandled => None,
            OtaEventOutcome::Unexpected => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::UnexpectedOtaEvent);
                Some(false)
            }
            OtaEventOutcome::Handled {
                keep_awake_ms,
                activation_pending,
            } => {
                self.apply_ota_outcome(keep_awake_ms, activation_pending);
                // OTA owns its deadline; do not replace it with the generic
                // commissioning-activity window.
                Some(false)
            }
        }
    }

    /// Handle one control-plane [`StackEvent`], explicitly — every variant
    /// has an arm, so a new variant fails to compile here rather than
    /// silently falling through a wildcard. Returns whether the event
    /// represents coordinator/network activity that should extend the
    /// fast-poll window.
    async fn handle_control_event(&mut self, event: &StackEvent) -> bool {
        if let Some(handled) = self.handle_ota_event(event).await {
            return handled;
        }

        match event {
            StackEvent::Joined {
                short_address,
                channel,
                pan_id,
            } => {
                self.resources.diagnostics.record(DiagnosticEvent::Joined {
                    short_address: *short_address,
                    channel: *channel,
                    pan_id: *pan_id,
                });
                self.reset_post_join_state();
                self.checkpoint_security();
                true
            }
            StackEvent::Left => {
                self.resources.diagnostics.record(DiagnosticEvent::Left);
                self.resources.status.set(SensorStatus::Off);
                // The stack already marked itself left (and `state_dirty`);
                // persist that promptly rather than waiting for the next
                // scheduled checkpoint. A requested leave (Mgmt_Leave without
                // rejoin, local Leave/FactoryReset) leaves the device
                // factory-new: wait for the user rather than searching.
                self.checkpoint_security();
                self.enter_factory_new_after_leave();
                false
            }
            StackEvent::AttributeReport {
                src_addr,
                endpoint,
                cluster_id,
                attr_id,
            } => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::AttributeReport {
                        src_addr: *src_addr,
                        endpoint: *endpoint,
                        cluster_id: *cluster_id,
                        attr_id: *attr_id,
                    });
                false
            }
            StackEvent::ReportingConfigured { cluster_id, .. } => {
                // The runtime emits this only for a Configure Reporting
                // command whose every record succeeded. Profile progress is
                // derived from the shared expected-ID contract, so unrelated
                // clusters retained by generic runtime state cannot inflate
                // the displayed count or satisfy completion.
                let expected = self.node.expected_report_clusters();
                let configured = self.node.remote_reporting_cluster_count();
                let complete = self.node.remote_reporting_is_complete();
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::ReportingConfigured {
                        cluster_id: *cluster_id,
                        configured,
                        expected,
                    });
                if complete && !self.interview_done {
                    self.resources.diagnostics.record(
                        DiagnosticEvent::InterviewConfigurationComplete {
                            configured,
                            expected,
                        },
                    );
                    self.start_fast_poll(self.policy.interview_complete_grace_ms);
                    self.interview_done = true;
                    self.resources
                        .status
                        .set(SensorStatus::Joined { active: false });
                }
                // Generic fast-poll extension only while reporting progress is
                // still incomplete: once complete, returning `false` here lets
                // the caller retain the 5-second completion grace instead of
                // overwriting it with the generic 120-second window (which
                // would keep a sleepy end device awake for no reason).
                policy::configure_reporting_requests_generic_extension(complete)
            }
            StackEvent::CommandReceived {
                src_addr,
                cluster_id,
                command_id,
                ..
            } => {
                if *command_id == 0x06 {
                    // A Configure Reporting the stack did not fully accept
                    // (malformed, unsupported/unreportable attribute, invalid
                    // data type, no capacity). It is coordinator activity but
                    // explicitly not interview progress: keep the generic
                    // window open only while reporting remains incomplete,
                    // and never overwrite the completed interview's 5-second
                    // grace.
                    self.resources
                        .diagnostics
                        .record(DiagnosticEvent::ReportingRejected {
                            cluster_id: *cluster_id,
                            configured: self.node.remote_reporting_cluster_count(),
                            expected: self.node.expected_report_clusters(),
                        });
                    return policy::configure_reporting_requests_generic_extension(
                        self.node.remote_reporting_is_complete(),
                    );
                }
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::UnhandledCommand {
                        src_addr: *src_addr,
                        cluster_id: *cluster_id,
                        command_id: *command_id,
                    });
                false
            }
            StackEvent::CommissioningComplete { success: true } => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::CommissioningComplete { success: true });
                self.consecutive_rejoin_failures = 0;
                self.checkpoint_security();
                false
            }
            StackEvent::CommissioningComplete { success: false } => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::CommissioningComplete { success: false });
                if self.node.device().secure_rejoin_pending() {
                    self.record_failed_rejoin().await
                } else {
                    false
                }
            }
            StackEvent::DefaultResponse {
                src_addr,
                cluster_id,
                command_id,
                status,
                ..
            } => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::DefaultResponse {
                        src_addr: *src_addr,
                        cluster_id: *cluster_id,
                        command_id: *command_id,
                        status: *status,
                    });
                false
            }
            StackEvent::PermitJoinChanged { open } => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::PermitJoinChanged { open: *open });
                false
            }
            StackEvent::ApsSecurityIndication(_) => false,
            StackEvent::DeviceAnnounced { .. } => false,
            StackEvent::ReportSent => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::ReportSent);
                false
            }
            StackEvent::OtaImageAvailable { .. }
            | StackEvent::OtaProgress { .. }
            | StackEvent::OtaComplete
            | StackEvent::OtaFailed
            | StackEvent::OtaDelayedActivation { .. } => {
                debug_assert!(is_ota_event(event));
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::UnexpectedOtaEvent);
                false
            }
            StackEvent::LeaveRequested => {
                // NWK Leave without rejoin, or Trust Center Remove-Device.
                // The runtime has already persisted the factory-new security
                // state; clear the in-memory network state the same way and
                // then wait for the user instead of rejoining automatically.
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::LeaveRequested);
                self.factory_reset().await;
                self.enter_factory_new_after_leave();
                false
            }
            StackEvent::BasicResetToFactoryDefaults => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::BasicResetToFactoryDefaults);
                false
            }
            StackEvent::RejoinRequested => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::RejoinRequested);
                let rejoined = self.secure_rejoin().await;
                if rejoined {
                    self.reset_post_join_state();
                }
                rejoined
            }
        }
    }

    /// Bounded (four-round) MAC receive/poll window for a joined sleepy end
    /// device: drain up to four queued indirect frames per outer-loop
    /// iteration, same as the original firmware.
    async fn service_joined_polls(&mut self) {
        self.last_poll = Some(self.mark());
        for _poll_round in 0..4u8 {
            match self.node.device_mut().poll().await {
                Ok(Some(indication)) => {
                    let event = match self.node.process_incoming(&indication).await {
                        Ok(event) => event,
                        Err(error) => self.node_failure(error),
                    };
                    if let Some(event) = &event {
                        // Matches the original firmware's early `break`: a
                        // rejoin/leave/factory-reset event stops this
                        // iteration's poll round immediately, without also
                        // running the interview check or the response-flush
                        // tick below against network state that has just
                        // changed underneath them.
                        let recommission = matches!(
                            event,
                            StackEvent::RejoinRequested | StackEvent::LeaveRequested
                        );
                        if self.handle_control_event(event).await {
                            self.start_fast_poll(self.policy.fresh_join_fast_ms);
                        }
                        if recommission {
                            break;
                        }
                    }
                    // Tick to send any queued ZCL responses.
                    self.tick_and_handle(0).await;
                }
                Ok(None) => break,
                Err(_) => break,
            }
        }
    }

    /// Whole seconds of real time not yet charged to the runtime clock.
    ///
    /// The former 60-second cap silently lost time whenever a product policy
    /// slept longer than a minute (slow poll or sample interval), which
    /// stretched every runtime timer. `last_tick` advances by exactly the
    /// charged amount, so the sub-second remainder carries forward.
    fn take_runtime_elapsed_secs(&mut self) -> u16 {
        let secs = (self.elapsed_ms(self.last_tick) / 1_000).min(u32::from(u16::MAX));
        if secs != 0 {
            self.last_tick = W::add_ms(self.last_tick, secs * 1_000);
        }
        secs as u16
    }

    /// Charge the finished wait to the runtime clock before the poll window.
    ///
    /// The runtime stamps each poll with its own whole-second clock, which
    /// only advances in `tick()`. Polling first would stamp the poll one wait
    /// in the past; the following tick then saw `since_poll == wait` and
    /// asked for `RunAgain(slow_poll - wait)`, so a single shortened wait
    /// (e.g. one cut short by the sample deadline) locked the slow cadence
    /// into a `wait / slow_poll - wait` alternation — an extra Data Request
    /// per period. Returns the charged seconds.
    async fn charge_runtime_clock(&mut self) -> u16 {
        let secs = self.take_runtime_elapsed_secs();
        if secs != 0 {
            self.tick_and_handle(secs).await;
        }
        secs
    }

    /// Periodic joined-state tasks: sensor read/report, the main runtime
    /// tick, Identify LED, and Device_annce retries.
    ///
    /// Sensor sampling keeps the original `last_report` cadence. Runtime
    /// elapsed time uses its own whole-second clock so `RunAgain` wakeups and
    /// incoming-frame flush ticks cannot repeatedly charge the stack for the
    /// same wall-clock interval.
    ///
    /// `precharged_secs` is runtime time already charged this iteration by
    /// [`charge_runtime_clock`](Self::charge_runtime_clock); OTA still sees
    /// the full elapsed time.
    async fn service_periodic_joined(&mut self, precharged_secs: u16) {
        if self.elapsed_ms(self.last_report) >= self.policy.sample_interval_ms {
            self.last_report = self.mark();
            self.read_sensors(SampleTrigger::Periodic).await;
        }

        let tick_elapsed = self.take_runtime_elapsed_secs();
        self.tick_and_handle(tick_elapsed).await;
        self.service_poll_control().await;

        if O::ENABLED {
            let ota = self
                .resources
                .ota
                .service(&mut self.node, tick_elapsed.saturating_add(precharged_secs))
                .await;
            if ota.keep_awake_ms.is_some() || ota.activation_pending {
                self.apply_ota_outcome(ota.keep_awake_ms, ota.activation_pending);
            }
        }

        // Matches the original firmware exactly: these two checks run
        // unconditionally after the tick above, even on the (rare) iteration
        // where that tick's event just left the network — `is_identifying`
        // naturally reads false and a stale `send_device_annce` is already
        // tolerated (`let _ = ...`) below.
        if St::PRESENT {
            let identifying = self.node.device().is_identifying(self.endpoint);
            if identifying {
                self.identify_phase_on = !self.identify_phase_on;
                self.resources.status.set(SensorStatus::Identifying {
                    on: self.identify_phase_on,
                });
            } else if self.was_identifying {
                self.identify_phase_on = false;
                let active = self.fast_poll_active();
                self.resources.status.set(SensorStatus::Joined { active });
            }
            self.was_identifying = identifying;
        }

        // Device_annce retry.
        if self.annce_retries_left > 0
            && self.elapsed_ms(self.last_annce) >= self.policy.announce_retry_ms
        {
            self.annce_retries_left -= 1;
            self.last_annce = self.mark();
            self.resources
                .diagnostics
                .record(DiagnosticEvent::DeviceAnnounceRetry {
                    retries_left: self.annce_retries_left,
                });
            self.checkpoint_security();
            let _ = self.node.device_mut().send_device_annce().await;
            self.checkpoint_security();
        }

        self.check_parent_loss().await;
    }

    /// Double blink shown only while a commissioning attempt actually runs.
    async fn blink_join_attempt(&mut self) {
        if !St::PRESENT {
            return;
        }
        // on, gap, on: one await site per wait keeps the future small.
        let mut second_flash = false;
        loop {
            self.resources
                .status
                .set(SensorStatus::Joining { on: true });
            self.resources
                .wake
                .delay_ms(self.policy.status.blink_on_ms)
                .await;
            self.resources
                .status
                .set(SensorStatus::Joining { on: false });
            if second_flash {
                break;
            }
            second_flash = true;
            self.resources
                .wake
                .delay_ms(self.policy.status.blink_gap_ms)
                .await;
        }
    }

    /// Unjoined lifecycle: sleep through the exponential backoff (or until
    /// the user button after an explicit leave) and make one bounded attempt
    /// when the measured backoff has elapsed.
    async fn service_unjoined(&mut self) {
        if self.network_up {
            // The network was lost while joined (parent loss, rejoin request,
            // failed secure rejoin). The event that caused it already made
            // one immediate attempt, so continue with the backoff.
            self.network_up = false;
            self.fast_poll = None;
            if !self.awaiting_button {
                self.reset_join_backoff();
                self.schedule_join_backoff();
            }
            return;
        }
        if self.awaiting_button || self.join_wait_ms != 0 {
            return;
        }
        self.attempt_unjoined_commissioning().await;
    }

    /// One unjoined commissioning attempt.
    ///
    /// `tick_and_handle()` owns scheduled secure-rejoin retries; do not also
    /// call `secure_rejoin()` in the same cycle or each failure would produce
    /// two over-the-air attempts. Fresh commissioning is started only when no
    /// durable secure-rejoin retry remains pending.
    async fn attempt_unjoined_commissioning(&mut self) {
        self.rejoin_count = self.rejoin_count.wrapping_add(1);
        self.resources
            .diagnostics
            .record(DiagnosticEvent::JoinRetry {
                attempt: self.rejoin_count,
            });
        self.blink_join_attempt().await;
        let had_secure_rejoin_pending = self.node.device().secure_rejoin_pending();
        self.tick_and_handle(0).await;

        let joined_now = if self.node.device().is_joined() {
            true
        } else if self.awaiting_button {
            // The tick delivered an explicit leave; do not search.
            false
        } else if had_secure_rejoin_pending || self.node.device().secure_rejoin_pending() {
            // The tick above either performed the scheduled secure
            // rejoin, or its bounded-failure fallback already attempted
            // the fresh join. Do not duplicate either operation in the
            // same retry cycle.
            false
        } else {
            self.join_or_resume_with_rejoin_tracking().await
        };
        self.finish_unjoined_attempt(joined_now);
    }

    fn finish_unjoined_attempt(&mut self, joined_now: bool) {
        if joined_now || self.node.device().is_joined() {
            self.reset_post_join_state();
        } else {
            self.resources.status.set(SensorStatus::Off);
            if !self.awaiting_button {
                self.schedule_join_backoff();
            }
        }
    }

    /// User-requested commissioning: leave the wait-for-button state, restart
    /// the backoff at its initial delay, and try immediately.
    async fn join_from_button(&mut self) {
        self.resources
            .diagnostics
            .record(DiagnosticEvent::ButtonJoin);
        self.awaiting_button = false;
        self.reset_join_backoff();
        self.blink_join_attempt().await;
        let joined_now = self.join_or_resume_with_rejoin_tracking().await;
        self.finish_unjoined_attempt(joined_now);
    }

    async fn force_report_from_button(&mut self) {
        let configured = self.node.remote_reporting_cluster_count();
        let expected = self.node.expected_report_clusters();
        self.resources
            .diagnostics
            .record(DiagnosticEvent::ForceReport {
                configured,
                expected,
            });
        self.resources
            .status
            .set(SensorStatus::Reporting { on: true });
        self.read_sensors(SampleTrigger::UserRequest).await;
        self.node.device_mut().reporting_mut().force_all_due();
        self.tick_and_handle(0).await;
        self.last_report = self.mark();
        self.start_fast_poll(self.policy.interview_complete_grace_ms);
        self.resources
            .status
            .set(SensorStatus::Reporting { on: false });
    }

    /// Next wait timeout, whether the joined fast-poll period is active, and
    /// the sleep depth to request.
    #[inline(never)]
    fn next_wait_ms(&mut self) -> (u32, bool, SleepDepth) {
        #[inline(always)]
        fn shorten(current_ms: u32, candidate_ms: u32) -> u32 {
            current_ms.min(candidate_ms.max(1))
        }

        if !self.node.device().is_joined() {
            // Unjoined: sleep at the steady-state depth between attempts,
            // in bounded chunks whose measured time is accumulated.
            let mut wait_ms = if self.awaiting_button {
                policy::MAX_WAIT_MS
            } else {
                self.join_wait_ms.min(policy::MAX_WAIT_MS)
            };
            if let Some(watchdog_ms) = self.resources.supervisor.max_wait_ms() {
                wait_ms = shorten(wait_ms, watchdog_ms);
            }
            self.last_bound_check = self.mark();
            return (wait_ms.max(1), false, self.policy.slow_sleep_depth);
        }

        let now = self.mark();
        let mut fast_poll_remaining_ms = self.fast_poll_remaining_ms();
        let (pending_protocol_work, ota_active, identifying, windows_closed) =
            self.apply_awake_bounds(now, fast_poll_remaining_ms.is_some());
        if windows_closed {
            fast_poll_remaining_ms = None;
        }
        let in_fast_poll =
            fast_poll_remaining_ms.is_some() || identifying || ota_active || pending_protocol_work;
        let mut wait_ms = if in_fast_poll {
            self.fast_poll_ms
        } else {
            self.slow_poll_ms
        };

        if let Some(remaining_ms) = fast_poll_remaining_ms {
            wait_ms = shorten(wait_ms, remaining_ms);
        }

        if let Some((started, delay_ms)) = self.run_again {
            wait_ms = shorten(
                wait_ms,
                policy::remaining_ms(W::elapsed_ms(now, started), delay_ms),
            );
        }

        wait_ms = shorten(
            wait_ms,
            policy::remaining_ms(
                W::elapsed_ms(now, self.last_report),
                self.policy.sample_interval_ms,
            ),
        );
        if self.annce_retries_left > 0 {
            wait_ms = shorten(
                wait_ms,
                policy::remaining_ms(
                    W::elapsed_ms(now, self.last_annce),
                    self.policy.announce_retry_ms,
                ),
            );
        }

        if O::ENABLED
            && let Some(ota_ms) = self.resources.ota.next_deadline_ms(self.node.profile())
        {
            wait_ms = shorten(wait_ms, ota_ms);
        }
        if let Some(watchdog_ms) = self.resources.supervisor.max_wait_ms() {
            wait_ms = shorten(wait_ms, watchdog_ms);
        }

        let sleep_depth = if ota_active || pending_protocol_work {
            SleepDepth::Active
        } else if in_fast_poll {
            self.policy.fast_sleep_depth
        } else {
            self.policy.slow_sleep_depth
        };
        (wait_ms.max(1), in_fast_poll, sleep_depth)
    }

    /// Whether a wake should run the bounded MAC poll window.
    ///
    /// Timer and button wakes keep the original behavior. An `Activity` wake
    /// means only that a platform skipped or shortened its sleep, so it polls
    /// the parent only once the current poll interval has really elapsed;
    /// otherwise every vetoed sleep would transmit an extra Data Request.
    fn poll_due(&self, wake_reason: WakeReason, in_fast_poll: bool) -> bool {
        match wake_reason {
            WakeReason::Timer | WakeReason::Button => true,
            WakeReason::Activity => match self.last_poll {
                None => true,
                Some(last_poll) => {
                    let interval_ms = if in_fast_poll {
                        self.fast_poll_ms
                    } else {
                        self.slow_poll_ms
                    };
                    self.elapsed_ms(last_poll) >= interval_ms
                }
            },
        }
    }

    async fn handle_button_press(&mut self) {
        let held_long = match self.policy.button.long_press_ms {
            Some(duration_ms) => self.resources.wake.button_held_for(duration_ms).await,
            None => false,
        };

        if held_long {
            self.resources
                .diagnostics
                .record(DiagnosticEvent::FactoryResetRequested);
            if self.factory_reset().await {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::SecurityResetRebooting);
            }
            if St::PRESENT {
                for _ in 0..self.policy.status.reset_blinks {
                    self.resources
                        .status
                        .set(SensorStatus::Resetting { on: true });
                    self.resources
                        .wake
                        .delay_ms(self.policy.status.reset_phase_ms)
                        .await;
                    self.resources
                        .status
                        .set(SensorStatus::Resetting { on: false });
                    self.resources
                        .wake
                        .delay_ms(self.policy.status.reset_phase_ms)
                        .await;
                }
            }
            self.resources.supervisor.reset();
        } else {
            match A::SHORT_PRESS {
                ShortPressAction::None => {}
                ShortPressAction::JoinOnly => {
                    if !self.node.device().is_joined() {
                        self.join_from_button().await;
                    }
                }
                ShortPressAction::ForceReport => {
                    if self.node.device().is_joined() {
                        self.force_report_from_button().await;
                    } else {
                        self.join_from_button().await;
                    }
                }
                ShortPressAction::ToggleJoin => {
                    if self.node.device().is_joined() {
                        self.resources
                            .diagnostics
                            .record(DiagnosticEvent::ButtonLeave);
                        if self.factory_reset().await {
                            // Stay factory-new until the next press.
                            self.enter_factory_new_after_leave();
                        }
                    } else {
                        self.join_from_button().await;
                    }
                }
            }
        }
        self.resources
            .wake
            .delay_ms(self.policy.button.debounce_ms)
            .await;
    }

    /// Perform the one-time boot/resume lifecycle.
    ///
    /// Event-loop integrations that own their outer scheduler call this once,
    /// then invoke [`step`](Self::step) whenever they want the application to
    /// advance by one finite wait/service iteration.
    pub async fn initialize(&mut self) -> Result<(), SensorLifecycleError> {
        if self.initialized {
            return Err(SensorLifecycleError::AlreadyInitialized);
        }
        self.cold_start().await;
        self.initialized = true;
        Ok(())
    }

    async fn step_initialized(&mut self) {
        let joined = self.node.device().is_joined();
        let (poll_ms, in_fast_poll, mut sleep_depth) = self.next_wait_ms();
        if self.sleep_disabled {
            sleep_depth = SleepDepth::Active;
        }

        // Log transition from fast→slow poll.
        if !joined {
            // Unjoined waits have no poll period; a later join restarts the
            // fast-poll window explicitly.
            self.was_fast_polling = false;
        } else if self.was_fast_polling && !in_fast_poll {
            let cfg = self.node.remote_reporting_cluster_count();
            self.resources
                .diagnostics
                .record(DiagnosticEvent::FastPollStopped {
                    configured: cfg,
                    expected: self.node.expected_report_clusters(),
                });
            self.was_fast_polling = false;
            if !self.was_identifying {
                self.resources
                    .status
                    .set(SensorStatus::Joined { active: false });
            }
        } else if in_fast_poll {
            self.was_fast_polling = true;
        }

        self.resources.supervisor.heartbeat();
        let wait_started = self.mark();
        let wait_result = {
            let node = &mut self.node;
            let resources = &mut self.resources;
            let mac = node.device_mut().mac_mut();
            resources
                .wake
                .wait(
                    mac,
                    WaitRequest {
                        timeout_ms: poll_ms,
                        sleep_depth,
                    },
                )
                .await
        };
        let wake_reason = match wait_result {
            Ok(wake_reason) => {
                if sleep_depth != SleepDepth::Active {
                    self.sleep_failures = 0;
                }
                wake_reason
            }
            Err(error) => self.recover_wake_failure(error, poll_ms).await,
        };
        self.resources.supervisor.heartbeat();
        if !joined {
            // Each unjoined wait is at most one chunk, so this measurement
            // never spans a native counter rollover.
            let waited_ms = self.elapsed_ms(wait_started);
            self.join_wait_ms = self.join_wait_ms.saturating_sub(waited_ms);
        }

        let poll_due = self.poll_due(wake_reason, in_fast_poll);
        if wake_reason == WakeReason::Button {
            self.handle_button_press().await;
        }

        if self.node.device().is_joined() {
            let mut precharged_secs = 0;
            if poll_due {
                precharged_secs = self.charge_runtime_clock().await;
                if self.node.device().is_joined() {
                    self.service_joined_polls().await;
                }
            }
            self.service_periodic_joined(precharged_secs).await;
        } else {
            self.service_unjoined().await;
        }
    }

    /// Advance one finite wait/service iteration.
    pub async fn step(&mut self) -> Result<(), SensorLifecycleError> {
        if !self.initialized {
            return Err(SensorLifecycleError::NotInitialized);
        }
        self.step_initialized().await;
        Ok(())
    }

    /// Complete an iteration whose wait ended by a reset-on-wake timer wake.
    ///
    /// A platform whose deepest sleep never returns from
    /// [`WakeController::wait`] (for example TLSR8258 LOW32K retention,
    /// which re-enters through a fresh executor root) calls this once after
    /// each validated timer wake, before resuming [`step`](Self::step).
    /// Calling `step` directly would start a new wait without the poll,
    /// reports or unjoined backoff accounting of the iteration that slept.
    ///
    /// The lifecycle state is the retained pre-sleep state, so the request
    /// that slept is recomputed; the platform has already verified that the
    /// timer, not an early event, ended it.
    pub async fn resume_after_timer_wake(&mut self) -> Result<(), SensorLifecycleError> {
        if !self.initialized {
            return Err(SensorLifecycleError::NotInitialized);
        }
        self.resources.supervisor.heartbeat();
        // The service half of `step_initialized` for a timer wake, whose
        // poll is always due.
        if self.node.device().is_joined() {
            let precharged_secs = self.charge_runtime_clock().await;
            if self.node.device().is_joined() {
                self.service_joined_polls().await;
            }
            self.service_periodic_joined(precharged_secs).await;
        } else {
            // Unjoined state is unchanged since the sleep, so this is the
            // backoff chunk that was requested and slept.
            let (waited_ms, _, _) = self.next_wait_ms();
            self.join_wait_ms = self.join_wait_ms.saturating_sub(waited_ms);
            self.service_unjoined().await;
        }
        Ok(())
    }

    /// Convenience wrapper identical to one [`initialize`](Self::initialize)
    /// followed by an infinite sequence of [`step`](Self::step) iterations.
    pub async fn run(&mut self) -> ! {
        if self.initialize().await.is_err() {
            core::panic!();
        }
        loop {
            self.step_initialized().await;
        }
    }
}
