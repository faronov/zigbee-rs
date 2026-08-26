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
//! | `nrf52840_sensor_product::profile::SensorProfile` | `P: EnvironmentalSensorProfile` |
//! | `nrf52840_sensor_product::ENDPOINT` | `profile.endpoint()` (same value) |
//! | nRF SAADC + product battery curve | `B: BatterySource` |
//! | `Temp` / `crate::sensor::Sensor` (cfg-selected) | `E: EnvironmentSource` |
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
//!   runtime-driven attempt and fall back to a fresh join after the bounded
//!   product policy limit.
//! - Construction rejects a device whose runtime automatic parent polling is
//!   still enabled, so the bounded manual poll window has exactly one owner.

use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::profile::TemperatureHumidityMeasurement;
use zigbee_runtime::role::EndDevice;
use zigbee_runtime::security_store::SecurityStateStore;

use crate::battery::BatterySource;
use crate::capabilities::{
    SensorStatus, StatusSink, Supervisor, WaitRequest, WakeController, WakeReason,
};
use crate::diagnostics::{DiagnosticEvent, Diagnostics};
use crate::environment::{EnvironmentSource, EnvironmentalSensorProfile};
use crate::ota::{OtaActivationOutcome, OtaEventOutcome, OtaLifecycle, is_ota_event};
use crate::parts::{SensorSedParts, SensorSedResources};
use crate::policy::{self, SensorPolicy, ShortPressAction, SleepDepth, UserActionPolicy};

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

pub struct SensorApp<'a, M, S, P, R>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: EnvironmentalSensorProfile,
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
    fast_poll_started: R::Mark,
    fast_poll_duration_ms: u32,
    last_rejoin_attempt: R::Mark,
    last_status: R::Mark,
    rejoin_count: u8,
    annce_retries_left: u8,
    last_annce: R::Mark,
    was_fast_polling: bool,
    was_identifying: bool,
    identify_phase_on: bool,
    interview_done: bool,
    consecutive_rejoin_failures: u8,
    run_again: Option<(R::Mark, u32)>,
    initialized: bool,
}

impl<'a, M, S, P, W, St, E, B, O, A, Sv, D>
    SensorApp<'a, M, S, P, SensorSedParts<W, St, E, B, O, A, Sv, D>>
where
    M: MacDriver,
    S: SecurityStateStore,
    P: EnvironmentalSensorProfile,
    W: WakeController<M>,
    St: StatusSink,
    E: EnvironmentSource,
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

        let now = parts.wake.mark();
        let endpoint = node.profile().endpoint();
        Ok(Self {
            node,
            policy,
            resources: parts,
            endpoint,
            last_report: now,
            last_tick: now,
            fast_poll_started: now,
            fast_poll_duration_ms: 0,
            last_rejoin_attempt: now,
            last_status: now,
            rejoin_count: 0,
            annce_retries_left: 0,
            last_annce: now,
            was_fast_polling: false,
            was_identifying: false,
            identify_phase_on: false,
            interview_done: false,
            consecutive_rejoin_failures: 0,
            run_again: None,
            initialized: false,
        })
    }

    fn mark(&self) -> W::Mark {
        self.resources.wake.mark()
    }

    fn elapsed_ms(&self, since: W::Mark) -> u32 {
        W::elapsed_ms(self.mark(), since)
    }

    fn start_fast_poll(&mut self, duration_ms: u32) {
        self.fast_poll_started = self.mark();
        self.fast_poll_duration_ms = duration_ms;
    }

    fn fast_poll_active(&self) -> bool {
        self.elapsed_ms(self.fast_poll_started) < self.fast_poll_duration_ms
    }

    #[inline(never)]
    fn wake_failure(&mut self) -> ! {
        self.resources
            .diagnostics
            .record(DiagnosticEvent::WakeFailed);
        self.resources.status.set(SensorStatus::Fault);
        self.resources.supervisor.reset()
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
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::ProfileFailure(error));
                self.resources.status.set(SensorStatus::Fault);
                core::panic!("profile error");
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
        self.last_rejoin_attempt = now;
        self.last_status = now;
        self.annce_retries_left = self.policy.announce_retries;
        self.last_annce = now;
        self.interview_done = false;
        self.was_identifying = false;
        self.identify_phase_on = false;
        // A fresh join/rejoin means the coordinator re-runs its interview.
        self.node.reset_remote_reporting();
        self.was_fast_polling = true;
        self.consecutive_rejoin_failures = 0;
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
    /// fresh join. Used for both a coordinator-driven Leave/factory-reset
    /// request and the repeated-secure-rejoin-failure fallback.
    async fn rejoin_after_reset(&mut self) -> bool {
        if self.factory_reset().await && self.join_or_resume().await {
            self.reset_post_join_state();
            true
        } else {
            false
        }
    }

    /// Count one failed secure-rejoin attempt and, once the bounded limit is
    /// reached, fall back to a durable reset and fresh join.
    ///
    /// Both direct `secure_rejoin()` failures and runtime-driven retry
    /// failures use this path, so one over-the-air attempt is counted once.
    async fn record_failed_rejoin(&mut self) -> bool {
        self.consecutive_rejoin_failures = self.consecutive_rejoin_failures.saturating_add(1);
        if self.consecutive_rejoin_failures < self.policy.secure_rejoin_failure_limit {
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

    /// Read all sensors and update the profile's environment/battery
    /// clusters — unchanged from the original firmware's `read_sensors`:
    /// a failed environmental read still leaves the previous cluster
    /// values in place and still proceeds to the battery measurement.
    async fn read_sensors(&mut self) {
        match self.resources.environment.sample().await {
            Ok(reading) => {
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::Environment(reading));
                self.node
                    .profile_mut()
                    .update_environment(TemperatureHumidityMeasurement {
                        temperature_centi_celsius: reading.temperature_centi_celsius,
                        humidity_centi_percent: reading.humidity_centi_percent,
                    });
                if let Some(pressure) = reading.pressure_tenth_kpa {
                    self.node.profile_mut().update_pressure(pressure);
                }
            }
            Err(_) => self
                .resources
                .diagnostics
                .record(DiagnosticEvent::EnvironmentReadFailed),
        }

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

        if self.join_or_resume_with_rejoin_tracking().await {
            self.resources
                .status
                .set(SensorStatus::Joined { active: true });
        }

        self.read_sensors().await;

        if let Err(error) = self.node.configure_default_reporting() {
            self.node_failure(NodeError::Profile(error));
        }
        self.resources
            .diagnostics
            .record(DiagnosticEvent::DefaultReportingConfigured);

        let now = self.mark();
        self.last_report = now;
        self.last_tick = now;
        self.last_rejoin_attempt = now;
        self.last_status = now;
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
            self.fast_poll_started = now;
            self.fast_poll_duration_ms = 0;
            self.resources.status.set(SensorStatus::Off);
        }
        self.annce_retries_left = if joined && !resumed_at_boot {
            self.policy.announce_retries
        } else {
            0
        };
    }

    fn apply_ota_outcome(&mut self, keep_awake_ms: Option<u32>, activation_pending: bool) -> bool {
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
        true
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
            } => Some(self.apply_ota_outcome(keep_awake_ms, activation_pending)),
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
                // scheduled checkpoint. No rejoin action here — the normal
                // unjoined retry loop picks it back up.
                self.checkpoint_security();
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
                self.resources
                    .diagnostics
                    .record(DiagnosticEvent::LeaveRequested);
                self.rejoin_after_reset().await
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

    /// Periodic joined-state tasks: sensor read/report, the main runtime
    /// tick, Identify LED, and Device_annce retries.
    ///
    /// Sensor sampling keeps the original `last_report` cadence. Runtime
    /// elapsed time uses its own whole-second clock so `RunAgain` wakeups and
    /// incoming-frame flush ticks cannot repeatedly charge the stack for the
    /// same wall-clock interval.
    async fn service_periodic_joined(&mut self) {
        if self.elapsed_ms(self.last_report) >= self.policy.sample_interval_ms {
            self.last_report = self.mark();
            self.read_sensors().await;
        }

        let tick_elapsed = (self.elapsed_ms(self.last_tick) / 1_000).min(60);
        if tick_elapsed != 0 {
            self.last_tick = W::add_ms(self.last_tick, tick_elapsed * 1_000);
        }
        self.tick_and_handle(tick_elapsed as u16).await;

        if O::ENABLED {
            let ota = self
                .resources
                .ota
                .service(&mut self.node, tick_elapsed as u16)
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
                self.resources.status.set(SensorStatus::Joined {
                    active: self.fast_poll_active(),
                });
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
    }

    /// Not-joined blink + bounded retry cadence.
    ///
    /// `tick_and_handle()` owns scheduled secure-rejoin retries; do not also
    /// call `secure_rejoin()` in the same cycle or each failure would produce
    /// two over-the-air attempts. Fresh commissioning is started only when no
    /// durable secure-rejoin retry remains pending.
    async fn service_unjoined(&mut self) {
        if St::PRESENT
            && self.elapsed_ms(self.last_status) >= self.policy.status.unjoined_blink_period_ms
        {
            self.last_status = self.mark();
            // Double blink.
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
            self.resources
                .wake
                .delay_ms(self.policy.status.blink_gap_ms)
                .await;
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
        }

        if self.elapsed_ms(self.last_rejoin_attempt) >= self.policy.join_retry_ms {
            self.rejoin_count = self.rejoin_count.wrapping_add(1);
            self.last_rejoin_attempt = self.mark();
            self.resources
                .diagnostics
                .record(DiagnosticEvent::JoinRetry {
                    attempt: self.rejoin_count,
                });
            let had_secure_rejoin_pending = self.node.device().secure_rejoin_pending();
            self.tick_and_handle(0).await;

            let joined_now = if self.node.device().is_joined() {
                true
            } else if had_secure_rejoin_pending || self.node.device().secure_rejoin_pending() {
                // The tick above either performed the scheduled secure
                // rejoin, or its bounded-failure fallback already attempted
                // the fresh join. Do not duplicate either operation in the
                // same retry cycle.
                false
            } else {
                self.join_or_resume_with_rejoin_tracking().await
            };
            if joined_now {
                self.reset_post_join_state();
            }
        }
    }

    async fn join_from_button(&mut self) {
        self.resources
            .diagnostics
            .record(DiagnosticEvent::ButtonJoin);
        if self.join_or_resume_with_rejoin_tracking().await {
            self.reset_post_join_state();
        }
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
        self.read_sensors().await;
        self.node.device_mut().reporting_mut().force_all_due();
        self.tick_and_handle(0).await;
        self.last_report = self.mark();
        self.start_fast_poll(self.policy.interview_complete_grace_ms);
        self.resources
            .status
            .set(SensorStatus::Reporting { on: false });
    }

    #[inline(never)]
    fn next_wait_ms(&self) -> (u32, bool) {
        #[inline(always)]
        fn shorten(current_ms: u32, candidate_ms: u32) -> u32 {
            current_ms.min(candidate_ms.max(1))
        }

        let now = self.mark();
        let joined = self.node.device().is_joined();
        let identifying = St::PRESENT && self.node.device().is_identifying(self.endpoint);
        let ota_active = O::ENABLED && self.resources.ota.is_active(self.node.profile());
        let fast_poll_elapsed_ms = W::elapsed_ms(now, self.fast_poll_started);
        let timed_fast_poll = fast_poll_elapsed_ms < self.fast_poll_duration_ms;
        let in_fast_poll = timed_fast_poll || identifying || ota_active;
        let mut wait_ms = if in_fast_poll {
            self.policy.fast_poll_ms
        } else {
            self.policy.slow_poll_ms
        };

        if timed_fast_poll {
            wait_ms = shorten(
                wait_ms,
                policy::remaining_ms(fast_poll_elapsed_ms, self.fast_poll_duration_ms),
            );
        }

        if let Some((started, delay_ms)) = self.run_again {
            wait_ms = shorten(
                wait_ms,
                policy::remaining_ms(W::elapsed_ms(now, started), delay_ms),
            );
        }

        if joined {
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
        } else {
            wait_ms = shorten(
                wait_ms,
                policy::remaining_ms(
                    W::elapsed_ms(now, self.last_rejoin_attempt),
                    self.policy.join_retry_ms,
                ),
            );
            if St::PRESENT {
                wait_ms = shorten(
                    wait_ms,
                    policy::remaining_ms(
                        W::elapsed_ms(now, self.last_status),
                        self.policy.status.unjoined_blink_period_ms,
                    ),
                );
            }
        }

        if O::ENABLED
            && let Some(ota_ms) = self.resources.ota.next_deadline_ms(self.node.profile())
        {
            wait_ms = shorten(wait_ms, ota_ms);
        }
        if let Some(watchdog_ms) = self.resources.supervisor.max_wait_ms() {
            wait_ms = shorten(wait_ms, watchdog_ms);
        }

        (wait_ms.max(1), in_fast_poll)
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
                            self.resources.status.set(SensorStatus::Off);
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
        let (poll_ms, in_fast_poll) = self.next_wait_ms();

        // Log transition from fast→slow poll.
        if self.was_fast_polling && !in_fast_poll {
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
        let ota_active = O::ENABLED && self.resources.ota.is_active(self.node.profile());
        let sleep_depth = if self.node.device().is_joined() && !ota_active {
            if in_fast_poll {
                self.policy.fast_sleep_depth
            } else {
                self.policy.slow_sleep_depth
            }
        } else {
            SleepDepth::Active
        };
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
            Ok(wake_reason) => wake_reason,
            Err(_) => self.wake_failure(),
        };
        self.resources.supervisor.heartbeat();

        match wake_reason {
            WakeReason::Button => self.handle_button_press().await,
            WakeReason::Timer => {}
        }

        if self.node.device().is_joined() {
            self.service_joined_polls().await;
            self.service_periodic_joined().await;
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
