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
//! | `nrf52840_sensor_product::profile::SensorProfile` | `DeviceProfile<C>` |
//! | `nrf52840_sensor_product::ENDPOINT` | `profile.endpoint()` (same value) |
//! | nRF SAADC + product battery curve | `B: BatterySource` |
//! | `Temp` / `crate::sensor::Sensor` (cfg-selected) | `E: EnvironmentSource` |
//! | Embassy clock/button/LED/reset | `I: LifecyclePlatform` |
//! | `NrfMac::enter_low_power_idle` | `R: RadioPower<M>` |
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
//! - [`TickResult::RunAgain`] now records an absolute deadline and shortens
//!   the next poll/sleep wait (see `crate::policy`) instead of being
//!   discarded. Runtime elapsed time is tracked independently from the
//!   sensor-report cadence, so those additional wakeups cannot advance stack
//!   timers faster than wall clock.
//! - Secure-rejoin failures are counted once per boot-time, direct, or
//!   runtime-driven attempt and fall back to a fresh join after the bounded
//!   `SECURE_REJOIN_FAILURE_LIMIT`.

use zigbee_mac::MacDriver;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::profile::{
    ApplicationProfile, DeviceProfile, ProfileComponent, TemperatureHumidityMeasurement,
};
use zigbee_runtime::security_store::SecurityStateStore;

use crate::battery::BatterySource;
use crate::capabilities::{LifecyclePlatform, RadioPower, WakeReason};
use crate::diagnostics::{DiagnosticEvent, Diagnostics};
use crate::environment::{EnvironmentSink, EnvironmentSource};
use crate::policy;

const REPORT_INTERVAL_SECS: u64 = 60;
const FAST_POLL_MS: u64 = 250;
const SLOW_POLL_SECS: u64 = 30;
const FAST_POLL_DURATION_SECS: u64 = 120;
const JOIN_RETRY_SECS: u64 = 15;
const ANNCE_RETRY_SECS: u64 = 8;
const ANNCE_RETRIES: u8 = 5;
const BUTTON_LONG_PRESS_SECS: u64 = 3;
/// Consecutive failed secure-rejoin attempts (each retried on the
/// `JOIN_RETRY_SECS` cadence, or immediately on a fresh
/// `CommissioningComplete { success: false }`) before falling back to a full
/// factory reset and fresh join. Matches the bound already proven on the
/// EFR32MG1 and ESP32-H2 sensors.
const SECURE_REJOIN_FAILURE_LIMIT: u8 = 4;

type SensorNode<'a, M, S, C> = ZigbeeNode<'a, M, S, DeviceProfile<C>>;

pub struct SensorApp<'a, M, S, C, E, B, I, R, D>
where
    M: MacDriver,
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink,
    E: EnvironmentSource,
    B: BatterySource,
    I: LifecyclePlatform,
    R: RadioPower<M>,
    D: Diagnostics,
{
    node: SensorNode<'a, M, S, C>,
    /// Cached `profile.endpoint()`. Identical to the product's `ENDPOINT`
    /// constant this used to import; read once so the hot paths do not
    /// re-borrow the profile just to name the endpoint.
    endpoint: u8,
    platform: I,
    radio_power: R,
    diagnostics: D,
    environment: E,
    battery: B,
    last_report: I::Instant,
    last_tick: I::Instant,
    fast_poll_until: I::Instant,
    last_rejoin_attempt: I::Instant,
    rejoin_count: u8,
    annce_retries_left: u8,
    last_annce: I::Instant,
    was_fast_polling: bool,
    interview_done: bool,
    consecutive_rejoin_failures: u8,
    /// Absolute deadline from the most recent `TickResult::RunAgain`.
    ///
    /// Storing a platform instant, rather than retaining the relative duration,
    /// ensures time spent processing incoming frames or button activity does
    /// not restart the runtime's requested delay.
    run_again_deadline: Option<I::Instant>,
}

impl<'a, M, S, C, E, B, I, R, D> SensorApp<'a, M, S, C, E, B, I, R, D>
where
    M: MacDriver,
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink,
    E: EnvironmentSource,
    B: BatterySource,
    I: LifecyclePlatform,
    R: RadioPower<M>,
    D: Diagnostics,
{
    pub fn new(
        node: SensorNode<'a, M, S, C>,
        platform: I,
        radio_power: R,
        diagnostics: D,
        environment: E,
        battery: B,
    ) -> Self {
        let now = platform.now();
        let endpoint = node.profile().endpoint();
        Self {
            node,
            endpoint,
            platform,
            radio_power,
            diagnostics,
            environment,
            battery,
            last_report: now,
            last_tick: now,
            fast_poll_until: now,
            last_rejoin_attempt: now,
            rejoin_count: 0,
            annce_retries_left: 0,
            last_annce: now,
            was_fast_polling: false,
            interview_done: false,
            consecutive_rejoin_failures: 0,
            run_again_deadline: None,
        }
    }

    #[inline(never)]
    fn persistence_failure(
        &mut self,
        error: zigbee_runtime::security_store::SecurityStoreError,
    ) -> ! {
        self.diagnostics
            .record(DiagnosticEvent::SecurityFailure(error));
        core::panic!("security persistence failure");
    }

    #[inline(never)]
    fn node_failure(&mut self, error: NodeError) -> ! {
        match error {
            NodeError::Persistence(error) => self.persistence_failure(error),
            NodeError::Profile(error) => {
                self.diagnostics
                    .record(DiagnosticEvent::ProfileFailure(error));
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
        self.run_again_deadline = None;
        let result = match self.node.tick(elapsed_secs).await {
            Ok(result) => result,
            Err(error) => self.node_failure(error),
        };
        if let Some(delay_ms) = policy::run_again_delay_ms(&result) {
            self.run_again_deadline = Some(I::add_millis(self.platform.now(), u64::from(delay_ms)));
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
            self.fast_poll_until =
                I::add_millis(self.platform.now(), FAST_POLL_DURATION_SECS * 1_000);
        }
    }

    fn reset_post_join_state(&mut self) {
        let now = self.platform.now();
        self.platform.led_on();
        self.fast_poll_until = I::add_millis(now, FAST_POLL_DURATION_SECS * 1_000);
        self.last_tick = now;
        self.last_rejoin_attempt = now;
        self.annce_retries_left = ANNCE_RETRIES;
        self.last_annce = now;
        self.interview_done = false;
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
        self.run_again_deadline = None;
        match self.node.start_or_resume().await {
            Ok(short_address) => {
                self.diagnostics.record(DiagnosticEvent::JoinedOrResumed {
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
                self.diagnostics
                    .record(DiagnosticEvent::ZigbeeInitializationFailed);
                false
            }
            Err(StartError::CommissioningFailed(status)) => {
                self.diagnostics
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
        self.run_again_deadline = None;
        match self.node.secure_rejoin().await {
            Ok(short_address) => {
                self.diagnostics
                    .record(DiagnosticEvent::SecureRejoinSucceeded { short_address });
                self.checkpoint_security();
                true
            }
            Err(StartError::InitFailed) => {
                self.diagnostics
                    .record(DiagnosticEvent::SecureRejoinInitializationFailed);
                self.record_failed_rejoin().await
            }
            Err(StartError::CommissioningFailed(status)) => {
                self.diagnostics
                    .record(DiagnosticEvent::SecureRejoinFailed {
                        status: status as u8,
                    });
                self.record_failed_rejoin().await
            }
            Err(StartError::PersistenceFailed(error)) => self.persistence_failure(error),
        }
    }

    async fn factory_reset(&mut self) -> bool {
        self.run_again_deadline = None;
        match self.node.factory_reset().await {
            Ok(()) => true,
            Err(StartError::InitFailed) => {
                self.diagnostics
                    .record(DiagnosticEvent::FactoryResetInitializationFailed);
                false
            }
            Err(StartError::CommissioningFailed(status)) => {
                self.diagnostics
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
        if self.consecutive_rejoin_failures < SECURE_REJOIN_FAILURE_LIMIT {
            self.diagnostics
                .record(DiagnosticEvent::SecureRejoinPending {
                    failures: self.consecutive_rejoin_failures,
                });
            return false;
        }
        self.diagnostics
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
        let reading = self.environment.sample().await;
        match &reading {
            Some(reading) => self.environment.log_reading(reading),
            None => self
                .diagnostics
                .record(DiagnosticEvent::EnvironmentReadFailed),
        }

        let environment = self.node.profile_mut().component_mut();
        if let Some(reading) = reading {
            environment.update_environment(TemperatureHumidityMeasurement {
                temperature_centi_celsius: reading.temperature_centi_celsius,
                humidity_centi_percent: reading.humidity_centi_percent,
            });
            if let Some(pressure) = reading.pressure_tenth_kpa {
                environment.update_pressure(pressure);
            }
        }

        let battery = self.battery.sample().await;
        self.diagnostics.record(DiagnosticEvent::Battery {
            millivolts: battery.millivolts,
            percentage: battery.measurement.percentage_remaining / 2,
        });
        self.node
            .profile_mut()
            .component_mut()
            .update_battery(battery.measurement);
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
            self.platform.led_on();
        }

        self.read_sensors().await;

        if let Err(error) = self.node.configure_default_reporting() {
            self.node_failure(NodeError::Profile(error));
        }
        self.diagnostics
            .record(DiagnosticEvent::DefaultReportingConfigured);

        let now = self.platform.now();
        self.last_report = now;
        self.last_tick = now;
        self.last_rejoin_attempt = now;
        self.rejoin_count = 0;
        self.last_annce = now;
        self.interview_done = false;
        self.node.reset_remote_reporting();
        let joined = self.node.device().is_joined();
        self.was_fast_polling = joined;
        self.fast_poll_until = if joined {
            self.diagnostics.record(DiagnosticEvent::FastPollStarted {
                duration_secs: FAST_POLL_DURATION_SECS,
            });
            self.platform.led_on();
            I::add_millis(now, FAST_POLL_DURATION_SECS * 1_000)
        } else {
            now
        };
        self.annce_retries_left = if joined && !resumed_at_boot {
            ANNCE_RETRIES
        } else {
            0
        };
    }

    /// Handle one control-plane [`StackEvent`], explicitly — every variant
    /// has an arm, so a new variant fails to compile here rather than
    /// silently falling through a wildcard. Returns whether the event
    /// represents coordinator/network activity that should extend the
    /// fast-poll window.
    async fn handle_control_event(&mut self, event: &StackEvent) -> bool {
        match event {
            StackEvent::Joined {
                short_address,
                channel,
                pan_id,
            } => {
                self.diagnostics.record(DiagnosticEvent::Joined {
                    short_address: *short_address,
                    channel: *channel,
                    pan_id: *pan_id,
                });
                self.reset_post_join_state();
                self.checkpoint_security();
                true
            }
            StackEvent::Left => {
                self.diagnostics.record(DiagnosticEvent::Left);
                self.platform.led_off();
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
                self.diagnostics.record(DiagnosticEvent::AttributeReport {
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
                self.diagnostics
                    .record(DiagnosticEvent::ReportingConfigured {
                        cluster_id: *cluster_id,
                        configured,
                        expected,
                    });
                if complete && !self.interview_done {
                    self.diagnostics
                        .record(DiagnosticEvent::InterviewConfigurationComplete {
                            configured,
                            expected,
                        });
                    self.fast_poll_until = I::add_millis(self.platform.now(), 5_000);
                    self.interview_done = true;
                    self.platform.led_off();
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
                    self.diagnostics.record(DiagnosticEvent::ReportingRejected {
                        cluster_id: *cluster_id,
                        configured: self.node.remote_reporting_cluster_count(),
                        expected: self.node.expected_report_clusters(),
                    });
                    return policy::configure_reporting_requests_generic_extension(
                        self.node.remote_reporting_is_complete(),
                    );
                }
                self.diagnostics.record(DiagnosticEvent::UnhandledCommand {
                    src_addr: *src_addr,
                    cluster_id: *cluster_id,
                    command_id: *command_id,
                });
                false
            }
            StackEvent::CommissioningComplete { success: true } => {
                self.diagnostics
                    .record(DiagnosticEvent::CommissioningComplete { success: true });
                self.consecutive_rejoin_failures = 0;
                self.checkpoint_security();
                false
            }
            StackEvent::CommissioningComplete { success: false } => {
                self.diagnostics
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
                self.diagnostics.record(DiagnosticEvent::DefaultResponse {
                    src_addr: *src_addr,
                    cluster_id: *cluster_id,
                    command_id: *command_id,
                    status: *status,
                });
                false
            }
            StackEvent::PermitJoinChanged { open } => {
                self.diagnostics
                    .record(DiagnosticEvent::PermitJoinChanged { open: *open });
                false
            }
            StackEvent::ReportSent => {
                self.diagnostics.record(DiagnosticEvent::ReportSent);
                false
            }
            StackEvent::OtaImageAvailable { .. }
            | StackEvent::OtaProgress { .. }
            | StackEvent::OtaComplete
            | StackEvent::OtaFailed
            | StackEvent::OtaDelayedActivation { .. } => {
                // This first shared lifecycle slice accepts a bare
                // `DeviceProfile<C>`, not an OTA-decorated profile. These
                // events should therefore be unreachable, but remain
                // explicit so a future OTA capability cannot be added
                // without deliberately defining its lifecycle.
                self.diagnostics.record(DiagnosticEvent::OtaEventIgnored);
                false
            }
            StackEvent::LeaveRequested => {
                self.diagnostics.record(DiagnosticEvent::LeaveRequested);
                self.rejoin_after_reset().await
            }
            StackEvent::BasicResetToFactoryDefaults => {
                self.diagnostics
                    .record(DiagnosticEvent::BasicResetToFactoryDefaults);
                false
            }
            StackEvent::RejoinRequested => {
                self.diagnostics.record(DiagnosticEvent::RejoinRequested);
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
                            self.fast_poll_until =
                                I::add_millis(self.platform.now(), FAST_POLL_DURATION_SECS * 1_000);
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
    async fn service_periodic_joined(&mut self, now: I::Instant) {
        let elapsed_s = I::elapsed_millis(now, self.last_report) / 1_000;
        if elapsed_s >= REPORT_INTERVAL_SECS {
            self.last_report = now;
            self.read_sensors().await;
        }

        let tick_elapsed = (I::elapsed_millis(now, self.last_tick) / 1_000).min(60);
        if tick_elapsed != 0 {
            self.last_tick = I::add_millis(self.last_tick, tick_elapsed * 1_000);
        }
        self.tick_and_handle(tick_elapsed as u16).await;

        // Matches the original firmware exactly: these two checks run
        // unconditionally after the tick above, even on the (rare) iteration
        // where that tick's event just left the network — `is_identifying`
        // naturally reads false and a stale `send_device_annce` is already
        // tolerated (`let _ = ...`) below.
        // Identify LED blink.
        if self.node.device().is_identifying(self.endpoint) {
            self.platform.led_toggle();
        }

        // Device_annce retry.
        let now2 = self.platform.now();
        if self.annce_retries_left > 0
            && I::elapsed_millis(now2, self.last_annce) / 1_000 >= ANNCE_RETRY_SECS
        {
            self.annce_retries_left -= 1;
            self.last_annce = now2;
            self.diagnostics
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
    async fn service_unjoined(&mut self, now: I::Instant) {
        if I::elapsed_millis(now, self.last_rejoin_attempt) >= 1_000 {
            // Double blink.
            self.platform.led_on();
            self.platform.delay_ms(80).await;
            self.platform.led_off();
            self.platform.delay_ms(120).await;
            self.platform.led_on();
            self.platform.delay_ms(80).await;
            self.platform.led_off();
        }

        if I::elapsed_millis(now, self.last_rejoin_attempt) / 1_000 >= JOIN_RETRY_SECS {
            self.rejoin_count = self.rejoin_count.wrapping_add(1);
            self.last_rejoin_attempt = self.platform.now();
            self.diagnostics.record(DiagnosticEvent::JoinRetry {
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

    async fn handle_button_press(&mut self) {
        // Check for long press (3s = factory reset).
        let held_long = self
            .platform
            .button_held_for(BUTTON_LONG_PRESS_SECS * 1_000)
            .await;

        if held_long {
            self.diagnostics
                .record(DiagnosticEvent::FactoryResetRequested);
            if self.factory_reset().await {
                self.diagnostics
                    .record(DiagnosticEvent::SecurityResetRebooting);
            }
            for _ in 0..5u8 {
                self.platform.led_on();
                self.platform.delay_ms(100).await;
                self.platform.led_off();
                self.platform.delay_ms(100).await;
            }
            self.platform.reset();
        } else if self.node.device().is_joined() {
            let configured = self.node.remote_reporting_cluster_count();
            let expected = self.node.expected_report_clusters();
            self.diagnostics.record(DiagnosticEvent::ForceReport {
                configured,
                expected,
            });
            self.platform.led_on();
            self.read_sensors().await;
            self.node.device_mut().reporting_mut().force_all_due();
            self.tick_and_handle(0).await;
            self.last_report = self.platform.now();
            self.fast_poll_until = I::add_millis(self.platform.now(), 5_000);
            self.platform.led_off();
        } else {
            self.diagnostics.record(DiagnosticEvent::ButtonJoin);
            if self.join_or_resume_with_rejoin_tracking().await {
                self.reset_post_join_state();
            }
        }
        self.platform.delay_ms(300).await;
    }

    pub async fn run(&mut self) -> ! {
        self.cold_start().await;

        loop {
            let now = self.platform.now();
            let in_fast_poll = now < self.fast_poll_until;
            let base_poll_ms = if in_fast_poll {
                FAST_POLL_MS
            } else {
                SLOW_POLL_SECS * 1000
            };
            let run_again_ms = self
                .run_again_deadline
                .map(|deadline| I::elapsed_millis(deadline, now));
            let poll_ms = policy::resolve_poll_delay_ms(base_poll_ms, run_again_ms);

            // Log transition from fast→slow poll.
            if self.was_fast_polling && !in_fast_poll {
                let cfg = self.node.remote_reporting_cluster_count();
                self.diagnostics.record(DiagnosticEvent::FastPollStopped {
                    configured: cfg,
                    expected: self.node.expected_report_clusters(),
                });
                self.was_fast_polling = false;
                if !self.interview_done {
                    self.platform.led_off();
                }
            } else if in_fast_poll {
                self.was_fast_polling = true;
            }

            // Sleep until button or poll timer wake.
            if self.node.device().is_joined() {
                let prepare_result = {
                    let mac = self.node.device_mut().mac_mut();
                    self.radio_power.prepare_for_sleep(mac)
                };
                if prepare_result.is_err() {
                    self.diagnostics
                        .record(DiagnosticEvent::RadioSleepPreparationFailed);
                }
            }
            match self.platform.wait_for_wake(poll_ms).await {
                WakeReason::Button => self.handle_button_press().await,
                WakeReason::Timer => {}
            }

            if self.node.device().is_joined() {
                self.service_joined_polls().await;
                self.service_periodic_joined(self.platform.now()).await;
            } else {
                self.service_unjoined(self.platform.now()).await;
            }
        }
    }
}
