//! nRF52 sensor application state machine, shared by every nRF sensor
//! product (nRF52840-DK and nRF52833-DK today).
//!
//! Extracted unchanged in behavior from the single-file `main.rs` the
//! nRF52840 firmware used before: the same bounded (four-round) MAC
//! receive/poll window, the same two-level fast/slow polling with a
//! post-join fast-poll window ended early by a real `ConfigureReporting`
//! from the coordinator, the same Device_annce retry policy (skipped only
//! for the very first cold-boot silent resume — see
//! [`SensorApp::cold_start`]), the same sensor/report cadence, button
//! behavior, and durable checkpointing at the same points in the
//! lifecycle.
//!
//! Generalizing it out of `examples/nrf52840-sensor` changed no state
//! transition, timing constant, or persistence point. The only difference
//! is that the four product-specific couplings it used to import directly
//! are now type parameters bound by the composition root:
//!
//! | was | now |
//! |-----|-----|
//! | `nrf52840_sensor_product::storage::SecurityStore` | `S: SecurityStateStore` |
//! | `nrf52840_sensor_product::profile::SensorProfile` | `DeviceProfile<C>` |
//! | `nrf52840_sensor_product::ENDPOINT` | `profile.endpoint()` (same value) |
//! | `nrf52840_sensor_product::battery::*` | `B: BatteryPolicy` |
//! | `Temp` / `crate::sensor::Sensor` (cfg-selected) | `E: EnvironmentSource` |
//!
//! All four are zero-cost: every parameter is monomorphized, and the
//! battery policy resolves to the same product arithmetic that used to be
//! called directly.
//!
//! Three behaviors are new relative to the original `main.rs`:
//!
//! - Every [`StackEvent`] variant is now matched explicitly in
//!   [`handle_control_event`](SensorApp::handle_control_event), instead of a
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
//!   [`SECURE_REJOIN_FAILURE_LIMIT`].

use embassy_futures::select::{Either, select};
use embassy_nrf::gpio;
use embassy_nrf::saadc::Saadc;
use embassy_time::{Duration, Instant, Timer};

use defmt::{debug, info, warn};

use zigbee_mac::nrf::NrfMac;
use zigbee_runtime::event_loop::{StackEvent, StartError, TickResult};
use zigbee_runtime::node::{NodeError, ZigbeeNode};
use zigbee_runtime::profile::{
    ApplicationProfile, DeviceProfile, ProfileComponent, TemperatureHumidityMeasurement,
};
use zigbee_runtime::security_store::{SecurityStateStore, SecurityStoreError};

use crate::battery::BatteryPolicy;
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

/// Radio + RNG-backed MAC driver every nRF sensor product uses.
pub type SensorMac =
    NrfMac<'static, embassy_nrf::peripherals::RADIO, embassy_nrf::peripherals::RNG>;

type SensorNode<'a, S, C> = ZigbeeNode<'a, SensorMac, S, DeviceProfile<C>>;

pub fn persistence_failure(error: SecurityStoreError) -> ! {
    match error {
        SecurityStoreError::NotFound => defmt::error!("Security persistence failed: not found"),
        SecurityStoreError::Corrupt => defmt::error!("Security persistence failed: corrupt state"),
        SecurityStoreError::Full => defmt::error!("Security persistence failed: full"),
        SecurityStoreError::Hardware => defmt::error!("Security persistence failed: hardware"),
        SecurityStoreError::CounterExhausted => {
            defmt::error!("Security persistence failed: counter exhausted")
        }
        SecurityStoreError::GenerationExhausted => {
            defmt::error!("Security persistence failed: generation exhausted")
        }
    }
    core::panic!("security persistence failure");
}

#[inline(never)]
fn node_failure(error: NodeError) -> ! {
    match error {
        NodeError::Persistence(error) => persistence_failure(error),
        NodeError::Profile(error) => {
            defmt::error!("Profile error: {:?}", defmt::Debug2Format(&error));
            core::panic!("profile error");
        }
    }
}

pub struct SensorApp<'a, S, C, E, B>
where
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink,
    E: EnvironmentSource,
    B: BatteryPolicy,
{
    node: SensorNode<'a, S, C>,
    /// Cached `profile.endpoint()`. Identical to the product's `ENDPOINT`
    /// constant this used to import; read once so the hot paths do not
    /// re-borrow the profile just to name the endpoint.
    endpoint: u8,
    led: gpio::Output<'static>,
    button: gpio::Input<'static>,
    environment: E,
    saadc: Saadc<'static, 1>,
    last_report: Instant,
    last_tick: Instant,
    fast_poll_until: Instant,
    last_rejoin_attempt: Instant,
    rejoin_count: u8,
    annce_retries_left: u8,
    last_annce: Instant,
    was_fast_polling: bool,
    interview_done: bool,
    consecutive_rejoin_failures: u8,
    /// Absolute deadline from the most recent `TickResult::RunAgain`.
    ///
    /// Storing an `Instant`, rather than retaining the relative duration,
    /// ensures time spent processing incoming frames or button activity does
    /// not restart the runtime's requested delay.
    run_again_deadline: Option<Instant>,
    battery: core::marker::PhantomData<B>,
}

impl<'a, S, C, E, B> SensorApp<'a, S, C, E, B>
where
    S: SecurityStateStore,
    C: ProfileComponent + EnvironmentSink,
    E: EnvironmentSource,
    B: BatteryPolicy,
{
    pub fn new(
        node: SensorNode<'a, S, C>,
        led: gpio::Output<'static>,
        button: gpio::Input<'static>,
        environment: E,
        saadc: Saadc<'static, 1>,
    ) -> Self {
        let now = Instant::now();
        let endpoint = node.profile().endpoint();
        Self {
            node,
            endpoint,
            led,
            button,
            environment,
            saadc,
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
            battery: core::marker::PhantomData,
        }
    }

    fn checkpoint_security(&mut self) {
        if let Err(error) = self.node.checkpoint_security() {
            persistence_failure(error);
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
            Err(error) => node_failure(error),
        };
        if let Some(delay_ms) = policy::run_again_delay_ms(&result) {
            self.run_again_deadline =
                Some(Instant::now() + Duration::from_millis(u64::from(delay_ms)));
        }
        result
    }

    /// Tick the runtime, and if it produced a control event, run it through
    /// [`handle_control_event`](Self::handle_control_event).
    async fn tick_and_handle(&mut self, elapsed_secs: u16) {
        let result = self.tick(elapsed_secs).await;
        if let TickResult::Event(event) = &result {
            if self.handle_control_event(event).await {
                self.fast_poll_until =
                    Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
            }
        }
    }

    fn reset_post_join_state(&mut self) {
        let now = Instant::now();
        self.led.set_low(); // ON
        self.fast_poll_until = now + Duration::from_secs(FAST_POLL_DURATION_SECS);
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
                info!(
                    "Joined/resumed network: addr=0x{:04X} ch={} pan=0x{:04X}",
                    short_address,
                    self.node.device().channel(),
                    self.node.device().pan_id()
                );
                // The runtime owns the R22 End Device Timeout lifecycle: a
                // fresh join or secured rejoin sends exactly one initial
                // request, and a silent resume reuses the persisted parent
                // relationship. Sending one here as well would duplicate
                // the negotiation.
                self.checkpoint_security();
                true
            }
            Err(StartError::InitFailed) => {
                warn!("Zigbee initialization failed");
                false
            }
            Err(StartError::CommissioningFailed(status)) => {
                warn!("Commissioning failed: status=0x{:02X}", status as u8);
                false
            }
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
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
                info!("Secure rejoin succeeded: addr=0x{:04X}", short_address);
                self.checkpoint_security();
                true
            }
            Err(StartError::InitFailed) => {
                warn!("Secure rejoin initialization failed");
                self.record_failed_rejoin().await
            }
            Err(StartError::CommissioningFailed(status)) => {
                warn!("Secure rejoin failed: status=0x{:02X}", status as u8);
                self.record_failed_rejoin().await
            }
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
        }
    }

    async fn factory_reset(&mut self) -> bool {
        self.run_again_deadline = None;
        match self.node.factory_reset().await {
            Ok(()) => true,
            Err(StartError::InitFailed) => {
                warn!("Factory reset initialization failed");
                false
            }
            Err(StartError::CommissioningFailed(status)) => {
                warn!("Factory reset failed: status=0x{:02X}", status as u8);
                false
            }
            Err(StartError::PersistenceFailed(error)) => persistence_failure(error),
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
            warn!(
                "Secure rejoin pending — will retry (failures={})",
                self.consecutive_rejoin_failures
            );
            return false;
        }
        warn!(
            "Secure rejoin failed {} times — resetting and rejoining fresh",
            self.consecutive_rejoin_failures
        );
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
            None => self.environment.log_failure(),
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

        // Battery
        let mut buf = [0i16; 1];
        self.saadc.sample(&mut buf).await;
        let measurement = B::measurement(buf[0]);
        info!(
            "Battery: {}mV ({}%)",
            B::millivolts(buf[0]),
            measurement.percentage_remaining / 2
        );
        self.node
            .profile_mut()
            .component_mut()
            .update_battery(measurement);
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
            Err(error) => persistence_failure(error),
        };

        if self.join_or_resume_with_rejoin_tracking().await {
            self.led.set_low();
        }

        self.read_sensors().await;

        if let Err(error) = self.node.configure_default_reporting() {
            node_failure(NodeError::Profile(error));
        }
        info!("Default reporting configured");

        let now = Instant::now();
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
            info!("Fast poll ON ({}s) — post-join", FAST_POLL_DURATION_SECS);
            self.led.set_low();
            now + Duration::from_secs(FAST_POLL_DURATION_SECS)
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
                info!(
                    "Joined! addr=0x{:04X} ch={} pan=0x{:04X}",
                    short_address, channel, pan_id
                );
                self.reset_post_join_state();
                self.checkpoint_security();
                true
            }
            StackEvent::Left => {
                info!("Left network");
                self.led.set_high(); // OFF
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
                info!(
                    "Attribute report src=0x{:04X} ep={} cluster=0x{:04X} attr=0x{:04X}",
                    src_addr, endpoint, cluster_id, attr_id
                );
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
                info!(
                    "Remote ConfigureReporting: cluster=0x{:04X} {}/{} clusters",
                    cluster_id, configured, expected
                );
                if complete && !self.interview_done {
                    info!(
                        "Interview configuration complete: {}/{} clusters",
                        configured, expected
                    );
                    self.fast_poll_until = Instant::now() + Duration::from_secs(5);
                    self.interview_done = true;
                    self.led.set_high();
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
                    warn!(
                        "Remote ConfigureReporting rejected: cluster=0x{:04X} ({}/{} clusters)",
                        cluster_id,
                        self.node.remote_reporting_cluster_count(),
                        self.node.expected_report_clusters()
                    );
                    return policy::configure_reporting_requests_generic_extension(
                        self.node.remote_reporting_is_complete(),
                    );
                }
                info!(
                    "Unhandled command src=0x{:04X} cluster=0x{:04X} cmd=0x{:02X}",
                    src_addr, cluster_id, command_id
                );
                false
            }
            StackEvent::CommissioningComplete { success: true } => {
                info!("Commissioning: ok");
                self.consecutive_rejoin_failures = 0;
                self.checkpoint_security();
                false
            }
            StackEvent::CommissioningComplete { success: false } => {
                info!("Commissioning: failed");
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
                info!(
                    "Default response src=0x{:04X} cluster=0x{:04X} cmd=0x{:02X} status=0x{:02X}",
                    src_addr, cluster_id, command_id, status
                );
                false
            }
            StackEvent::PermitJoinChanged { open } => {
                info!("Permit join changed: open={}", open);
                false
            }
            StackEvent::ReportSent => {
                info!("Report sent");
                false
            }
            StackEvent::OtaImageAvailable { .. }
            | StackEvent::OtaProgress { .. }
            | StackEvent::OtaComplete
            | StackEvent::OtaFailed
            | StackEvent::OtaDelayedActivation { .. } => {
                // This product has no OTA client wired up (see
                // `products/nrf52840-sensor/src/profile.rs`); the runtime
                // should never actually emit these for this build, but log
                // rather than silently drop in case that ever changes.
                debug!("OTA event ignored — this firmware build has no OTA client");
                false
            }
            StackEvent::LeaveRequested => {
                info!("Leave requested by coordinator — resetting and rejoining");
                self.rejoin_after_reset().await
            }
            StackEvent::BasicResetToFactoryDefaults => {
                info!("Basic cluster attributes reset to factory defaults");
                false
            }
            StackEvent::RejoinRequested => {
                info!("Coordinator requested secure rejoin");
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
                        Err(error) => node_failure(error),
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
                                Instant::now() + Duration::from_secs(FAST_POLL_DURATION_SECS);
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
    async fn service_periodic_joined(&mut self, now: Instant) {
        let elapsed_s = now.saturating_duration_since(self.last_report).as_secs();
        if elapsed_s >= REPORT_INTERVAL_SECS {
            self.last_report = now;
            self.read_sensors().await;
        }

        let tick_elapsed = now
            .saturating_duration_since(self.last_tick)
            .as_secs()
            .min(60);
        if tick_elapsed != 0 {
            self.last_tick += Duration::from_secs(tick_elapsed);
        }
        self.tick_and_handle(tick_elapsed as u16).await;

        // Matches the original firmware exactly: these two checks run
        // unconditionally after the tick above, even on the (rare) iteration
        // where that tick's event just left the network — `is_identifying`
        // naturally reads false and a stale `send_device_annce` is already
        // tolerated (`let _ = ...`) below.
        // Identify LED blink.
        if self.node.device().is_identifying(self.endpoint) {
            self.led.toggle();
        }

        // Device_annce retry.
        let now2 = Instant::now();
        if self.annce_retries_left > 0
            && now2.saturating_duration_since(self.last_annce).as_secs() >= ANNCE_RETRY_SECS
        {
            self.annce_retries_left -= 1;
            self.last_annce = now2;
            info!("Re-sending Device_annce ({} left)", self.annce_retries_left);
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
    async fn service_unjoined(&mut self, now: Instant) {
        if now
            .saturating_duration_since(self.last_rejoin_attempt)
            .as_secs()
            >= 1
        {
            // Double blink.
            self.led.set_low();
            Timer::after(Duration::from_millis(80)).await;
            self.led.set_high();
            Timer::after(Duration::from_millis(120)).await;
            self.led.set_low();
            Timer::after(Duration::from_millis(80)).await;
            self.led.set_high();
        }

        if now
            .saturating_duration_since(self.last_rejoin_attempt)
            .as_secs()
            >= JOIN_RETRY_SECS
        {
            self.rejoin_count = self.rejoin_count.wrapping_add(1);
            self.last_rejoin_attempt = Instant::now();
            info!("Not joined — retrying (attempt {})…", self.rejoin_count);
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
        let held_long = matches!(
            select(
                self.button.wait_for_rising_edge(),
                Timer::after(Duration::from_secs(BUTTON_LONG_PRESS_SECS)),
            )
            .await,
            Either::Second(_)
        );

        if held_long {
            info!("FACTORY RESET");
            if self.factory_reset().await {
                info!("Security state reset — rebooting");
            }
            for _ in 0..5u8 {
                self.led.set_low();
                Timer::after(Duration::from_millis(100)).await;
                self.led.set_high();
                Timer::after(Duration::from_millis(100)).await;
            }
            cortex_m::peripheral::SCB::sys_reset();
        } else if self.node.device().is_joined() {
            let configured = self.node.remote_reporting_cluster_count();
            let expected = self.node.expected_report_clusters();
            info!(
                "Button → force report (interview configuration {}/{})",
                configured, expected
            );
            self.led.set_low();
            self.read_sensors().await;
            self.node.device_mut().reporting_mut().force_all_due();
            self.tick_and_handle(0).await;
            self.last_report = Instant::now();
            self.fast_poll_until = Instant::now() + Duration::from_secs(5);
            self.led.set_high();
        } else {
            info!("Button → join");
            if self.join_or_resume_with_rejoin_tracking().await {
                self.reset_post_join_state();
            }
        }
        Timer::after(Duration::from_millis(300)).await;
    }

    pub async fn run(&mut self) -> ! {
        self.cold_start().await;

        loop {
            let now = Instant::now();
            let in_fast_poll = now < self.fast_poll_until;
            let base_poll_ms = if in_fast_poll {
                FAST_POLL_MS
            } else {
                SLOW_POLL_SECS * 1000
            };
            let run_again_ms = self
                .run_again_deadline
                .map(|deadline| deadline.saturating_duration_since(now).as_millis());
            let poll_ms = policy::resolve_poll_delay_ms(base_poll_ms, run_again_ms);

            // Log transition from fast→slow poll.
            if self.was_fast_polling && !in_fast_poll {
                let cfg = self.node.remote_reporting_cluster_count();
                info!(
                    "Fast poll OFF — remote client configured {}/{} report clusters",
                    cfg,
                    self.node.expected_report_clusters()
                );
                self.was_fast_polling = false;
                if !self.interview_done {
                    self.led.set_high(); // OFF
                }
            } else if in_fast_poll {
                self.was_fast_polling = true;
            }

            // Sleep until button or poll timer wake.
            if self.node.device().is_joined()
                && self
                    .node
                    .device_mut()
                    .mac_mut()
                    .enter_low_power_idle()
                    .is_err()
            {
                warn!("Failed to disable RADIO before poll sleep");
            }
            match select(
                self.button.wait_for_falling_edge(),
                Timer::after(Duration::from_millis(poll_ms)),
            )
            .await
            {
                Either::First(_) => self.handle_button_press().await,
                Either::Second(_) => {} // Normal timeout — proceed to poll.
            }

            if self.node.device().is_joined() {
                self.service_joined_polls().await;
                self.service_periodic_joined(Instant::now()).await;
            } else {
                self.service_unjoined(Instant::now()).await;
            }
        }
    }
}
