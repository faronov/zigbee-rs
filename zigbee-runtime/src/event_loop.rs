//! Event loop — drives the Zigbee stack processing pipeline.
//!
//! The event loop is the heartbeat of a Zigbee device. It:
//! 1. Processes pending user actions (join/leave)
//! 2. Ticks the ZCL reporting engine
//! 3. Sends any due attribute reports via APS→NWK→MAC
//! 4. Manages sleep/wake for end devices
//!
//! # Usage
//! The application drives the event loop by calling `tick()` periodically
//! and `receive()` + `process_incoming()` for incoming frames:
//!
//! ```rust,no_run,ignore
//! loop {
//!     match select(device.receive(), Timer::after(Duration::from_secs(10))).await {
//!         Either::First(Ok(frame)) => {
//!             if let Some(event) = device.process_incoming(&frame) {
//!                 handle_event(event);
//!             }
//!         }
//!         Either::First(Err(_)) => {} // MAC error
//!         Either::Second(_) => {
//!             // Timer fired — tick reporting and read sensor
//!             let result = device.tick(10).await;
//!             match result {
//!                 TickResult::Event(evt) => handle_event(evt),
//!                 _ => {}
//!             }
//!         }
//!     }
//! }
//! ```

use core::future::Future;

use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsStatus, ApsTxOptions};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;
#[cfg(any(feature = "finding-binding", feature = "finding-binding-target"))]
use zigbee_zcl::clusters::Cluster;

use crate::UserAction;

fn advance_millis(now_ms: u32, elapsed_secs: u16) -> u32 {
    now_ms.wrapping_add((elapsed_secs as u32) * 1000)
}

pub(crate) fn automatic_poll_due(
    automatic_polling: bool,
    sleepy: bool,
    commissioning_active: bool,
    interval_due: bool,
) -> bool {
    automatic_polling && sleepy && (commissioning_active || interval_due)
}

/// Map a power-manager [`SleepDecision`](crate::power::SleepDecision) to the
/// [`TickResult`] returned by the joined tick.
///
/// Factored out (and kept non-generic) so the two joined-tick tails share one
/// behaviour: a routing role reaches it through the out-of-line
/// [`ZigbeeDevice::tick_power_state`], while a sleepy end device inlines it at
/// the single tail return of [`ZigbeeDevice::tick_joined`] (compile-time
/// `CAN_ROUTE` split) so the large `TickResult` is built straight into the
/// caller instead of copied back through an extra call frame.
#[inline]
pub(crate) fn sleep_decision_to_tick(decision: crate::power::SleepDecision) -> TickResult {
    match decision {
        crate::power::SleepDecision::StayAwake => TickResult::Idle,
        crate::power::SleepDecision::LightSleep(ms) => TickResult::RunAgain(ms),
        crate::power::SleepDecision::DeepSleep(ms) => TickResult::RunAgain(ms),
    }
}

/// Events that the stack can generate for the application.
#[derive(Debug)]
pub enum StackEvent {
    /// Device joined the network successfully.
    Joined {
        short_address: u16,
        channel: u8,
        pan_id: u16,
    },
    /// Device left the network.
    Left,
    /// Attribute report received from another device.
    AttributeReport {
        src_addr: u16,
        endpoint: u8,
        cluster_id: u16,
        attr_id: u16,
    },
    /// Command received from another device.
    CommandReceived {
        src_addr: u16,
        /// Remote APS endpoint that sent the command.
        source_endpoint: u8,
        /// Local endpoint that received the command.
        endpoint: u8,
        cluster_id: u16,
        /// Whether this is a ZCL foundation or cluster-specific command.
        frame_type: zigbee_zcl::frame::ZclFrameType,
        command_id: u8,
        /// ZCL sequence number (needed for response frames).
        seq_number: u8,
        payload: heapless::Vec<u8, 64>,
    },
    /// BDB commissioning completed.
    CommissioningComplete { success: bool },
    /// Default Response received from a remote device.
    DefaultResponse {
        src_addr: u16,
        endpoint: u8,
        cluster_id: u16,
        /// The command ID that this is responding to.
        command_id: u8,
        /// Status code from the remote device.
        status: u8,
    },
    /// A remote ZCL client successfully configured attribute reporting.
    ///
    /// Emitted **only** after a non-empty, well-formed global Configure
    /// Reporting (0x06, client→server) command made entirely of Send-direction
    /// records was fully processed and *every* status record returned
    /// `Success`. An empty or malformed command, a receive-only or mixed
    /// command, an unsupported or unreportable attribute, an invalid/disabled
    /// data type, a reporting-table capacity failure, or any other
    /// unsuccessful record still produces the generic
    /// [`CommandReceived`](Self::CommandReceived) event instead, so an
    /// application keying interview completion off this event can never count
    /// a rejected or inbound-reporting-only configuration.
    ///
    /// This is what "the remote client finished configuring reporting"
    /// actually means; it is unrelated to defaults the product configured for
    /// itself (see [`crate::remote_reporting`]).
    ReportingConfigured {
        src_addr: u16,
        /// Remote APS endpoint that sent the command.
        source_endpoint: u8,
        /// Local endpoint whose cluster was configured.
        endpoint: u8,
        cluster_id: u16,
        /// Distinct clusters a remote client has now configured on
        /// `endpoint`, including this one and any unrelated server clusters.
        /// This generic count is diagnostic only; profile completion must
        /// check the profile's exact expected cluster IDs.
        configured_clusters: usize,
    },
    /// Permit joining status changed.
    PermitJoinChanged { open: bool },
    /// A parsed APSME security command is ready for Trust Center policy.
    ///
    /// The APS layer has already authenticated the transport and validated
    /// the command's wire-level invariants. Coordinator policy decides the
    /// resulting admission, key, or removal action.
    ApsSecurityIndication(zigbee_aps::apsme::ApsmeSecurityIndication),
    /// A validated, NWK-authenticated Device_annce from an operational peer.
    DeviceAnnounced {
        address: zigbee_types::IeeeAddress,
        short_address: zigbee_types::ShortAddress,
        capabilities: u8,
    },
    /// Attribute report was sent successfully.
    ReportSent,
    /// OTA: New image available from server.
    OtaImageAvailable { version: u32, size: u32 },
    /// OTA: Download progress update.
    OtaProgress { percent: u8 },
    /// OTA: Image is verified and ready for application-controlled activation.
    OtaComplete,
    /// OTA: Upgrade failed.
    OtaFailed,
    /// OTA: Server requested delayed activation — reboot after `delay_secs`.
    OtaDelayedActivation { delay_secs: u32 },
    /// Basic cluster Reset to Factory Defaults: reset application attributes
    /// only; network state, frame counters, groups, and bindings stay intact.
    BasicResetToFactoryDefaults,
    /// NWK Leave command received from coordinator — device should rejoin.
    LeaveRequested,
    /// NWK Leave command explicitly requested a secured network rejoin.
    RejoinRequested,
}

/// Stack tick result — tells the application what to do next.
#[derive(Debug)]
pub enum TickResult {
    /// Nothing happened, consider sleeping.
    Idle,
    /// Event(s) occurred — process them.
    Event(StackEvent),
    /// Stack needs to run again soon (within ms).
    RunAgain(u32),
}

/// Errors from device start/join/leave operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StartError {
    /// BDB initialization failed.
    InitFailed,
    /// BDB commissioning (steering/formation) failed, with BDB status code.
    CommissioningFailed(zigbee_bdb::BdbStatus),
    /// Durable security-state storage failed.
    PersistenceFailed(crate::security_store::SecurityStoreError),
}

/// Static fresh-start selection for pending user actions.
///
/// The marker is consumed by the store-backed tick before async lowering.
/// Typed router application frontends therefore materialize only their allowed
/// join future even though the legacy generic tick remains device-type aware.
pub(crate) trait ActionStartup<R: crate::role::DeviceRole> {
    fn start<M: MacDriver>(
        device: &mut crate::ZigbeeDevice<M, R>,
    ) -> impl Future<Output = Result<u16, StartError>>;

    fn start_with_security_store<'a, M, S>(
        device: &'a mut crate::ZigbeeDevice<M, R>,
        store: &'a mut S,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: crate::security_store::SecurityStateStore;
}

pub(crate) struct DynamicActionStartup;
pub(crate) struct SteeringActionStartup;
#[cfg(any(feature = "router", test))]
pub(crate) struct CoordinatorActionStartup;
#[cfg(any(feature = "router", test))]
pub(crate) struct DistributedActionStartup;

impl<R: crate::role::DeviceRole> ActionStartup<R> for DynamicActionStartup {
    fn start<M: MacDriver>(
        device: &mut crate::ZigbeeDevice<M, R>,
    ) -> impl Future<Output = Result<u16, StartError>> {
        device.start()
    }

    fn start_with_security_store<'a, M, S>(
        device: &'a mut crate::ZigbeeDevice<M, R>,
        store: &'a mut S,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: crate::security_store::SecurityStateStore,
    {
        device.start_or_resume_with_security_store(store)
    }
}

impl<R: crate::role::DeviceRole> ActionStartup<R> for SteeringActionStartup {
    fn start<M: MacDriver>(
        device: &mut crate::ZigbeeDevice<M, R>,
    ) -> impl Future<Output = Result<u16, StartError>> {
        device.start_steering()
    }

    fn start_with_security_store<'a, M, S>(
        device: &'a mut crate::ZigbeeDevice<M, R>,
        store: &'a mut S,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: crate::security_store::SecurityStateStore,
    {
        device.start_or_resume_steering_with_security_store(store)
    }
}

#[cfg(any(feature = "router", test))]
impl<R: crate::role::ParentRole> ActionStartup<R> for CoordinatorActionStartup {
    fn start<M: MacDriver>(
        device: &mut crate::ZigbeeDevice<M, R>,
    ) -> impl Future<Output = Result<u16, StartError>> {
        device.start_coordinator()
    }

    fn start_with_security_store<'a, M, S>(
        device: &'a mut crate::ZigbeeDevice<M, R>,
        store: &'a mut S,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: crate::security_store::SecurityStateStore,
    {
        device.start_or_resume_coordinator_with_security_store(store)
    }
}

#[cfg(any(feature = "router", test))]
impl<R: crate::role::ParentRole> ActionStartup<R> for DistributedActionStartup {
    fn start<M: MacDriver>(
        device: &mut crate::ZigbeeDevice<M, R>,
    ) -> impl Future<Output = Result<u16, StartError>> {
        device.start_distributed_network()
    }

    fn start_with_security_store<'a, M, S>(
        device: &'a mut crate::ZigbeeDevice<M, R>,
        store: &'a mut S,
    ) -> impl Future<Output = Result<u16, StartError>> + 'a
    where
        M: MacDriver,
        S: crate::security_store::SecurityStateStore,
    {
        device.start_or_resume_distributed_network_with_security_store(store)
    }
}

/// Largest Report Attributes ZCL frame: one unfragmented APS payload.
const REPORT_FRAME_CAP: usize = zigbee_aps::apsde::APS_MAX_PAYLOAD;
/// Global frame, server→client, disable default response (ZCL r8 §2.4.1.1).
const REPORT_FRAME_CONTROL: u8 = 0x18;
/// Report Attributes (ZCL r8 §2.5.11).
const REPORT_ATTRIBUTES_COMMAND: u8 = 0x0A;

/// Result of sending a set of collected attribute reports.
pub(crate) struct ReportSendOutcome {
    /// Report Attributes frames accepted by APS.
    pub(crate) frames_sent: usize,
    /// A record too large for any single frame was dropped.
    pub(crate) skipped_oversize: bool,
    /// First record not handed to APS and the error that stopped sending.
    pub(crate) failure: Option<(usize, SendError)>,
}

/// Errors returned while sending application ZCL traffic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendError {
    /// The device has not joined a network yet.
    NotJoined,
    /// The ZCL frame could not be serialized.
    Serialization,
    /// The serialized payload exceeded the fixed ZCL frame capacity.
    PayloadTooLong,
    /// APS rejected or failed the data request.
    Aps(ApsStatus),
}

/// Run one iteration of the Zigbee stack event loop.
///
/// This is designed for cooperative async scheduling:
/// - Call `tick()` periodically from your main loop
/// - It processes pending user actions and generates reports
/// - Returns quickly, never blocks indefinitely
///
/// The `elapsed_secs` parameter tells the reporting engine how much time
/// has passed since the last tick. Use the actual timer interval.
///
/// Pass registered cluster instances so the runtime can automatically
/// send attribute reports when they are due.
pub async fn stack_tick<M: MacDriver, R: crate::role::DeviceRole>(
    device: &mut crate::ZigbeeDevice<M, R>,
    elapsed_secs: u16,
    clusters: &mut [crate::ClusterRef<'_>],
) -> TickResult {
    device.tick(elapsed_secs, clusters).await
}

impl<M: MacDriver, R: crate::role::DeviceRole> crate::ZigbeeDevice<M, R> {
    /// Tick the Zigbee stack — process pending actions, send reports.
    ///
    /// Call this periodically. `elapsed_secs` is the time since the last tick.
    /// Pass registered cluster instances for automatic attribute reporting.
    #[inline(never)]
    pub async fn tick(
        &mut self,
        elapsed_secs: u16,
        clusters: &mut [crate::ClusterRef<'_>],
    ) -> TickResult {
        self.tick_identify_clusters(elapsed_secs);
        if let Some(action) = self.pending_action.take() {
            return self.handle_action(action).await;
        }
        if self.secure_rejoin_retry_due() {
            return self.retry_secure_rejoin().await;
        }

        self.flush_pending_responses().await;
        if !self.is_joined() {
            return TickResult::Idle;
        }

        // Keep this path direct: another async wrapper adds several KiB of
        // transient stack on small Series-1 devices.
        self.run_aps_maintenance().await;
        self.run_nwk_maintenance(elapsed_secs).await;
        R::ed_advance_timers(self, elapsed_secs);

        self.reporting.tick(elapsed_secs);
        #[cfg(any(feature = "finding-binding", feature = "finding-binding-target"))]
        self.apply_fb_target_request();
        #[cfg(feature = "finding-binding")]
        self.run_finding_binding_tick(elapsed_secs).await;
        #[cfg(all(feature = "finding-binding-target", not(feature = "finding-binding")))]
        self.run_finding_binding_target_tick(elapsed_secs);
        self.send_due_reports(clusters).await;
        self.update_pending_tx_flag();

        // GSDK-style event-driven commissioning: advance the unique Trust
        // Center link-key handshake before normal tick result generation.
        // A terminal transition is returned immediately. If it is still in
        // progress, continue through polling and preserve any application event
        // produced there for this tick.
        #[cfg(feature = "centralized-tclk")]
        if self.bdb.tclk_exchange_active()
            && let Some(event) = self.advance_commissioning().await
        {
            return TickResult::Event(event);
        }

        let now_ms = self.advance_power_clock(elapsed_secs);
        // The poll runs first so an End Device Timeout Response that arrives
        // this tick cancels the response wait before it is serviced. Reached
        // through the role hook so a routing monomorphization never names the
        // poll future (and therefore never links the second receive path).
        let poll_event = R::ed_run_poll(self, now_ms, clusters).await;
        R::ed_service(self).await;

        let result = if let Some(event) = poll_event {
            TickResult::Event(event)
        } else {
            self.tick_power_state(now_ms)
        };
        self.commissioning_tick_hint(result)
    }

    /// Advance post-network commissioning without a durable security store.
    ///
    /// This is the platform-independent equivalent of GSDK's scheduled
    /// update-tc-link-key event: normal maintenance runs first, then exactly
    /// one bounded security step is performed before polling/result generation.
    /// Returns `Some` only for a terminal transition.
    #[cfg(feature = "centralized-tclk")]
    async fn advance_commissioning(&mut self) -> Option<StackEvent> {
        match self.bdb.advance_tclk_exchange(None).await {
            zigbee_bdb::TclkProgress::InProgress => None,
            zigbee_bdb::TclkProgress::Complete => {
                self.state_dirty = true;
                Some(StackEvent::CommissioningComplete { success: true })
            }
            zigbee_bdb::TclkProgress::Failed(_) => {
                self.mark_left();
                Some(StackEvent::CommissioningComplete { success: false })
            }
        }
    }

    /// Shorten a non-event tick result while commissioning security is running.
    ///
    /// The handshake advances one bounded step per tick, so the application must
    /// come back quickly; an application event is never replaced by the hint.
    pub(crate) fn commissioning_tick_hint(&self, result: TickResult) -> TickResult {
        if !self.bdb.tclk_exchange_active() {
            return result;
        }
        match result {
            TickResult::Event(_) => result,
            TickResult::RunAgain(ms) => TickResult::RunAgain(ms.min(Self::COMMISSIONING_POLL_MS)),
            _ => TickResult::RunAgain(Self::COMMISSIONING_POLL_MS),
        }
    }

    async fn retry_secure_rejoin(&mut self) -> TickResult {
        log::info!("[Runtime] Retrying secure rejoin");
        match self.secure_rejoin().await {
            Ok(addr) => TickResult::Event(StackEvent::Joined {
                short_address: addr,
                channel: self.channel(),
                pan_id: self.pan_id(),
            }),
            Err(_) => TickResult::Event(StackEvent::CommissioningComplete { success: false }),
        }
    }

    #[inline(never)]
    pub(crate) async fn flush_pending_responses(&mut self) {
        while let Some(resp) = self.pending_responses.pop() {
            rt_trace!(
                "[RT] zcl_tx dst=0x{:04X} src_ep={} dst_ep={} cluster=0x{:04X} len={}",
                resp.dst_addr.0,
                resp.src_endpoint,
                resp.dst_endpoint,
                resp.cluster_id,
                resp.zcl_data.len(),
            );
            log::info!(
                "[Runtime] Sending ZCL response: dst=0x{:04X} ep={} cluster=0x{:04X} len={}",
                resp.dst_addr.0,
                resp.dst_endpoint,
                resp.cluster_id,
                resp.zcl_data.len(),
            );
            if let Err(_e) = self
                .send_zcl_frame(
                    resp.dst_addr,
                    resp.dst_endpoint,
                    resp.src_endpoint,
                    resp.cluster_id,
                    &resp.zcl_data,
                )
                .await
            {
                rt_trace!(
                    "[RT] zcl_tx_err dst=0x{:04X} cluster=0x{:04X}",
                    resp.dst_addr.0,
                    resp.cluster_id,
                );
                log::warn!(
                    "[Runtime] ZCL response send failed: dst=0x{:04X} ep={} cluster=0x{:04X}",
                    resp.dst_addr.0,
                    resp.dst_endpoint,
                    resp.cluster_id,
                );
            } else {
                rt_trace!(
                    "[RT] zcl_tx_ok dst=0x{:04X} cluster=0x{:04X}",
                    resp.dst_addr.0,
                    resp.cluster_id,
                );
            }
        }
    }

    #[inline(never)]
    pub(crate) async fn run_aps_maintenance(&mut self) {
        let aps = self.bdb.zdo_mut().aps_mut();
        let retransmit_frames = aps.age_ack_table();
        let radius = aps.nwk().nib().max_depth.saturating_mul(2);
        for retransmission in retransmit_frames.iter() {
            // An APS retry repeats the *original unicast* (R22 §2.2.5.2.2).
            // Broadcasting it instead would flood the network with a frame
            // only one device expects, and the acknowledgement being waited
            // for would still never arrive.
            let _ = aps
                .nwk_mut()
                .nlde_data_request(
                    retransmission.dst_addr,
                    radius,
                    &retransmission.frame,
                    true,
                    true,
                )
                .await;
        }
        aps.age_dup_table();
        // `age_ack_table` above also drives an in-flight fragmented
        // transaction; collect its final confirm once it finished.
        #[cfg(feature = "router")]
        self.collect_fragmented_send_confirm();
    }

    /// Drive periodic NWK maintenance for this device's role.
    ///
    /// All maintenance is now selected by *static dispatch* through
    /// [`DeviceRole::run_role_nwk_maintenance`](crate::role::DeviceRole::run_role_nwk_maintenance):
    /// - a leaf [`EndDevice`](crate::role::EndDevice) ages only its small
    ///   neighbour cache (no router/parent subgraph),
    /// - a [`RelayRouter`](crate::role::RelayRouter) runs routing-only
    ///   maintenance (permit-join expiry, router / link-status / route-table /
    ///   concentrator maintenance, pending routing TX),
    /// - a [`Router`](crate::role::Router) runs the full parent maintenance
    ///   sequence plus a due Parent Announce.
    ///
    /// Because the split is by role type, a non-parent monomorphization's `tick`
    /// future never contains the child-serving futures. The routing/parent
    /// bodies are additionally gated on the `router` capability feature, so a
    /// sensor build removes them from the image entirely.
    #[inline(never)]
    pub(crate) async fn run_nwk_maintenance(&mut self, elapsed_secs: u16) {
        R::run_role_nwk_maintenance(self, elapsed_secs).await;
    }

    /// End-device neighbour-cache aging — the leaf role's only NWK maintenance.
    ///
    /// A routing device ages the same table inside
    /// [`NwkLayer::tick_router_maintenance`], so this runs *only* for the
    /// [`EndDevice`](crate::role::EndDevice) role (dispatched from
    /// [`DeviceRole::run_role_nwk_maintenance`](crate::role::DeviceRole::run_role_nwk_maintenance))
    /// to preserve the LRU eviction ordering of its small neighbour cache
    /// without linking the routing / BTR / indirect / link-status subgraph.
    ///
    /// [`NwkLayer::tick_router_maintenance`]: zigbee_nwk::NwkLayer::tick_router_maintenance
    #[inline]
    pub(crate) fn run_end_device_nwk_maintenance(&mut self, elapsed_secs: u16) {
        self.bdb
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .tick_end_device_maintenance(elapsed_secs);
    }

    /// Forwarding-only (relay) NWK maintenance — the routing subset shared by a
    /// [`RelayRouter`](crate::role::RelayRouter) and a
    /// [`Router`](crate::role::Router).
    ///
    /// Runs permit-join expiry, router / link-status / route-table /
    /// concentrator maintenance and pending routing transmission, but **no**
    /// child End Device Timeout aging, MAC parent-command servicing or Parent
    /// Announce — a relay cannot accept or serve children. Present only in
    /// `router` builds; dispatched from
    /// [`DeviceRole::run_role_nwk_maintenance`](crate::role::DeviceRole::run_role_nwk_maintenance)
    /// for the [`RelayRouter`](crate::role::RelayRouter) role.
    #[cfg(feature = "router")]
    pub(crate) async fn run_relay_nwk_maintenance(&mut self, elapsed_secs: u16) {
        let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
        let _ = nwk.tick_permit_joining(elapsed_secs).await;
        nwk.tick_router_maintenance(elapsed_secs);
        await_out_of_line!(nwk.process_pending_routing());
    }

    /// Parent/router periodic maintenance — present only in `router` builds and
    /// dispatched only for the [`Router`](crate::role::Router) parent role.
    ///
    /// This is verbatim the pre-split maintenance sequence (Parent Announce
    /// *sending* excepted — that runs immediately after this in the role
    /// dispatch), so a router/coordinator executes exactly the same work in the
    /// same order. It extends the routing subset with the parent-only steps:
    /// End Device Timeout child aging and coupled eviction cleanup, MAC
    /// parent-command servicing and Parent Announce transaction aging. A relay
    /// or sensor never runs (or links) any of it.
    #[cfg(feature = "router")]
    pub(crate) async fn run_parent_nwk_maintenance(&mut self, elapsed_secs: u16)
    where
        R: crate::role::ParentRole,
    {
        let evicted = {
            let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
            let _ = nwk.tick_permit_joining(elapsed_secs).await;
            nwk.tick_router_maintenance(elapsed_secs);
            // R22 End Device Timeout aging: evict end-device children that
            // stopped keeping alive. Returns the evicted short addresses so
            // the runtime can drop the coupled deferred Update-Device state it
            // owns; the NWK layer already cleaned the indirect queue, routing,
            // replay counters and MAC Frame Pending for each one.
            let evicted = nwk.age_end_device_children(elapsed_secs);
            await_out_of_line!(nwk.process_pending_routing());
            evicted
        };
        for child in evicted {
            self.forget_evicted_child(child);
        }
        let _ = await_out_of_line!(self.service_parent_commands_inner());
        self.bdb
            .zdo_mut()
            .tick_parent_annce_transactions(elapsed_secs);
    }

    #[cfg(any(feature = "finding-binding", feature = "finding-binding-target"))]
    #[inline(never)]
    pub(crate) fn apply_fb_target_request(&mut self) {
        if let Some((ep, time_secs)) = self.bdb.fb_target_request.take()
            && let Some(entry) = self
                .identify_clusters
                .iter_mut()
                .find(|entry| entry.endpoint == ep)
        {
            let _ = entry.cluster.attributes_mut().set(
                zigbee_zcl::AttributeId(0x0000),
                zigbee_zcl::data_types::ZclValue::U16(time_secs),
            );
            log::info!(
                "[Runtime] F&B target: set IdentifyTime={}s on ep {}",
                time_secs,
                ep,
            );
        }
    }

    #[cfg(feature = "finding-binding")]
    #[inline(never)]
    pub(crate) async fn run_finding_binding_tick(&mut self, elapsed_secs: u16) {
        let _ = self.bdb.tick_finding_binding(elapsed_secs).await;
    }

    #[cfg(all(feature = "finding-binding-target", not(feature = "finding-binding")))]
    #[inline(never)]
    pub(crate) fn run_finding_binding_target_tick(&mut self, elapsed_secs: u16) {
        let _ = self.bdb.tick_finding_binding_target(elapsed_secs);
    }

    #[inline(never)]
    pub(crate) async fn send_due_reports(&mut self, clusters: &[crate::ClusterRef<'_>]) {
        for cr in clusters.iter() {
            let ep = cr.endpoint;
            let cid = cr.cluster.cluster_id().0;
            self.check_and_send_cluster_reports(ep, cid, cr.cluster.attributes())
                .await;
        }
    }

    pub(crate) fn update_pending_tx_flag(&mut self) {
        self.power
            .set_pending_tx(!self.pending_responses.is_empty());
    }

    pub(crate) fn advance_power_clock(&mut self, elapsed_secs: u16) -> u32 {
        self.power_now_ms = advance_millis(self.power_now_ms, elapsed_secs);
        self.power_now_ms
    }

    #[inline(never)]
    pub(crate) async fn run_sleepy_poll(
        &mut self,
        now_ms: u32,
        clusters: &mut [crate::ClusterRef<'_>],
    ) -> Option<StackEvent> {
        // Only a role that has a parent to poll materializes this branch. A
        // router/relay keeps its receiver on and takes every frame through the
        // durable receive path, so compiling the branch out removes a whole
        // second copy of `process_incoming` (and its NWK/APS instantiations)
        // from every routing image instead of leaving it as unreachable code.
        if !R::POLLS_PARENT {
            return None;
        }
        // A forced poll fetches an indirect End Device Timeout Response (or a
        // command the parent queued while we slept) and deliberately bypasses
        // the automatic-polling and sleepy gates: it is a keepalive obligation,
        // not an application poll.
        let forced = R::ed_take_forced_poll(self);
        if forced
            || automatic_poll_due(
                self.automatic_polling,
                self.is_sleepy(),
                self.bdb.tclk_exchange_active(),
                self.power.should_poll(now_ms),
            )
        {
            let indication = self.poll().await;
            // Failure accounting and recovery now live in the single `poll()`
            // choke point (which also covers application-driven OTA fast polls
            // that call `poll()` directly), so this path only needs to consume a
            // delivered frame. `forced` still selects the keepalive poll cadence
            // in the gate above.
            if let Ok(Some(frame)) = indication {
                return self.process_incoming(&frame, clusters).await;
            }
        }
        None
    }

    /// Map the power manager's sleep decision to a [`TickResult`] for a routing
    /// role.
    ///
    /// A routing device's joined tick is a large standalone future, so keeping
    /// this an out-of-line call avoids growing it. A sleepy end device instead
    /// inlines the same decision at the single tail return in
    /// [`Self::tick_joined`] (selected by the `CAN_ROUTE` role constant), which
    /// lets the large `TickResult` be constructed directly into the caller
    /// rather than copied back through an extra call frame.
    #[inline(never)]
    pub(crate) fn tick_power_state(&mut self, now_ms: u32) -> TickResult {
        sleep_decision_to_tick(self.power.decide(now_ms))
    }

    /// Handle a user-initiated action.
    ///
    /// `Toggle` is resolved to the join or leave it means for the current
    /// membership state *before* the dispatch below, so `start()` and
    /// `leave()` are each awaited from exactly one place. An `.await` embeds
    /// the awaited future's state machine in this one, so a second textual
    /// copy of either call is a second copy of the whole join or leave
    /// sequence in flash.
    #[inline(never)]
    pub(crate) async fn handle_action(&mut self, action: UserAction) -> TickResult {
        self.handle_action_with::<DynamicActionStartup>(action)
            .await
    }

    /// Handle one action with a statically selected fresh-start path.
    #[inline(never)]
    pub(crate) async fn handle_action_with<A>(&mut self, action: UserAction) -> TickResult
    where
        A: ActionStartup<R>,
    {
        let action = match action {
            UserAction::Toggle if self.is_joined() => {
                log::info!("[Runtime] User action: Toggle → Leave");
                UserAction::Leave
            }
            UserAction::Toggle => {
                log::info!("[Runtime] User action: Toggle → Join");
                UserAction::Join
            }
            other => other,
        };
        match action {
            // `Toggle` cannot reach here — it was resolved above — and is
            // folded into the join arm rather than into an unreachable branch
            // so no panic path is linked for a state that cannot occur.
            UserAction::Join | UserAction::Toggle => {
                if self.secure_rejoin_pending() {
                    return self.retry_secure_rejoin().await;
                }
                log::info!("[Runtime] User action: Join");
                match A::start(self).await {
                    Ok(addr) => {
                        // The selected fresh-start path owns the single initial
                        // End Device Timeout Request for this join.
                        let ch = self.channel();
                        let pan = self.pan_id();
                        TickResult::Event(StackEvent::Joined {
                            short_address: addr,
                            channel: ch,
                            pan_id: pan,
                        })
                    }
                    Err(_) => {
                        TickResult::Event(StackEvent::CommissioningComplete { success: false })
                    }
                }
            }

            UserAction::Rejoin => {
                log::info!("[Runtime] User action: Rejoin");
                self.retry_secure_rejoin().await
            }
            UserAction::Leave => {
                log::info!("[Runtime] User action: Leave");
                let _ = self.leave().await;
                TickResult::Event(StackEvent::Left)
            }
            UserAction::PermitJoin(duration) => {
                log::info!("[Runtime] User action: PermitJoin({}s)", duration);
                let _ = self.bdb.zdo_mut().nlme_permit_joining(duration).await;
                TickResult::Event(StackEvent::PermitJoinChanged { open: duration > 0 })
            }
            UserAction::FactoryReset => {
                log::info!("[Runtime] User action: Factory Reset");
                self.factory_reset(None).await;
                TickResult::Event(StackEvent::Left)
            }
        }
    }

    /// Send a ZCL Report Attributes command for the given endpoint and cluster.
    ///
    /// Records are packed whole into as many Report Attributes frames as
    /// needed, each no larger than one unfragmented APS payload, and sent via
    /// APS→NWK→MAC. Returns [`SendError::PayloadTooLong`] when a single record
    /// can never fit a frame (it is skipped; the others are still sent), or the
    /// first send error (later records are then not sent).
    pub async fn send_report(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        report: &zigbee_zcl::foundation::reporting::ReportAttributes,
    ) -> Result<(), SendError> {
        let outcome = self
            .send_report_records(endpoint, cluster_id, &report.reports)
            .await;
        match outcome.failure {
            Some((_, error)) => Err(error),
            None if outcome.skipped_oversize => Err(SendError::PayloadTooLong),
            None => Ok(()),
        }
    }

    /// Send `records` as one or more whole-record Report Attributes frames.
    ///
    /// Sending stops at the first failure; [`ReportSendOutcome::failure`]
    /// then identifies the first record that was not handed to APS so the
    /// caller can keep it pending.
    pub(crate) async fn send_report_records(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        records: &[zigbee_zcl::foundation::reporting::AttributeReport],
    ) -> ReportSendOutcome {
        let mut outcome = ReportSendOutcome {
            frames_sent: 0,
            skipped_oversize: false,
            failure: None,
        };
        if !self.is_joined() {
            outcome.failure = Some((0, SendError::NotJoined));
            return outcome;
        }

        // One buffer holds the whole ZCL frame: 3-byte global header followed
        // by whole records. Its size is the largest unfragmented APS payload.
        let mut zcl_buf = [0u8; REPORT_FRAME_CAP];
        let mut next = 0usize;
        while next < records.len() {
            let chunk_start = next;
            let (payload_len, after) =
                crate::zcl_wire::pack_report_records(records, next, &mut zcl_buf[3..]);
            next = after;
            if payload_len == 0 {
                log::warn!(
                    "[Runtime] Report record too large for one frame: ep={} cluster=0x{:04X}",
                    endpoint,
                    cluster_id
                );
                outcome.skipped_oversize = true;
                continue;
            }

            // Frame control: global, server→client, disable default response.
            zcl_buf[0] = REPORT_FRAME_CONTROL;
            zcl_buf[1] = self.next_zcl_seq();
            zcl_buf[2] = REPORT_ATTRIBUTES_COMMAND;

            let req = ApsdeDataRequest {
                dst_addr_mode: ApsAddressMode::Short,
                dst_address: ApsAddress::Short(ShortAddress::COORDINATOR),
                dst_endpoint: endpoint,
                profile_id: 0x0104, // Home Automation
                cluster_id,
                src_endpoint: endpoint,
                payload: &zcl_buf[..3 + payload_len],
                tx_options: ApsTxOptions {
                    use_nwk_key: true,
                    ..ApsTxOptions::default()
                },
                radius: 0,
                alias_src_addr: None,
                alias_seq: None,
            };

            if let Err(e) = self.bdb.zdo_mut().aps_mut().apsde_data_request(&req).await {
                log::warn!("[Runtime] Report send failed: {:?}", e);
                outcome.failure = Some((chunk_start, SendError::Aps(e)));
                return outcome;
            }
            outcome.frames_sent += 1;
            log::debug!(
                "[Runtime] Report sent: ep={} cluster=0x{:04X}",
                endpoint,
                cluster_id
            );
        }
        outcome
    }
}

#[cfg(test)]
mod tests {
    use super::{TickResult, advance_millis, automatic_poll_due, sleep_decision_to_tick};
    use crate::power::SleepDecision;

    #[test]
    fn power_clock_accumulates_elapsed_deltas() {
        let mut now_ms = 0;
        for elapsed_secs in [1, 0, 0, 1] {
            now_ms = advance_millis(now_ms, elapsed_secs);
        }
        assert_eq!(now_ms, 2_000);
    }

    #[test]
    fn commissioning_forces_automatic_sleepy_polling() {
        assert!(automatic_poll_due(true, true, true, false));
        assert!(!automatic_poll_due(false, true, true, false));
        assert!(!automatic_poll_due(true, false, true, false));
        assert!(automatic_poll_due(true, true, false, true));
        assert!(!automatic_poll_due(true, true, false, false));
    }

    #[test]
    fn sleep_decision_maps_to_tick_result() {
        // Both joined-tick tails (the router's out-of-line `tick_power_state`
        // and the sleepy end device's inlined `CAN_ROUTE` tail) funnel through
        // this one mapping, so locking it keeps the two role paths identical.
        assert!(matches!(
            sleep_decision_to_tick(SleepDecision::StayAwake),
            TickResult::Idle
        ));
        assert!(matches!(
            sleep_decision_to_tick(SleepDecision::LightSleep(1_500)),
            TickResult::RunAgain(1_500)
        ));
        assert!(matches!(
            sleep_decision_to_tick(SleepDecision::DeepSleep(60_000)),
            TickResult::RunAgain(60_000)
        ));
    }
}

#[cfg(all(test, feature = "centralized-tclk"))]
mod commissioning_tick_tests {
    //! Event-driven commissioning-security progress from the tick loop.
    //!
    //! GSDK advances the update-tc-link-key handshake from a scheduled event,
    //! independently of whatever else the stack is doing. These tests pin the
    //! platform-independent equivalent: every tick advances the handshake by
    //! exactly one bounded step while it is active, a terminal transition is
    //! reported immediately, and an ordinary application event is never
    //! replaced by the commissioning poll hint.

    use core::future::Future;
    use core::task::{Context, Poll, Waker};

    use zigbee_bdb::TclkStage;
    use zigbee_mac::PlatformServices;
    use zigbee_mac::mock::MockMac;
    use zigbee_mac::primitives::ZigbeeBeaconPayload;
    use zigbee_mac::primitives::{
        AssociationStatus, MacFrame, MlmeAssociateConfirm, PanDescriptor, SuperframeSpec,
    };
    use zigbee_nwk::DeviceType;
    use zigbee_nwk::frames::{NwkFrameControl, NwkFrameType, NwkHeader};
    use zigbee_types::{IeeeAddress, MacAddress, PanId, ShortAddress};

    use super::{StackEvent, TickResult};
    use crate::ZigbeeDevice;
    use crate::role::EndDevice;

    const LOCAL_IEEE: IeeeAddress = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
    const TC_IEEE: IeeeAddress = [0xAA; 8];
    const NETWORK_KEY: [u8; 16] = [0x5A; 16];
    const PAN: u16 = 0x1234;
    const SHORT: u16 = 0x1A2B;
    const CHANNEL: u8 = 15;

    fn block_on<F: Future>(future: F) -> F::Output {
        let mut context = Context::from_waker(Waker::noop());
        let mut future = std::pin::pin!(future);
        loop {
            if let Poll::Ready(output) = future.as_mut().poll(&mut context) {
                return output;
            }
            std::thread::yield_now();
        }
    }

    /// A plain NWK data frame as relayed by the parent, used to model
    /// "Transport-Key received" together with the pre-installed network key.
    fn parent_relayed_frame() -> heapless::Vec<u8, 32> {
        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: false,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: ShortAddress(SHORT),
            src_addr: ShortAddress::COORDINATOR,
            radius: 30,
            seq_number: 1,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };
        let mut buf = [0u8; 32];
        let header_len = header.serialize(&mut buf);
        let aps = [0x00u8, 0x01, 0x00, 0x00, 0x04, 0x01, 0x01, 0x2A];
        buf[header_len..header_len + aps.len()].copy_from_slice(&aps);
        let mut frame = heapless::Vec::new();
        let _ = frame.extend_from_slice(&buf[..header_len + aps.len()]);
        frame
    }

    /// A sleepy end device parked one poll away from a joinable coordinator.
    ///
    /// BDB initialization resets the lower layers (and therefore the mock's
    /// scripted radio), so it runs *before* the coordinator is scripted.
    fn joinable_device() -> ZigbeeDevice<MockMac, EndDevice> {
        let mut device = ZigbeeDevice::builder(MockMac::new(LOCAL_IEEE))
            .device_type(DeviceType::EndDevice)
            .build();
        device.bdb_mut().initialize().expect("BDB initialize");
        {
            let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
            nwk.set_rx_on_when_idle(false);
            let mac = nwk.mac_mut();
            mac.add_beacon(PanDescriptor {
                channel: CHANNEL,
                coord_address: MacAddress::Short(PanId(PAN), ShortAddress::COORDINATOR),
                superframe_spec: SuperframeSpec {
                    association_permit: true,
                    pan_coordinator: true,
                    ..Default::default()
                },
                lqi: 200,
                security_use: false,
                zigbee_beacon: ZigbeeBeaconPayload {
                    protocol_id: 0,
                    stack_profile: 2,
                    protocol_version: 2,
                    router_capacity: true,
                    device_depth: 0,
                    end_device_capacity: true,
                    extended_pan_id: [0xBB; 8],
                    tx_offset: [0xFF; 3],
                    update_id: 0,
                },
            });
            mac.set_associate_response(MlmeAssociateConfirm {
                short_address: ShortAddress(SHORT),
                status: AssociationStatus::Success,
            });
            let frame = parent_relayed_frame();
            mac.enqueue_poll_response(MacFrame::from_slice(&frame).unwrap());
        }
        let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
        nwk.security_mut().set_network_key(NETWORK_KEY, 0);
        nwk.nib_mut().security_enabled = true;
        nwk.nib_mut().outgoing_frame_counter_limit = 0x400;
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .aib_mut()
            .aps_trust_center_address = TC_IEEE;
        device
    }

    fn advance_time(device: &mut ZigbeeDevice<MockMac, EndDevice>, micros: u32) {
        block_on(
            device
                .bdb_mut()
                .zdo_mut()
                .aps_mut()
                .nwk_mut()
                .mac_mut()
                .delay_micros(micros),
        );
    }

    fn tick(device: &mut ZigbeeDevice<MockMac, EndDevice>) -> TickResult {
        block_on(device.tick(1, &mut []))
    }

    /// Join and leave the device parked on the armed post-network handshake.
    ///
    /// This is `start()` minus its leading `initialize()` (already done while
    /// building the joinable device): the same pre-network steering and the
    /// same join completion the production entry point runs.
    fn commissioning_device() -> ZigbeeDevice<MockMac, EndDevice> {
        let mut device = joinable_device();
        block_on(device.bdb_mut().network_steering()).expect("network steering");
        assert_eq!(block_on(device.finish_join()).ok(), Some(SHORT));
        assert!(
            device.bdb().tclk_exchange_active(),
            "network-up must arm the unique-TCLK handshake"
        );
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::StartDelay)
        );
        device
    }

    /// R22 §2.2.5.2.2: an APS retransmission repeats the original *unicast*.
    /// It used to be re-sent to `0xFFFF` with radius 0, which flooded the
    /// network with a frame only one device was expecting while the
    /// acknowledgement being waited for still never arrived.
    ///
    /// It is also paced by `apscAckWaitDuration` rather than by how often
    /// maintenance happens to run: nothing goes out inside the wait window.
    #[test]
    fn an_unacknowledged_aps_unicast_is_retransmitted_to_its_own_destination() {
        let mut device = commissioning_device();
        const PEER: u16 = 0x4763;

        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .register_ack_pending(
                0x31,
                PEER,
                &[0x40, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x31],
            )
            .expect("a free ACK slot");
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .clear_tx_history();

        // Maintenance inside the acknowledgement window transmits nothing, no
        // matter how often the application runs it.
        for _ in 0..10 {
            block_on(device.run_aps_maintenance());
        }
        assert!(
            device.bdb().zdo().aps().nwk().mac().tx_history().is_empty(),
            "no retry may be sent before apscAckWaitDuration has elapsed"
        );

        advance_time(&mut device, zigbee_aps::APS_ACK_WAIT_DURATION_US);
        block_on(device.run_aps_maintenance());

        let history = device.bdb().zdo().aps().nwk().mac().tx_history();
        assert_eq!(history.len(), 1, "exactly one retransmission");
        let (_nwk, _len) = zigbee_nwk::frames::NwkHeader::parse(history[0].payload.as_slice())
            .expect("a parsable NWK frame");
        assert_eq!(
            _nwk.dst_addr,
            ShortAddress(PEER),
            "the retry keeps the original unicast destination, never 0xFFFF"
        );
        assert!(
            _nwk.radius > 0,
            "a retransmission must carry a usable radius"
        );
    }

    #[test]
    fn every_tick_advances_the_tclk_handshake_by_one_step() {
        let mut device = commissioning_device();

        // The start delay is short, but it is still enforced by the monotonic
        // clock rather than by tick counting.
        assert!(matches!(tick(&mut device), TickResult::RunAgain(50)));
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::StartDelay)
        );

        advance_time(&mut device, 300_000);
        assert!(matches!(tick(&mut device), TickResult::RunAgain(50)));
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::SendNodeDesc)
        );

        assert!(matches!(tick(&mut device), TickResult::RunAgain(50)));
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::AwaitNodeDesc),
            "the tick loop must transmit Node_Desc without an extra application step"
        );
    }

    #[test]
    fn an_application_event_is_never_replaced_by_the_commissioning_hint() {
        let device = commissioning_device();
        assert!(device.bdb().tclk_exchange_active());

        // An ordinary tick result that carries an application event survives.
        assert!(matches!(
            device.commissioning_tick_hint(TickResult::Event(StackEvent::LeaveRequested)),
            TickResult::Event(StackEvent::LeaveRequested)
        ));
        // Idle and long sleeps are shortened to the commissioning cadence.
        assert!(matches!(
            device.commissioning_tick_hint(TickResult::Idle),
            TickResult::RunAgain(50)
        ));
        assert!(matches!(
            device.commissioning_tick_hint(TickResult::RunAgain(60_000)),
            TickResult::RunAgain(50)
        ));
        // A shorter deadline than the hint is preserved.
        assert!(matches!(
            device.commissioning_tick_hint(TickResult::RunAgain(10)),
            TickResult::RunAgain(10)
        ));
    }

    /// The durable tick path must behave exactly like the plain one: one
    /// bounded handshake step per tick, terminal transition reported at once.
    #[test]
    fn the_durable_tick_path_advances_the_handshake_identically() {
        use crate::security_store::RamSecurityStateStore;

        let mut device = joinable_device();
        let mut store = RamSecurityStateStore::new();
        {
            let mut persistence =
                crate::CommissioningSecurityPersistence::new(&mut store).expect("persistence");
            block_on(
                device
                    .bdb_mut()
                    .network_steering_with_persistence(&mut persistence),
            )
            .expect("network steering");
            assert!(persistence.take_error().is_none());
        }
        assert_eq!(block_on(device.finish_join()).ok(), Some(SHORT));
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::StartDelay)
        );

        let mut durable_tick = |device: &mut ZigbeeDevice<MockMac, EndDevice>| {
            block_on(device.tick_with_security_store(1, &mut [], &mut store)).expect("durable tick")
        };

        assert!(matches!(
            durable_tick(&mut device),
            TickResult::RunAgain(50)
        ));
        advance_time(&mut device, 300_000);
        assert!(matches!(
            durable_tick(&mut device),
            TickResult::RunAgain(50)
        ));
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::SendNodeDesc)
        );
        assert!(matches!(
            durable_tick(&mut device),
            TickResult::RunAgain(50)
        ));
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::AwaitNodeDesc)
        );

        // Nothing answers, so the handshake fails strictly and the durable path
        // reports it exactly like the plain tick.
        let mut terminal = None;
        for _ in 0..512 {
            match durable_tick(&mut device) {
                TickResult::Event(event) => {
                    terminal = Some(event);
                    break;
                }
                _ => advance_time(&mut device, 500_000),
            }
        }
        assert!(matches!(
            terminal,
            Some(StackEvent::CommissioningComplete { success: false })
        ));
        assert!(!device.bdb().tclk_exchange_active());
        assert!(!device.is_joined());
    }

    // ── KEY-03: durable replay commits during the unique-TCLK exchange ──

    const UNIQUE_TCLK: [u8; 16] = [0x5C; 16];

    fn sleepy_joinable_device() -> ZigbeeDevice<MockMac, EndDevice> {
        let mut device = ZigbeeDevice::builder(MockMac::new(LOCAL_IEEE))
            .device_type(DeviceType::EndDevice)
            .power_mode(crate::power::PowerMode::Sleepy {
                poll_interval_ms: 1000,
                wake_duration_ms: 100,
            })
            .build();
        device.bdb_mut().initialize().expect("BDB initialize");
        {
            let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
            nwk.set_rx_on_when_idle(false);
            let mac = nwk.mac_mut();
            mac.add_beacon(PanDescriptor {
                channel: CHANNEL,
                coord_address: MacAddress::Short(PanId(PAN), ShortAddress::COORDINATOR),
                superframe_spec: SuperframeSpec {
                    association_permit: true,
                    pan_coordinator: true,
                    ..Default::default()
                },
                lqi: 200,
                security_use: false,
                zigbee_beacon: ZigbeeBeaconPayload {
                    protocol_id: 0,
                    stack_profile: 2,
                    protocol_version: 2,
                    router_capacity: true,
                    device_depth: 0,
                    end_device_capacity: true,
                    extended_pan_id: [0xBB; 8],
                    tx_offset: [0xFF; 3],
                    update_id: 0,
                },
            });
            mac.set_associate_response(MlmeAssociateConfirm {
                short_address: ShortAddress(SHORT),
                status: AssociationStatus::Success,
            });
            let frame = parent_relayed_frame();
            mac.enqueue_poll_response(MacFrame::from_slice(&frame).unwrap());
        }
        let nwk = device.bdb_mut().zdo_mut().aps_mut().nwk_mut();
        nwk.security_mut().set_network_key(NETWORK_KEY, 0);
        nwk.nib_mut().security_enabled = true;
        nwk.nib_mut().outgoing_frame_counter_limit = 0x400;
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .aib_mut()
            .aps_trust_center_address = TC_IEEE;
        device
    }

    /// An NWK-secured data frame from the Trust Center (0x0000, `TC_IEEE`).
    fn tc_nwk_frame(nwk_counter: u32, aps: &[u8]) -> MacFrame {
        use zigbee_nwk::security::{NwkSecurity, NwkSecurityHeader};

        let header = NwkHeader {
            frame_control: NwkFrameControl {
                frame_type: NwkFrameType::Data as u8,
                protocol_version: 0x02,
                discover_route: 0,
                multicast: false,
                security: true,
                source_route: false,
                dst_ieee_present: false,
                src_ieee_present: false,
                end_device_initiator: false,
            },
            dst_addr: ShortAddress(SHORT),
            src_addr: ShortAddress::COORDINATOR,
            radius: 5,
            seq_number: nwk_counter as u8,
            dst_ieee: None,
            src_ieee: None,
            multicast_control: None,
            source_route: None,
        };
        let security = NwkSecurityHeader {
            security_control: NwkSecurityHeader::ZIGBEE_DEFAULT,
            frame_counter: nwk_counter,
            source_address: TC_IEEE,
            key_seq_number: 0,
        };
        let mut frame = [0u8; 127];
        let header_len = header.serialize(&mut frame);
        let security_len = security.serialize(&mut frame[header_len..]);
        let aad_len = header_len + security_len;
        let ciphertext = NwkSecurity::new()
            .encrypt(&frame[..aad_len], aps, &NETWORK_KEY, &security)
            .expect("TC frame encrypts");
        frame[aad_len..aad_len + ciphertext.len()].copy_from_slice(&ciphertext);
        frame[header_len] &= !0x07;
        MacFrame::from_slice(&frame[..aad_len + ciphertext.len()]).expect("TC frame fits")
    }

    /// Node_Desc_rsp from the Trust Center advertising stack revision 22.
    fn node_desc_rsp(tsn: u8) -> heapless::Vec<u8, 64> {
        use zigbee_aps::frames::{ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};

        let header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Data as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: false,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: Some(0),
            group_address: None,
            cluster_id: Some(zigbee_zdo::NODE_DESC_RSP),
            profile_id: Some(0x0000),
            src_endpoint: Some(0),
            aps_counter: 0x41,
            extended_header: None,
        };
        let server_mask: u16 = (22 << 9) | 0x0001;
        let mut payload = [0u8; 17];
        payload[0] = tsn;
        payload[1] = 0x00; // SUCCESS
        payload[2..4].copy_from_slice(&0x0000u16.to_le_bytes());
        payload[4] = 0x00; // coordinator
        payload[5] = 0x40; // 2.4 GHz
        payload[6] = 0x8E;
        payload[7..9].copy_from_slice(&0x1234u16.to_le_bytes());
        payload[9] = 0x52;
        payload[10..12].copy_from_slice(&0x0052u16.to_le_bytes());
        payload[12..14].copy_from_slice(&server_mask.to_le_bytes());
        payload[14..16].copy_from_slice(&0x0052u16.to_le_bytes());
        payload[16] = 0x00;
        let mut frame = [0u8; 64];
        let header_len = header.serialize(&mut frame);
        frame[header_len..header_len + payload.len()].copy_from_slice(&payload);
        heapless::Vec::from_slice(&frame[..header_len + payload.len()]).unwrap()
    }

    /// An APS command from the Trust Center, APS-secured with `key` under
    /// `key_id`, as the APS payload of an NWK frame.
    fn tc_aps_command(
        key_id: u8,
        key: &[u8; 16],
        aps_counter: u32,
        command: &[u8],
    ) -> heapless::Vec<u8, 96> {
        use zigbee_aps::frames::{ApsDeliveryMode, ApsFrameControl, ApsFrameType, ApsHeader};
        use zigbee_aps::security::{ApsSecurity, ApsSecurityHeader, SEC_LEVEL_ENC_MIC_32};

        let header = ApsHeader {
            frame_control: ApsFrameControl {
                frame_type: ApsFrameType::Command as u8,
                delivery_mode: ApsDeliveryMode::Unicast as u8,
                ack_format: false,
                security: true,
                ack_request: false,
                extended_header: false,
            },
            dst_endpoint: None,
            group_address: None,
            cluster_id: None,
            profile_id: None,
            src_endpoint: None,
            aps_counter: aps_counter as u8,
            extended_header: None,
        };
        let security = ApsSecurityHeader {
            security_control: (key_id << 3) | (1 << 5),
            frame_counter: aps_counter,
            source_address: Some(TC_IEEE),
            key_seq_number: None,
        };
        let mut frame = [0u8; 96];
        let header_len = header.serialize(&mut frame);
        let security_len = security.serialize(&mut frame[header_len..]);
        let aad_len = header_len + security_len;
        let mut authenticated = [0u8; 32];
        authenticated[..aad_len].copy_from_slice(&frame[..aad_len]);
        authenticated[header_len] |= SEC_LEVEL_ENC_MIC_32;
        let encrypted = ApsSecurity::new()
            .encrypt(&authenticated[..aad_len], command, key, &security)
            .expect("APS command encrypts");
        frame[aad_len..aad_len + encrypted.len()].copy_from_slice(&encrypted);
        heapless::Vec::from_slice(&frame[..aad_len + encrypted.len()]).unwrap()
    }

    /// Transport-Key carrying the unique TCLK under the key-load key of the
    /// default ZigBeeAlliance09 link key that protected the join.
    fn tclk_transport_key(aps_counter: u32) -> heapless::Vec<u8, 96> {
        use zigbee_aps::frames::ApsCommandId;
        use zigbee_aps::security::{KEY_ID_KEY_LOAD, derive_key_load_key};

        let mut command = [0u8; 34];
        command[0] = ApsCommandId::TransportKey as u8;
        command[1] = 0x04;
        command[2..18].copy_from_slice(&UNIQUE_TCLK);
        command[18..26].copy_from_slice(&LOCAL_IEEE);
        command[26..34].copy_from_slice(&TC_IEEE);
        let key_load = derive_key_load_key(&zigbee_aps::security::DEFAULT_TC_LINK_KEY);
        tc_aps_command(KEY_ID_KEY_LOAD, &key_load, aps_counter, &command)
    }

    /// Confirm-Key SUCCESS under the freshly delivered unique TCLK.
    fn confirm_key_success(aps_counter: u32) -> heapless::Vec<u8, 96> {
        use zigbee_aps::frames::ApsCommandId;
        use zigbee_aps::security::KEY_ID_DATA_KEY;

        let mut command = [0u8; 11];
        command[0] = ApsCommandId::ConfirmKey as u8;
        command[1] = 0x00;
        command[2] = 0x04;
        command[3..11].copy_from_slice(&LOCAL_IEEE);
        tc_aps_command(KEY_ID_DATA_KEY, &UNIQUE_TCLK, aps_counter, &command)
    }

    fn pending_node_desc_tsn(device: &ZigbeeDevice<MockMac, EndDevice>) -> Option<u8> {
        (0..4).find_map(|slot| device.bdb().zdo().pending_tsn(slot))
    }

    /// Outcome of driving the durable unique-TCLK handshake against a
    /// scripted Trust Center.
    struct DurableTclkRun {
        device: ZigbeeDevice<MockMac, EndDevice>,
        terminal: Option<StackEvent>,
        errors: heapless::Vec<(TclkStage, crate::SecurityStoreError), 8>,
        last_nwk_counter: u32,
        last_aps_counter: u32,
        node_desc_tsn: Option<u8>,
        node_desc_nwk_counter: u32,
        transport_key_counter: u32,
        confirm_key_counter: u32,
    }

    /// Which Trust Center responses the scripted coordinator sends.
    #[derive(Clone, Copy)]
    struct TrustCenterScript {
        confirm_key: bool,
        /// Stop driving, as a power loss would, once this stage is reached.
        stop_at: Option<TclkStage>,
        /// Last NWK/APS frame counter the Trust Center used before this run.
        nwk_counter: u32,
        aps_counter: u32,
    }

    const FULL_EXCHANGE: TrustCenterScript = TrustCenterScript {
        confirm_key: true,
        stop_at: None,
        nwk_counter: 100,
        aps_counter: 10,
    };

    /// Join with a durable commissioning reservation; the unique-TCLK
    /// exchange is then active against `TC_IEEE`.
    fn join_durably<S: crate::SecurityStateStore>(
        store: &mut S,
    ) -> ZigbeeDevice<MockMac, EndDevice> {
        let mut device = sleepy_joinable_device();
        {
            let mut persistence =
                crate::CommissioningSecurityPersistence::new(&mut *store).expect("persistence");
            block_on(
                device
                    .bdb_mut()
                    .network_steering_with_persistence(&mut persistence),
            )
            .expect("network steering");
            assert!(persistence.take_error().is_none());
        }
        assert_eq!(block_on(device.finish_join()).ok(), Some(SHORT));
        assert!(device.bdb().tclk_exchange_active());
        assert!(
            !store.load().unwrap().expect("reserved state").commissioned,
            "network-up reserves security but is not yet commissioned"
        );
        device
    }

    /// Join with durable reservation, then let a scripted R22 Trust Center
    /// answer every step of the unique-TCLK exchange through parent polls,
    /// with every tick on the durable security-store path.
    fn drive_durable_tclk_exchange<S: crate::SecurityStateStore>(
        store: &mut S,
        script: TrustCenterScript,
    ) -> DurableTclkRun {
        let device = join_durably(store);
        drive_tclk_exchange(device, store, script)
    }

    fn drive_tclk_exchange<S: crate::SecurityStateStore>(
        mut device: ZigbeeDevice<MockMac, EndDevice>,
        store: &mut S,
        script: TrustCenterScript,
    ) -> DurableTclkRun {
        let mut nwk_counter = script.nwk_counter;
        let mut aps_counter = script.aps_counter;
        let mut node_desc_tsn = None;
        let mut node_desc_nwk_counter = 0;
        let mut node_desc_sent = false;
        let mut transport_key_sent = false;
        let mut confirm_key_sent = false;
        let mut transport_key_counter = 0;
        let mut confirm_key_counter = 0;
        let mut errors = heapless::Vec::new();
        let mut terminal = None;
        for _ in 0..2048 {
            advance_time(&mut device, 100_000);
            let stage = device.bdb().tclk_exchange_stage();
            if script.stop_at.is_some() && stage == script.stop_at {
                break;
            }
            let response = match stage {
                Some(TclkStage::AwaitNodeDesc) if !node_desc_sent => {
                    node_desc_sent = true;
                    node_desc_tsn = pending_node_desc_tsn(&device);
                    node_desc_nwk_counter = nwk_counter + 1;
                    node_desc_tsn.map(node_desc_rsp)
                }
                Some(TclkStage::AwaitTclk) if !transport_key_sent => {
                    transport_key_sent = true;
                    aps_counter += 1;
                    transport_key_counter = aps_counter;
                    Some(heapless::Vec::from_slice(&tclk_transport_key(aps_counter)).unwrap())
                }
                Some(TclkStage::AwaitConfirmKey) if script.confirm_key && !confirm_key_sent => {
                    confirm_key_sent = true;
                    aps_counter += 1;
                    confirm_key_counter = aps_counter;
                    Some(heapless::Vec::from_slice(&confirm_key_success(aps_counter)).unwrap())
                }
                _ => None,
            };
            if let Some(aps) = response {
                nwk_counter += 1;
                device
                    .bdb_mut()
                    .zdo_mut()
                    .aps_mut()
                    .nwk_mut()
                    .mac_mut()
                    .enqueue_poll_response(tc_nwk_frame(nwk_counter, &aps));
            }
            match block_on(device.tick_with_security_store(1, &mut [], &mut *store)) {
                Ok(TickResult::Event(event @ StackEvent::CommissioningComplete { .. })) => {
                    terminal = Some(event);
                    break;
                }
                Ok(_) => {}
                Err(error) => {
                    let _ = errors.push((stage.unwrap_or(TclkStage::Failed), error));
                }
            }
        }
        DurableTclkRun {
            device,
            terminal,
            errors,
            last_nwk_counter: nwk_counter,
            last_aps_counter: aps_counter,
            node_desc_tsn,
            node_desc_nwk_counter,
            transport_key_counter,
            confirm_key_counter,
        }
    }

    fn persisted_tc_nwk_floor<S: crate::SecurityStateStore>(store: &mut S) -> Option<u32> {
        let mut floor = None;
        store
            .visit_replay_counters(&mut |replay| {
                if let crate::security_store::PersistentReplayCounter::Nwk(replay) = replay
                    && replay.source == TC_IEEE
                {
                    floor = Some(floor.map_or(replay.counter, |old: u32| old.max(replay.counter)));
                }
            })
            .expect("replay counters are readable");
        floor
    }

    /// KEY-03: a persistent device must be able to complete the unique-TCLK
    /// exchange. Every Trust Center response arrives NWK-secured while the
    /// reserved record is still `commissioned = false`; its replay floor must
    /// become durable *without* the record being marked commissioned early.
    #[test]
    fn durable_sleepy_end_device_completes_the_unique_tclk_exchange() {
        use crate::security_store::RamSecurityStateStore;

        let mut store = RamSecurityStateStore::new();
        let run = drive_durable_tclk_exchange(&mut store, FULL_EXCHANGE);
        assert_completed_exchange(&mut store, &run);
    }

    fn assert_completed_exchange<S: crate::SecurityStateStore>(
        store: &mut S,
        run: &DurableTclkRun,
    ) {
        assert!(
            run.errors.is_empty(),
            "no durable tick may fail during the exchange: {:?}",
            run.errors
        );
        assert!(
            matches!(
                run.terminal,
                Some(StackEvent::CommissioningComplete { success: true })
            ),
            "the exchange must complete successfully, got {:?}",
            run.terminal
        );
        assert!(run.device.is_joined());
        let state = store.load().unwrap().expect("committed state");
        assert!(state.commissioned);
        assert!(state.tclk_present);
        assert_eq!(state.trust_center_link_key, UNIQUE_TCLK);
        assert!(state.tclk_incoming_counter_valid);
        assert!(state.tclk_incoming_counter >= run.confirm_key_counter);
        assert!(
            persisted_tc_nwk_floor(store).is_some_and(|floor| floor >= run.last_nwk_counter),
            "every accepted TC NWK frame must leave a durable replay floor"
        );
    }

    fn fresh_end_device() -> ZigbeeDevice<MockMac, EndDevice> {
        ZigbeeDevice::builder(MockMac::new(LOCAL_IEEE))
            .device_type(DeviceType::EndDevice)
            .power_mode(crate::power::PowerMode::Sleepy {
                poll_interval_ms: 1000,
                wake_duration_ms: 100,
            })
            .build()
    }

    /// The production flash journal, not only the RAM model, must let the
    /// provisional exchange persist its floors and carry them into the
    /// commissioned record.
    #[test]
    fn durable_journal_end_device_completes_the_unique_tclk_exchange() {
        use crate::security_journal::tests::MockFlash;
        use crate::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

        let mut journal =
            SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32);
        let run = drive_durable_tclk_exchange(&mut journal, FULL_EXCHANGE);
        assert_completed_exchange(&mut journal, &run);
    }

    /// A store that cannot make any incoming replay floor durable.
    struct ReplayFailingStore(crate::security_store::RamSecurityStateStore);

    impl crate::SecurityStateStore for ReplayFailingStore {
        fn load(
            &mut self,
        ) -> Result<Option<crate::PersistentSecurityState>, crate::SecurityStoreError> {
            self.0.load()
        }

        fn store(
            &mut self,
            state: &crate::PersistentSecurityState,
        ) -> Result<(), crate::SecurityStoreError> {
            self.0.store(state)
        }

        fn visit_replay_counters(
            &mut self,
            visitor: &mut dyn FnMut(crate::security_store::PersistentReplayCounter),
        ) -> Result<(), crate::SecurityStoreError> {
            self.0.visit_replay_counters(visitor)
        }

        fn commit_replay_counter(
            &mut self,
            _replay: crate::security_store::PersistentReplayCounter,
        ) -> Result<(), crate::SecurityStoreError> {
            Err(crate::SecurityStoreError::Hardware)
        }

        fn tombstone_replay_counters(
            &mut self,
            tombstone: crate::security_store::ReplayCounterTombstone,
        ) -> Result<(), crate::SecurityStoreError> {
            self.0.tombstone_replay_counters(tombstone)
        }
    }

    /// Accepting the provisional exchange's traffic is conditional on its
    /// floor being durable: a storage failure drops the Trust Center
    /// response, so the handshake can only time out and nothing commits.
    #[test]
    fn a_volatile_provisional_replay_floor_fails_the_exchange_closed() {
        use crate::SecurityStateStore;

        let mut store = ReplayFailingStore(crate::security_store::RamSecurityStateStore::new());
        let run = drive_durable_tclk_exchange(&mut store, FULL_EXCHANGE);

        assert_eq!(
            run.errors.first(),
            Some(&(
                TclkStage::AwaitNodeDesc,
                crate::SecurityStoreError::Hardware
            )),
            "the first Trust Center response must be refused, not acted on"
        );
        assert!(
            matches!(
                run.terminal,
                Some(StackEvent::CommissioningComplete { success: false })
            ),
            "an exchange whose replies cannot be made durable must fail, got {:?}",
            run.terminal
        );
        let state = store.load().unwrap().expect("reserved state");
        assert!(!state.commissioned);
        assert!(!state.tclk_present, "no TCLK may be installed");
        assert_eq!(persisted_tc_nwk_floor(&mut store), None);
    }

    /// A power loss mid-exchange leaves a provisional record whose floors
    /// are durable but which never restores as an operational network.
    #[test]
    fn a_reboot_inside_the_provisional_exchange_restores_no_network() {
        use crate::SecurityStateStore;
        use crate::security_store::RamSecurityStateStore;

        let mut store = RamSecurityStateStore::new();
        let run = drive_durable_tclk_exchange(
            &mut store,
            TrustCenterScript {
                confirm_key: true,
                stop_at: Some(TclkStage::AwaitConfirmKey),
                ..FULL_EXCHANGE
            },
        );
        assert!(run.errors.is_empty(), "{:?}", run.errors);
        assert!(run.terminal.is_none());
        let provisional = store.load().unwrap().expect("provisional state");
        assert!(!provisional.commissioned);
        assert!(provisional.tclk_present, "the delivered TCLK is reserved");
        assert!(
            persisted_tc_nwk_floor(&mut store).is_some_and(|floor| floor >= run.last_nwk_counter),
            "the provisional exchange's accepted frames already have durable floors"
        );

        let mut rebooted = fresh_end_device();
        assert_eq!(rebooted.restore_security_state(&mut store), Ok(false));
        assert!(!rebooted.is_joined());
        assert_eq!(store.load().unwrap(), Some(provisional));
    }

    /// Every frame accepted during the exchange stays a replay after the
    /// commissioned network is restored.
    #[test]
    fn exchange_traffic_stays_replay_rejected_after_reboot() {
        use crate::SecurityStateStore;
        use crate::security_store::{PersistentReplayCounter, RamSecurityStateStore};
        use zigbee_aps::security::{ApsKeyType, ApsReplayOrigin};

        let mut store = RamSecurityStateStore::new();
        let run = drive_durable_tclk_exchange(&mut store, FULL_EXCHANGE);
        assert_completed_exchange(&mut store, &run);

        let mut rebooted = fresh_end_device();
        assert_eq!(rebooted.restore_security_state(&mut store), Ok(true));

        let mut persisted = heapless::Vec::<PersistentReplayCounter, 8>::new();
        store
            .visit_replay_counters(&mut |replay| persisted.push(replay).unwrap())
            .unwrap();
        let aps = rebooted.bdb().zdo().aps();
        let nwk = aps.nwk();
        let mut key_load_floor = None;
        for replay in &persisted {
            match replay {
                PersistentReplayCounter::Nwk(replay) => {
                    assert!(!nwk.security().check_frame_counter_for_key(
                        &replay.source,
                        replay.key_sequence,
                        replay.counter
                    ));
                }
                PersistentReplayCounter::Aps(replay) => {
                    assert!(!aps.security().check_replay_counter(replay));
                    if replay.origin == (ApsReplayOrigin::PreconfiguredGlobal { source: TC_IEEE }) {
                        key_load_floor = Some(replay.counter);
                    }
                }
            }
        }
        assert!(
            !nwk.security()
                .check_frame_counter(&TC_IEEE, run.last_nwk_counter),
            "the Confirm-Key NWK frame is a replay after reboot"
        );
        assert!(
            nwk.security()
                .check_frame_counter(&TC_IEEE, run.last_nwk_counter + 1)
        );
        assert_eq!(
            key_load_floor,
            Some(run.transport_key_counter),
            "the TCLK Transport-Key floor survives into the commissioned epoch"
        );
        assert!(!aps.security().check_frame_counter(
            &TC_IEEE,
            ApsKeyType::TrustCenterLinkKey,
            run.confirm_key_counter
        ));
    }

    /// A Trust Center that never confirms the key leaves no commissioned
    /// network behind, durable or live.
    #[test]
    fn an_unconfirmed_exchange_never_commits_the_network() {
        use crate::SecurityStateStore;
        use crate::security_store::RamSecurityStateStore;

        let mut store = RamSecurityStateStore::new();
        let run = drive_durable_tclk_exchange(
            &mut store,
            TrustCenterScript {
                confirm_key: false,
                stop_at: None,
                ..FULL_EXCHANGE
            },
        );
        assert!(run.errors.is_empty(), "{:?}", run.errors);
        assert!(matches!(
            run.terminal,
            Some(StackEvent::CommissioningComplete { success: false })
        ));
        assert!(!run.device.is_joined());
        assert!(!store.load().unwrap().expect("reserved state").commissioned);
        let mut rebooted = fresh_end_device();
        assert_eq!(rebooted.restore_security_state(&mut store), Ok(false));
        assert!(!rebooted.is_joined());
    }

    /// Tick on the durable path until the exchange reaches `stage`.
    fn tick_durably_until<S: crate::SecurityStateStore>(
        device: &mut ZigbeeDevice<MockMac, EndDevice>,
        store: &mut S,
        stage: TclkStage,
    ) {
        for _ in 0..256 {
            if device.bdb().tclk_exchange_stage() == Some(stage) {
                return;
            }
            advance_time(device, 100_000);
            let result = block_on(device.tick_with_security_store(1, &mut [], &mut *store));
            assert!(
                matches!(result, Ok(TickResult::Idle | TickResult::RunAgain(_))),
                "unexpected tick result before {stage:?}: {result:?}"
            );
        }
        panic!("the exchange never reached {stage:?}");
    }

    /// Deliver `frame` through the next parent poll on the durable path,
    /// then let the exchange observe it for a few more ticks.
    fn deliver_durably<S: crate::SecurityStateStore>(
        device: &mut ZigbeeDevice<MockMac, EndDevice>,
        store: &mut S,
        frame: MacFrame,
    ) -> heapless::Vec<Result<TickResult, crate::SecurityStoreError>, 8> {
        let polls = device.bdb().zdo().aps().nwk().mac().poll_count();
        device
            .bdb_mut()
            .zdo_mut()
            .aps_mut()
            .nwk_mut()
            .mac_mut()
            .enqueue_poll_response(frame);
        let mut results = heapless::Vec::new();
        for _ in 0..64 {
            advance_time(device, 100_000);
            let result = block_on(device.tick_with_security_store(1, &mut [], &mut *store));
            if device.bdb().zdo().aps().nwk().mac().poll_count() > polls {
                let _ = results.push(result);
                break;
            }
            assert!(result.is_ok(), "tick failed before delivery: {result:?}");
        }
        assert!(!results.is_empty(), "the parent was never polled");
        for _ in 0..3 {
            advance_time(device, 100_000);
            let _ = results.push(block_on(device.tick_with_security_store(
                1,
                &mut [],
                &mut *store,
            )));
        }
        results
    }

    fn persisted_tclk_floor<S: crate::SecurityStateStore>(store: &mut S) -> Option<u32> {
        use zigbee_aps::security::{ApsKeyType, ApsReplayOrigin};

        let mut floor = None;
        store
            .visit_replay_counters(&mut |replay| {
                if let crate::security_store::PersistentReplayCounter::Aps(replay) = replay
                    && replay.origin
                        == (ApsReplayOrigin::KeyPair {
                            partner: TC_IEEE,
                            key_type: ApsKeyType::TrustCenterLinkKey,
                        })
                {
                    floor = Some(floor.map_or(replay.counter, |old: u32| old.max(replay.counter)));
                }
            })
            .expect("replay counters are readable");
        floor
    }

    /// KEY-03 review: a power loss after a provisional NWK floor became
    /// durable must not let the same security epoch accept that frame again.
    ///
    /// The first attempt accepts the Trust Center's Node_Desc_rsp while the
    /// record is `commissioned = false`, then loses power. The second attempt
    /// reserves the same EPID, local IEEE, network key and key sequence, so
    /// the durable floor carries over — and the exact old frame must be
    /// rejected before it can answer the new Node_Desc_req.
    fn a_provisional_nwk_floor_rejects_its_frame_after_reboot<S: crate::SecurityStateStore>(
        store: &mut S,
    ) {
        let first = drive_durable_tclk_exchange(
            store,
            TrustCenterScript {
                stop_at: Some(TclkStage::AwaitTclk),
                ..FULL_EXCHANGE
            },
        );
        assert!(first.errors.is_empty(), "{:?}", first.errors);
        let tsn = first.node_desc_tsn.expect("the Node_Desc_req was answered");
        let counter = first.node_desc_nwk_counter;
        let provisional = store.load().unwrap().expect("provisional state");
        assert!(!provisional.commissioned);
        assert_eq!(
            persisted_tc_nwk_floor(store),
            Some(counter),
            "the accepted Node_Desc_rsp left a durable floor"
        );
        drop(first);

        // Power loss: the provisional record restores no network.
        let mut rebooted = fresh_end_device();
        assert_eq!(rebooted.restore_security_state(store), Ok(false));
        drop(rebooted);

        let mut device = join_durably(store);
        let reserved = store.load().unwrap().expect("reserved state");
        assert!(
            provisional.replay_domain_continues_into(&reserved),
            "the second attempt reserves the same security epoch"
        );
        assert_eq!(persisted_tc_nwk_floor(store), Some(counter));
        tick_durably_until(&mut device, store, TclkStage::AwaitNodeDesc);
        assert_eq!(
            pending_node_desc_tsn(&device),
            Some(tsn),
            "the replayed frame answers the new request exactly"
        );

        let replay = deliver_durably(
            &mut device,
            store,
            tc_nwk_frame(counter, &node_desc_rsp(tsn)),
        );
        assert!(
            !replay
                .iter()
                .any(|result| matches!(result, Ok(TickResult::Event(_)))),
            "{replay:?}"
        );
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::AwaitNodeDesc),
            "a replayed Node_Desc_rsp must not advance the exchange"
        );
        assert_eq!(
            pending_node_desc_tsn(&device),
            Some(tsn),
            "a replayed Node_Desc_rsp must not reach ZDO"
        );
        assert_eq!(persisted_tc_nwk_floor(store), Some(counter));

        // A fresh response from the same Trust Center still progresses.
        let fresh = deliver_durably(
            &mut device,
            store,
            tc_nwk_frame(counter + 1, &node_desc_rsp(tsn)),
        );
        assert!(fresh.iter().all(Result::is_ok), "{fresh:?}");
        assert_ne!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::AwaitNodeDesc)
        );
        assert_eq!(persisted_tc_nwk_floor(store), Some(counter + 1));
    }

    #[test]
    fn a_provisional_nwk_floor_rejects_its_frame_after_reboot_ram() {
        let mut store = crate::security_store::RamSecurityStateStore::new();
        a_provisional_nwk_floor_rejects_its_frame_after_reboot(&mut store);
    }

    #[test]
    fn a_provisional_nwk_floor_rejects_its_frame_after_reboot_journal() {
        a_provisional_nwk_floor_rejects_its_frame_after_reboot(&mut journal());
    }

    /// What the device does after its commissioning commit fails.
    #[derive(Clone, Copy, Debug)]
    enum AfterCommitFailure {
        /// Power is lost: no later flash operation happens before reboot.
        PowerLoss,
        /// The device stays up, leaves, and keeps running durable ticks.
        KeepRunning,
    }

    /// A store whose write of a `commissioned` record fails while `cut` is
    /// set, optionally freezing all later writes as a power loss would.
    struct CommitCutStore<S> {
        inner: S,
        cut: bool,
        cut_hit: bool,
        after: AfterCommitFailure,
    }

    impl<S> CommitCutStore<S> {
        fn powered_off(&self) -> bool {
            self.cut_hit && matches!(self.after, AfterCommitFailure::PowerLoss)
        }

        fn reboot(&mut self) {
            self.cut = false;
            self.cut_hit = false;
        }
    }

    impl<S: crate::SecurityStateStore> crate::SecurityStateStore for CommitCutStore<S> {
        fn load(
            &mut self,
        ) -> Result<Option<crate::PersistentSecurityState>, crate::SecurityStoreError> {
            self.inner.load()
        }

        fn store(
            &mut self,
            state: &crate::PersistentSecurityState,
        ) -> Result<(), crate::SecurityStoreError> {
            if self.powered_off() || (self.cut && state.commissioned) {
                self.cut_hit = true;
                return Err(crate::SecurityStoreError::Hardware);
            }
            self.inner.store(state)
        }

        fn visit_replay_counters(
            &mut self,
            visitor: &mut dyn FnMut(crate::security_store::PersistentReplayCounter),
        ) -> Result<(), crate::SecurityStoreError> {
            self.inner.visit_replay_counters(visitor)
        }

        fn commit_replay_counter(
            &mut self,
            replay: crate::security_store::PersistentReplayCounter,
        ) -> Result<(), crate::SecurityStoreError> {
            if self.powered_off() {
                return Err(crate::SecurityStoreError::Hardware);
            }
            self.inner.commit_replay_counter(replay)
        }

        fn tombstone_replay_counters(
            &mut self,
            tombstone: crate::security_store::ReplayCounterTombstone,
        ) -> Result<(), crate::SecurityStoreError> {
            if self.powered_off() {
                return Err(crate::SecurityStoreError::Hardware);
            }
            self.inner.tombstone_replay_counters(tombstone)
        }

        fn retain_replay_counters(
            &mut self,
            retain: &dyn Fn(crate::security_store::PersistentReplayCounter) -> bool,
        ) -> Result<usize, crate::SecurityStoreError> {
            if self.powered_off() {
                return Err(crate::SecurityStoreError::Hardware);
            }
            self.inner.retain_replay_counters(retain)
        }
    }

    /// KEY-03 review, APS variant: the Confirm-Key accepted under the unique
    /// TCLK is durably floored before `commit_network`. If power is lost at
    /// that commit, re-commissioning with the same network key and the same
    /// unique TCLK must reject the old Confirm-Key once that TCLK is live.
    ///
    /// `KeepRunning` additionally covers a commit failure without power
    /// loss: the device leaves and keeps ticking, and the provisional floors
    /// must survive that as well, since the record still owns the epoch.
    fn a_provisional_tclk_floor_rejects_its_frame_after_reboot<S: crate::SecurityStateStore>(
        inner: S,
        after: AfterCommitFailure,
    ) {
        use crate::SecurityStateStore;
        use zigbee_aps::security::ApsKeyType;

        let mut store = CommitCutStore {
            inner,
            cut: true,
            cut_hit: false,
            after,
        };
        let first = drive_durable_tclk_exchange(&mut store, FULL_EXCHANGE);
        assert!(store.cut_hit, "power is lost at the commissioning commit");
        assert_eq!(
            first.errors.first(),
            Some(&(
                TclkStage::AwaitConfirmKey,
                crate::SecurityStoreError::Hardware
            )),
            "the commissioning commit is the first failure"
        );
        assert!(first.terminal.is_none(), "{:?}", first.terminal);
        assert_eq!(
            persisted_tc_nwk_floor(&mut store),
            Some(first.last_nwk_counter),
            "the provisional NWK floor stays durable"
        );
        let provisional = store.load().unwrap().expect("provisional state");
        assert!(!provisional.commissioned);
        assert!(provisional.tclk_present);
        assert_eq!(provisional.trust_center_link_key, UNIQUE_TCLK);
        assert_eq!(
            persisted_tclk_floor(&mut store),
            Some(first.confirm_key_counter),
            "the accepted Confirm-Key left a durable unique-TCLK floor"
        );
        drop(first.device);
        store.reboot();

        let mut rebooted = fresh_end_device();
        assert_eq!(rebooted.restore_security_state(&mut store), Ok(false));
        drop(rebooted);

        let device = join_durably(&mut store);
        let second = drive_tclk_exchange(
            device,
            &mut store,
            TrustCenterScript {
                stop_at: Some(TclkStage::AwaitConfirmKey),
                nwk_counter: first.last_nwk_counter,
                aps_counter: first.last_aps_counter,
                ..FULL_EXCHANGE
            },
        );
        assert!(second.errors.is_empty(), "{:?}", second.errors);
        assert!(second.terminal.is_none(), "{:?}", second.terminal);
        let mut device = second.device;
        let live = device
            .bdb()
            .zdo()
            .aps()
            .security()
            .find_key(&TC_IEEE, ApsKeyType::TrustCenterLinkKey)
            .expect("the unique TCLK is live again");
        assert_eq!(live.key, UNIQUE_TCLK, "the second attempt reuses the TCLK");
        let reserved = store.load().unwrap().expect("reserved state");
        assert!(provisional.replay_domain_continues_into(&reserved));

        // The old Confirm-Key, re-wrapped in a fresh NWK frame so only its
        // APS counter can identify the replay.
        let successes = device
            .bdb()
            .zdo()
            .aps()
            .security_handshake_stats()
            .confirm_key_successes;
        let replay = deliver_durably(
            &mut device,
            &mut store,
            tc_nwk_frame(
                second.last_nwk_counter + 1,
                &confirm_key_success(first.confirm_key_counter),
            ),
        );
        assert!(
            !replay.iter().any(|result| matches!(
                result,
                Ok(TickResult::Event(StackEvent::CommissioningComplete { .. }))
            )),
            "{replay:?}"
        );
        assert_eq!(
            device
                .bdb()
                .zdo()
                .aps()
                .security_handshake_stats()
                .confirm_key_successes,
            successes,
            "a replayed Confirm-Key must not authenticate the TCLK"
        );
        assert_eq!(
            device.bdb().tclk_exchange_stage(),
            Some(TclkStage::AwaitConfirmKey)
        );
        assert!(!store.load().unwrap().expect("reserved state").commissioned);

        // A fresh Confirm-Key completes the exchange.
        let fresh = deliver_durably(
            &mut device,
            &mut store,
            tc_nwk_frame(
                second.last_nwk_counter + 2,
                &confirm_key_success(second.last_aps_counter + 1),
            ),
        );
        assert!(
            fresh.iter().any(|result| matches!(
                result,
                Ok(TickResult::Event(StackEvent::CommissioningComplete {
                    success: true
                }))
            )),
            "{fresh:?}"
        );
        let committed = store.load().unwrap().expect("committed state");
        assert!(committed.commissioned);
        assert_eq!(committed.trust_center_link_key, UNIQUE_TCLK);
        assert!(committed.tclk_incoming_counter > second.last_aps_counter);
    }

    fn journal() -> crate::security_journal::SecurityStateJournal<
        crate::security_journal::tests::MockFlash,
        { crate::security_journal::SECURITY_JOURNAL_SECTOR_SIZE },
    > {
        use crate::security_journal::tests::MockFlash;
        use crate::security_journal::{SECURITY_JOURNAL_SECTOR_SIZE, SecurityStateJournal};

        SecurityStateJournal::new(MockFlash::new(), 0, SECURITY_JOURNAL_SECTOR_SIZE as u32)
    }

    #[test]
    fn a_provisional_tclk_floor_rejects_its_frame_after_power_loss_ram() {
        a_provisional_tclk_floor_rejects_its_frame_after_reboot(
            crate::security_store::RamSecurityStateStore::new(),
            AfterCommitFailure::PowerLoss,
        );
    }

    #[test]
    fn a_provisional_tclk_floor_rejects_its_frame_after_power_loss_journal() {
        a_provisional_tclk_floor_rejects_its_frame_after_reboot(
            journal(),
            AfterCommitFailure::PowerLoss,
        );
    }

    #[test]
    fn a_provisional_tclk_floor_rejects_its_frame_after_a_failed_commit_ram() {
        a_provisional_tclk_floor_rejects_its_frame_after_reboot(
            crate::security_store::RamSecurityStateStore::new(),
            AfterCommitFailure::KeepRunning,
        );
    }

    #[test]
    fn a_provisional_tclk_floor_rejects_its_frame_after_a_failed_commit_journal() {
        a_provisional_tclk_floor_rejects_its_frame_after_reboot(
            journal(),
            AfterCommitFailure::KeepRunning,
        );
    }

    #[test]
    fn a_terminal_handshake_failure_is_reported_from_the_tick_loop() {
        let mut device = commissioning_device();

        // The mock coordinator answers nothing, so every message budget runs
        // out and the exchange fails inside the overall deadline.
        let mut terminal = None;
        for _ in 0..512 {
            match tick(&mut device) {
                TickResult::Event(event) => {
                    terminal = Some(event);
                    break;
                }
                _ => advance_time(&mut device, 500_000),
            }
        }

        assert!(
            matches!(
                terminal,
                Some(StackEvent::CommissioningComplete { success: false })
            ),
            "a failed R21+ initial join must be reported, not silently retried"
        );
        assert!(!device.bdb().tclk_exchange_active());
        assert!(
            !device.is_joined(),
            "a failed initial join must never stay commissioned"
        );
    }
}
