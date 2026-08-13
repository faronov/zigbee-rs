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

use zigbee_aps::apsde::ApsdeDataRequest;
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsStatus, ApsTxOptions};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, CommandId};

use crate::UserAction;

fn advance_millis(now_ms: u32, elapsed_secs: u16) -> u32 {
    now_ms.wrapping_add((elapsed_secs as u32) * 1000)
}

fn automatic_poll_due(
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
    /// Basic cluster: factory reset requested by coordinator.
    FactoryResetRequested,
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
        self.apply_fb_target_request(clusters);
        self.run_finding_binding_tick(elapsed_secs).await;
        self.send_due_reports(clusters).await;
        self.update_pending_tx_flag();

        // GSDK-style event-driven commissioning: advance the unique Trust
        // Center link-key handshake before normal tick result generation.
        // A terminal transition is returned immediately. If it is still in
        // progress, continue through polling and preserve any application event
        // produced there for this tick.
        if self.bdb.tclk_exchange_active()
            && let Some(event) = self.advance_commissioning().await
        {
            return TickResult::Event(event);
        }

        let now_ms = self.advance_power_clock(elapsed_secs);
        // The poll runs first so an End Device Timeout Response that arrives
        // this tick cancels the response wait before it is serviced.
        let poll_event = self.run_sleepy_poll(now_ms, clusters).await;
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
        aps.fragment_rx_mut().age_entries();
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
        nwk.process_pending_routing().await;
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
            nwk.process_pending_routing().await;
            evicted
        };
        for child in evicted {
            self.forget_evicted_child(child);
        }
        let _ = self.service_parent_commands_inner().await;
        self.bdb
            .zdo_mut()
            .tick_parent_annce_transactions(elapsed_secs);
    }

    #[inline(never)]
    pub(crate) fn apply_fb_target_request(&mut self, clusters: &mut [crate::ClusterRef<'_>]) {
        if let Some((ep, time_secs)) = self.bdb.fb_target_request.take()
            && self
                .with_cluster_mut(ep, zigbee_zcl::ClusterId::IDENTIFY, clusters, |cluster| {
                    cluster.attributes_mut().set(
                        zigbee_zcl::AttributeId(0x0000),
                        zigbee_zcl::data_types::ZclValue::U16(time_secs),
                    )
                })
                .is_some()
        {
            log::info!(
                "[Runtime] F&B target: set IdentifyTime={}s on ep {}",
                time_secs,
                ep,
            );
        }
    }

    #[inline(never)]
    pub(crate) async fn run_finding_binding_tick(&mut self, elapsed_secs: u16) {
        let _ = self.bdb.tick_finding_binding(elapsed_secs).await;
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
    #[inline(never)]
    pub(crate) async fn handle_action(&mut self, action: UserAction) -> TickResult {
        match action {
            UserAction::Join => {
                if self.secure_rejoin_pending() {
                    return self.retry_secure_rejoin().await;
                }
                log::info!("[Runtime] User action: Join");
                match self.start().await {
                    Ok(addr) => {
                        // `start()` owns the single initial End Device Timeout
                        // Request for this join.
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
            UserAction::Toggle => {
                if self.is_joined() {
                    log::info!("[Runtime] User action: Toggle → Leave");
                    let _ = self.leave().await;
                    TickResult::Event(StackEvent::Left)
                } else {
                    if self.secure_rejoin_pending() {
                        return self.retry_secure_rejoin().await;
                    }
                    log::info!("[Runtime] User action: Toggle → Join");
                    match self.start().await {
                        Ok(addr) => {
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
    /// Serializes the report into a ZCL frame and sends via APS→NWK→MAC.
    pub async fn send_report(
        &mut self,
        endpoint: u8,
        cluster_id: u16,
        report: &zigbee_zcl::foundation::reporting::ReportAttributes,
    ) -> Result<(), SendError> {
        if !self.is_joined() {
            return Err(SendError::NotJoined);
        }

        // Build ZCL Report Attributes frame (command 0x0A, server→client)
        let seq = self.next_zcl_seq();
        let mut zcl_frame = ZclFrame::new_global(
            seq,
            CommandId(0x0A), // Report Attributes
            ClusterDirection::ServerToClient,
            true, // disable default response
        );

        // Serialize report payload into ZCL frame
        let mut payload_buf = [0u8; 128];
        let payload_len = report.serialize(&mut payload_buf);
        for &b in &payload_buf[..payload_len] {
            zcl_frame
                .payload
                .push(b)
                .map_err(|_| SendError::PayloadTooLong)?;
        }

        // Serialize ZCL frame
        let mut zcl_buf = [0u8; 128];
        let zcl_len = match zcl_frame.serialize(&mut zcl_buf) {
            Ok(len) => len,
            Err(_) => return Err(SendError::Serialization),
        };

        // Send via APS to the coordinator (0x0000)
        let req = ApsdeDataRequest {
            dst_addr_mode: ApsAddressMode::Short,
            dst_address: ApsAddress::Short(ShortAddress::COORDINATOR),
            dst_endpoint: endpoint,
            profile_id: 0x0104, // Home Automation
            cluster_id,
            src_endpoint: endpoint,
            payload: &zcl_buf[..zcl_len],
            tx_options: ApsTxOptions {
                use_nwk_key: true,
                ..ApsTxOptions::default()
            },
            radius: 0,
            alias_src_addr: None,
            alias_seq: None,
        };

        match self.bdb.zdo_mut().aps_mut().apsde_data_request(&req).await {
            Ok(_) => {
                log::debug!(
                    "[Runtime] Report sent: ep={} cluster=0x{:04X}",
                    endpoint,
                    cluster_id
                );
                Ok(())
            }
            Err(e) => {
                log::warn!("[Runtime] Report send failed: {:?}", e);
                Err(SendError::Aps(e))
            }
        }
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

#[cfg(test)]
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
