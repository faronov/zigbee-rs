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
pub async fn stack_tick<M: MacDriver>(
    device: &mut crate::ZigbeeDevice<M>,
    elapsed_secs: u16,
    clusters: &mut [crate::ClusterRef<'_>],
) -> TickResult {
    device.tick(elapsed_secs, clusters).await
}

impl<M: MacDriver> crate::ZigbeeDevice<M> {
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

        self.reporting.tick(elapsed_secs);
        self.apply_fb_target_request(clusters);
        self.run_finding_binding_tick(elapsed_secs).await;
        self.send_due_reports(clusters).await;
        self.update_pending_tx_flag();

        let now_ms = self.advance_power_clock(elapsed_secs);
        let result = if let Some(event) = self.run_sleepy_poll(now_ms, clusters).await {
            TickResult::Event(event)
        } else {
            self.tick_power_state(now_ms)
        };
        self.advance_commissioning(result).await
    }

    /// Advance post-network commissioning without a durable security store.
    ///
    /// This is the platform-independent equivalent of GSDK's scheduled
    /// update-tc-link-key event: normal ZDO/ZCL and polling work runs first,
    /// then one bounded security step is performed.
    async fn advance_commissioning(&mut self, result: TickResult) -> TickResult {
        if matches!(result, TickResult::Event(_)) || !self.bdb.tclk_exchange_active() {
            return result;
        }

        match self.bdb.advance_tclk_exchange(None).await {
            zigbee_bdb::TclkProgress::InProgress => {
                if matches!(result, TickResult::Idle) {
                    TickResult::RunAgain(Self::COMMISSIONING_POLL_MS)
                } else {
                    result
                }
            }
            zigbee_bdb::TclkProgress::Complete => {
                self.state_dirty = true;
                TickResult::Event(StackEvent::CommissioningComplete { success: true })
            }
            zigbee_bdb::TclkProgress::Failed(_) => {
                self.mark_left();
                TickResult::Event(StackEvent::CommissioningComplete { success: false })
            }
        }
    }

    pub(crate) async fn tick_without_secure_rejoin(
        &mut self,
        elapsed_secs: u16,
        clusters: &mut [crate::ClusterRef<'_>],
    ) -> TickResult {
        // Phase 1: Handle pending user actions
        if let Some(action) = self.pending_action.take() {
            return self.handle_action(action).await;
        }

        self.flush_pending_responses().await;

        // Phase 3: Only do reporting/maintenance if joined
        if !self.is_joined() {
            return TickResult::Idle;
        }

        self.tick_joined(elapsed_secs, clusters).await
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
    async fn flush_pending_responses(&mut self) {
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
    async fn tick_joined(
        &mut self,
        elapsed_secs: u16,
        clusters: &mut [crate::ClusterRef<'_>],
    ) -> TickResult {
        self.run_aps_maintenance().await;
        self.run_nwk_maintenance(elapsed_secs).await;

        self.reporting.tick(elapsed_secs);
        self.apply_fb_target_request(clusters);
        self.run_finding_binding_tick(elapsed_secs).await;
        self.send_due_reports(clusters).await;
        self.update_pending_tx_flag();

        let now_ms = self.advance_power_clock(elapsed_secs);
        if let Some(event) = self.run_sleepy_poll(now_ms, clusters).await {
            return TickResult::Event(event);
        }
        self.tick_power_state(now_ms)
    }

    #[inline(never)]
    async fn run_aps_maintenance(&mut self) {
        let aps = self.bdb.zdo_mut().aps_mut();
        let retransmit_frames = aps.age_ack_table();
        for frame in retransmit_frames.iter() {
            let _ = aps
                .nwk_mut()
                .nlde_data_request(zigbee_types::ShortAddress(0xFFFF), 0, frame, true, false)
                .await;
        }
        aps.age_dup_table();
        aps.fragment_rx_mut().age_entries();
    }

    #[inline(never)]
    async fn run_nwk_maintenance(&mut self, elapsed_secs: u16) {
        let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
        nwk.tick_router_maintenance(elapsed_secs);
        nwk.process_pending_routing().await;
    }

    #[inline(never)]
    fn apply_fb_target_request(&mut self, clusters: &mut [crate::ClusterRef<'_>]) {
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
    async fn run_finding_binding_tick(&mut self, elapsed_secs: u16) {
        let _ = self.bdb.tick_finding_binding(elapsed_secs).await;
    }

    #[inline(never)]
    async fn send_due_reports(&mut self, clusters: &[crate::ClusterRef<'_>]) {
        for cr in clusters.iter() {
            let ep = cr.endpoint;
            let cid = cr.cluster.cluster_id().0;
            self.check_and_send_cluster_reports(ep, cid, cr.cluster.attributes())
                .await;
        }
    }

    #[inline(never)]
    fn update_pending_tx_flag(&mut self) {
        self.power
            .set_pending_tx(!self.pending_responses.is_empty());
    }

    #[inline(never)]
    fn advance_power_clock(&mut self, elapsed_secs: u16) -> u32 {
        self.power_now_ms = advance_millis(self.power_now_ms, elapsed_secs);
        self.power_now_ms
    }

    #[inline(never)]
    async fn run_sleepy_poll(
        &mut self,
        now_ms: u32,
        clusters: &mut [crate::ClusterRef<'_>],
    ) -> Option<StackEvent> {
        if automatic_poll_due(
            self.automatic_polling,
            self.is_sleepy(),
            self.bdb.tclk_exchange_active(),
            self.power.should_poll(now_ms),
        ) {
            let indication = self.poll().await;
            if let Ok(Some(frame)) = indication {
                return self.process_incoming(&frame, clusters).await;
            }
        }
        None
    }

    #[inline(never)]
    fn tick_power_state(&mut self, now_ms: u32) -> TickResult {
        match self.power.decide(now_ms) {
            crate::power::SleepDecision::StayAwake => TickResult::Idle,
            crate::power::SleepDecision::LightSleep(ms) => TickResult::RunAgain(ms),
            crate::power::SleepDecision::DeepSleep(ms) => TickResult::RunAgain(ms),
        }
    }

    /// Handle a user-initiated action.
    #[inline(never)]
    async fn handle_action(&mut self, action: UserAction) -> TickResult {
        match action {
            UserAction::Join => {
                if self.secure_rejoin_pending() {
                    return self.retry_secure_rejoin().await;
                }
                log::info!("[Runtime] User action: Join");
                match self.start().await {
                    Ok(addr) => {
                        // Send ED Timeout Request to parent (max ~11 days)
                        self.send_ed_timeout_request().await;
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
    use super::{advance_millis, automatic_poll_due};

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
}
