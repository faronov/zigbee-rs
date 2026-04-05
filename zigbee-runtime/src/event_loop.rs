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
use zigbee_aps::{ApsAddress, ApsAddressMode, ApsTxOptions};
use zigbee_mac::MacDriver;
use zigbee_types::ShortAddress;
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, CommandId};

use crate::UserAction;

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
    /// OTA: Upgrade completed successfully — reboot to apply.
    OtaComplete,
    /// OTA: Upgrade failed.
    OtaFailed,
    /// OTA: Server requested delayed activation — reboot after `delay_secs`.
    OtaDelayedActivation { delay_secs: u32 },
    /// Basic cluster: factory reset requested by coordinator.
    FactoryResetRequested,
    /// NWK Leave command received from coordinator — device should rejoin.
    LeaveRequested,
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
    /// BDB commissioning (steering/formation) failed.
    CommissioningFailed,
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
    pub async fn tick(
        &mut self,
        elapsed_secs: u16,
        clusters: &mut [crate::ClusterRef<'_>],
    ) -> TickResult {
        // Phase 1: Handle pending user actions
        if let Some(action) = self.pending_action.take() {
            return self.handle_action(action).await;
        }

        // Phase 2: Send any queued ZCL responses
        while let Some(resp) = self.pending_responses.pop() {
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
                log::warn!(
                    "[Runtime] ZCL response send failed: dst=0x{:04X} ep={} cluster=0x{:04X}",
                    resp.dst_addr.0,
                    resp.dst_endpoint,
                    resp.cluster_id,
                );
            }
        }

        // Phase 3: Only do reporting/maintenance if joined
        if !self.is_joined() {
            return TickResult::Idle;
        }

        // Phase 4: APS layer maintenance — ACK retransmission and fragment aging
        {
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

        // Phase 4b: NWK router maintenance — BTR aging, link status, routing expiry
        {
            let nwk = self.bdb.zdo_mut().aps_mut().nwk_mut();
            nwk.tick_router_maintenance(elapsed_secs);
            // Send link status if due (async operation)
            nwk.process_pending_routing().await;
        }

        // Phase 5: Tick the reporting engine timers
        self.reporting.tick(elapsed_secs);

        // Phase 5b: Handle F&B target request — set IdentifyTime on Identify cluster
        if let Some((ep, time_secs)) = self.bdb.fb_target_request.take() {
            for cr in clusters.iter_mut() {
                if cr.endpoint == ep && cr.cluster.cluster_id().0 == 0x0003 {
                    let _ = cr.cluster.attributes_mut().set(
                        zigbee_zcl::AttributeId(0x0000), // IdentifyTime
                        zigbee_zcl::data_types::ZclValue::U16(time_secs),
                    );
                    log::info!(
                        "[Runtime] F&B target: set IdentifyTime={}s on ep {}",
                        time_secs,
                        ep,
                    );
                    break;
                }
            }
        }

        // Phase 5c: Tick F&B initiator response window
        let _ = self.bdb.tick_finding_binding(elapsed_secs).await;

        // Phase 6: Check and send due attribute reports for each cluster
        for cr in clusters.iter() {
            let ep = cr.endpoint;
            let cid = cr.cluster.cluster_id().0;
            self.check_and_send_cluster_reports(ep, cid, cr.cluster.attributes())
                .await;
        }

        // Phase 7: Power management — SED auto-poll and sleep decision
        // Record that we had activity if reports were sent
        if !self.pending_responses.is_empty() {
            self.power.set_pending_tx(true);
        } else {
            self.power.set_pending_tx(false);
        }

        // Auto-poll for sleepy end devices
        let now_ms = (elapsed_secs as u32) * 1000; // approximate
        if self.is_sleepy() && self.power.should_poll(now_ms) {
            if let Ok(Some(_frame)) = self.bdb.zdo_mut().nwk_mut().mac_mut().mlme_poll().await {
                self.power.record_activity(now_ms);
            }
            self.power.record_poll(now_ms);
        }

        // Decide sleep vs stay awake
        match self.power.decide(now_ms) {
            crate::power::SleepDecision::StayAwake => TickResult::Idle,
            crate::power::SleepDecision::LightSleep(ms) => TickResult::RunAgain(ms),
            crate::power::SleepDecision::DeepSleep(ms) => TickResult::RunAgain(ms),
        }
    }

    /// Handle a user-initiated action.
    async fn handle_action(&mut self, action: UserAction) -> TickResult {
        match action {
            UserAction::Join => {
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
                match self.rejoin().await {
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
    ) -> Result<(), ()> {
        if !self.is_joined() {
            return Err(());
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
            let _ = zcl_frame.payload.push(b);
        }

        // Serialize ZCL frame
        let mut zcl_buf = [0u8; 128];
        let zcl_len = match zcl_frame.serialize(&mut zcl_buf) {
            Ok(len) => len,
            Err(_) => return Err(()),
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
                Err(())
            }
        }
    }
}
