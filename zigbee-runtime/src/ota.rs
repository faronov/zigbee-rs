//! OTA Manager — runtime integration for OTA firmware upgrades.
//!
//! Combines the ZCL OTA cluster (state machine + command parsing) with
//! a FirmwareWriter (platform flash abstraction) to handle the complete
//! OTA upgrade flow.
//!
//! Enabled with the `ota` feature flag.

use crate::event_loop::StackEvent;
use crate::firmware_writer::FirmwareWriter;
use zigbee_zcl::clusters::ota::{
    self, ImageBlockRequest, OtaAction, OtaCluster, OtaState, QueryNextImageRequest,
    UpgradeEndRequest,
};
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, CommandId};

/// OTA configuration.
#[derive(Debug, Clone)]
pub struct OtaConfig {
    /// Manufacturer code for this device.
    pub manufacturer_code: u16,
    /// Image type for this device.
    pub image_type: u16,
    /// Current firmware version.
    pub current_version: u32,
    /// Endpoint where OTA cluster lives.
    pub endpoint: u8,
    /// Block size for image requests (default: 48).
    pub block_size: u8,
    /// Auto-accept OTA images (if false, app must call accept_ota()).
    pub auto_accept: bool,
}

impl Default for OtaConfig {
    fn default() -> Self {
        Self {
            manufacturer_code: 0x0000,
            image_type: 0x0000,
            current_version: 0x00000001,
            endpoint: 1,
            block_size: ota::DEFAULT_BLOCK_SIZE,
            auto_accept: true,
        }
    }
}

/// Pending OTA ZCL frame to be sent.
pub struct PendingOtaFrame {
    /// Serialized ZCL frame bytes.
    pub zcl_data: heapless::Vec<u8, 128>,
    /// Source/destination endpoint.
    pub endpoint: u8,
    /// Cluster ID (always 0x0019).
    pub cluster_id: u16,
}

/// OTA Manager — coordinates OTA cluster + firmware writer.
pub struct OtaManager<F: FirmwareWriter> {
    /// OTA ZCL cluster (state machine + attributes).
    cluster: OtaCluster,
    /// Platform firmware writer.
    writer: F,
    /// OTA configuration.
    config: OtaConfig,
    /// Pending outgoing frame (queued for sending in tick()).
    pending_frame: Option<PendingOtaFrame>,
    /// Whether we need to request the next block after a write.
    need_next_block: bool,
    /// ZCL sequence counter (borrowed from device).
    zcl_seq: u8,
}

impl<F: FirmwareWriter> OtaManager<F> {
    /// Create a new OTA manager.
    pub fn new(writer: F, config: OtaConfig) -> Self {
        let mut cluster = OtaCluster::new(
            config.manufacturer_code,
            config.image_type,
            config.current_version,
        );
        cluster.set_block_size(config.block_size);

        Self {
            cluster,
            writer,
            config,
            pending_frame: None,
            need_next_block: false,
            zcl_seq: 0,
        }
    }

    fn next_seq(&mut self) -> u8 {
        let s = self.zcl_seq;
        self.zcl_seq = self.zcl_seq.wrapping_add(1);
        s
    }

    /// Get the current OTA state.
    pub fn state(&self) -> OtaState {
        self.cluster.state()
    }

    /// Get download progress (0-100%).
    pub fn progress(&self) -> u8 {
        self.cluster.progress_percent()
    }

    /// Get the OTA cluster (for attribute reads).
    pub fn cluster(&self) -> &OtaCluster {
        &self.cluster
    }

    /// Initiate an OTA image query.
    pub fn start_query(&mut self) -> Option<StackEvent> {
        let action = self.cluster.start_query();
        self.process_action(action)
    }

    /// Process an incoming OTA server→client command.
    pub fn handle_incoming(&mut self, cmd_id: u8, payload: &[u8]) -> Option<StackEvent> {
        let action = self.cluster.process_server_command(cmd_id, payload);
        self.process_action(action)
    }

    /// Tick the OTA engine (called from runtime tick).
    pub fn tick(&mut self, elapsed_secs: u16) -> Option<StackEvent> {
        // Handle pending next-block request after a write
        if self.need_next_block {
            self.need_next_block = false;

            // Check if download is complete
            if self.cluster.is_download_complete() {
                self.cluster.mark_download_complete();
                // Verify the image
                match self
                    .writer
                    .verify(self.cluster.state().download_total(), None)
                {
                    Ok(()) => {
                        let action = self.cluster.mark_verified();
                        return self.process_action(action);
                    }
                    Err(e) => {
                        log::warn!("[OTA] Verify failed: {:?}", e);
                        let _ = self.writer.abort();
                        let action = self.cluster.mark_failed();
                        return self.process_action(action);
                    }
                }
            }

            // Request next block
            let action = self.cluster.next_block_request();
            return self.process_action(action);
        }

        // Handle WaitForData countdown
        let action = self.cluster.tick(elapsed_secs);
        self.process_action(action)
    }

    /// Take the pending outgoing frame (consumed by runtime to send via APS).
    pub fn take_pending_frame(&mut self) -> Option<PendingOtaFrame> {
        self.pending_frame.take()
    }

    /// Abort the current OTA.
    pub fn abort(&mut self) {
        self.cluster.abort();
        let _ = self.writer.abort();
        self.pending_frame = None;
        self.need_next_block = false;
    }

    /// Process an OtaAction into a StackEvent and/or queue an outgoing frame.
    fn process_action(&mut self, action: OtaAction) -> Option<StackEvent> {
        match action {
            OtaAction::SendQuery(req) => {
                self.build_and_queue_request(ota::CMD_QUERY_NEXT_IMAGE_REQUEST, &req);
                None
            }
            OtaAction::SendBlockRequest(req) => {
                self.build_and_queue_block_request(&req);
                None
            }
            OtaAction::WriteBlock { offset, data } => {
                match self.writer.write_block(offset, &data) {
                    Ok(()) => {
                        self.need_next_block = true;
                        let progress = self.cluster.progress_percent();
                        Some(StackEvent::OtaProgress { percent: progress })
                    }
                    Err(e) => {
                        log::warn!("[OTA] Write failed at offset {}: {:?}", offset, e);
                        let _ = self.writer.abort();
                        let fail_action = self.cluster.mark_failed();
                        // Process the fail action (sends UpgradeEndRequest with error)
                        self.process_action(fail_action);
                        Some(StackEvent::OtaFailed)
                    }
                }
            }
            OtaAction::SendEndRequest(req) => {
                self.build_and_queue_end_request(&req);
                None
            }
            OtaAction::ActivateImage => match self.writer.activate() {
                Ok(()) => Some(StackEvent::OtaComplete),
                Err(e) => {
                    log::warn!("[OTA] Activate failed: {:?}", e);
                    Some(StackEvent::OtaFailed)
                }
            },
            OtaAction::Wait(_secs) => None,
            OtaAction::None => None,
        }
    }

    fn build_and_queue_request(&mut self, cmd_id: CommandId, req: &QueryNextImageRequest) {
        let seq = self.next_seq();
        let mut frame =
            ZclFrame::new_cluster_specific(seq, cmd_id, ClusterDirection::ClientToServer, false);
        let mut buf = [0u8; 16];
        let len = req.serialize(&mut buf);
        for &b in &buf[..len] {
            let _ = frame.payload.push(b);
        }
        self.queue_frame(frame);
    }

    fn build_and_queue_block_request(&mut self, req: &ImageBlockRequest) {
        let seq = self.next_seq();
        let mut frame = ZclFrame::new_cluster_specific(
            seq,
            ota::CMD_IMAGE_BLOCK_REQUEST,
            ClusterDirection::ClientToServer,
            false,
        );
        let mut buf = [0u8; 16];
        let len = req.serialize(&mut buf);
        for &b in &buf[..len] {
            let _ = frame.payload.push(b);
        }
        self.queue_frame(frame);
    }

    fn build_and_queue_end_request(&mut self, req: &UpgradeEndRequest) {
        let seq = self.next_seq();
        let mut frame = ZclFrame::new_cluster_specific(
            seq,
            ota::CMD_UPGRADE_END_REQUEST,
            ClusterDirection::ClientToServer,
            false,
        );
        let mut buf = [0u8; 12];
        let len = req.serialize(&mut buf);
        for &b in &buf[..len] {
            let _ = frame.payload.push(b);
        }
        self.queue_frame(frame);
    }

    fn queue_frame(&mut self, frame: ZclFrame) {
        let mut zcl_buf = [0u8; 128];
        if let Ok(len) = frame.serialize(&mut zcl_buf) {
            let mut data = heapless::Vec::new();
            for &b in &zcl_buf[..len] {
                let _ = data.push(b);
            }
            self.pending_frame = Some(PendingOtaFrame {
                zcl_data: data,
                endpoint: self.config.endpoint,
                cluster_id: 0x0019,
            });
        }
    }
}
