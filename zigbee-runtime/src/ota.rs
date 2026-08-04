//! OTA Manager — runtime integration for OTA firmware upgrades.
//!
//! Combines the ZCL OTA cluster (state machine + command parsing) with
//! a FirmwareWriter (platform flash abstraction) to handle the complete
//! OTA upgrade flow.
//!
//! Key responsibilities:
//! - Erases flash slot before download starts
//! - Parses OTA image header from first blocks, validates manufacturer/image_type
//! - Strips OTA header + sub-element header, writes only firmware payload to flash
//! - Tracks actual firmware bytes written for correct verification
//!
//! Enabled with the `ota` feature flag.

use crate::event_loop::StackEvent;
use crate::firmware_writer::FirmwareWriter;
use zigbee_zcl::clusters::ota::{
    self, ImageBlockRequest, OtaAction, OtaCluster, OtaState, QueryNextImageRequest,
    UpgradeEndRequest,
};
use zigbee_zcl::clusters::ota_image::{OtaImageHeader, OtaSubElement, OtaTagId};
use zigbee_zcl::frame::ZclFrame;
use zigbee_zcl::{ClusterDirection, CommandId};

const OTA_RESPONSE_TIMEOUT_SECS: u32 = 120;
const OTA_BLOCK_RETRY_INTERVAL_SECS: u32 = 2;
const OTA_BLOCK_MAX_RETRIES: u8 = 3;

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
    /// Hardware version (included in QueryNextImageRequest if set).
    pub hardware_version: Option<u16>,
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
            hardware_version: None,
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

struct BlockRetry {
    request: ImageBlockRequest,
    zcl_seq: u8,
    elapsed_secs: u32,
    retries_sent: u8,
}

/// OTA Manager — coordinates OTA cluster + firmware writer.
///
/// Handles the OTA file format: parses the OTA image header from the
/// first received blocks, validates manufacturer/image_type, then strips
/// the header and sub-element overhead — writing only raw firmware bytes
/// to the flash slot.
pub struct OtaManager<F: FirmwareWriter> {
    /// OTA ZCL cluster (state machine + attributes).
    cluster: OtaCluster,
    /// Platform firmware writer.
    writer: F,
    /// OTA configuration.
    config: OtaConfig,
    /// Pending outgoing frame queued for the transport.
    pending_frame: Option<PendingOtaFrame>,
    /// ZCL sequence counter (borrowed from device).
    zcl_seq: u8,
    /// Download context — tracks header parsing and payload offset.
    download_ctx: OtaDownloadCtx,
    /// Whether an image query was accepted (for auto_accept=false mode).
    query_pending_accept: bool,
    /// Time spent waiting for the next OTA server response.
    response_wait_secs: u32,
    /// Logical retry state for the currently outstanding block request.
    block_retry: Option<BlockRetry>,
    /// Whether the verified image is waiting for application-controlled activation.
    activation_pending: bool,
}

/// Tracks OTA file header parsing and firmware write offset during download.
struct OtaDownloadCtx {
    /// Whether the OTA image header has been parsed from initial blocks.
    header_parsed: bool,
    /// Buffer for accumulating header bytes from early blocks.
    header_buf: heapless::Vec<u8, 128>,
    /// Bytes to skip at start of OTA file (header + sub-element headers before firmware).
    skip_bytes: u32,
    /// Actual firmware payload size (total_image_size - skip_bytes).
    firmware_size: u32,
    /// Firmware bytes actually written to flash.
    firmware_written: u32,
    /// Whether erase_slot() has been called.
    slot_erased: bool,
}

impl OtaDownloadCtx {
    fn new() -> Self {
        Self {
            header_parsed: false,
            header_buf: heapless::Vec::new(),
            skip_bytes: 0,
            firmware_size: 0,
            firmware_written: 0,
            slot_erased: false,
        }
    }

    fn reset(&mut self) {
        self.header_parsed = false;
        self.header_buf.clear();
        self.skip_bytes = 0;
        self.firmware_size = 0;
        self.firmware_written = 0;
        self.slot_erased = false;
    }
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
        if let Some(hw) = config.hardware_version {
            cluster.set_hardware_version(hw);
        }

        Self {
            cluster,
            writer,
            config,
            pending_frame: None,
            zcl_seq: 0,
            download_ctx: OtaDownloadCtx::new(),
            query_pending_accept: false,
            response_wait_secs: 0,
            block_retry: None,
            activation_pending: false,
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

    /// Get mutable access to the OTA cluster for runtime attribute dispatch.
    pub fn cluster_mut(&mut self) -> &mut OtaCluster {
        &mut self.cluster
    }

    pub const fn endpoint(&self) -> u8 {
        self.config.endpoint
    }

    /// Borrow the platform firmware writer (staging slot state, diagnostics).
    pub fn writer(&self) -> &F {
        &self.writer
    }

    /// Initiate an OTA image query.
    pub fn start_query(&mut self) -> Option<StackEvent> {
        self.response_wait_secs = 0;
        self.block_retry = None;
        self.activation_pending = false;
        let action = self.cluster.start_query();
        self.process_action(action)
    }

    /// Process an incoming OTA server→client command.
    ///
    /// `server_ieee` is the IEEE address of the sender (for UpgradeServerID).
    pub fn handle_incoming(
        &mut self,
        cmd_id: u8,
        payload: &[u8],
        server_ieee: Option<u64>,
    ) -> Option<StackEvent> {
        self.handle_incoming_with_sequence(cmd_id, payload, None, server_ieee)
    }

    /// Process an incoming OTA command with its ZCL transaction sequence.
    pub fn handle_incoming_with_sequence(
        &mut self,
        cmd_id: u8,
        payload: &[u8],
        zcl_seq: Option<u8>,
        server_ieee: Option<u64>,
    ) -> Option<StackEvent> {
        if cmd_id == ota::CMD_IMAGE_BLOCK_RESPONSE.0
            && payload.first().is_some_and(|status| *status != 0x00)
            && zcl_seq.is_some()
            && self.block_retry.as_ref().map(|retry| retry.zcl_seq) != zcl_seq
        {
            return None;
        }

        let outcome = self
            .cluster
            .process_server_command_with_outcome(cmd_id, payload);
        if outcome.accepted {
            self.response_wait_secs = 0;
            // Only a matching response proves that the outstanding request
            // reached the server. Stale blocks must not erase its retry copy.
            self.pending_frame = None;
            self.block_retry = None;
            if let Some(ieee) = server_ieee {
                self.cluster.set_upgrade_server_id(ieee);
            }
        }
        self.process_action(outcome.action)
    }

    /// Tick the OTA engine (called from runtime tick).
    pub fn tick(&mut self, elapsed_secs: u16) -> Option<StackEvent> {
        if matches!(
            self.cluster.state(),
            OtaState::QuerySent
                | OtaState::Downloading { .. }
                | OtaState::Verifying
                | OtaState::WaitingActivate
        ) {
            self.response_wait_secs = self.response_wait_secs.saturating_add(elapsed_secs as u32);
            if self.response_wait_secs >= OTA_RESPONSE_TIMEOUT_SECS {
                self.abort();
                return Some(StackEvent::OtaFailed);
            }
        } else {
            self.response_wait_secs = 0;
        }

        // Handle WaitForData countdown
        let action = self.cluster.tick(elapsed_secs);
        if !matches!(&action, OtaAction::None) {
            return self.process_action(action);
        }

        let retry_request = if self.pending_frame.is_none()
            && matches!(self.cluster.state(), OtaState::Downloading { .. })
        {
            self.block_retry.as_mut().and_then(|retry| {
                retry.elapsed_secs = retry.elapsed_secs.saturating_add(elapsed_secs as u32);
                if retry.elapsed_secs >= OTA_BLOCK_RETRY_INTERVAL_SECS
                    && retry.retries_sent < OTA_BLOCK_MAX_RETRIES
                {
                    retry.elapsed_secs = 0;
                    retry.retries_sent += 1;
                    Some((retry.request.clone(), retry.zcl_seq))
                } else {
                    None
                }
            })
        } else {
            None
        };
        if let Some((request, zcl_seq)) = retry_request {
            self.build_and_queue_block_request(&request, zcl_seq);
        }
        None
    }

    /// Take the pending outgoing frame (consumed by runtime to send via APS).
    pub fn take_pending_frame(&mut self) -> Option<PendingOtaFrame> {
        self.pending_frame.take()
    }

    /// Requeue a frame after a transient transport failure.
    ///
    /// OTA requests are idempotent because they carry the requested file
    /// offset. The response timeout remains the upper bound for retries.
    pub fn requeue_pending_frame(&mut self, frame: PendingOtaFrame) -> bool {
        if self.pending_frame.is_some() {
            return false;
        }
        self.pending_frame = Some(frame);
        true
    }

    /// Activate a verified image after the application has persisted any state
    /// that must survive the bootloader reset.
    pub fn activate(&mut self) -> Result<(), crate::firmware_writer::FirmwareError> {
        if !self.activation_pending {
            return Err(crate::firmware_writer::FirmwareError::ActivateFailed);
        }
        self.activation_pending = false;
        self.writer.activate()
    }

    /// Abort the current OTA.
    pub fn abort(&mut self) {
        self.cluster.abort();
        let _ = self.writer.abort();
        self.pending_frame = None;
        self.download_ctx.reset();
        self.query_pending_accept = false;
        self.response_wait_secs = 0;
        self.block_retry = None;
        self.activation_pending = false;
    }

    /// Accept a pending OTA image (for auto_accept=false mode).
    /// Call this after receiving OtaImageAvailable to start the download.
    pub fn accept_ota(&mut self) -> Option<StackEvent> {
        if self.query_pending_accept {
            self.query_pending_accept = false;
            // Re-query — the cluster will transition to Downloading
            self.start_query()
        } else {
            None
        }
    }

    /// Set the OTA server's IEEE address (UpgradeServerID attribute).
    pub fn set_upgrade_server_id(&mut self, ieee: u64) {
        self.cluster.set_upgrade_server_id(ieee);
    }

    /// Process an OtaAction into a StackEvent and/or queue an outgoing frame.
    fn process_action(&mut self, action: OtaAction) -> Option<StackEvent> {
        match action {
            OtaAction::SendQuery(req) => {
                // Reset download context for new OTA session
                self.download_ctx.reset();
                self.block_retry = None;
                self.build_and_queue_request(ota::CMD_QUERY_NEXT_IMAGE_REQUEST, &req);
                None
            }
            OtaAction::SendBlockRequest(req) => {
                // If auto_accept is false and this is the start of download,
                // pause and wait for app to call accept_ota()
                if !self.config.auto_accept
                    && req.file_offset == 0
                    && !self.download_ctx.slot_erased
                {
                    self.query_pending_accept = true;
                    let version = self.cluster.target_version();
                    let total = match self.cluster.state() {
                        OtaState::Downloading { total_size, .. } => total_size,
                        _ => 0,
                    };
                    self.cluster.abort(); // go back to idle until accepted
                    return Some(StackEvent::OtaImageAvailable {
                        version,
                        size: total,
                    });
                }

                // Erase slot before first block if not done yet
                if !self.download_ctx.slot_erased {
                    match self.writer.erase_slot() {
                        Ok(()) => {
                            log::info!("[OTA] Flash slot erased, ready for download");
                            self.download_ctx.slot_erased = true;
                        }
                        Err(e) => {
                            log::warn!("[OTA] Erase slot failed: {:?}", e);
                            let fail_action = self.cluster.mark_failed();
                            return self.process_action(fail_action);
                        }
                    }
                }
                let zcl_seq = self.next_seq();
                self.block_retry = Some(BlockRetry {
                    request: req.clone(),
                    zcl_seq,
                    elapsed_secs: 0,
                    retries_sent: 0,
                });
                self.build_and_queue_block_request(&req, zcl_seq);
                // Emit OtaImageAvailable on first block request (start of download)
                if req.file_offset == 0 {
                    let total = match self.cluster.state() {
                        OtaState::Downloading { total_size, .. } => total_size,
                        _ => 0,
                    };
                    let version = self.cluster.target_version();
                    return Some(StackEvent::OtaImageAvailable {
                        version,
                        size: total,
                    });
                }
                None
            }
            OtaAction::WriteBlock { offset, data } => match self.write_ota_block(offset, &data) {
                Ok(()) => {
                    let progress = self.cluster.progress_percent();
                    if self.cluster.is_download_complete() {
                        let verify_size = if self.download_ctx.firmware_size > 0 {
                            self.download_ctx.firmware_size
                        } else {
                            self.download_ctx.firmware_written
                        };
                        self.cluster.mark_download_complete();
                        match self.writer.verify(verify_size, None) {
                            Ok(()) => {
                                let action = self.cluster.mark_verified();
                                let status = self.process_action(action);
                                debug_assert!(status.is_none());
                                if status.is_some() {
                                    return status;
                                }
                            }
                            Err(e) => {
                                log::warn!("[OTA] Verify failed: {:?}", e);
                                let action = self.cluster.mark_failed();
                                return self.process_action(action);
                            }
                        }
                    } else {
                        // Queue the next stop-and-wait request now. The
                        // transport that delivered this block can send it
                        // before the application enters another poll cycle.
                        let action = self.cluster.next_block_request();
                        let followup = self.process_action(action);
                        debug_assert!(followup.is_none());
                        if followup.is_some() {
                            return followup;
                        }
                    }
                    Some(StackEvent::OtaProgress { percent: progress })
                }
                Err(e) => {
                    log::warn!("[OTA] Write failed at offset {}: {:?}", offset, e);
                    let fail_action = self.cluster.mark_failed();
                    self.process_action(fail_action);
                    Some(StackEvent::OtaFailed)
                }
            },
            OtaAction::SendEndRequest(req) => {
                self.block_retry = None;
                let failed = req.status != 0;
                self.build_and_queue_end_request(&req);
                failed.then_some(StackEvent::OtaFailed)
            }
            OtaAction::ActivateImage => {
                self.block_retry = None;
                self.activation_pending = true;
                Some(StackEvent::OtaComplete)
            }
            OtaAction::Wait(secs) => Some(StackEvent::OtaDelayedActivation { delay_secs: secs }),
            OtaAction::None => None,
        }
    }

    /// Write an OTA block, handling header parsing and payload stripping.
    ///
    /// The OTA file format is: [OTA header] [sub-element header] [firmware payload]
    /// We parse the header from the first block(s), validate manufacturer/image_type,
    /// then write only the firmware payload bytes to flash.
    fn write_ota_block(
        &mut self,
        ota_offset: u32,
        data: &[u8],
    ) -> Result<(), crate::firmware_writer::FirmwareError> {
        use crate::firmware_writer::FirmwareError;

        if !self.download_ctx.header_parsed {
            // Accumulate bytes for header parsing
            for &b in data {
                let _ = self.download_ctx.header_buf.push(b);
            }

            // Try to parse OTA header once we have minimum header bytes (56)
            if self.download_ctx.header_buf.len() >= 56 {
                match OtaImageHeader::parse(&self.download_ctx.header_buf) {
                    Ok((header, header_len)) => {
                        // Validate manufacturer and image type
                        if header.manufacturer_code != self.config.manufacturer_code {
                            log::warn!(
                                "[OTA] Manufacturer mismatch: got 0x{:04X}, expected 0x{:04X}",
                                header.manufacturer_code,
                                self.config.manufacturer_code
                            );
                            return Err(FirmwareError::VerifyFailed);
                        }
                        if header.image_type != self.config.image_type {
                            log::warn!(
                                "[OTA] Image type mismatch: got 0x{:04X}, expected 0x{:04X}",
                                header.image_type,
                                self.config.image_type
                            );
                            return Err(FirmwareError::VerifyFailed);
                        }

                        // Scan sub-element headers to find the UpgradeImage payload.
                        // OTA file layout: [header][sub-elem1][sub-elem2]...
                        // Each sub-element: tag(2) + length(4) + data(length)
                        let mut scan_offset = header_len;
                        let mut found_upgrade_image = false;
                        let mut fw_size = 0u32;

                        while scan_offset + 6 <= self.download_ctx.header_buf.len() {
                            match OtaSubElement::parse(&self.download_ctx.header_buf[scan_offset..])
                            {
                                Ok((sub_elem, _)) => {
                                    if sub_elem.tag == OtaTagId::UpgradeImage {
                                        // Found the firmware payload sub-element
                                        fw_size = sub_elem.length;
                                        found_upgrade_image = true;
                                        scan_offset += 6; // skip sub-element header
                                        break;
                                    }
                                    // Skip this sub-element entirely (header + data)
                                    scan_offset += 6 + sub_elem.length as usize;
                                }
                                Err(_) => break,
                            }
                        }

                        if !found_upgrade_image {
                            // Fallback: assume single sub-element right after header
                            scan_offset = header_len + 6;
                            fw_size = header.total_image_size.saturating_sub(scan_offset as u32);
                            log::warn!(
                                "[OTA] No UpgradeImage tag found, assuming single sub-element"
                            );
                        }

                        let skip = scan_offset as u32;

                        log::info!(
                            "[OTA] Header parsed: version=0x{:08X} header={}B skip={}B firmware={}B",
                            header.file_version,
                            header_len,
                            skip,
                            fw_size,
                        );

                        // Check firmware fits in flash slot
                        if fw_size > self.writer.slot_size() {
                            log::warn!(
                                "[OTA] Firmware too large: {}B > slot {}B",
                                fw_size,
                                self.writer.slot_size()
                            );
                            return Err(FirmwareError::ImageTooLarge);
                        }

                        self.download_ctx.skip_bytes = skip;
                        self.download_ctx.firmware_size = fw_size;
                        self.download_ctx.header_parsed = true;

                        // Write any payload bytes that are past the header in the buffer
                        let buf_len = self.download_ctx.header_buf.len() as u32;
                        if buf_len > skip {
                            let payload_start = skip as usize;
                            let buf_ref = &self.download_ctx.header_buf;
                            // Copy payload bytes to a temp buffer to avoid borrow conflict
                            let mut tmp = [0u8; 128];
                            let plen = buf_ref.len() - payload_start;
                            tmp[..plen].copy_from_slice(&buf_ref[payload_start..]);
                            self.writer.write_block(0, &tmp[..plen])?;
                            self.download_ctx.firmware_written = plen as u32;
                        }
                    }
                    Err(e) => {
                        log::warn!("[OTA] Header parse failed: {:?}", e);
                        return Err(FirmwareError::VerifyFailed);
                    }
                }
            }
            // Still accumulating header bytes — nothing to write yet
            return Ok(());
        }

        // Header already parsed — write firmware payload bytes
        let skip = self.download_ctx.skip_bytes;
        let block_end = ota_offset + data.len() as u32;

        if block_end <= skip {
            // Entire block is still in header/sub-element area — skip
            return Ok(());
        }

        let (data_start, flash_offset) = if ota_offset < skip {
            // Block partially overlaps header — write only the payload portion
            let data_skip = (skip - ota_offset) as usize;
            (&data[data_skip..], 0u32)
        } else {
            // Block is entirely in payload area
            (data, ota_offset - skip)
        };

        // Sanity check: don't write past firmware size
        let max_write = self.download_ctx.firmware_size.saturating_sub(flash_offset) as usize;
        let write_data = if data_start.len() > max_write {
            &data_start[..max_write]
        } else {
            data_start
        };

        if !write_data.is_empty() {
            self.writer.write_block(flash_offset, write_data)?;
            self.download_ctx.firmware_written = flash_offset + write_data.len() as u32;
        }

        Ok(())
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

    fn build_and_queue_block_request(&mut self, req: &ImageBlockRequest, zcl_seq: u8) {
        let mut frame = ZclFrame::new_cluster_specific(
            zcl_seq,
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
                cluster_id: zigbee_zcl::ClusterId::OTA_UPGRADE.0,
            });
        }
    }
}
